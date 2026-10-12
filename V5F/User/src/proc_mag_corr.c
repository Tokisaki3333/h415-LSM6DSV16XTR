/* 处理函数 4：重力环 + 两级地磁修正
 *
 * 入口选择（A/B 严格互斥，否则同一观测量被两个通道重复注入）：
 *   门开（重力可信）  -> 入口A：只修偏航（倾斜交给重力环，磁不动它）
 *   门关（重力丢失）  -> 入口B：3D 矢量（磁成为唯一绝对参考，必须管倾斜）
 *
 * 入口A（仅偏航）：
 *   pb = R(q)·BM_N，取水平投影
 *   dpsi = atan2(hb.x*hm.y - hb.y*hm.x, hb.x*hm.x + hb.y*hm.y)
 *   q <- quat(cos(by),0,0,sin(by)) ⊗ q,  by = 0.5*K*dpsi
 *   ★ 限制：水平投影在大角度倾斜下退化（实测 |H|/|M| < 0.2 占 1.9~4.2%），
 *     所以入口A 的服务窗口必须短 —— 靠协方差门控限制，而非靠它自己撑住。
 *
 * 入口B（3D 矢量，世界系左乘）：
 *   pb = R(q)·BM_N
 *   z  = cross(pb, y_meas)          <- 新息，正比于世界系姿态误差
 *   aa = atan2(|z|, dot(pb, y_meas))
 *   q  <- quat(cos(h), u*sin(h)) ⊗ q,  h = 0.5*K*aa
 *   ★ 符号经单帧验证：z=(R·BM)×y 时残差从 18.11->0.300、11.85->0.084、
 *     28.22->1.128（K=1.0）；反向定义会使残差变大。
 *   ★ K_MAX < 1 是硬约束：K=1 时姿态输出完全等于磁强计读数。
 *
 * 重力环（与磁无关，独立工作）：
 *   pb = R(q)·UP_N, av = normalize(a)
 *   aa = atan2(|cross|, dot)，世界系左乘 0.5*K_GRAV*aa
 *   ★ K_GRAV = 0.10 经增益扫描确认最优：降增益不减小抖动，反而使静态偏差增大。
 *
 * 全部状态为文件静态，仅被 V5F 处理主循环单线程调用，无需加锁。 */
#include "proc.h"
#include "proc_tune.h"
#include "proc_tune.h"
#include "mipc_shm.h"
#include <math.h>

#define DEG2RAD     0.0174532925f
#define RAD2DEG     57.29577951f

static const float s_mag_ref[3] = { MAG_REF_NX, MAG_REF_NY, MAG_REF_NZ };

/* 世界系左乘小旋转：q <- dq ⊗ q */
static void q_world_rotate(float ux, float uy, float uz, float angle)
{
    float h = 0.5f * angle;
    float c = cosf(h), s_ = sinf(h);
    float dw = c, dx = ux * s_, dy = uy * s_, dz = uz * s_;
    float w_ = g_proc.q[0], x_ = g_proc.q[1];
    float y_ = g_proc.q[2], z_ = g_proc.q[3];
    float nw = dw*w_ - dx*x_ - dy*y_ - dz*z_;
    float nx = dw*x_ + dx*w_ + dy*z_ - dz*y_;
    float ny = dw*y_ - dx*z_ + dy*w_ + dz*x_;
    float nz = dw*z_ + dx*y_ - dy*x_ + dz*w_;
    float sn = 1.0f / sqrtf(nw*nw + nx*nx + ny*ny + nz*nz);

    g_proc.q[0] = nw * sn; g_proc.q[1] = nx * sn;
    g_proc.q[2] = ny * sn; g_proc.q[3] = nz * sn;
}

/* 把导航系矢量转到体坐标：R(q)^T · v */
static void nav_to_body(const float *v, float *out)
{
    float w_ = g_proc.q[0], x_ = g_proc.q[1];
    float y_ = g_proc.q[2], z_ = g_proc.q[3];
    float r00 = 1.0f - 2.0f*(y_*y_ + z_*z_);
    float r01 = 2.0f*(x_*y_ + w_*z_);
    float r02 = 2.0f*(x_*z_ - w_*y_);
    float r10 = 2.0f*(x_*y_ - w_*z_);
    float r11 = 1.0f - 2.0f*(x_*x_ + z_*z_);
    float r12 = 2.0f*(y_*z_ + w_*x_);
    float r20 = 2.0f*(x_*z_ + w_*y_);
    float r21 = 2.0f*(y_*z_ - w_*x_);
    float r22 = 1.0f - 2.0f*(x_*x_ + y_*y_);

    out[0] = r00*v[0] + r10*v[1] + r20*v[2];
    out[1] = r01*v[0] + r11*v[1] + r21*v[2];
    out[2] = r02*v[0] + r12*v[1] + r22*v[2];
}

void proc_mag_corr_init(void)
{
    g_proc.mag_bad_acc  = 0u;
    g_proc.grav_err_deg = 0.0f;
    g_proc.grav_applied = 0u;
}

/* 每 IMU 帧调用：先重力环，再按入口修正 */
void proc_mag_corr_step(const volatile shared_mem_t *shm, float dt_s)
{
    float av[3], up[3] = { 0.0f, 0.0f, 1.0f };
    float pb[3];
    float an, k;

    (void)dt_s;
    if (g_proc.q_valid == 0u) return;
    g_proc.grav_applied = 0u;               /* 先清零，正常路径里置 1 */

    /* ---- 加计可信性（削顶/大加速度时重力与磁都不可信） ---- */
    av[0] = (float)shm->imu.ax * ACCEL_G_PER_LSB;
    av[1] = (float)shm->imu.ay * ACCEL_G_PER_LSB;
    av[2] = (float)shm->imu.az * ACCEL_G_PER_LSB;
    an = sqrtf(av[0]*av[0] + av[1]*av[1] + av[2]*av[2]);
    g_proc.mag_bad_acc = (fabsf(an - 1.0f) > MAG_ACC_LIMIT_G) ? 1u : 0u;

    /* ---- 重力环：只要有可信加计就工作，与门控无关 ---- */
    if (g_proc.mag_bad_acc == 0u && an > 1e-3f)
    {
        float cr[3], nn, aa;
        av[0] /= an; av[1] /= an; av[2] /= an;
        nav_to_body(up, pb);
        cr[0] = pb[1]*av[2] - pb[2]*av[1];
        cr[1] = pb[2]*av[0] - pb[0]*av[2];
        cr[2] = pb[0]*av[1] - pb[1]*av[0];
        nn = sqrtf(cr[0]*cr[0] + cr[1]*cr[1] + cr[2]*cr[2]);
        if (nn > 1e-6f)
        {
            aa = atan2f(nn, pb[0]*av[0] + pb[1]*av[1] + pb[2]*av[2]);
            g_proc.grav_err_deg = aa * RAD2DEG;   /* 上报本帧新息（估计 up 与实测 a 的夹角） */
            /* ★ 符号开关 GRAV_CORR_SIGN：
             *   推导上 cr = pb × av 是正确的转轴（把估计的 up 转向实测 a）。
             *   若实测方向相反（表现为"倾斜后立刻被拉向反方向"），
             *   把 GRAV_CORR_SIGN 改成 -1.0f 即可，无需改结构。 */
            q_world_rotate(cr[0]/nn, cr[1]/nn, cr[2]/nn,
                           GRAV_CORR_SIGN * CORR_K_GRAV * aa);
            g_proc.grav_applied = 1u;       /* ★ 重力矢量本帧参与了修正 */
        }
    }

    /* ---- 地磁修正 ---- */
    /* ★ 地磁隔离开关：置 0 时完全不做磁修正（用于对照实验：抖动是否消失） */
    if (g_proc.mag_corr_en == 0u) { g_proc.mag_fix = MAG_FIX_OFF; return; }
    if (g_proc.mag_valid == 0u)  { g_proc.mag_fix = MAG_FIX_OFF; return; }

    k = g_proc.K;
    nav_to_body(s_mag_ref, pb);

    if (g_proc.entry_B != 0u)
    {
        /* 入口B：3D 矢量，世界系左乘 */
        float y_[3], cr[3], nn, aa;

        nn = sqrtf(g_proc.mag_norm_g);
        if (nn < 1e-6f) { g_proc.mag_fix = MAG_FIX_OFF; return; }
        y_[0] = g_proc.mag_g[0] / nn;
        y_[1] = g_proc.mag_g[1] / nn;
        y_[2] = g_proc.mag_g[2] / nn;

        cr[0] = pb[1]*y_[2] - pb[2]*y_[1];
        cr[1] = pb[2]*y_[0] - pb[0]*y_[2];
        cr[2] = pb[0]*y_[1] - pb[1]*y_[0];
        nn = sqrtf(cr[0]*cr[0] + cr[1]*cr[1] + cr[2]*cr[2]);
        if (nn > 1e-6f)
        {
            aa = atan2f(nn, pb[0]*y_[0] + pb[1]*y_[1] + pb[2]*y_[2]);
            q_world_rotate(cr[0]/nn, cr[1]/nn, cr[2]/nn, k * aa);
        }
        g_proc.mag_fix = MAG_FIX_ENTRY_B;
    }
    else
    {
        /* 入口A：仅偏航（绕世界 z 轴左乘）。水平投影退化则本帧失能。 */
        float hb[2], hm[2], nb, nm, dpsi;

        hb[0] = pb[0]; hb[1] = pb[1];
        hm[0] = g_proc.mag_g[0]; hm[1] = g_proc.mag_g[1];
        nb = sqrtf(hb[0]*hb[0] + hb[1]*hb[1]);
        nm = sqrtf(hm[0]*hm[0] + hm[1]*hm[1]);

        if (nb < MAG_MIN_PERP || nm < MAG_MIN_PERP)
        {
            g_proc.mag_fix = MAG_FIX_OFF;
            return;
        }
        hb[0] /= nb; hb[1] /= nb;
        hm[0] /= nm; hm[1] /= nm;
        dpsi = atan2f(hb[0]*hm[1] - hb[1]*hm[0], hb[0]*hm[0] + hb[1]*hm[1]);
        q_world_rotate(0.0f, 0.0f, 1.0f, k * dpsi);
        g_proc.mag_fix = MAG_FIX_ENTRY_A;
    }
}
