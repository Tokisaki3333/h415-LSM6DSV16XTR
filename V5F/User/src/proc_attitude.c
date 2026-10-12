/* 处理函数 3：姿态传播 + 两级门控（协方差标量）
 *
 * 传播    q <- q ⊗ dq(w·dt)     体坐标右乘（局部增量）
 * 门控    Pg 是唯一的标量，两条规则：
 *           有重力帧：Pg *= (1 - PG_DECAY)，下限 PG_FLOOR
 *           无重力帧：Pg += (INFL * SG_PPM * max(|w|, W_FLOOR) * DEG2RAD)^2 * dt
 *                     上限 PG_CEIL
 *         entry_B = (Pg > PG_THR)
 *         K       = clamp(Pg / (Pg + SIG_M2), K_MIN, K_MAX)
 *
 * ★ 为什么 Pg 必须饱和：继续增长会让 K -> 1，修正律退化成"把姿态精确转到磁
 *   矢量方向"，姿态输出完全等于磁强计读数，磁噪声与失真 1:1 灌进姿态。
 * ★ 为什么膨胀率来自实测：SG_PPM = 246 ppm 是本次实测的陀螺标度误差
 *   （门 B 关闭 19.857 s，倾角误差 0.045 -> 7.364 度）。按此预测 19.857 s 后
 *   误差 7.319 度，与实测 7.364 度偏差 0.6%。
 *
 * 全部状态为文件静态，仅被 V5F 处理主循环单线程调用，无需加锁。 */
#include "proc.h"
#include "proc_tune.h"
#include "proc_tune.h"
#include <math.h>

#define DEG2RAD     0.0174532925f
#define RAD2DEG     57.29577951f

static float s_Pg;          /* rad^2 */

void proc_attitude_init(void)
{
    s_Pg = PG_FLOOR_DEG * DEG2RAD * PG_FLOOR_DEG * DEG2RAD;
    g_proc.Pg_deg  = PG_FLOOR_DEG;
    g_proc.K       = CORR_K_MIN;
    g_proc.entry_B = 0u;
    g_proc.mag_fix = MAG_FIX_OFF;
    g_proc.q[0] = 1.0f; g_proc.q[1] = 0.0f;
    g_proc.q[2] = 0.0f; g_proc.q[3] = 0.0f;
    g_proc.q_valid = 0u;
}

/* 用加计定初始姿态：把重力对齐到导航系 UP，偏航置零（等磁来纠正） */
void proc_attitude_align(const float *a_g)
{
    float n = a_g[0]*a_g[0] + a_g[1]*a_g[1] + a_g[2]*a_g[2];
    float w_, x_, y_, z_, sn;

    if (n < 0.5f || n > 1.5f) return;       /* |a| 不接近 1g 不用于对齐 */

    n = 1.0f / sqrtf(n);
    {
        float ux = a_g[0] * n, uy = a_g[1] * n, uz = a_g[2] * n;

        if (uz > -0.999999f)
        {
            w_ = 1.0f + uz; x_ = uy; y_ = -ux; z_ = 0.0f;
        }
        else
        {
            w_ = 0.0f; x_ = 1.0f; y_ = 0.0f; z_ = 0.0f;
        }
    }
    sn = 1.0f / sqrtf(w_*w_ + x_*x_ + y_*y_ + z_*z_);
    g_proc.q[0] = w_ * sn; g_proc.q[1] = x_ * sn;
    g_proc.q[2] = y_ * sn; g_proc.q[3] = z_ * sn;
    g_proc.q_valid = 1u;
}

/* 体坐标右乘小旋转增量：q <- q ⊗ dq */
void proc_attitude_prop(float dt_s)
{
    float half, dw, dx, dy, dz, w_, x_, y_, z_, nw, nx, ny, nz, sn;

    if (g_proc.q_valid == 0u) return;
    if (dt_s <= 0.0f || dt_s > 0.2f) return;

    half = 0.5f * dt_s * DEG2RAD;
    dw = 1.0f;
    dx = g_proc.gyro_dps[0] * half;
    dy = g_proc.gyro_dps[1] * half;
    dz = g_proc.gyro_dps[2] * half;

    w_ = g_proc.q[0]; x_ = g_proc.q[1]; y_ = g_proc.q[2]; z_ = g_proc.q[3];
    nw = w_*dw - x_*dx - y_*dy - z_*dz;
    nx = w_*dx + x_*dw + y_*dz - z_*dy;
    ny = w_*dy - x_*dz + y_*dw + z_*dx;
    nz = w_*dz + x_*dy - y_*dx + z_*dw;

    sn = 1.0f / sqrtf(nw*nw + nx*nx + ny*ny + nz*nz);
    g_proc.q[0] = nw * sn; g_proc.q[1] = nx * sn;
    g_proc.q[2] = ny * sn; g_proc.q[3] = nz * sn;
}

/* 门控推进（每 IMU 帧调用一次） */
void proc_gate_step(float dt_s)
{
    float thr = PG_THR_DEG * DEG2RAD * PG_THR_DEG * DEG2RAD;
    float sig = thr;                        /* SIG_M2 与阈值同值 */
    float wm, d, k;

    if (g_gate.is_static != 0u)
    {
        s_Pg *= (1.0f - PG_DECAY);
        if (s_Pg < PG_FLOOR_DEG * DEG2RAD * PG_FLOOR_DEG * DEG2RAD)
            s_Pg = PG_FLOOR_DEG * DEG2RAD * PG_FLOOR_DEG * DEG2RAD;
    }
    else
    {
        wm = g_proc.gyro_dps[0]*g_proc.gyro_dps[0]
           + g_proc.gyro_dps[1]*g_proc.gyro_dps[1]
           + g_proc.gyro_dps[2]*g_proc.gyro_dps[2];
        wm = sqrtf(wm);
        if (wm < W_FLOOR_DPS) wm = W_FLOOR_DPS;
        d  = INFL * SG_PPM * wm * DEG2RAD;
        s_Pg += d * d * dt_s;
        if (s_Pg > PG_CEIL_DEG * DEG2RAD * PG_CEIL_DEG * DEG2RAD)
            s_Pg = PG_CEIL_DEG * DEG2RAD * PG_CEIL_DEG * DEG2RAD;
    }

    g_proc.Pg_deg  = sqrtf(s_Pg) * RAD2DEG;
    g_proc.entry_B = (s_Pg > thr) ? 1u : 0u;

    k = s_Pg / (s_Pg + sig);
    if (k < CORR_K_MIN) k = CORR_K_MIN;
    if (k > CORR_K_MAX) k = CORR_K_MAX;
    g_proc.K = k;
}
