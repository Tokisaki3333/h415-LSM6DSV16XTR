/* proc_eskf.c —— 陀螺 + 地磁 误差状态卡尔曼滤波（重力牵引已关闭）
 *
 * ★★ 设计要点（全部由 PC 端单一判据「纯陀螺积分」验证）★★
 *
 * 1. 零偏分工
 *      角速度零偏: proc_gyro_bias 每帧牵引负责，gyro_dps 已扣除。
 *                  ESKF 【不】估计、也【不】再扣一次。
 *      ESKF 状态里没有零偏项（三维纯姿态误差）。
 *
 * 2. 启动即 identity（消除一切矩阵/转置环节）
 *      0.5 s ~ 2.5 s 窗口内约定静止，取均值:
 *          v_up  = 窗口内加计均值（单位矢量）
 *          v_mag = 窗口内地磁均值（单位矢量）
 *          q0    = [1, 0, 0, 0]         ← 导航系 ≡ 启动时刻机体系
 *      ⟹ 静止时 R = I，h = v_nav 与实测完全相等，残差 0
 *      ⟹ 磁倾角既不需要、也不出现
 *
 * 3. 状态(误差) x = [ dtheta(3) ]
 *      预测:  q <- q ⊗ exp( gyro_dps·DEG2RAD·dt )
 *             F = I − [w]×dt ;  Q = SIG_G²·dt·I3
 *             P = F P Fᵀ + Q
 *      观测(单位矢量):  h = Rᵀ·v_nav
 *             H = +[h]×                 ← 符号由 PC 数值实验判定
 *             r = z − h , 去沿 h 分量
 *             S = H P Hᵀ + R_obs        (R_obs 各向异性: 垂直 h / 沿 h)
 *             K = P Hᵀ S⁻¹
 *             dx = K r ;  q <- q ⊗ exp(dx)
 *             P = (I − K H) P
 *      唯一观测: 地磁（v_nav = v_mag，z = 净场单位矢量），每 IMU 帧一次
 *
 * 4. 重力牵引 = 关闭
 *      PC 实测（判据=纯陀螺积分）：
 *        陀螺+地磁  : 013304 max 21.97°、053236 max 31.41°，有界
 *        加计三传感器: 013304 16.2→32.5°、053236 20.4→117.4°，显著更差
 *      ⟹ 加计只用于启动窗口定 v_up，此后不参与姿态修正。
 */
#include "proc.h"
#include "proc_tune.h"
#include "mipc_shm.h"
#include <math.h>

#define ESKF_DEG2RAD    0.0174532925f
#define ESKF_RAD2DEG    57.29577951f

/* ---------------- 参数 ---------------- */
/* ★★ 重力牵引总开关 ★★
 * 1 = 开启（本版）：加计作为第二个观测，只做【倾斜修正】，三条件门控：
 *      ① 模长门  | |a| − 1 | < ESKF_ACC_MAGLIM_G   （紧急阀，滤极端值）
 *      ② 残差门  resid < acc_thr，且 acc_thr 有上限 ESKF_ACC_MAX_RAD
 *      ③ 角速率门 |gyro_dps| < ESKF_GRAV_WLIM_DPS
 *         —— 该判据由【被地磁修正过的陀螺】提供
 * 0 = 关闭：只用陀螺积分提供倾斜（上一版）。 */
#define ESKF_USE_GRAVITY    1

/* 陀螺白噪声: 手册 ARW 3.8 mdps/√Hz */
#define ESKF_SIG_G          0.0038f
/* 地磁观测噪声（度）：各向异性。沿 h 方向本身不可观测，用大方差使其自动失效 */
#define ESKF_MAG_PERP_DEG   0.44f
#define ESKF_MAG_PAR_DEG    4.00f
/* 加计观测噪声（度）*/
#define ESKF_ACC_SIG_DEG    0.50f
/* 自适应残差门（弧度）：★ 必须有上限，无上限会涨到 0.159 造成限制运动误开 */
#define ESKF_ACC0_RAD       0.03f
#define ESKF_ACC_MIN_RAD    0.02f
#define ESKF_ACC_MAX_RAD    0.05f
#define ESKF_ACC_DECAY      0.9995f
#define ESKF_ACC_GROW       1.0e-6f
/* 角速率硬门（dps）*/
#define ESKF_GRAV_WLIM_DPS  2.0f
/* 模长门（g）*/
#define ESKF_ACC_MAGLIM_G   0.05f

/* 启动标定窗口（10 ns 计数）*/
#define ESKF_START_T0_TICKS 50000000ULL     /* 0.5 s */
#define ESKF_START_T1_TICKS 250000000ULL    /* 2.5 s */
#define ESKF_START_MAGLIM_G 0.05f           /* |a| 偏离 1g 上限 */
#define ESKF_START_STD      0.0087f         /* 方向稳定性上限 (rad, 0.5°) */
#define ESKF_START_MIN_N    200u            /* 窗口内最少样本数 */
#define ESKF_START_LEN_TICKS (ESKF_START_T1_TICKS - ESKF_START_T0_TICKS)   /* 窗口长度 */
/* ★ 启动激活阈值: 累积到这么多个【有效磁样本】后激活 EKF。
 *   15000 个样本 ≈ 2 s（实测磁有效率约 7.7 kHz）。
 *   ★ 用样本计数而不用 tick 划窗口: tick 一旦不连续,
 *     tick 窗口会被整个跳过、n 一个都攒不到,
 *     却仍置 ready=1 并用初值参考矢量跑。 */
#define ESKF_START_NWIN     15000u

typedef struct {
    float   P[3][3];
    float   q[4];
    /* 启动标定累加器 */
    float   s_m[3], s_a[3];
    uint32_t n;
    /* 导航系参考矢量（启动窗口实测，无任何矩阵转换）*/
    float   v_up[3];
    float   v_mag[3];
    float   acc_thr;            /* 自适应残差门当前值 */
    uint8_t grav_open;          /* 本帧重力观测是否施加 */
    uint8_t calibrating;
    uint8_t ready;
    uint8_t valid;
    /* 输出诊断 */
    float   mag_resid_deg;
    float   acc_resid_deg;      /* 加计残差（度），仅诊断 */
    float   mag_corr_deg;
    float   p_perp, p_par;
} eskf_t;

static eskf_t s_eskf;

/* ---------------- 小型线性代数 ---------------- */
static uint8_t mat3_inv(const float *A, float *Ai)
{
    float d = A[0]*(A[4]*A[8]-A[5]*A[7]) - A[1]*(A[3]*A[8]-A[5]*A[6]) + A[2]*(A[3]*A[7]-A[4]*A[6]);
    float id;
    if (fabsf(d) < 1e-20f) return 0u;
    id = 1.0f / d;
    Ai[0]=(A[4]*A[8]-A[5]*A[7])*id; Ai[1]=(A[2]*A[7]-A[1]*A[8])*id; Ai[2]=(A[1]*A[5]-A[2]*A[4])*id;
    Ai[3]=(A[5]*A[6]-A[3]*A[8])*id; Ai[4]=(A[0]*A[8]-A[2]*A[6])*id; Ai[5]=(A[2]*A[3]-A[0]*A[5])*id;
    Ai[6]=(A[3]*A[7]-A[4]*A[6])*id; Ai[7]=(A[1]*A[6]-A[0]*A[7])*id; Ai[8]=(A[0]*A[4]-A[1]*A[3])*id;
    return 1u;
}

/* Rᵀ v ：导航系矢量 -> 体坐标
 * ★ 标准 R(q)（w,x,y,z）:
 *     R01 = 2(xy − wz)   R10 = 2(xy + wz)
 *     R02 = 2(xz + wy)   R20 = 2(xz − wy)
 *     R12 = 2(yz − wx)   R21 = 2(yz + wx)
 *   此处曾经把 r01/r10/r12/r21 的符号写反（等价于误用 R 代 Rᵀ），
 *   导致 h = Rᵀ·v 连续但恒定错误 —— 静止时因近 180° 旋转下 R≈Rᵀ 而看不出来，
 *   一转动就整体错向。 */
static void nav_to_body(const float *q, const float *v, float *out)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float r00 = 1.0f - 2.0f*(y*y + z*z), r01 = 2.0f*(x*y - w*z), r02 = 2.0f*(x*z + w*y);
    float r10 = 2.0f*(x*y + w*z),        r11 = 1.0f - 2.0f*(x*x + z*z), r12 = 2.0f*(y*z - w*x);
    float r20 = 2.0f*(x*z - w*y),        r21 = 2.0f*(y*z + w*x),        r22 = 1.0f - 2.0f*(x*x + y*y);
    out[0] = r00*v[0] + r10*v[1] + r20*v[2];
    out[1] = r01*v[0] + r11*v[1] + r21*v[2];
    out[2] = r02*v[0] + r12*v[1] + r22*v[2];
}

/* q <- q ⊗ exp(v)  （体坐标右乘小旋转） */
static void q_body_rotate(float *q, const float *v)
{
    /* q <- q (x) exp(v)  体坐标右乘（Hamilton）。
     * ★ 原写法 nx/ny 两式实为 dq (x) q（左乘），每步把旋转轴镜像一下：
     *   陀螺明明只在绕 z 转，四元数却出现 xy 大分量，且误差随转角累积
     *   （实测 14.94/29.88/44.82/59.76 度处误差 10.19/20.23/29.97/39.24 度）。
     *   纯数学对照：显式右乘与解析 q0*exp((0,0,psi)) 的误差 = 0.0000 度。
     *   现直接展开显式 Hamilton 乘积, 不再手写各项符号。 */
    float a = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    float e[3], sh, ch, dw, dx, dy, dz, nw, nx, ny, nz, sn;
    uint8_t i;
    if (a < 1e-12f) return;
    for (i = 0u; i < 3u; i++) e[i] = v[i] / a;
    sh = sinf(a * 0.5f); ch = cosf(a * 0.5f);
    dw = ch; dx = e[0]*sh; dy = e[1]*sh; dz = e[2]*sh;
    {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        nw = q0*dw - q1*dx - q2*dy - q3*dz;
        nx = q0*dx + q1*dw + q2*dz - q3*dy;
        ny = q0*dy - q1*dz + q2*dw + q3*dx;
        nz = q0*dz + q1*dy - q2*dx + q3*dw;
    }
    sn = 1.0f / sqrtf(nw*nw + nx*nx + ny*ny + nz*nz);
    q[0] = nw*sn; q[1] = nx*sn; q[2] = ny*sn; q[3] = nz*sn;
}

/* ---------------- 单位矢量观测（三维状态） ---------------- */
static float eskf_update(const float *v_nav, const float *z, float r_perp, float r_par)
{
    float h[3], r[3], H[3][3], PHt[3][3], S[3][3], Si[3][3], K[3][3];
    float KH[3][3], Pn[3][3], dx[3];
    float dp;
    uint8_t i, j, k;

    nav_to_body(s_eskf.q, v_nav, h);

    for (i = 0; i < 3u; i++) r[i] = z[i] - h[i];
    dp = r[0]*h[0] + r[1]*h[1] + r[2]*h[2];
    for (i = 0; i < 3u; i++) r[i] -= dp * h[i];

    /* H = [h]× 的负形（与 PC 端已验证的 _eskf.c 的 make_skew() 完全一致）
     * ★ 此处曾经写成正形，导致静止时看不出来（残差≈0、修正≈0），
     *   一转动残差变大就把姿态往反方向推 —— 表现为"转的轴向错"。 */
    H[0][0] =  0.0f;   H[0][1] = -h[2];  H[0][2] =  h[1];
    H[1][0] =  h[2];   H[1][1] =  0.0f;  H[1][2] = -h[0];
    H[2][0] = -h[1];   H[2][1] =  h[0];  H[2][2] =  0.0f;

    for (i = 0; i < 3u; i++)
        for (j = 0; j < 3u; j++) {
            float s = 0.0f;
            for (k = 0; k < 3u; k++) s += s_eskf.P[i][k] * H[j][k];
            PHt[i][j] = s;
        }
    for (i = 0; i < 3u; i++)
        for (j = 0; j < 3u; j++) {
            float s = 0.0f;
            for (k = 0; k < 3u; k++) s += H[i][k] * PHt[k][j];
            S[i][j] = s;
        }
    for (i = 0; i < 3u; i++)
        for (j = 0; j < 3u; j++) {
            float d = (i == j) ? 1.0f : 0.0f;
            float eye = d - h[i]*h[j];
            S[i][j] += r_perp * eye + r_par * (d - eye);
        }
    if (mat3_inv(&S[0][0], &Si[0][0]) == 0u) return 0.0f;

    for (i = 0; i < 3u; i++)
        for (j = 0; j < 3u; j++) {
            float s = 0.0f;
            for (k = 0; k < 3u; k++) s += PHt[i][k] * Si[k][j];
            K[i][j] = s;
        }
    for (i = 0; i < 3u; i++) {
        float s = 0.0f;
        for (k = 0; k < 3u; k++) s += K[i][k] * r[k];
        dx[i] = s;
    }
    q_body_rotate(s_eskf.q, dx);

    for (i = 0; i < 3u; i++)
        for (j = 0; j < 3u; j++) {
            float s = 0.0f;
            for (k = 0; k < 3u; k++) s += K[i][k] * H[k][j];
            KH[i][j] = ((i == j) ? 1.0f : 0.0f) - s;
        }
    for (i = 0; i < 3u; i++)
        for (j = 0; j < 3u; j++) {
            float s = 0.0f;
            for (k = 0; k < 3u; k++) s += KH[i][k] * s_eskf.P[k][j];
            Pn[i][j] = s;
        }
    for (i = 0; i < 3u; i++) for (j = 0; j < 3u; j++) s_eskf.P[i][j] = Pn[i][j];

    return sqrtf(dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2]);
}

/* 由启动窗口均值构造 (q0, v_mag) —— 与 PC 仿真原版逐字一致
 *   ẑ = a_mean , x̂ = normalize(u_mean − (u_mean·ẑ)ẑ) , ŷ = ẑ × x̂
 *   R0 = [x̂; ŷ; ẑ]  (体->导航)
 *   q0 = R0 转四元数 ； v_mag = R0 · u_mean
 * ★ 不得改成 q0=identity：PC 实测两者姿态差恒定 179.63°。 */
static uint8_t eskf_build_q0(const float *a_mean, const float *u_mean)
{
    float zb[3], yb[3], xb[3], R0[9], um[3];
    float na, nu, dp, ny, tr, w0, sn;
    uint8_t i;

    na = sqrtf(a_mean[0]*a_mean[0] + a_mean[1]*a_mean[1] + a_mean[2]*a_mean[2]);
    if (na < 1e-6f) return 0u;
    for (i = 0; i < 3u; i++) zb[i] = a_mean[i] / na;

    nu = sqrtf(u_mean[0]*u_mean[0] + u_mean[1]*u_mean[1] + u_mean[2]*u_mean[2]);
    if (nu < 1e-9f) return 0u;
    for (i = 0; i < 3u; i++) um[i] = u_mean[i] / nu;

    dp = um[0]*zb[0] + um[1]*zb[1] + um[2]*zb[2];
    for (i = 0; i < 3u; i++) yb[i] = um[i] - dp * zb[i];
    ny = sqrtf(yb[0]*yb[0] + yb[1]*yb[1] + yb[2]*yb[2]);
    if (ny < 1e-6f) return 0u;
    for (i = 0; i < 3u; i++) yb[i] /= ny;

    xb[0] = zb[1]*yb[2] - zb[2]*yb[1];
    xb[1] = zb[2]*yb[0] - zb[0]*yb[2];
    xb[2] = zb[0]*yb[1] - zb[1]*yb[0];

    R0[0]=xb[0]; R0[1]=xb[1]; R0[2]=xb[2];
    R0[3]=yb[0]; R0[4]=yb[1]; R0[5]=yb[2];
    R0[6]=zb[0]; R0[7]=zb[1]; R0[8]=zb[2];

    /* ★ 稳健四元数提取：分支法（对 tr<0 / 接近 180° 旋转有效）
     *   原写法 w0 = 0.5*sqrtf(tr) 在 tr<0 时 sqrtf 返回 NaN，
     *   于是落入 else 分支被置成 identity —— 实测 013304 的 R0 迹为
     *   −1.86（接近绕水平轴 180°），q0 实际变成了 [1,0,0,0]，姿态从错误
     *   起点累积，表现为"EKF 内部稳定但输出四元数方向不对"。 */
    {
        float t = R0[0] + R0[4] + R0[8];
        if (t > 0.0f) {
            float s = sqrtf(t + 1.0f) * 2.0f;      /* s = 4w */
            s_eskf.q[0] = 0.25f * s;
            s_eskf.q[1] = (R0[7] - R0[5]) / s;
            s_eskf.q[2] = (R0[2] - R0[6]) / s;
            s_eskf.q[3] = (R0[3] - R0[1]) / s;
        } else if ((R0[0] > R0[4]) && (R0[0] > R0[8])) {
            float s = sqrtf(1.0f + R0[0] - R0[4] - R0[8]) * 2.0f;   /* s = 4x */
            s_eskf.q[0] = (R0[7] - R0[5]) / s;
            s_eskf.q[1] = 0.25f * s;
            s_eskf.q[2] = (R0[1] + R0[3]) / s;
            s_eskf.q[3] = (R0[2] + R0[6]) / s;
        } else if (R0[4] > R0[8]) {
            float s = sqrtf(1.0f + R0[4] - R0[0] - R0[8]) * 2.0f;   /* s = 4y */
            s_eskf.q[0] = (R0[2] - R0[6]) / s;
            s_eskf.q[1] = (R0[1] + R0[3]) / s;
            s_eskf.q[2] = 0.25f * s;
            s_eskf.q[3] = (R0[5] + R0[7]) / s;
        } else {
            float s = sqrtf(1.0f + R0[8] - R0[0] - R0[4]) * 2.0f;   /* s = 4z */
            s_eskf.q[0] = (R0[3] - R0[1]) / s;
            s_eskf.q[1] = (R0[2] + R0[6]) / s;
            s_eskf.q[2] = (R0[5] + R0[7]) / s;
            s_eskf.q[3] = 0.25f * s;
        }
    }
    sn = 1.0f / sqrtf(s_eskf.q[0]*s_eskf.q[0] + s_eskf.q[1]*s_eskf.q[1]
                    + s_eskf.q[2]*s_eskf.q[2] + s_eskf.q[3]*s_eskf.q[3]);
    for (i = 0; i < 4u; i++) s_eskf.q[i] *= sn;

    for (i = 0; i < 3u; i++)
        s_eskf.v_mag[i] = R0[i*3+0]*um[0] + R0[i*3+1]*um[1] + R0[i*3+2]*um[2];
    sn = 1.0f / sqrtf(s_eskf.v_mag[0]*s_eskf.v_mag[0]
                    + s_eskf.v_mag[1]*s_eskf.v_mag[1]
                    + s_eskf.v_mag[2]*s_eskf.v_mag[2]);
    for (i = 0; i < 3u; i++) s_eskf.v_mag[i] *= sn;

    /* v_up 仍取 (0,0,1)（R0 已把 ẑ 对齐到加计方向）*/
    s_eskf.v_up[0] = 0.0f; s_eskf.v_up[1] = 0.0f; s_eskf.v_up[2] = 1.0f;
    return 1u;
}

/* ---------------- 启动标定 ---------------- */
static void eskf_calib_reset(void)
{
    uint8_t i, j;
    for (i = 0; i < 3u; i++) { s_eskf.s_m[i] = 0.0f; s_eskf.s_a[i] = 0.0f; }
    s_eskf.n = 0u;
    for (i = 0; i < 3u; i++) for (j = 0; j < 3u; j++) s_eskf.P[i][j] = 0.0f;
    /* ★ 启动即 identity */
    s_eskf.q[0] = 1.0f; s_eskf.q[1] = 0.0f; s_eskf.q[2] = 0.0f; s_eskf.q[3] = 0.0f;
    s_eskf.v_up[0] = 0.0f; s_eskf.v_up[1] = 0.0f; s_eskf.v_up[2] = 1.0f;
    s_eskf.v_mag[0] = 0.0f; s_eskf.v_mag[1] = 1.0f; s_eskf.v_mag[2] = 0.0f;
    s_eskf.calibrating = 1u;
    s_eskf.ready = 0u;
    s_eskf.valid = 0u;
    s_eskf.acc_thr = ESKF_ACC0_RAD;
    s_eskf.grav_open = 0u;
}

void proc_eskf_init(void)
{
    eskf_calib_reset();
    s_eskf.mag_resid_deg = 0.0f;
    s_eskf.acc_resid_deg = 0.0f;
    s_eskf.mag_corr_deg  = 0.0f;
    s_eskf.p_perp = 0.0f; s_eskf.p_par = 0.0f;
}

uint8_t proc_eskf_ready(void)   { return s_eskf.ready; }
uint8_t proc_eskf_valid(void)   { return s_eskf.valid; }
uint8_t proc_eskf_grav_open(void) { return s_eskf.grav_open; }
float   proc_eskf_mag_resid(void) { return s_eskf.mag_resid_deg; }
float   proc_eskf_acc_resid(void) { return s_eskf.acc_resid_deg; }
float   proc_eskf_mag_corr(void)  { return s_eskf.mag_corr_deg; }
float   proc_eskf_p_perp(void)    { return s_eskf.p_perp; }
float   proc_eskf_p_par(void)     { return s_eskf.p_par; }
uint8_t proc_eskf_calib_done(void) { return (s_eskf.calibrating == 0u) ? 1u : 0u; }

/* ★ 输出四元数并强制半球连续（w ≥ 0 只对提取那一帧，之后必须跟着上一帧走）
 *   q 与 −q 表示同一姿态；若输出侧在分支边界翻号，上位机会看到
 *   "转到某角度抽风、转回去也恢复不了" —— 正是半球不连续的表现。
 *   这里保证每次输出的四元数与上一次输出同半球（点积 ≥ 0）。 */
static float s_q_last[4];
static uint8_t s_q_last_valid;

void proc_eskf_get_q(float *q_out)
{
    float d;
    uint8_t i;

    if (s_q_last_valid != 0u) {
        d = s_eskf.q[0]*s_q_last[0] + s_eskf.q[1]*s_q_last[1]
          + s_eskf.q[2]*s_q_last[2] + s_eskf.q[3]*s_q_last[3];
        if (d < 0.0f) {
            q_out[0] = -s_eskf.q[0]; q_out[1] = -s_eskf.q[1];
            q_out[2] = -s_eskf.q[2]; q_out[3] = -s_eskf.q[3];
        } else {
            q_out[0] = s_eskf.q[0]; q_out[1] = s_eskf.q[1];
            q_out[2] = s_eskf.q[2]; q_out[3] = s_eskf.q[3];
        }
    } else {
        q_out[0] = s_eskf.q[0]; q_out[1] = s_eskf.q[1];
        q_out[2] = s_eskf.q[2]; q_out[3] = s_eskf.q[3];
    }
    for (i = 0; i < 4u; i++) s_q_last[i] = q_out[i];
    s_q_last_valid = 1u;
}

/* 启动标定完成时清掉半球记忆，使第一次输出按提取结果的符号 */
void proc_eskf_clear_qhemi(void)
{
    s_q_last_valid = 0u;
}

void proc_eskf_sync_gproc(void)
{
    g_proc.q[0] = s_eskf.q[0]; g_proc.q[1] = s_eskf.q[1];
    g_proc.q[2] = s_eskf.q[2]; g_proc.q[3] = s_eskf.q[3];
    g_proc.q_valid = 1u;
}

/* R = R(q) 体->导航（行主序） */
void proc_eskf_get_R(float *R_out)
{
    float w = s_eskf.q[0], x = s_eskf.q[1], y = s_eskf.q[2], z = s_eskf.q[3];
    R_out[0] = 1.0f - 2.0f*(y*y + z*z);
    R_out[1] = 2.0f*(x*y - w*z);
    R_out[2] = 2.0f*(x*z + w*y);
    R_out[3] = 2.0f*(x*y + w*z);
    R_out[4] = 1.0f - 2.0f*(x*x + z*z);
    R_out[5] = 2.0f*(y*z - w*x);
    R_out[6] = 2.0f*(x*z - w*y);
    R_out[7] = 2.0f*(y*z + w*x);
    R_out[8] = 1.0f - 2.0f*(x*x + y*y);
}

/*
 * 每 IMU 帧调用一次。
 *   gyro_dps : 已由 proc_gyro_bias 扣除角速度零偏的角速度（dps）
 *   gyro_lsb : 保留参数，本版不使用
 *   a_g      : 加计（g）—— 仅启动窗口使用
 *   mag_u    : 净场单位矢量（体坐标），mag_valid=0 表示本帧无新磁
 *   t10ns    : 本帧 10 ns 时间戳
 */
void proc_eskf_step(float dt_s, const float *gyro_dps, const float *gyro_lsb,
                    const float *a_g, const float *mag_u, uint8_t mag_valid,
                    uint8_t new_mag, uint64_t t10ns)
{
    float w[3], F[3][3], FP[3][3], FPFt[3][3];
    uint8_t i, j, k;

    (void)gyro_lsb;

    /* ---------------- 启动标定: 按【有效样本计数】激活 EKF ----------------
     * 结构就是"每个有效样本令计数器加一, 到阈值后激活 EKF":
     *   每来一个有效磁样本: s_m += mag_u , s_a += a_g , n++
     *   n 达到 ESKF_START_NWIN 时: 用均值构造 q0/v_up/v_mag, 置 ready=1
     * ★ 不再用 t10ns 划窗口。原写法用 t10ns ∈ [T0,T1]:
     *   一旦 tick 出现不连续, 窗口被整个跳过, n 一个都没攒到,
     *   却仍然置 ready=1, 而 q0/v_up/v_mag 还是初值
     *   (1,0,0,0)/(0,0,1)/(0,1,0) —— 与真实姿态无关。
     * ★ 已在 PC 侧用同一份录制验证: n=15000, |a|=1.002128, |u|=0.999992, valid=1。 */
    if (s_eskf.calibrating != 0u) {
        if (s_eskf.n < ESKF_START_NWIN) {
            if (mag_valid != 0u) {
                for (i = 0; i < 3u; i++) {
                    s_eskf.s_m[i] += mag_u[i];
                    s_eskf.s_a[i] += a_g[i];
                }
                s_eskf.n++;
            }
            return;
        }
        {
            float invn = 1.0f / (float)s_eskf.n;
            float a_mean[3], u_mean[3];
            float an_mean, unorm;
            for (i = 0; i < 3u; i++) {
                a_mean[i] = s_eskf.s_a[i] * invn;
                u_mean[i] = s_eskf.s_m[i] * invn;
            }
            an_mean = sqrtf(a_mean[0]*a_mean[0] + a_mean[1]*a_mean[1] + a_mean[2]*a_mean[2]);
            unorm   = sqrtf(u_mean[0]*u_mean[0] + u_mean[1]*u_mean[1] + u_mean[2]*u_mean[2]);
            s_eskf.valid = ((fabsf(an_mean - 1.0f) < ESKF_START_MAGLIM_G)
                            && (unorm > 1e-6f)) ? 1u : 0u;
            /* ★ 与 PC 仿真原版逐字一致：由 (a_mean, u_mean) 构造 q0 与 v_mag */
            if (eskf_build_q0(a_mean, u_mean) == 0u) s_eskf.valid = 0u;
        }
        /* 初始协方差: 姿态 2° */
        for (i = 0; i < 3u; i++) for (j = 0; j < 3u; j++) s_eskf.P[i][j] = 0.0f;
        for (i = 0; i < 3u; i++) s_eskf.P[i][i] = 0.001218f;   /* (2°)² */
        s_eskf.acc_thr = ESKF_ACC0_RAD;
        s_eskf.calibrating = 0u;
        s_eskf.ready = 1u;
        return;
    }

    if (s_eskf.ready == 0u) return;

    /* ---------------- 预测（不做任何角速度零偏修正） ---------------- */
    for (i = 0; i < 3u; i++) w[i] = gyro_dps[i] * ESKF_DEG2RAD;
    {
        float v[3];
        for (i = 0; i < 3u; i++) v[i] = w[i] * dt_s;
        q_body_rotate(s_eskf.q, v);
    }
    /* F = I − [w]×dt */
    for (i = 0; i < 3u; i++) for (j = 0; j < 3u; j++) F[i][j] = 0.0f;
    for (i = 0; i < 3u; i++) F[i][i] = 1.0f;
    F[0][1] =  w[2]*dt_s; F[0][2] = -w[1]*dt_s;
    F[1][0] = -w[2]*dt_s; F[1][2] =  w[0]*dt_s;
    F[2][0] =  w[1]*dt_s; F[2][1] = -w[0]*dt_s;

    for (i = 0; i < 3u; i++)
        for (j = 0; j < 3u; j++) {
            float s = 0.0f;
            for (k = 0; k < 3u; k++) s += F[i][k] * s_eskf.P[k][j];
            FP[i][j] = s;
        }
    for (i = 0; i < 3u; i++)
        for (j = 0; j < 3u; j++) {
            float s = 0.0f;
            for (k = 0; k < 3u; k++) s += FP[i][k] * F[j][k];
            FPFt[i][j] = s;
        }
    {
        float qg = ESKF_SIG_G * ESKF_SIG_G * dt_s;
        for (i = 0; i < 3u; i++)
            for (j = 0; j < 3u; j++)
                s_eskf.P[i][j] = FPFt[i][j] + (((i == j) ? qg : 0.0f));
    }

    /* ---------------- 唯一观测: 地磁 ----------------
     * ★ 只在【本帧真有新磁拍】时做观测（new_mag = 1）。
     *   实测磁拍 1039.5 Hz、EKF 7670 Hz ⟹ 一份样本会被 5 帧共享；
     *   若每帧都注入，地磁的有效权重被放大约 5 倍
     *   （等效于把仿真定下的 σ_mag=0.44° 压到 0.44°/√5），
     *   姿态会被地磁拽得过紧、抖动增大。
     *   这正是"只有真正有磁的一拍才做观测"的默认做法。 */
    s_eskf.mag_corr_deg = 0.0f;
    /* ★★ 【临时开关】地磁观测总闸 ★★
     *   ESKF_USE_MAG_OBS = 0 时完全不做地磁观测（不影响启动标定）：
     *     · 启动窗口仍然采 mag_u 用于构造 v_mag 与 q0
     *       （否则导航系无法建立）
     *     · 但姿态推进中不再用地磁修正
     *   改回 1 即恢复地磁观测。 */
#ifndef ESKF_USE_MAG_OBS
#define ESKF_USE_MAG_OBS    0
#endif
#if ESKF_USE_MAG_OBS
    if ((mag_valid != 0u) && (new_mag != 0u)) {
        float h[3], z[3], c;
        for (i = 0; i < 3u; i++) z[i] = mag_u[i];
        nav_to_body(s_eskf.q, s_eskf.v_mag, h);
        c = z[0]*h[0] + z[1]*h[1] + z[2]*h[2];
        if (c > 1.0f) c = 1.0f; if (c < -1.0f) c = -1.0f;
        s_eskf.mag_resid_deg = acosf(c) * ESKF_RAD2DEG;
        {
            float rp = ESKF_MAG_PERP_DEG * ESKF_DEG2RAD;
            float rq = ESKF_MAG_PAR_DEG  * ESKF_DEG2RAD;
            s_eskf.mag_corr_deg = eskf_update(s_eskf.v_mag, z, rp*rp, rq*rq) * ESKF_RAD2DEG;
        }
    }
#endif  /* ESKF_USE_MAG_OBS */

    /* ---------------- 观测2: 加计（只做倾斜修正，三条件门控） ---------------- */
    s_eskf.grav_open = 0u;
#if ESKF_USE_GRAVITY
    {
        float h[3], z[3], c, resid, an, avn;
        an = sqrtf(a_g[0]*a_g[0] + a_g[1]*a_g[1] + a_g[2]*a_g[2]);
        nav_to_body(s_eskf.q, s_eskf.v_up, h);
        if (an > 1e-6f) {
            for (i = 0; i < 3u; i++) z[i] = a_g[i] / an;
        } else {
            for (i = 0; i < 3u; i++) z[i] = 0.0f;
        }
        c = z[0]*h[0] + z[1]*h[1] + z[2]*h[2];
        if (c > 1.0f) c = 1.0f;
        if (c < -1.0f) c = -1.0f;
        resid = acosf(c);

        avn = sqrtf(gyro_dps[0]*gyro_dps[0] + gyro_dps[1]*gyro_dps[1]
                  + gyro_dps[2]*gyro_dps[2]);

        /* 三条件同时满足才开门：
         *   ① 模长门（紧急阀） ② 自适应残差门（有上限） ③ 角速率门 */
        if ((fabsf(an - 1.0f) < ESKF_ACC_MAGLIM_G) && (resid < s_eskf.acc_thr)
            && (avn < ESKF_GRAV_WLIM_DPS)) {
            float rs = ESKF_ACC_SIG_DEG * ESKF_DEG2RAD;
            s_eskf.grav_open = 1u;
            (void)eskf_update(s_eskf.v_up, z, rs*rs, rs*rs);
            s_eskf.acc_thr *= ESKF_ACC_DECAY;
            if (s_eskf.acc_thr < ESKF_ACC_MIN_RAD) s_eskf.acc_thr = ESKF_ACC_MIN_RAD;
        } else {
            s_eskf.acc_thr += ESKF_ACC_GROW;
            if (s_eskf.acc_thr > ESKF_ACC_MAX_RAD) s_eskf.acc_thr = ESKF_ACC_MAX_RAD;
        }
    }
#endif

    /* ---------------- 加计残差（仅诊断上报，不改 q/P）---------------- */
    {
        float hh[3], zz[3], cc, an;
        an = sqrtf(a_g[0]*a_g[0] + a_g[1]*a_g[1] + a_g[2]*a_g[2]);
        nav_to_body(s_eskf.q, s_eskf.v_up, hh);
        if (an > 1e-6f) {
            for (i = 0; i < 3u; i++) zz[i] = a_g[i] / an;
            cc = zz[0]*hh[0] + zz[1]*hh[1] + zz[2]*hh[2];
            if (cc > 1.0f) cc = 1.0f;
            if (cc < -1.0f) cc = -1.0f;
            s_eskf.acc_resid_deg = acosf(cc) * ESKF_RAD2DEG;
        }
    }

    /* ---------------- 输出诊断 ---------------- */
    {
        float Bb[3], vp = 0.0f, tr;
        nav_to_body(s_eskf.q, s_eskf.v_mag, Bb);
        for (i = 0; i < 3u; i++)
            for (j = 0; j < 3u; j++) vp += Bb[i] * s_eskf.P[i][j] * Bb[j];
        tr = s_eskf.P[0][0] + s_eskf.P[1][1] + s_eskf.P[2][2];
        s_eskf.p_par  = sqrtf(vp > 0.0f ? vp : 0.0f);
        s_eskf.p_perp = sqrtf(((tr - vp) * 0.5f) > 0.0f ? (tr - vp) * 0.5f : 0.0f);
    }
}
