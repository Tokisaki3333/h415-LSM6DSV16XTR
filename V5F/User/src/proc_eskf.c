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
/* ★★ 单位修正 + 实测标度项（2026-10-13）
 *
 * ESKF_SIG_G 在 qg = ESKF_SIG_G²·dt_s 里与 P(rad²) 配套 ⟹ 它必须是 rad/√s。
 * 原值 0.0038 比实测 ARW 大 103 倍（若本意是 dps/√s 而漏了 DEG2RAD 则 Q 偏大 3283 倍）。
 * 实测 ARW = 0.127 °/√hr = 0.127·(π/180)/√3600 = 3.6943e-05 rad/√s。
 *
 * ESKF_KS_SCALE = 636 ppm：实测【残余标度误差】，做法是四次运动录制上做
 *   "偏航误差 vs 累计转角"回归：偏航误差 = 6.365e-04 × θ_rot，残差 rms 1.45°，
 *   与均|ω| 相关仅 +0.245 ⟹ 误差 ∝ 累计转角、【无速度敏感性】。
 *   （逐轴标度修正前是 5030/2400/1150 ppm，修正后降到 636 ppm，落在文档 §6 的 S3 目标内。）
 * 在正确的 ARW 之下，运动时(|ω|=2000dps) k_s 项占 Q 的 98% ⟹ 它才是运动期 P 的主驱动。
 * 若沿用旧的 0.0038，k_s 项只占 1/225，加了等于没加。 */
#define ESKF_SIG_G          3.6943e-05f
#define ESKF_KS_SCALE       636e-6f
/* 地磁观测噪声（度）：各向异性。沿 h 方向本身不可观测，用大方差使其自动失效 */
#define ESKF_MAG_PERP_DEG   0.44f
#define ESKF_MAG_PAR_DEG    4.00f
/* 加计观测噪声（度）*/
/* ★ 实测：三次运动录制(065442/072104/072538)的【静止段 acc_resid 中位 = 0.1312°】
 *   完全一致 ⟹ 这就是加计观测噪声 σ_a。原值 0.50° 比实测大 3.8 倍。 */
#define ESKF_ACC_SIG_DEG    0.1312f
/* 自适应残差门（弧度）：★ 必须有上限，无上限会涨到 0.159 造成限制运动误开 */
#define ESKF_ACC0_RAD       0.03f
#define ESKF_ACC_MIN_RAD    0.02f
#define ESKF_ACC_MAX_RAD    0.05f
#define ESKF_ACC_DECAY      0.9995f
#define ESKF_ACC_GROW       1.0e-6f
/* 角速率硬门（dps）*/
/* ★★ 速率门 2.0 -> 10.0（2026-10-13，五份录制逐级消融）
 * 依据一(解碎片化): 手端平段 |w| 中位 2.1 dps、95% 48 dps ⟹ 旧的 2 dps 门把手端平
 *   切成中位 7 帧的碎片, 开门率仅 48.3%; 提到 10 后 78.2%（+29.9 pp，实测）。
 * 依据二(拦运动凑巧帧): 运动中 |g+a_lin| 凑巧 =1g 的解集是一个球面，不罕见 ——
 *   剧烈段有 1.9~3.1% 的帧满足 | |a|-1 |<0.05 却方向错。这类帧实测在 |w|>10 之外,
 *   故 10 的阈值即可 100% 拦掉（实测 |w|>10 的帧开门数 = 0）。
 * 依据三(不能更高): 按 |w| 分箱看残差, 拐点在 |w|≈10 dps（[5,10) 中位 1.69°,
 *   [10,20) 中位 3.27° 且 max 可达 151°）。50 dps 会放进大量被线加速度主导的帧。 */
/* 放松: 10 -> 20 dps（2026-10-13）。实测手端平段开门率 78.2%(W=10) -> 85.5%(W=20)，
 * 而开门帧 resid 中位仅从 0.291° 升到 0.315°（方向仍正确）。 */
#define ESKF_GRAV_WLIM_DPS  20.0f
/* NIS 门限: chi2(2 DOF, 95%) = 5.991。S = H P H^T + R 随 P 自适应 ⟹
 *   运动后 P 诚实变大时大残差统计上仍"合理" ⟹ 门照开 ⟹ 恢复能力不丢。
 * 实测贡献小（手端平 ±0.0 pp、灰区 −1.4%），保留作廉价相容性检查。 */
/* 放松: chi2(2 DOF) 95%=5.991 -> 99%=9.210（2026-10-13）。P 尚未完全诚实前留余量。 */
/* ★ NIS 门已关闭（2026-10-13，实测依据 rec_20261013_082033）
 * 把限幅从 dx 移到 K 之后收敛确实发生了（末段 resid 13.478° -> 2.999°，4.5 倍），
 * 但留下一个 3° 平台：更新把 P 的倾斜 σ 压到 ≈R 量级后，NIS = resid²/(P_⊥h+R) 变得极严
 *   实测末段 resid=2.999°, P倾斜1σ=0.192°  ⟹ NIS = 8.99/0.0541 = 166 ≫ 阈值
 *   要开门需 resid < sqrt(阈值 × (0.192²+0.1312²)) = 0.706°
 * ⟹ 门的"可通过残差"被自限在 ~0.7°，误差 3° 就永远修不掉。
 * 而 NIS 原有的作用（拦运动污染帧）已由速率门(20 dps)+模长门(0.05g)完整覆盖：
 *   实测 |w|>20 的帧开门率 0%、运动凑巧帧拦截 100%；NIS 自身贡献仅
 *   手端平 ±0.0pp、灰区 −1.4%。⟹ 关闭它，只保留诊断上报。
 * 阈值设为 1e9（等效关闭）；NIS 仍逐帧计算并可用探针读取。 */
#define ESKF_GRAV_NIS_MAX   1.0e9f
/* 单帧修正量上限 0.5°（rad）。实测: 不限幅时开门帧 resid 95% 分位 7.8°、max 25.5°;
 *   限幅后单帧 <=0.5°, 而残余误开是瞬态(中位 2~3 帧) ⟹ 累计误注入 <=1~1.5°。
 *   合法的大修正(灰区实测姿态差可达 64°)分摊到 ~128 帧(17 ms)完成，可接受。 */
#define ESKF_GRAV_DX_MAX_RAD 0.0087266f

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
    uint8_t mag_ref_ok;     /* ★ v_mag 是否由【实测地磁】构造（0 ⟹ 禁止地磁观测） */
    /* 输出诊断 */
    float   mag_resid_deg;
    float   acc_resid_deg;      /* 加计残差（度），仅诊断 */
    float   mag_corr_deg;
    float   p_perp, p_par;
} eskf_t;

static eskf_t s_eskf;
/* ★ 启动标定的【地磁样本计数】，与 s_eskf.n（IMU 样本计数）解耦。
 *   地磁缺失时只降级偏航参考 v_mag，不再把整条重力链卡死。
 *   ★ 必须声明在 eskf_calib_reset() 之前——该函数会复位它。 */
static uint32_t s_cal_n_mag;

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
/* ★★ 二维更新重写（2026-10-13）——重力观测天生就是【二维牵引】
 *
 * 为什么原来的三维写法会错（实测 rec_20261013_062531：t=8~10 s 沿世界 z 被注入 6.2° 偏航）：
 *   原式 dx = P Hᵀ (H P Hᵀ + R)⁻¹ r。其中 Hᵀ(...)r 确实落在 ⊥h 平面内，
 *   但【左乘 P 之后离开平面】—— 因为 P 在"可观子空间 ⊥h"与"不可观子空间 h(偏航)"
 *   之间存在互协方差。实测该比值在运动期达 0.356 ⟹ 每次更新都往偏航里塞一点。
 *   更糟：P ← (I−KH)P 会把 P_hh 缩小。重力对偏航【零信息】，不可观方向的
 *   不确定度【不得】被降低；违反它形成正反馈：
 *       P_hh 被错误缩小 ⟹ 下次增益更"自信" ⟹ 泄漏更大 …
 *   单次泄漏极小（差值高通中位 0.0004°），正是这个正反馈把它放大成 6.2° 台阶。
 *
 * 而且 P 的那个互协方差【本身是伪影】：误差状态定义在体系、不可观方向 h 随姿态
 * 旋转，过程模型 Φ=I−[ω]×dt 每步都在搅动协方差，而 Q 各向同性 —— 相关性不来自
 * 任何物理误差源。所以按它去"修正"偏航没有物理依据。"重力不可观测偏航"是系统
 * 【可观测性】的属性，与先验协方差长什么样无关；让先验把信息漏进不可观方向即为
 * 不一致 —— 这正是 OC-EKF（可观测性约束卡尔曼滤波）的理据。
 *
 * 本实现：在 ⊥h 的正交基 {e1,e2} 上做真正的二维更新
 *   H2 = Eᵀ H E (2×2, 可逆)   P2 = Eᵀ P E   r2 = Eᵀ r
 *   K2 = P2 H2ᵀ (H2 P2 H2ᵀ + r_perp I)⁻¹
 *   θ2 = K2 r2  ⟹  dx = E θ2 【构造上 ⊥ h】⟹ 偏航注入恒为 0
 *   P += E (P2⁺ − P2) Eᵀ      ⟹ P_hh 与 h-倾斜互协方差【逐位不变】
 * 泄漏通道在结构上不存在（不是被"清零"），正反馈随之消失。
 * ★ 与原三维式在 ⊥h 子空间上严格等价（因 E Eᵀ = I − hhᵀ 且 H(I − hhᵀ) = H）。 */
/* ★★ 回退说明（2026-10-13）——二维更新尝试【失败并已撤销】
 *
 * 曾把本函数改写为"在 ⊥h 正交基上做二维更新、只缩小倾斜块、冻结 P_hh 与
 * h↔倾斜互协方差"，理由是：重力观测秩 2、不可观方向 h(偏航) 不该被降低
 * 不确定度，且 P 的互协方差是体系参数化的伪影。
 *
 * 实测（rec_20261013_064624，二维版固件）该做法【破坏 P 的正定性】：
 *     P对角  1.43e-03 → 3.28e-03 单调增长
 *     P非对角 2.2e-05 → 1.65e-03（75 倍），|非对角|/对角 达 0.9938（近奇异）
 *     ★ P_xx 出现【负值】-1.86e-04 ⟹ 协方差非法 ⟹ 大运动时滤波器发散
 *   对照 rec_20261013_060408（原三维式）同一比值仅 0.356。
 * 症状与用户观察一致：小运动正常，大运动后姿态崩溃 + 小幅乱闪。
 *
 * 根因：协方差被缩减时，与 h 的互协方差【必须同步缩减】才能保持正定；
 * "冻结"它们会构造出非 PSD 矩阵。若日后仍要消除偏航泄漏，正确方向是
 * OC-EKF 那种【修改 H / Φ 使线性化系统的可观测性与非线性系统一致】，
 * 或采用 Joseph 形式 + 【秩 1 的 PSD 安全修正】恢复 h 方向方差，
 * 而不是冻结子块。在拿到正确的做法之前，本函数保持原三维式。
 */
#ifdef PC_PROBE
extern float g_pc_nis, g_pc_applied, g_pc_throt_set;
extern float g_pc_p_pre[3], g_pc_p_post[3];
extern float g_pc_p6pre[6], g_pc_p6post[6];
#endif
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
    /* ★★ NIS 门（2026-10-13）: NIS = r^T S^-1 r, S = H P H^T + R 随 P 自适应。
     * 超 chi2(2 DOF,95%%)=5.991 则本帧不施加修正, 返回 -1 让调用方把 grav_open 置 0。
     * 放在求逆之后、算 K 之前 ⟹ 拒绝时完全不改 q/P。 */
    {
        float nis = 0.0f;
        for (i = 0; i < 3u; i++) {
            float ss = 0.0f;
            for (j = 0; j < 3u; j++) ss += Si[i][j] * r[j];
            nis += r[i] * ss;
        }
#ifdef PC_PROBE
        g_pc_nis = nis;
#endif
#ifdef PC_PROBE
        if (!(nis <= ESKF_GRAV_NIS_MAX)) { g_pc_applied = -1.0f; return -1.0f; }
#else
        if (!(nis <= ESKF_GRAV_NIS_MAX)) return -1.0f;
#endif
    }

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
    /* ★★ 限幅必须作用在【增益】上，不能只截断 dx（2026-10-13 修正）
     * 只截断 dx ⟹ 协方差按【全额增益】收缩而姿态只动了一点点 ⟹ 不一致：
     * 倾斜方差被压到近零、真实倾斜误差仍在 ⟹ S=HPHᵀ+R 塌掉 ⟹ NIS 爆大 ⟹ 永久拒绝。
     * PC 复刻实测(rec_20261013_080750, 注入板载初值):
     *   t=16  倾斜1σ=12.72°, resid=10.62°, NIS=0.70   ⟹ 开门 ✓
     *   t=20  倾斜1σ 塌到 0.36°, resid 仍 9.07°, NIS=635 ⟹ 此后 100% 拒绝
     *   特征分解佐证 λmax·h=1.000（精确秩 1，⊥h 被抹平）。
     * 正解：按比例缩放整个增益 K，使估计与协方差【同步】受限。 */
    {
        float n2 = dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2];
        if (n2 > ESKF_GRAV_DX_MAX_RAD * ESKF_GRAV_DX_MAX_RAD) {
            float sc = ESKF_GRAV_DX_MAX_RAD / sqrtf(n2);
            for (i = 0; i < 3u; i++)
                for (j = 0; j < 3u; j++) K[i][j] *= sc;
            for (i = 0; i < 3u; i++) {
                float s = 0.0f;
                for (k = 0; k < 3u; k++) s += K[i][k] * r[k];
                dx[i] = s;
            }
        }
    }
    /* ★★ 偏航注入归零（2026-10-13）——【只改估计，不动协方差路径】
     *
     * 重力观测秩 2，零空间 = h（= 当前体系下的重力方向 = 偏航）。但 K = P Hᵀ S⁻¹
     * 左乘 P 后，因 P 在"可观子空间 ⊥h"与"不可观子空间 h"之间有互协方差
     * （实测运动期 |非对角|/对角 达 0.356），dx 会长出沿 h 的分量 ——
     * 即凭空修正一个【没有任何传感器观测它】的方向。
     * 实测后果（rec_20261013_062531 板载）：t=8~10 s 沿世界 z 被灌入 6.2° 偏航。
     *
     * 修法：把 dx 投影到 ⊥h。这只影响【估计】，不触碰协方差的更新路径
     * （P ← (I−KH)P 保持原样，060408 已证明该路径使 P 有界且正定）。
     * ★ 早期曾把"二维更新 + 冻结 P 子块"一起改，后者破坏 P 正定性
     *   （P_xx 变负、|非对角|/对角 达 0.99）导致大运动发散，已撤销。
     *   结论：估计与协方差必须分开处理，不能靠冻结协方差子块来实现可观测性约束。
     *
     * 注：倾斜分量不受影响 —— 投影只去掉沿 h 的部分，⊥h 部分原样保留。 */
    {
        float dh = dx[0]*h[0] + dx[1]*h[1] + dx[2]*h[2];
        for (i = 0; i < 3u; i++) dx[i] -= dh * h[i];
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
#ifdef PC_PROBE
    g_pc_p_post[0]=s_eskf.P[0][0]; g_pc_p_post[1]=s_eskf.P[1][1]; g_pc_p_post[2]=s_eskf.P[2][2];
    g_pc_p6post[0]=s_eskf.P[0][0]; g_pc_p6post[1]=s_eskf.P[1][1]; g_pc_p6post[2]=s_eskf.P[2][2];
    g_pc_p6post[3]=s_eskf.P[0][1]; g_pc_p6post[4]=s_eskf.P[0][2]; g_pc_p6post[5]=s_eskf.P[1][2];
#endif

#ifdef PC_PROBE
    {   /* 仅诊断: 偏航注入分量与 P_hh（三维式下两者均非零，属已知现象） */
        float s = 0.0f, dh = 0.0f; int p, q2;
        for (p = 0; p < 3; p++) dh += dx[p]*h[p];
        for (p = 0; p < 3; p++) for (q2 = 0; q2 < 3; q2++) s += h[p]*s_eskf.P[p][q2]*h[q2];
        g_pc_upd[0] = dh; g_pc_upd[1] = sqrtf(dx[0]*dx[0]+dx[1]*dx[1]+dx[2]*dx[2]); g_pc_upd[3] = s;
    }
#endif
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
    /* ★★ 启动鲁棒性（2026-10-13）：地磁不可用时【仅用重力】构造 q0。
     *
     * 原实现此处直接 `return 0u`，而调用方 (:492) 的 ready 仍【无条件置 1】，
     * 于是带着一个【失败的 q0】开机运行。实测后果（rec_20261013_060408, v3 帧）：
     *     valid = 0%（|a_mean|−1 = 0.0064 < 0.05 通过 ⟹ 只能是本条失败）
     *     初始倾斜偏差 7~11° ⟹ 重力残差 10° >> acc_thr 2.86°
     *     ⟹ 重力门静止期 grav_open = 0.0% ⟹ 姿态纯开环 ⟹ 水平闭合误差 3.4°
     * 是"初始姿态错、而非过程漂移"的指纹：绝对偏差 7~11° 而段间差仅 3.4°。
     *
     * 兜底做法：倾斜维（roll/pitch）由重力唯一确定 —— 这就是重力能给的，
     * 取与 zb 夹角【最大】的体轴做正交化，得到良态的 yb（= 任意偏航参考）。
     * 偏航维本无观测，取任意值不影响正确性；水平不再从一开始就偏。
     *
     * ★ 已知限制：此时 v_mag 是【按该任意参考】算出的，不是真实磁场。
     *   当前 mag_valid ≡ 0，地磁观测被门控而不会使用，故安全。
     *   若日后启用磁观测，必须加一个 mag_ref_ok 标志，在地磁参考非由实测
     *   地磁构造时【禁止】地磁更新，否则会把偏航拉向错误方向。 */
    if (nu < 1e-9f) {
        uint8_t kk = 0u;
        float amin = fabsf(zb[0]);
        if (fabsf(zb[1]) < amin) { amin = fabsf(zb[1]); kk = 1u; }
        if (fabsf(zb[2]) < amin) { kk = 2u; }
        {
            float ek[3];
            float d2, n2;
            ek[0] = 0.0f; ek[1] = 0.0f; ek[2] = 0.0f;
            ek[kk] = 1.0f;
            d2 = ek[0]*zb[0] + ek[1]*zb[1] + ek[2]*zb[2];
            for (i = 0; i < 3u; i++) um[i] = ek[i] - d2 * zb[i];
            n2 = sqrtf(um[0]*um[0] + um[1]*um[1] + um[2]*um[2]);
            if (n2 < 1e-6f) return 0u;      /* 理论上取最不正交的轴后不会发生 */
            for (i = 0; i < 3u; i++) um[i] /= n2;
        }
        s_eskf.mag_ref_ok = 0u;
    } else {
        for (i = 0; i < 3u; i++) um[i] = u_mean[i] / nu;
        s_eskf.mag_ref_ok = 1u;
    }

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
    s_eskf.mag_ref_ok = 0u;
    s_cal_n_mag  = 0u;
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

/* ★ 持久状态上报（2026-10-13）：这三个量是【纯历史量】，PC 侧截取片段复刻
 *   时无法反推，导致 grav_open 8.2% vs 板载 53.6%、姿态发散。
 *     acc_thr : 自适应重力残差门限（接受则衰减、拒绝则增长）—— 直接决定门的开合率
 *     P[3][3] : 协方差。非对角项 Pxy/Pxz/Pyz 就是"重力更新经互协方差漏到偏航"
 *               的那条通道，是判定"偏航是否被正确懒更新"的关键观测量
 *     n       : 启动标定累计样本数（诊断标定是否完成） */
float proc_eskf_acc_thr(void) { return s_eskf.acc_thr; }
float proc_eskf_calib_n(void) { return (float)s_eskf.n; }
void  proc_eskf_p_sym(float *out6)
{
    out6[0] = s_eskf.P[0][0]; out6[1] = s_eskf.P[1][1]; out6[2] = s_eskf.P[2][2];
    out6[3] = s_eskf.P[0][1]; out6[4] = s_eskf.P[0][2]; out6[5] = s_eskf.P[1][2];
}
uint8_t proc_eskf_calibrating(void) { return s_eskf.calibrating; }
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

#ifdef PC_PROBE
/* PC 侧注入接口(仅在 PC 构建存在): 从录制第 0 帧预置 ESKF 状态, 使复刻与板载同起点 */
float g_pc_throt_set = -1.0f;
float g_pc_nis = 0.0f, g_pc_applied = 0.0f;
float g_pc_p_pre[3] = {0,0,0}, g_pc_p_post[3] = {0,0,0};
float g_pc_p6pre[6] = {0,0,0,0,0,0}, g_pc_p6post[6] = {0,0,0,0,0,0};
void proc_eskf_pc_inject(const float *q4, const float *P6, float acc_thr, float throt)
{
    int a, b;
    for (a = 0; a < 4; a++) s_eskf.q[a] = q4[a];
    s_eskf.P[0][0] = P6[0]; s_eskf.P[1][1] = P6[1]; s_eskf.P[2][2] = P6[2];
    s_eskf.P[0][1] = s_eskf.P[1][0] = P6[3];
    s_eskf.P[0][2] = s_eskf.P[2][0] = P6[4];
    s_eskf.P[1][2] = s_eskf.P[2][1] = P6[5];
    for (a = 0; a < 3; a++) for (b = 0; b < 3; b++) { }
    s_eskf.acc_thr = acc_thr;
    s_eskf.ready = 1u; s_eskf.valid = 1u; s_eskf.calibrating = 0u;
    s_eskf.n = 15000u;
    g_pc_throt_set = throt;
}
#endif
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
            /* ★★ 关键修正（2026-10-13）：IMU 样本【一律】推进主计数。
             *   原写法只在 mag_valid != 0 时 n++，而当前固件 proc_mag.c 从不置
             *   mag_valid（mag_u 硬写 0）⟹ n 恒 0、NWIN 永不达到、:474 的
             *   ready=1 永不可达 ⟹ grav_open ≡ 0，整条重力链失效。
             *   实测证据：1,377,054 帧（4 份录制）flag bit0/bit2/bit3 全为 0。
             *   地磁只推进自己的计数，仅用于决定 u_mean（偏航参考）是否可用。 */
            for (i = 0; i < 3u; i++) s_eskf.s_a[i] += a_g[i];
            s_eskf.n++;
            if (mag_valid != 0u) {
                for (i = 0; i < 3u; i++) s_eskf.s_m[i] += mag_u[i];
                s_cal_n_mag++;
            }
            return;
        }
        {
            float invn = 1.0f / (float)s_eskf.n;
            /* ★ 地磁与 IMU 的样本数不同（磁拍 ~1020 Hz / IMU 7673 Hz），
             *   必须用各自的计数归一，否则 u_mean 会小 7.5 倍。 */
            float invm = 1.0f / (float)((s_cal_n_mag != 0u) ? s_cal_n_mag : 1u);
            float a_mean[3], u_mean[3];
            float an_mean, unorm;
            for (i = 0; i < 3u; i++) {
                a_mean[i] = s_eskf.s_a[i] * invn;
                u_mean[i] = s_eskf.s_m[i] * invm;
            }
            an_mean = sqrtf(a_mean[0]*a_mean[0] + a_mean[1]*a_mean[1] + a_mean[2]*a_mean[2]);
            unorm   = sqrtf(u_mean[0]*u_mean[0] + u_mean[1]*u_mean[1] + u_mean[2]*u_mean[2]);
            /* ★ valid 只由【重力】决定。原写法把它与地磁绑定，使"地磁坏
             *   ⟹ 整个启动标定判失败"，这正是"地磁拖坏重力链"的结构性根源。
             *   地磁缺失时 valid 仍可为 1，只是 v_mag 不可用（偏航降级）。 */
            s_eskf.valid = (fabsf(an_mean - 1.0f) < ESKF_START_MAGLIM_G) ? 1u : 0u;
            /* ★ 与 PC 仿真原版逐字一致：由 (a_mean, u_mean) 构造 q0 与 v_mag */
            /* ★ unorm≈0（地磁样本不足）时 build_q0 必然失败，valid 置 0 表示
             *   "参考系不完整（缺偏航）"；但 ready 已在下方无条件置位，
             *   重力环照常运行 —— 这正是本次解耦的目的。 */
            (void)unorm;
            if (eskf_build_q0(a_mean, u_mean) == 0u) s_eskf.valid = 0u;
        }
        /* ★★ 启动鲁棒性（2026-10-13）：只有 q0 【成功构造】后才接管姿态。
         * 原写法此处无条件 ready=1：build_q0 失败时带着未构造的 q0 运行，
         * 实测（rec_20261013_060408）初始倾斜偏差 7~11° ⟹ 重力残差 10° >>
         * acc_thr 2.86° ⟹ 重力门静止期永不打开 ⟹ 姿态纯开环。
         * valid==0 只可能来自：① 重力模长不在 ±5% 内（设备并非真静止）
         * ② build_q0 连"仅重力"都无法构造（极其退化）。两种情况都应当
         * 【继续累计、保持标定态、下一帧重试】，而不是带着坏初值开机。 */
        if (s_eskf.valid != 0u) {
            /* 初始协方差: 姿态 2° */
            for (i = 0; i < 3u; i++) for (j = 0; j < 3u; j++) s_eskf.P[i][j] = 0.0f;
            for (i = 0; i < 3u; i++) s_eskf.P[i][i] = 0.001218f;   /* (2°)² */
            s_eskf.acc_thr = ESKF_ACC0_RAD;
            s_eskf.calibrating = 0u;
            s_eskf.ready = 1u;
        } else {
            s_eskf.n = 0u;
            for (i = 0; i < 3u; i++) {
                s_eskf.s_a[i] = 0.0f;
                s_eskf.s_m[i] = 0.0f;
            }
            s_cal_n_mag = 0u;
            /* calibrating 保持 1：下一帧重新累计 */
        }
        return;
    }

    if (s_eskf.ready == 0u) return;

    /* ★ 运行累计转角(rad)：驱动【系统性】标度误差的方差增长。
     * 函数内 static；标定期 ready=0 ⟹ 预测不跑 ⟹ 自然从 0 起，无需复位。 */
    static float s_throt_acc = 0.0f;
#ifdef PC_PROBE
    if (g_pc_throt_set >= 0.0f) { s_throt_acc = g_pc_throt_set; g_pc_throt_set = -1.0f; }
#endif

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
        /* ★ Q_θθ = q_arw·dt + (k_s·|ω|·dt)²   （docs/ekf_native_gate_design.md §1）
         * 第二项让 P 随【累计转角】增长 ⟹ 积分后 P_θ ≈ (k_s·θ_rot)²，
         * 正是实测到的误差形态（∝θ_rot、无速度敏感）⟹ P 诚实化，
         * 运动后 NIS 门才会正确打开。原先只有常数项，导致运动 20 s 后
         * P 仍只有 0.36°(1σ) 而真实倾斜误差 7.8°。 */
        float qg = ESKF_SIG_G * ESKF_SIG_G * dt_s;
        {
            float wn = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);  /* rad/s */
            /* ★★ 修正（2026-10-13）：原写 kw=(k_s·|ω|·dt) 再 qg+=kw²，那是对
             * 【随机游走】的建模：累积 N 步 P = k_s²·Σ(|ω|dt)² ≈ k_s²·N·x̄²  ∝ N。
             * 而标度误差是【确定性】误差：θ_err = k_s·θ_rot，方差应为 k_s²·θ_rot²  ∝ N²。
             * 两者 σ 相差 √N ≈ 300 倍 —— 实测 P 倾斜 1σ 仅 0.49° 而真实倾斜误差 12.7°，
             * ⟹ S = HPHᵀ+R 塌到 R ⟹ NIS = 12.67²/(0.1312°)² = 9330 ≫ 阈值
             * ⟹ NIS 门每帧拒绝 ⟹ grav_open ≡ 0（比旧残差门更严的锁死，实测 080401）。
             * 要得到 P = (k_s·θ_rot)²，每步增量须为 dP = 2·k_s²·θ_rot·dθ_rot：
             *     Q_θθ = q_arw·dt + 2·k_s²·θ_rot·|ω|·dt
             * 文档 §2.4 的 P_θ=(k_s·θ_rot)²（线性）与 §1 字面的 Q（√N）本身矛盾；
             * 此处按 §2.4 的结论实现。 */
            /* ★★ NaN/离群守卫（2026-10-13）：s_throt_acc 是无保护累加器 ——
             * 单个非有限或离群样本会把它【永久】污染（累加器不自愈）：
             *   s_throt_acc=NaN ⟹ qg=2k_s²·θ|ω|dt=NaN ⟹ P 全 NaN
             *   ⟹ S=HPHᵀ+R=NaN ⟹ NIS=rᵀS⁻¹r=NaN
             *   ⟹ !(NaN<=阈值) 恒真 ⟹ 门【永久拒绝】。
             * PC 复刻实测（rec_20261013_080750 注入板载初值）：t≈16s 起
             *   NIS 变 NaN，applied 全为 -1，343637/343697 帧被拒（100%），
             *   复刻 resid 因此漂到 95~130°。板载 grav_open 也仅 13.7%。
             * 故此处必须：① 丢弃非有限/离群的 wn 与 dt ② 给累加器设上限
             *             ③ 给 qg 兜底（单步方差上限 1 rad²=57° 已属荒谬）。 */
            if ((wn == wn) && (wn < 1.0e4f) && (dt_s == dt_s) && (dt_s > 0.0f) && (dt_s < 0.05f)) {
                s_throt_acc += wn * dt_s;
                if (s_throt_acc > 1.0e5f) s_throt_acc = 1.0e5f;          /* ≈1.6e6 度上限 */
                qg += 2.0f * ESKF_KS_SCALE * ESKF_KS_SCALE * s_throt_acc * wn * dt_s;
            }
            if (!(qg == qg) || (qg > 1.0f)) qg = (qg == qg) ? 1.0f : 1.2246e-6f;
        }
        for (i = 0; i < 3u; i++)
            for (j = 0; j < 3u; j++)
                s_eskf.P[i][j] = FPFt[i][j] + (((i == j) ? qg : 0.0f));
    }
#ifdef PC_PROBE
    g_pc_p_pre[0]=s_eskf.P[0][0]; g_pc_p_pre[1]=s_eskf.P[1][1]; g_pc_p_pre[2]=s_eskf.P[2][2];
    g_pc_p6pre[0]=s_eskf.P[0][0]; g_pc_p6pre[1]=s_eskf.P[1][1]; g_pc_p6pre[2]=s_eskf.P[2][2];
    g_pc_p6pre[3]=s_eskf.P[0][1]; g_pc_p6pre[4]=s_eskf.P[0][2]; g_pc_p6pre[5]=s_eskf.P[1][2];
#endif

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
    if ((mag_valid != 0u) && (new_mag != 0u) && (s_eskf.mag_ref_ok != 0u)) {
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

        /* ★★ 诚实卡尔曼原生门控（2026-10-13）
         *    实现 docs/ekf_native_gate_design.md §1 / §2.4 / §3。
         *
         * 只保留【与姿态几何无关】的质量门：加计此刻是否真的在测重力。
         * 删掉自适应残差门 `resid < acc_thr` —— 它是文档 §1 明确要求删除的"质量门"：
         *   "把'质量门 + 抬 R + 失效计时器'全部删掉，改成只调 Q 与 R 的诚实卡尔曼"。
         *
         * 残差门为何【必然】锁死（实测 rec_20261013_072538）：
         *   门开条件 resid < acc_thr，而 acc_thr 有硬上限 ESKF_ACC_MAX_RAD = 0.05 rad = 2.8648°。
         *   运动中门本就该关 ⟹ 20 s 剧烈运动后倾斜开环漂移到 7.78°。
         *   静止后 7.78° > 2.8648° ⟹ 门不开；而唯一能减小 resid 的正是【这个门控制的修正】
         *   ⟹ 残差只能继续开环生长 ⟹ 永久锁死。
         *   文档 §7 已记录同一故障："两个不同的门曾混用导致交叉门控死锁：姿态歪 6.7°
         *   ⇒ 姿态回不来"。本次实测 7.78° 即此。
         *
         * 原生判据 K = P/(P+R) 自动给出交叉点（§2.4），无需任何阈值：
         *   本录制末段 P 垂直平面 1σ = 0.360°，加计噪声 σ_a = ESKF_ACC_SIG_DEG 取实测 0.13°
         *   ⟹ K = 0.885 ⟹ 门等效 88.5% 开 ⟹ 那 7.78° 会被修正掉。
         *   而 P 由 Q 驱动增长（§1: Q_θθ = q_arw·dt + (k_s·|ω|·dt)²）⟹ 运动越多越信重力，
         *   这正是"交叉点自动涌现"，与"误差越大门越关"的正反馈方向相反。
         * 保留的两个门都是"加计是否可信"的物理判据，与姿态误差大小无关。 */
        if ((fabsf(an - 1.0f) < ESKF_ACC_MAGLIM_G) && (avn < ESKF_GRAV_WLIM_DPS)) {
            float rs = ESKF_ACC_SIG_DEG * ESKF_DEG2RAD;
            float applied = eskf_update(s_eskf.v_up, z, rs*rs, rs*rs);
#ifdef PC_PROBE
            g_pc_applied = applied;
#endif
            s_eskf.grav_open = (applied >= 0.0f) ? 1u : 0u;   /* NIS 拒绝返回 -1 */
        }
        (void)resid;   /* resid 仍用于下方 acc_resid_deg 诊断上报 */
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
