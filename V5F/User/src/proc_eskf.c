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
/* ★ 构建版本标识（每次改动本文件必须同步更新，供"输出即证明版本"使用） */
#define ESKF_BUILD_VER  "ESKF-20261014-0715-nograv-3deg20s"

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
#define ESKF_KS_SCALE       7.5e-5f
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
    float   acc_thr;            /* 自适应残差门当前值（已废弃，仅留上报） */
    uint8_t grav_open;          /* 本帧重力观测是否施加 */
    float   nograv_s;           /* ★ 重力观测连续不可用的时长（s），用于无重力期 P 膨胀 */
    uint8_t grav_last;          /* ★ 上一帧 grav_open（预测步在重力门之前 ⟹ 只能看上帧） */
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
extern float g_pc_wigg;
#endif
/* ★★ IGG-III 三段鲁棒权常数（2026-10-13 重建）
 * 作用: d = rᵀS⁻¹r 落在 [K0,K1] 时按 (K0/rd)·tt² 降权，≥K1 时压到 WMIN（地板，不硬拒）。
 * 用法: 【重力】use_igg=1 启用；【地磁】use_igg=0 ⟹ w ≡ 1.0（地磁不用 IGG，
 *       依据: 2026-10-14 用户指示 + 实测"地磁 IGG 会把大新息判粗差压到 WMIN 而自锁"）。 */
#ifndef ESKF_IGG_K0
#define ESKF_IGG_K0     1.0f
#endif
#ifndef ESKF_IGG_K1
#define ESKF_IGG_K1     3.0f
#endif
#ifndef ESKF_IGG_WMIN
#define ESKF_IGG_WMIN   1.0e-3f
#endif
/* ★★ 强制恢复（recovery trigger）常数
 *   REJ_WTHR : 判为"被拒"的权阈（≤ 它则计数 +1）
 *   REJ_MAX  : 连续被拒帧数上限，超过则强制 w=1 放行一次
 *   取值依据：门开帧 7670 Hz 下 0.25 s ≈ 1900 帧 ⟹ 误判到恢复的时延 ~0.25 s，
 *   而"粗差是瞬态的"（实测中位 2~3 帧）⟹ 不会把真实粗差放进来。 */
#ifndef ESKF_REJ_WTHR
#define ESKF_REJ_WTHR   0.1f
#endif
#ifndef ESKF_REJ_MAX
#define ESKF_REJ_MAX    1900u
#endif
static uint16_t s_rej_cnt = 0u;
/* ★★ 无重力观测段（|ω| 大 / 加计不可信）倾斜 1σ 的增长率（rad/√s）。
 *   依据 docs/IGG-III稳健权设计记录.md §1：σ_tilt = ESKF_SIG_NOGRAV · t_nograv。
 *   ★ 取值以【原代码产物实测】为准（目标：与原来相同），两步定出：
 *     ① 反编译 x_025232_on.bin 最长无重力段（17.69 s, 帧 38455..174211）:
 *          P6pre 迹 3.8353e-07 → 6.7836e-04 ⟹ 该段增长 dP = 6.7797e-04 rad²
 *     ② 在本地构建同段量出 Σ t_nograv·dt = 47.8636（由 ΔP=2N²Σt·dt 反解）
 *        ⟹ N = √(dP / (2Σ)) = 2.6613e-03 rad/√s
 *   接力文档 §1.2 记的 2.384e-3 与之相差 12%，属同一量级（可能口径略异）。 */
#ifndef ESKF_SIG_NOGRAV
#define ESKF_SIG_NOGRAV     2.6180e-3f
#endif
/* ★★ 无重力期累积器（依据归档设计的 v2 模型，逐帧严格二选一）：
 *     门关 / 重力不可用 :  acc <- acc + GROW              （GROW = 1e-6，与 dt 无关）
 *     门开且牵引成功    :  acc <- max(ACC_MIN, acc*DECAY) （DECAY = 0.9995，ACC_MIN = 0.02）
 *   ★ 地板 ACC_MIN = 0.02 是关键：门抖动时 acc 贴着地板缓慢摆动，
 *     而【不会】被清零 —— 这正是"用比例衰减代替清零"的本意。
 *   注入映射：σ_tilt = ESKF_SIG_NOGRAV · sqrt(acc/ACC_MIN)（acc = ACC_MIN 时恰为 SIG_NOGRAV），
 *     门关期间每步把 ΔP = σ_tilt² = SIG_NOGRAV² · acc/ACC_MIN 加到倾斜对角。 */
static float s_last_w = 1.0f, s_last_nis = 0.0f;

/* ★★ PC 侧探针全局量（10-14 pc_chain_main.c 需要）
 * 说明：这些量【只用于 PC 复刻的上报列】，不参与任何算法判断。
 * g_pc_mv / g_pc_mn / g_pc_mr / g_pc_caln / g_pc_calmag 定义在 proc_mag.c；
 * 以 g_pc_q_ 开头的、以及 stage / upd、mvcall / mncall 由 pc_chain_main.c 定义。 */
#ifdef PC_PROBE
float g_pc_wigg = 1.0f;
float g_pc_nograv = 0.0f;
float g_pc_gate_an = 0.0f, g_pc_gate_avn = 0.0f, g_pc_gate_res = 0.0f;
float g_pc_ready = 0.0f, g_pc_calmag2 = 0.0f, g_pc_valid2 = 0.0f;
float g_pc_mres = 0.0f, g_pc_mux = 0.0f, g_pc_muy = 0.0f, g_pc_muz = 0.0f;
float g_pc_vmx = 0.0f, g_pc_vmy = 0.0f, g_pc_vmz = 0.0f, g_pc_mrok = 0.0f;
float g_pc_probe_unorm = 0.0f, g_pc_probe_nmag = 0.0f, g_pc_probe_anmean = 0.0f;
const char *g_pc_eskf_ver = ESKF_BUILD_VER;
#endif

/* ★★ 导航系磁场参考 v_mag 的【静止窗累加器】（2026-10-14）
 * ★ 这是产品逻辑（板载与 PC 都要用），绝不可放进 #ifdef PC_PROBE：
 *   MCU 构建无 PC_PROBE ⟹ 会全部"未声明"（v5f 编译失败的根因）。
 * 窗内累加实测 a_g 与磁单位矢量；窗满后用与 eskf_build_q0 等价的公式
 * 构造 v_mag（见下方磁观测分支）。 */
static float s_vmag_acc[3]  = {0.0f, 0.0f, 0.0f};   /* 磁单位矢量累加 */
static float s_vmag_aacc[3] = {0.0f, 0.0f, 0.0f};   /* 加计累加（供体->导航参考系） */
static uint16_t s_vmag_cnt = 0u;
static uint8_t  s_vmag_ready = 0u;
#ifndef ESKF_VMAG_NWIN
#define ESKF_VMAG_NWIN  1000u     /* 累加窗：1000 个磁样本 ≈ 0.83 s */
#endif
float proc_eskf_grav_w(void)   { return s_last_w; }
float proc_eskf_grav_nis(void) { return s_last_nis; }
void proc_eskf_pc_set_vmag(const float *v3)
{
    float n;
    uint8_t a;
    for (a = 0; a < 3u; a++) s_eskf.v_mag[a] = v3[a];
    n = sqrtf(s_eskf.v_mag[0]*s_eskf.v_mag[0] + s_eskf.v_mag[1]*s_eskf.v_mag[1]
            + s_eskf.v_mag[2]*s_eskf.v_mag[2]);
    if (n > 1e-9f) { for (a = 0; a < 3u; a++) s_eskf.v_mag[a] /= n; }
}
static float eskf_update(const float *v_nav, const float *z, float r_perp, float r_par,
                         uint8_t use_igg)
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
    /* ★★ NIS（2026-10-14 重建）: 只【计算并上报】，不再作为生死门。
     *   原写法 `if (nis > ESKF_GRAV_NIS_MAX) return -1` 会造成：
     *     P 被压到 ≈R 量级后 NIS = resid²/(P⊥h+R) 极严 ⟹ 每帧拒绝 ⟹ 永久锁死
     *     （实测 rec_20261013_082033 留下 3° 平台）。
     *   拦截运动污染帧的职责已由【速率门(20 dps) + 模长门(0.05g)】承担。
     *   保留 nis 供上报（col24）与 IGG 降权使用，不返回 -1。 */
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
        s_last_nis = nis;                       /* ★ 上报用: d = rᵀ S⁻¹ r */
        {
            /* ★★ IGG-III 三段权（2026-10-13 重建；2026-10-14 加 use_igg 开关）
             *   rd = sqrt(nis/2)（χ²(2) 自由度归一）
             *   rd < K0        ⟹ w = 1
             *   K0 ≤ rd < K1   ⟹ w = (K0/rd)·((K1-rd)/(K1-K0))²
             *   rd ≥ K1        ⟹ w = WMIN（地板，永不归零 ⟹ 不会自锁）
             *   use_igg==0（地磁）⟹ w ≡ 1.0
             * K ← w·K（缩放 PHt 即缩放 K）⟹ 估计与协方差同步受限，一致性不破。 */
            float rd = sqrtf((nis > 0.0f ? nis : 0.0f) * 0.5f);
            float w;
#if defined(ESKF_IGG_OFF)
            /* 实验臂 A：完全不用 IGG（只靠门控拦污染）⟹ w ≡ 1 */
            w = 1.0f;
#elif defined(ESKF_IGG_SOFT)
            /* 实验臂 B：放宽 IGG —— 内点域 ×3、地板 0.001 → 0.3 */
            if (use_igg == 0u) {
                w = 1.0f;
            } else if (rd < ESKF_IGG_K0 * 3.0f) {
                w = 1.0f;
            } else if (rd < ESKF_IGG_K1 * 3.0f) {
                float k0 = ESKF_IGG_K0 * 3.0f, k1 = ESKF_IGG_K1 * 3.0f;
                float tt = (k1 - rd) / (k1 - k0);
                w = (k0 / (rd > 1e-6f ? rd : 1e-6f)) * tt * tt;
                if (w < 0.3f) w = 0.3f;
            } else {
                w = 0.3f;
            }
#else
            /* 基准臂：原 IGG-III 三段权（K0=1, K1=3, WMIN=1e-3） */
            if (use_igg == 0u) {
                w = 1.0f;
            } else if (rd < ESKF_IGG_K0) {
                w = 1.0f;
            } else if (rd < ESKF_IGG_K1) {
                float tt = (ESKF_IGG_K1 - rd) / (ESKF_IGG_K1 - ESKF_IGG_K0);
                w = (ESKF_IGG_K0 / (rd > 1e-6f ? rd : 1e-6f)) * tt * tt;
            } else {
                w = ESKF_IGG_WMIN;
            }
#endif
            if (!(w >= ESKF_IGG_WMIN)) w = ESKF_IGG_WMIN;
            /* ★★ 强制恢复（recovery trigger，2026-10-14）
             * 问题：IGG 的 w=WMIN 把【真实的大误差】与【粗差】当同一类。
             *   实测 rec_20261014_000422：重力门只开 0.04%，门开时 acc_resid 中位
             *   55.84°，落 WMIN 后永不再修 ⟹ 与文档 §L19/L144 描述的自锁同构。
             *   数学上无法靠膨胀 P 解决：要 rd≤3 需 P≥r²/9=(0.97rad)²/9=0.104 rad²
             *   （σ_tilt=18.5°）⟹ 那已不是滤波。
             * 判据：连续被降权的帧数超阈 ⟹ "重力一直在坚持同一个说法" ⟹ 错的是滤波器
             *   ⟹ 强制按全权接受一次。用【泄漏计数器】防逐帧抖动误触。
             *   w ≤ REJ_WTHR ⟹ cnt++；否则 cnt--（下限 0）。
             *   cnt > REJ_MAX ⟹ w 强制 1，并把 cnt 清零（重新计时）。
             * 只在门开帧可能触发（门关时本函数不被调用）⟹ 纯平动/剧烈运动"门本就关"
             *   的情况不会误触发；只有"门开着却一直不接受"才触发。
             * 设计依据：reference_implementations_survey.md 记的 Fusion recoveryTrigger
             *   （"连续被拒 +1、通过 −9，超过 rejectionTimeout 就强制放行一次"）。 */
            if (use_igg != 0u) {
                if (w <= ESKF_REJ_WTHR) {
                    if (s_rej_cnt < 0xFFFFu) s_rej_cnt++;
                } else if (s_rej_cnt > 0u) {
                    s_rej_cnt--;
                }
                if (s_rej_cnt > ESKF_REJ_MAX) {
                    w = 1.0f;            /* 强制放行 */
                    s_rej_cnt = 0u;
                }
            }
            if (w < 1.0f) {
                for (i = 0; i < 3u; i++)
                    for (j = 0; j < 3u; j++) PHt[i][j] *= w;
            }
            /* ★ s_last_w 必须在 #ifdef 之外 —— MCU 侧也要更新，否则上报恒为初值 1.0 */
            s_last_w = w;
#ifdef PC_PROBE
            g_pc_wigg = w;
#endif
        }
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
        /* ★★ 限幅只作用于【重力】通道（2026-10-14 用户要求）
         *   原始 0.5° 限幅是为重力设的（拦加计被线加速度污染的帧）。
         *   磁通道由用户明确要求【无限幅】—— 磁不存在"被运动污染"的同构问题，
         *   且限幅会让磁的大错位以 0.5°/帧 缓慢爬行（实测 54° 要 108 帧）。
         *   用 use_igg==0 作为"磁通道"标记（磁恒为 0，重力恒为 1）。 */
        if ((use_igg != 0u) && (n2 > ESKF_GRAV_DX_MAX_RAD * ESKF_GRAV_DX_MAX_RAD)) {
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
{    float zb[3], yb[3], xb[3], R0[9], um[3];
    float na, nu, dp, ny, sn;
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
    s_eskf.nograv_s = 0.0f;         /* ★ 无重力期计时（P 膨胀用） */
    s_eskf.grav_last = 0u;
    /* v_mag static-window accumulators reset.
     * MUST be on the product path: the board never calls proc_eskf_pc_inject,
     * so resetting only there would leave stale accumulators after a board reset. */
    s_vmag_acc[0] = s_vmag_acc[1] = s_vmag_acc[2] = 0.0f;
    s_vmag_aacc[0] = s_vmag_aacc[1] = s_vmag_aacc[2] = 0.0f;
    s_vmag_cnt = 0u;
    s_vmag_ready = 0u;
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
    /* ★★ 导航系磁场参考 v_mag 的数学等效重建（2026-10-14）
     * 问题：注入旁路了启动标定 ⟹ eskf_build_q0 从不执行 ⟹
     *       v_mag 停在初值 [0,1,0]（水平任意方向）、mag_ref_ok=0。
     *       于是磁观测拿【水平假参考】去拉姿态，把航向拉到错处
     *       （实测 mag_resid 56.4°、姿态歪 54°）。
     * 正解（与 eskf_build_q0 数学等价，只是数据来源改为实测）：
     *       v_mag = normalize( Σ_i R(q_i)·z_i )
     *   其中 z_i = 实测机体系磁场单位矢量（proc_mag 的 mag_u），
     *   R(q_i) = 由注入姿态定义的 体→导航 旋转。
     *   物理含义：把上电以来的实测机体系磁场，按当时的姿态转到导航系；
     *   姿态正确时它们应重合，其均值即导航系磁场方向（含真实倾角）。
     * 实现：开启累加窗，在磁观测分支里累加 R(q)·z；窗满后归一赋给 v_mag。 */
    s_vmag_acc[0] = s_vmag_acc[1] = s_vmag_acc[2] = 0.0f;
    s_vmag_aacc[0] = s_vmag_aacc[1] = s_vmag_aacc[2] = 0.0f;
    s_vmag_cnt = 0u;
    s_vmag_ready = 0u;
    s_eskf.mag_ref_ok = 0u;
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
#ifdef PC_PROBE
            g_pc_probe_unorm = unorm;
            g_pc_probe_nmag = (float)s_cal_n_mag;
            g_pc_probe_anmean = an_mean;
#endif
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

#ifdef PC_PROBE
    g_pc_ready = (float)s_eskf.ready; g_pc_calmag2 = (float)s_cal_n_mag; g_pc_valid2 = (float)s_eskf.valid;
#endif
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
        /* ★★ 无重力期倾斜方差膨胀（2026-10-14 重建）
         * 依据 docs/IGG-III稳健权设计记录.md §6.3 / §8:
         *   ESKF_SIG_NOGRAV —— "重力长期不可用时膨胀倾斜方差"
         *   §1 的物理式: σ_tilt = ESKF_SIG_NOGRAV · t_nograv
         *
         * ★ 设计原则（2026-10-14 定稿）：
         *   P 是【一致性诊断量】，不是可以随便拧的旋钮。
         *   IGG 的 rd = r/√S 判的是"新息相对滤波器自报不确定度是否异常"。
         *   若 P 诚实（= 真实误差量级），正常误差下 rd ≈ 1 ⟹ 门自然开 ⟹ 不自锁；
         *   若 P 撒谎（偏小），真实误差会被误判成 16σ 粗差 ⟹ w 落 WMIN ⟹ 锁死。
         *   所以这里【不追求把 rd 压到某个目标】，只做一件事：
         *   让 P 按【真实的倾角不确定度增长率】增长。
         *
         * ★ 增长率取值（用户 2026-10-14 定：直接用 3°）：
         *   σ_tilt 在 t_nograv = 20 s 时达到 3°（文档记录的最恶劣连续无重力时长）
         *   ⟹ N = 3° / 20 s = 0.052360 rad / 20 = 2.6180e-3 rad/√s
         *   对照：本工程实测（x_025232_on，P∝t²）为 1.473e-3（17.69 s 时 1.492°），
         *        文档陀螺漂移率 0.1366 °/s 为 2.384e-3。
         *   取 2.618e-3 满足：① 与手册量级一致；② 大于实测 ⟹ P 增长快于真实误差
         *   ⟹ 自带防锁裕度（这正是"P 不撒谎"的充要条件）。
         *
         * 注入方式【每步增量】: ΔP = 2·N²·t_nograv·dt_s，积分得 P = N²·t_nograv²
         *   （即 σ_tilt = N·t_nograv 的平方，与 §1 物理式一致）。
         *   grav_last 用于判断，因为预测步在本帧重力判定【之前】执行。
         * 安全性: 累加器有上限、非有限守卫；只加对角（与陀螺 qg 同形）。 */
        {
            float infl = 0.0f;
#ifdef ESKF_NOGRAV_INFL_OFF
            (void)infl;         /* 仅用于单变量 A/B */
#else
            if (s_eskf.nograv_s > 0.0f) {
                infl = 2.0f * ESKF_SIG_NOGRAV * ESKF_SIG_NOGRAV * s_eskf.nograv_s * dt_s;
                if (!(infl == infl) || (infl > 1.0f)) infl = 0.0f;
            }
#endif
            for (i = 0; i < 3u; i++)
                for (j = 0; j < 3u; j++)
                    s_eskf.P[i][j] = FPFt[i][j] + (((i == j) ? (qg + infl) : 0.0f));
        }
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
    /* NOTE: g_pc_mv / g_pc_mn / g_pc_mr / g_pc_caln / g_pc_calmag 由 proc_mag.c 维护；
     *       本文件只【定义】它们（PC_PROBE 下）供 pc_chain_main.c 链接，不在此处写入。 */
    /* ★★ 【临时开关】地磁观测总闸 ★★
     *   ESKF_USE_MAG_OBS = 0 时完全不做地磁观测（不影响启动标定）。
     *   默认 1（2026-10-14）：地磁观测为【唯一】观测，无门控。 */
#ifndef ESKF_USE_MAG_OBS
#define ESKF_USE_MAG_OBS    1
#endif
#if ESKF_USE_MAG_OBS
    /* ★★ 地磁观测【零门控】（2026-10-14，用户指示）
     *   原条件 `(mag_valid) && (new_mag) && (mag_ref_ok)` 里，
     *   mag_ref_ok 是【v_mag 是否由实测地磁构造】的标志；但 PC 复刻的 v_mag
     *   是从录制通道注入的（proc_eskf_pc_set_vmag），该标志恒为 0 ⟹ 整条磁观测
     *   被挡死（实测 col45 mv / col46 mn 全 0，attitude 对地磁零响应）。
     *   现只保留两个【数据存在性】条件，不再有任何门控/阈值/增益缩放：
     *     mag_valid : 本帧有有效磁样本（proc_mag.c 产出，SET 相）
     *     new_mag   : 本帧确实是【新】磁拍（防同一份样本被多帧重复注入）
     *   IGG 权已由 use_igg=0 关闭 ⟹ w ≡ 1.0；NIS 已改为只上报；
     *   单帧限幅 ESKF_GRAV_DX_MAX_RAD 保持不变（对磁/重力同样作用）。 */
    if ((mag_valid != 0u) && (new_mag != 0u)) {
        float h[3], z[3], c;
        for (i = 0; i < 3u; i++) z[i] = mag_u[i];
        /* ★★ 导航系磁场参考 v_mag 的构造（2026-10-14，静止窗公式）
         * 与 eskf_build_q0 的数学完全一致，只是数据源为【注入后的重启窗实测均值】：
         *     zb = a_mean/|a_mean|;  um = u_mean/|u_mean|
         *     yb = normalize(um − (um·zb)zb);  xb = zb × yb
         *     R0 = [xb; yb; zb];  v_mag = R0·um   （vb = R0ᵀ，故 90−…）
         * 物理含义：用【上电后实测的重力方向与地磁矢量方向】在世界系中定出姿态，
         *   再求出地磁矢量在世界系的方向 —— 即用户要求的"上电测地磁世界系方向 +
         *   自身上电姿态相较世界系的旋转"，无需假设上电后地磁在机体系的角度。
         * ★ 为什么不用 Σ R(q)·z：那会把【滤波姿态自身的误差】固化进参考，
         *   实测两者差 2.7°（Σ R(q)·z ⟹ 57.70°，静止窗公式 ⟹ 60.22° 口径下）。 */
        if (s_vmag_ready == 0u) {
            for (i = 0; i < 3u; i++) {
                s_vmag_acc[i] += z[i];
                s_vmag_aacc[i] += a_g[i];
            }
            s_vmag_cnt++;
            if (s_vmag_cnt >= ESKF_VMAG_NWIN) {
                float inv = 1.0f / (float)s_vmag_cnt;
                float am[3], um[3], zb[3], yb[3], xb[3];
                float na, nu, dp, ny;
                for (i = 0; i < 3u; i++) { am[i] = s_vmag_aacc[i] * inv; um[i] = s_vmag_acc[i] * inv; }
                /* ★ 只算 v_mag，【不调用 eskf_build_q0】—— 那个函数会覆盖 s_eskf.q，
                 *   而 PC 复刻的姿态是由录制注入的，不可被破坏。
                 *   公式与 eskf_build_q0 完全一致（R0 = [xb;yb;zb] 为体→导航）： */
                na = sqrtf(am[0]*am[0] + am[1]*am[1] + am[2]*am[2]);
                nu = sqrtf(um[0]*um[0] + um[1]*um[1] + um[2]*um[2]);
                if ((na > 1e-6f) && (nu > 1e-9f)) {
                    for (i = 0; i < 3u; i++) { zb[i] = am[i] / na; um[i] = um[i] / nu; }
                    dp = um[0]*zb[0] + um[1]*zb[1] + um[2]*zb[2];
                    for (i = 0; i < 3u; i++) yb[i] = um[i] - dp * zb[i];
                    ny = sqrtf(yb[0]*yb[0] + yb[1]*yb[1] + yb[2]*yb[2]);
                    if (ny > 1e-6f) {
                        for (i = 0; i < 3u; i++) yb[i] /= ny;
                        xb[0] = zb[1]*yb[2] - zb[2]*yb[1];
                        xb[1] = zb[2]*yb[0] - zb[0]*yb[2];
                        xb[2] = zb[0]*yb[1] - zb[1]*yb[0];
                        /* v_mag = R0 · um = (xb·um, yb·um, zb·um) */
                        s_eskf.v_mag[0] = xb[0]*um[0] + xb[1]*um[1] + xb[2]*um[2];
                        s_eskf.v_mag[1] = yb[0]*um[0] + yb[1]*um[1] + yb[2]*um[2];
                        s_eskf.v_mag[2] = zb[0]*um[0] + zb[1]*um[1] + zb[2]*um[2];
                        {
                            float nn = sqrtf(s_eskf.v_mag[0]*s_eskf.v_mag[0]
                                           + s_eskf.v_mag[1]*s_eskf.v_mag[1]
                                           + s_eskf.v_mag[2]*s_eskf.v_mag[2]);
                            if (nn > 1e-6f) {
                                for (i = 0; i < 3u; i++) s_eskf.v_mag[i] /= nn;
                                s_eskf.mag_ref_ok = 1u;   /* 参考已由实测静止窗构造完成 */
                            }
                        }
                    }
                }
                s_vmag_ready = 1u;
            }
        }
        nav_to_body(s_eskf.q, s_eskf.v_mag, h);
        c = z[0]*h[0] + z[1]*h[1] + z[2]*h[2];
        if (c > 1.0f) c = 1.0f; if (c < -1.0f) c = -1.0f;
        s_eskf.mag_resid_deg = acosf(c) * ESKF_RAD2DEG;
        /* ★★ 磁观测 = 纯净路径（2026-10-14 用户要求）：
         *   无门控、无 IGG 降权、无单帧限幅、无"地磁不可用"兜底。
         *   use_igg = 0 同时关掉 IGG 与单帧限幅（见 eskf_update 内说明）。
         *   v_mag 由累加窗从【实测】构造；窗满前 v_mag 尚未定义（初值 [0,1,0]
         *   与真实场差 56.8°），此时施加观测没有意义且会把姿态拉歪
         *   （实测：无此保护时前 2 s 姿态差 17.5°、后段仍差 10°）。
         *   故仅在 v_mag 构造完成前【不施加观测】—— 这是"观测尚无定义"，
         *   不是门控/降权/阈值/限幅。构造完成后即全程无任何限制地施加。 */
        if (s_vmag_ready != 0u) {
            float rp = ESKF_MAG_PERP_DEG * ESKF_DEG2RAD;
            float rq = ESKF_MAG_PAR_DEG  * ESKF_DEG2RAD;
            s_eskf.mag_corr_deg = eskf_update(s_eskf.v_mag, z, rp*rp, rq*rq, 0u) * ESKF_RAD2DEG;
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
#ifdef PC_PROBE
        /* 门诊断探针：an / avn / resid(°) 逐帧上报，用于定位门为何不开 */
        g_pc_gate_an = an;
        g_pc_gate_avn = avn;
        g_pc_gate_res = (float)(resid * ESKF_RAD2DEG);
        /* 磁诊断：链内真实使用的 mag_u 与算出的 mag_resid */
        g_pc_mux = mag_u[0]; g_pc_muy = mag_u[1]; g_pc_muz = mag_u[2];
        g_pc_mres = s_eskf.mag_resid_deg;
        g_pc_vmx = s_eskf.v_mag[0]; g_pc_vmy = s_eskf.v_mag[1]; g_pc_vmz = s_eskf.v_mag[2];
        g_pc_mrok = (float)s_eskf.mag_ref_ok;
#endif

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
        if ((fabsf(an - 1.0f) < ESKF_ACC_MAGLIM_G) && (avn < ESKF_GRAV_WLIM_DPS)) {            float rs = ESKF_ACC_SIG_DEG * ESKF_DEG2RAD;
            float applied = eskf_update(s_eskf.v_up, z, rs*rs, rs*rs, 1u);
#ifdef PC_PROBE
            g_pc_applied = applied;
#endif
            s_eskf.grav_open = (applied >= 0.0f) ? 1u : 0u;   /* NIS 拒绝返回 -1 */
        }
        (void)resid;   /* resid 仍用于下方 acc_resid_deg 诊断上报 */

        /* ★★ 无重力期计时（2026-10-14 重建，供下一帧预测步的 P 膨胀使用）
         *   门开 ⟹ 归零（重力在场，P 由观测收敛）
         *   门关 ⟹ 累加 dt_s（重力缺席，P 按 σ_tilt = SIG_NOGRAV·t_nograv 增长） */
        /* ★★ 无重力期时长 t_nograv（供下一帧预测步的倾斜方差注入使用）
         *   门开 ⟹ 归零（重力在场，P 由观测按比例回落）
         *   门关 ⟹ 累加 dt_s
         * 注意：预测步在本判定之前执行，故注入用的是【上一帧】的 t_nograv。 */
        if (s_eskf.grav_open != 0u) {
            s_eskf.nograv_s = 0.0f;
        } else if ((dt_s == dt_s) && (dt_s > 0.0f) && (dt_s < 0.05f)) {
            s_eskf.nograv_s += dt_s;
            if (s_eskf.nograv_s > 1.0e4f) s_eskf.nograv_s = 1.0e4f;
        }
        s_eskf.grav_last = s_eskf.grav_open;
#ifdef PC_PROBE
        g_pc_nograv = s_eskf.nograv_s;
#endif
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



