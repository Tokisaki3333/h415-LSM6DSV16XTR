#include "v5f_proc.h"
#include <math.h>   /* sqrtf：-Ofast 下编译成一条 fsqrt.s，不引入 libm 调用 */

/* =====================================================================
 * 处理函数 3：姿态四元数（陀螺积分）
 *
 *   输入   h->imu.gyro_dps[] —— 处理函数 1 的补偿输出
 *           （零偏牵引值 + 逐轴标度系数已载入，此处不再做任何定标/零偏处理）
 *   递推   q ← q × dq，右乘即"机体轴增量"（陀螺输出的就是机体角速度）
 *          dq = { 1, 0.5·wx·dt, 0.5·wy·dt, 0.5·wz·dt }      半角一阶近似
 *          ★ 这里的 w 必须是 rad/s：陀螺给的是 °/s，先乘 DEG2RAD 再进四元数。
 *            漏这一步不会报错，只会把姿态角整体放大 180/pi = 57.3 倍。
 *   近似误差  精确式 dq = { cos(θ/2), sin(θ/2)·(ω/|ω|) }，一阶式的相对角误差 ≈ θ^2/24；
 *          本机 dt = 124.55 us，满量程 |ω| = 2000 dps = 34.9 rad/s
 *          → θ ≤ 4.35e-3 rad → 相对误差 ≤ 8e-7（整圈 360° 累积 < 0.0004°，可忽略）。
 *          单帧 dt 越大误差按 θ^2 增长，故本函数依赖"每帧都被调用"这一前提。
 *   归一化  每帧一次：s = 1/sqrt(|q|^2)，q *= s。单精度下不算归一化，模长每帧
 *          漂移约 1e-7，几秒内就积累到不可忽略；V5F 有单精度 FPU，sqrt+div
 *          各一条指令，代价可忽略，故直接精确归一化，不做"仅在 |q|≈1 成立"的
 *          牛顿近似（那会在异常大 dt 下给出错误的负增益）。
 *   时间基准  dt 取自 h->imu.fresh.drdy_tick（10 ns 计数），与调用次数无关；
 *          上游时间戳回退帧（corr_valid = 0）本函数不更新姿态。
 *   参考系    导航系**由重力初始化成竖直**（z = 上），不取上电姿态。
 *          上电四元数取单位阵 = 把导航系定在"上电那一瞬的机体姿态"上，参考 z 与竖直的
 *          夹角完全取决于上电怎么摆放（实测各记录 1.3~178.8 deg 都有），于是去重力不得
 *          不额外带一个 up_ref 向量、姿态参考系也没有物理意义。
 *          改为在使能那一帧用重力定：u = accel_g/|accel_g| 是机体系里测到的"上"，
 *          取**最小旋转** q 使 R(q)·u = (0,0,1)。最小旋转不引入偏航（重力定不出偏航，
 *          只能定倾角），只把 z 轴摆到竖直。于是 up_ref 恒为 (0,0,1)，去重力退化成
 *          "减掉单位向量 (0,0,1)"。
 *   使能门    **只等一个可信的重力方向**（acc_valid + 本帧判静止
 *          + |accel_g| 在 1 g ±2% 内）才定参考系并开始积分，否则继续冻结、下一帧再试
 *          —— 水平机动时 |accel_g| 仍停在 1 g 附近（只有二阶敏感），必须靠"判静止"
 *          才能保证这个方向没被水平加速度污染。
 *          ★ 不再等 h->imu.bias_ok（零偏收敛）：把两者绑在一起时，上电后前 3 s
 *            （累计静止满 3τ_boot）q 恒为单位四元数 —— 倾斜上电那 3 s 根本没转到
 *            重力系，而且 up_ref_ok = 0 会让处理函数 4/5 整段冻结。零偏未收敛造成的
 *            角度误差上界只有 e0*窗口时长（最坏 0.035 dps × 3 s = 0.03 deg）。
 *   全部状态为文件静态，仅被 DMA1_Channel2_IRQHandler 主任务区单线程调用，无需加锁。
 * ===================================================================== */

#define ATT_DT_SCALE   1e-8f           /* 10 ns 计数 → s */
#define ATT_DEG2RAD    0.0174532925f   /* ° → rad（π/180） */

/* 逐轴加速度敏感度 [dps/g]，来源与风险见 v5f_proc.h 的"陀螺加速度敏感度补偿" */
const float g_v5f_gyro_gsens[3] = {
    V5F_GYRO_GSENS_X,
    V5F_GYRO_GSENS_Y,
    V5F_GYRO_GSENS_Z,
};

static float    s_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };   /* 未定参考系前的占位值 */
static uint64_t s_last_tick;
static uint8_t  s_started;

/* ---- 用重力把导航系定成竖直（z = 上）----
 * 求最小旋转 q 使 R(q)·u = (0,0,1)，u = 机体系里测到的"上"（比力方向，单位向量）。
 * 最小旋转 = 绕 (u x e_z) 转 (u 与 e_z 的夹角)，不引入任何偏航 —— 重力只能定倾角，
 * 偏航保持为"上电时机体 x 轴的水平投影方向"，这是物理上唯一能定的约定。
 * 解：axis = u x e_z = (u_y, -u_x, 0)，q = [1 + u·e_z, axis] 归一化。
 * 退化：u 朝下（u_z -> -1）时 1 + u_z -> 0，改取绕机体 x 轴转 180 deg。
 * 自检（两例都手算验证过 R(q)·u = (0,0,1)）：
 *   u = (1,0,0) -> q = (0.7071, 0, -0.7071, 0)
 *   u = (0,1,0) -> q = (0.7071, 0.7071, 0, 0) */
static void att_init_from_gravity(float ux, float uy, float uz)
{
    float w, nx, ny, nz, s;

    if (uz > -0.999999f) {
        w  = 1.0f + uz;
        nx = uy;
        ny = -ux;
        nz = 0.0f;
    } else {
        w  = 0.0f; nx = 1.0f; ny = 0.0f; nz = 0.0f;
    }
    s = 1.0f / sqrtf(w*w + nx*nx + ny*ny + nz*nz);
    s_q[0] = w * s;
    s_q[1] = nx * s;
    s_q[2] = ny * s;
    s_q[3] = nz * s;
}

uint8_t v5f_proc_attitude(volatile v5f_hold_t *h,
                            const volatile v5f_proc_gate_t *gate)
{
    uint64_t tick, dt;
    float    half, dw, dx, dy, dz, w, x, y, z, nw, nx, ny, nz, s;
    float    w0, w1, w2, qw, qx, qy, qz, vx, vy, vz, gx, gy, gz, ax, ay, az;
    float    am, a0, a1, a2, e0, e1, e2, mg, mg2, kt;
    uint8_t  i;

    for (i = 0u; i < 4u; i++) h->att.q[i] = s_q[i];   /* 无论本帧是否更新都发布当前姿态 */
    h->att.valid = 0u;

    if (h->imu.corr_valid == 0u) return V5F_PROC_OK;  /* 上游本帧无效：保持上一帧姿态 */

    /* ---- 参考系初始化：拿到可信重力方向就**立刻**定，不等 bias_ok ----
     * 以前这里是一句 `if (h->imu.bias_ok == 0u) return;`，把"定参考系"和"零偏收敛"
     * 绑在了一起。实测后果（20260914_060222_791，倾斜 19 deg 上电）：
     *   前 21651 帧（t = 0 ~ 2.699 s，等累计静止满 3*tau_boot = 3 s）q 恒为单位四元数
     *   —— 导航系就是机体系，**倾斜上电那 3 s 根本没转到重力系**；
     *   而且 up_ref_ok = 0 会让处理函数 4/5 整段冻结（a_lin = 0、不牵引、v = 0）。
     * 定参考系只需要"可信的重力方向"，与零偏是否收敛无关，所以拆开：
     *   可信 = acc_valid + stat.valid（判静滑窗已满）+ 本帧判静止 + |accel_g| 在 1 g ±2%。
     *   stat.valid 必须带上：滑窗未满时判静器把 is_static **强制置 1**（窗口还没数据），
     *   所以只查 is_static 会让"开机瞬间正在转动"也满足条件，拿被线加速度污染的方向
     *   定参考系。带上它只多花 16 ms（V5F_DET_W = 128 帧）。
     *   水平机动时 |accel_g| 仍停在 1 g 附近（只有二阶敏感），所以必须靠"判静止"
     *   保证这个方向没被线加速度污染 —— 定错了会把误差永久烙进导航系。
     * 不再等 bias_ok 才积分的理由：零偏未收敛期间的角度误差上界 = e0*窗口时长；e0 取
     *   最坏 0.035 dps（实测 060222 的常量偏差 125 deg/h = 0.0348 dps）时，3 s 也只有
     *   0.03 deg，远小于倾角环自身的稳态误差。bias_ok 现在只作为"零偏已收敛"的观察
     *   标志上报（flags bit0），不再用作姿态使能门。 */
    if (h->att.up_ref_ok == 0u) {
        if (h->imu.acc_valid != 0u && h->stat.valid != 0u && h->stat.is_static != 0u) {
            mg2 = h->imu.accel_g[0]*h->imu.accel_g[0]
                + h->imu.accel_g[1]*h->imu.accel_g[1]
                + h->imu.accel_g[2]*h->imu.accel_g[2];
            if (mg2 >= 0.9604f && mg2 <= 1.0404f) {      /* 0.98^2 / 1.02^2 */
                mg = sqrtf(mg2);
                att_init_from_gravity(h->imu.accel_g[0] / mg,
                                      h->imu.accel_g[1] / mg,
                                      h->imu.accel_g[2] / mg);

                /* 导航系 z 就是竖直 => 去重力 = 减掉单位向量 (0,0,1) */
                h->att.up_ref[0] = 0.0f;
                h->att.up_ref[1] = 0.0f;
                h->att.up_ref[2] = 1.0f;
                h->att.up_ref_ok = 1u;
                for (i = 0u; i < 4u; i++) h->att.q[i] = s_q[i];
            }
        }
        return V5F_PROC_OK;             /* 没拿到可信重力方向前，姿态保持不动 */
    }

    tick = h->imu.fresh.drdy_tick;
    if (s_started == 0u) {              /* 定好参考系后的第一帧：只建立 dt 基准 */
        s_started   = 1u;
        s_last_tick = tick;
        h->att.valid = 1u;
        return V5F_PROC_OK;
    }

    dt          = tick - s_last_tick;
    s_last_tick = tick;

    /* ---- 陀螺加速度敏感度（g 敏感度）补偿 ----
     *   陀螺输出 = 真实角速度 + Sg * a，a 是该时刻**全部非重力加速度**（单位 g）。
     *   a_lin = accel_g - R(q)^T * up_ref
     *     accel_g  —— 实测比力（处理函数 4 输出，单位 g）
     *     up_ref   —— 世界竖直在**导航系**里的表示，恒为 (0,0,1)：导航系已由上面的
     *                 att_init_from_gravity 用重力初始化成竖直。
     *   姿态用**本帧更新前**的 s_q（即上一帧结果），单帧滞后可忽略。
     *   闸门：acc_valid = 0 或 |a_lin| >= ALIM 时不补 —— 削顶帧的 a_lin 本身是错的，
     *         补进去等于把加速度计的饱和误差灌进陀螺；不补只是回到补偿前的行为。 */
    w0 = h->imu.gyro_dps[0];
    w1 = h->imu.gyro_dps[1];
    w2 = h->imu.gyro_dps[2];
    if (h->imu.acc_valid != 0u && h->att.up_ref_ok != 0u) {
        qw = s_q[0]; qx = s_q[1]; qy = s_q[2]; qz = s_q[3];
        vx = h->att.up_ref[0]; vy = h->att.up_ref[1]; vz = h->att.up_ref[2];
        gx = (1.0f - 2.0f*(qy*qy + qz*qz))*vx + 2.0f*(qx*qy + qw*qz)*vy + 2.0f*(qx*qz - qw*qy)*vz;
        gy = 2.0f*(qx*qy - qw*qz)*vx + (1.0f - 2.0f*(qx*qx + qz*qz))*vy + 2.0f*(qy*qz + qw*qx)*vz;
        gz = 2.0f*(qx*qz + qw*qy)*vx + 2.0f*(qy*qz - qw*qx)*vy + (1.0f - 2.0f*(qx*qx + qy*qy))*vz;
        ax = h->imu.accel_g[0] - gx;
        ay = h->imu.accel_g[1] - gy;
        az = h->imu.accel_g[2] - gz;
        if (ax*ax + ay*ay + az*az < V5F_GYRO_GSENS_ALIM * V5F_GYRO_GSENS_ALIM) {
            w0 -= g_v5f_gyro_gsens[0] * ax;
            w1 -= g_v5f_gyro_gsens[1] * ay;
            w2 -= g_v5f_gyro_gsens[2] * az;
        }

        /* ---- 重力方向持续测量（Mahony 式倾斜修正）----
         *   a_hat = accel_g / |accel_g|          加速度计测到的"上"（机体系）
         *   g_hat = (gx,gy,gz)                   = R(q)^T*up_ref，已是单位向量
         *   e = a_hat x g_hat,   omega_tilt = +K * e
         * 符号由数值验证：K 取正号时 2/17/60 deg 初倾角全部收敛到约 0.001 deg，
         *   取负号则发散到 180 deg。
         * ★★ 门控**不能用 gate->acc_traction**（这是交叉门控死锁，已实测）：
         *   那个门的判据是 |w| = |用离线常量去零偏后的比力 - 重力方向|，姿态一歪 w 就大，
         *   门就以为"机动中"而冻结；而倾斜环恰恰是唯一能把姿态转回来的东西，于是：
         *     姿态歪 -> |w| 大 -> acc_traction = 0 -> 倾斜环冻结 -> 姿态永远转不回来。
         *   实测 20260914_061955_820 末段：is_static 100%、level 0.021 dps（死静），
         *   而姿态偏 6.725 deg、|w| = 107 mg > OFF(100 mg)、acc_traction = 0%，
         *   |a_lin| 卡在 117 mg，加速度零偏牵引再也起不来。
         *   改用**本环专用的外部门控字段 gate->att_tilt**：由 proc_acc_gate.c（处理
         *   函数 5，下游）独立算出，判据 = |off| 在 1 g ±5%（off = 用离线标定常量换算
         *   的比力）且陀螺判静。这里**既不借用别的环的门、也不在环内现算** ——
         *   铁律见 v5f_proc.h 的门控结构说明。
         *   |off| 对"姿态歪"完全免疫（歪只改方向、不改模长），所以不会被姿态误差关掉；
         *   水平线加速度那一路（|off|-1g 只有二阶敏感）由陀螺判静挡。
         * ★ 两段 τ：启动段 tau_boot = 1 s（与陀螺零偏牵引启动段同值），bias_ok 置位后
         *   切工作段 5 s（见表）。启动段要快 —— 导航系刚由重力定出来，此时残余零偏
         *   最大，要在几秒内把四元数硬拉对齐重力系；之后放松，免得长期跟着加速度计
         *   的零偏走。本环是纯 P 环，对残余零偏有稳态倾角 tilt_eq = omega_res*tau_eff，
         *   那个倾角会直接把重力漏进 a_lin，所以工作段也不能太慢。 */
        if (gate->att_tilt != 0u) {
            kt = (h->imu.bias_ok != 0u) ? V5F_ATT_TILT_K : V5F_ATT_TILT_K_BOOT;
            am = sqrtf(h->imu.accel_g[0]*h->imu.accel_g[0] +
                       h->imu.accel_g[1]*h->imu.accel_g[1] +
                       h->imu.accel_g[2]*h->imu.accel_g[2]);
            if (am > 0.5f) {          /* 纯数值保护：a_hat 要除以它，不是功能门控 */
                a0 = h->imu.accel_g[0] / am;
                a1 = h->imu.accel_g[1] / am;
                a2 = h->imu.accel_g[2] / am;
                e0 = a1 * gz - a2 * gy;              /* (a_hat x g_hat)_x */
                e1 = a2 * gx - a0 * gz;
                e2 = a0 * gy - a1 * gx;
                w0 += kt * e0;               /* 速率，单位 dps；dt 由 half 统一乘 */
                w1 += kt * e1;
                w2 += kt * e2;
            }
        }
    }

    /* 本帧机体增量（半角）：θ/2 = 0.5 · ω[rad/s] · dt[s]，gyro_dps 是 °/s 故先转 rad */
    half = 0.5f * (float)dt * ATT_DT_SCALE * ATT_DEG2RAD;
    dw   = 1.0f;
    dx   = w0 * half;
    dy   = w1 * half;
    dz   = w2 * half;

    /* q ← q × dq（Hamilton 积，q = {w,x,y,z}） */
    w = s_q[0]; x = s_q[1]; y = s_q[2]; z = s_q[3];
    nw = w * dw - x * dx - y * dy - z * dz;
    nx = w * dx + x * dw + y * dz - z * dy;
    ny = w * dy - x * dz + y * dw + z * dx;
    nz = w * dz + x * dy - y * dx + z * dw;

    /* 归一化：s = 1/|q| */
    s = 1.0f / sqrtf(nw * nw + nx * nx + ny * ny + nz * nz);
    s_q[0] = nw * s;
    s_q[1] = nx * s;
    s_q[2] = ny * s;
    s_q[3] = nz * s;

    for (i = 0u; i < 4u; i++) h->att.q[i] = s_q[i];
    h->att.valid = 1u;
    return V5F_PROC_OK;
}
