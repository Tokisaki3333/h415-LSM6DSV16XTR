#include "v5f_proc.h"

/* =====================================================================
 * 处理函数 1：陀螺三轴零偏校正
 *
 *   out[i] = (gyro_lsb[i] - bias[i]) / g_v5f_gyro_lsb_per_dps[i]   （°/s，逐轴标度）
 *
 *   启动初值：直接取标定常量（GB_BIAS0_*，见下），上电即在零点附近，
 *              不需要运行时取均值；否则启动偏置（实测 z 轴 0.7 dps）会被
 *              动静判定当成运动，导致牵引被永久冻结。
 *   牵引    ：静止时一阶牵引 bias += (raw - bias) · dt/τ，分两段：
 *              启动段 τ_boot = 1 s —— 上电初值残差 e0（标定常量与本机实际零点之差，
 *                实测本机 0.026 dps、会话间散布 ≤0.037 dps、换温度可达 0.1~0.3 dps）
 *                按 e^(-t/τ) 衰减；τ 越小残余收敛越快，且牵引噪声
 *                σ_b = σ_raw·sqrt(Δt/2τ) 在 τ=1 s 时仅 9.6e-4 dps，相对 e0 可忽略。
 *               工作段 τ_work = 12 s —— **有效追踪时长**（静止时间减去被回退的粒度）
 *               满 3τ_boot = 3 s 后切入并长期保持：
 *                残余再按 τ_work 平滑，对偶发误判静止的抵抗力比 τ_boot 强 12 倍。
 *              两段在 3τ_boot 处按"已累计静止时长"切换，不用上电墙上时间
 *              （一动就冻结、时长不涨，自然不会误切）。
 *   收敛判定：**有效追踪时长** ≥ 3τ_boot = 3 s → s_boot_done 锁存 = h->imu.bias_ok。
 *              ★ 回退只扣减被回退的粒度数，**不清零**：否则开机头 3 s 内只要动过
 *                一次就永远锁不上（手持开机常见），零偏牵引会永远停在 τ_boot。
 *              锁存不因回退清除：回退恢复的是运动前（通常已收敛）的快照，
 *              且此时设备多半正在转，重新关掉收敛标志没有意义。
 *              实际残余 = e0·e^(-3) ≈ 5%·e0（本机 e0=0.026 dps → 1.3e-3 dps ≈ 4.7°/h，
 *              与实测 48 s 处 ≤1.9e-3 dps 的量级一致）。
 *              处理函数 3（四元数）以 bias_ok 为门：收敛前不出姿态。
 *              → 静止下校正输出趋于 0；运动时冻结（不牵引）。
 *   回溯记录：内部每 20 ms 存一份 bias 快照，环形 64 份 = 1.28 s；
 *              外部（处理函数 2）告知"静止→运动"边沿时回退若干粒度，
 *              即把 bias 恢复到该粒度之前的快照值，从该值继续牵引。
 *              回退粒度数由门控给出，并钳位到"本段静止已累计的粒度数"，
 *              绝不回退到上一段运动期间的快照。
 *
 *   全部状态为文件静态，仅被 DMA1_Channel2_IRQHandler 主任务区单线程调用，无需加锁。
 *   时间基准统一用 h->imu.fresh.drdy_tick（10 ns 计数），与调用次数无关。
 * ===================================================================== */

/* 本函数的工况可调参数（tau_boot / tau_work / 快照粒度）已集中到 v5f_tune.h 的 B 组 */

/* 零偏初值：由 151 s 静止记录 serial_runtime_20260912_215604 标定，单位 LSB。
 *   bias 通道中值：x=1.0334  y=0.8494  z=12.0910 LSB
 *                （= 0.0630 / 0.0518 / 0.7373 dps，×16.4 换算）
 *   bias 通道均值：x=1.0277  y=0.8499  z=12.0916 LSB（与中值差 < 0.006 LSB，取中值）*/
/* 别名到 v5f_proc.h 的器件标定组，避免同一常量两处各写一份（门B 也要用）。 */
#define GB_BIAS0_LSB_X      V5F_GYRO_BIAS_LSB_X
#define GB_BIAS0_LSB_Y      V5F_GYRO_BIAS_LSB_Y
#define GB_BIAS0_LSB_Z      V5F_GYRO_BIAS_LSB_Z

/* 逐轴标度表（标定出处与算式见 v5f_proc.h）：dps = LSB / g_v5f_gyro_lsb_per_dps[轴] */
const float g_v5f_gyro_lsb_per_dps[3] = {
    V5F_GYRO_LSB_PER_DPS_X,
    V5F_GYRO_LSB_PER_DPS_Y,
    V5F_GYRO_LSB_PER_DPS_Z
};

static float    s_bias[3] = { GB_BIAS0_LSB_X, GB_BIAS0_LSB_Y, GB_BIAS0_LSB_Z };
static float    s_hist[3][GB_HIST_DEPTH];         /* 每个 20 ms 粒度一份 bias 快照 */
static uint32_t s_head;                           /* s_hist 中最近一份快照的下标 */
static uint64_t s_last_tick;                      /* 上一帧 DRDY 时刻 */
static uint64_t s_next_snap;                      /* 下一个快照边界时刻 */
static uint64_t s_eff_ticks;                      /* ★ **有效追踪时长**：静止时累加 dt，
                                                   * 回退时按被回退的粒度数**扣减**（不是清零）。
                                                   * 启动段 tau_boot 的收敛判据与 s_boot_done 锁存都用它。
                                                   * 为什么不清零：见 gb_rollback 里的详细说明。 */
static uint8_t  s_boot_done;                      /* 启动牵引完成（累计静止 ≥ 3τ_boot）后锁存
                                                   * = h->imu.bias_ok；不回退清除 */
static uint8_t  s_prev_static;                    /* 上一帧的门控状态（边沿判定用） */
static uint16_t s_static_gran;                    /* 本段静止已累计的 20 ms 粒度数 */
static uint16_t s_evidence_gran;                  /* 累积静止证据（粒度数，回退时扣减）
                                                   * 供处理函数 2 查判静阈值表 */
#define GB_EVIDENCE_MAX  ((uint16_t)(V5F_DET_THR_NBUCK << V5F_DET_THR_SHIFT))  /* 16*128 = 2048 粒 */

static uint8_t  s_hist_init;                      /* 快照历史是否已预置 */

/* 快照历史预置：s_hist 是零初始化的，若不预置，开机 GB_HIST_DEPTH 格（1.28 s）内
 * 一旦发生回退，gb_rollback 可能把 idx 落到尚未写入的槽位，**把零偏退成全 0** ——
 * 表现为此后 gyro_dps 输出原始角速度（零偏不再扣除），看起来像"开机失败"。
 * 预置成"离线标定初值"，语义上等于"开机前一直就是这个值"。 */
static void gb_hist_init(void)
{
    uint32_t i, j;

    for (i = 0u; i < 3u; i++)
        for (j = 0u; j < GB_HIST_DEPTH; j++)
            s_hist[i][j] = s_bias[i];
    s_hist_init = 1u;
}

/* 写一份快照（在本帧牵引之后调用，记录该粒度边界的 bias） */
static void gb_snapshot(uint8_t is_static)
{
    uint32_t i;

    s_head = (s_head + 1u) % GB_HIST_DEPTH;
    for (i = 0u; i < 3u; i++) s_hist[i][s_head] = s_bias[i];

    if (is_static != 0u) {                        /* 静止段粒度数：回退钳位用 */
        if (s_static_gran < GB_HIST_DEPTH) s_static_gran++;
        if (s_evidence_gran < GB_EVIDENCE_MAX) s_evidence_gran++;   /* 证据只增不减 */
    } else {
        s_static_gran = 0u;                      /* 证据不因运动清零：它是 bias 的累积依据 */
    }
}

/* 回退 granules 个 20 ms 粒度：bias 取该粒度的快照，并覆盖当前槽使历史与现值一致 */
static void gb_rollback(uint32_t granules)
{
    uint32_t i, back, idx;

    back = (granules > GB_HIST_DEPTH) ? GB_HIST_DEPTH : granules;      /* 饱和到最深可用粒度 */
    idx  = (s_head + GB_HIST_DEPTH - back) % GB_HIST_DEPTH;
    for (i = 0u; i < 3u; i++) {
        s_bias[i]         = s_hist[i][idx];
        s_hist[i][s_head] = s_bias[i];
    }
    /* ★ 有效追踪时长只扣掉**被回退的那部分**，不清零。
     * 语义：回退把 bias 恢复成 back 个粒度之前的快照，等于丢掉这 back 个
     *   粒度的牵引成果，所以有效时长相应减 back 个粒度；但之前已经
     *   积累的收敛成果保留。
     * ★ 曾经写成"清零"，后果是：只要开机头 3 s 内动过一次
     *   （手持开机时静止<->运动反复），s_eff_ticks 就永远回不到 3 s，
     *   s_boot_done 永不锁存 -> ① 零偏牵引永远停在 τ_boot=1 s（快牵引，
     *   飞行中一次误判静止就把零偏拽偏）；② 倾斜增益永远停在
     *   V5F_ATT_TILT_K_BOOT。换成扣减后，反复小幅晃动只是把进度往回推
     *   一点，总有效时长仍然单调增长，最终一定锁存。 */
    {
        uint64_t loss = (uint64_t)back * GB_SNAP_TICKS;
        s_eff_ticks = (s_eff_ticks > loss) ? (s_eff_ticks - loss) : 0u;
    }
    /* 回退等于把这 back 个粒度的牵引成果丢掉：证据量同步扣减，
     * 于是处理函数 2 的阈值会相应回升（证据不足 -> 阈值抬高）。 */
    s_evidence_gran = (s_evidence_gran > back) ? (uint16_t)(s_evidence_gran - back) : 0u;
}

uint8_t v5f_proc_gyro_bias(volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)
{
    uint64_t tick, dt;
    uint32_t i;
    uint8_t  is_static, rb;
    float    dv[3], bv[3];                   /* 过对称交叉前的逐轴 dps（校正值 / 零偏值） */

    h->imu.corr_flags = 0u;
    h->imu.corr_valid = 0u;                  /* 先置无效，正常路径最后置 1 */

    tick = h->imu.fresh.drdy_tick;
    if (s_next_snap == 0u) {                 /* 首帧：对齐 20 ms 边界与 dt 基准 */
        s_last_tick = tick;
        s_next_snap = tick + GB_SNAP_TICKS;
    }

    if (s_hist_init == 0u) gb_hist_init();

    if (tick < s_last_tick) return V5F_PROC_ERR_TICK;   /* 时间戳回退：本帧不动内部状态 */

    dt        = tick - s_last_tick;
    /* 单帧 dt 钳位：时间戳大跳时 alpha=dt/tau 会 >1（零偏跳变），
     * 且一次大 dt 就可能把有效追踪时长瞬间灌满而假锁存 bias_ok。
     * 钳到 200 ms：真正丢帧时仍然计入真实静止时间，但不会被荒谬值灌满。 */
    if (dt > V5F_GB_DT_MAX_TICKS) dt = V5F_GB_DT_MAX_TICKS;
    is_static = (gate->is_static != 0u) ? 1u : 0u;
    rb        = 0u;

    /* 静止→运动边沿：按门控给的粒度数回退，并钳位到本段静止已有的粒度数
     * （否则会回退到上一段运动期间的快照） */
    if (s_prev_static != 0u && is_static == 0u && s_eff_ticks > 0u) {
        uint32_t back = gate->rollback_granules;
        if (back > s_static_gran) back = s_static_gran;
        if (back != 0u) {
            gb_rollback(back);
            rb = 1u;
        }
    }

    /* 静止：一阶牵引 bias += (raw - bias) · dt/τ；运动：冻结（于是只有静止时间计入）。
     * τ 分两段：启动段 τ_boot（快收敛），**有效追踪时长**满 3τ_boot 后切工作段
     * τ_work 并锁存 s_boot_done（= bias_ok）。
     * ★ "有效"的含义：只累加静止时间，且回退时按被回退的粒度扣减（不清零）。 */
    if (is_static != 0u) {
        float alpha = (float)dt * ((s_boot_done != 0u) ? GB_ALPHA_WORK_TICK
                                                       : GB_ALPHA_BOOT_TICK);
        for (i = 0u; i < 3u; i++) {
            s_bias[i] += ((float)h->imu.gyro_lsb[i] - s_bias[i]) * alpha;
        }
        s_eff_ticks += dt;
        if (s_eff_ticks >= GB_BOOT_TICKS) s_boot_done = 1u;   /* 有效追踪 >= 3τ_boot */
    }

    /* 20 ms 粒度快照：帧恒定 125 us 到来 => 每 160 帧正好跨过一次边界 */
    if (tick >= s_next_snap) {
        gb_snapshot(is_static);
        s_next_snap += GB_SNAP_TICKS;
    }

    /* 输出：逐轴标度 -> 对称交叉 -> 校正结果 + 零偏估计 + 有效性标志。
     * 物理模型 dps_true = K · (LSB / S)，K 对角=1、对称（见 v5f_proc.h 的 V5F_GYRO_K*）。
     * K 必须放在标度之后：交叉灵敏是"通道 i 混进了通道 j 的角速度"，混的是已换算成
     * dps 的量。零偏那一路同样过 K，这样"校正后 = 原始 - 零偏"在 K 之后依然成立。 */
    for (i = 0u; i < 3u; i++) {
        dv[i] = ((float)h->imu.gyro_lsb[i] - s_bias[i]) / g_v5f_gyro_lsb_per_dps[i];
        bv[i] = s_bias[i] / g_v5f_gyro_lsb_per_dps[i];
    }
    h->imu.gyro_dps[0]      = dv[0] + V5F_GYRO_KXY * dv[1] + V5F_GYRO_KXZ * dv[2];
    h->imu.gyro_dps[1]      = dv[1] + V5F_GYRO_KXY * dv[0] + V5F_GYRO_KYZ * dv[2];
    h->imu.gyro_dps[2]      = dv[2] + V5F_GYRO_KXZ * dv[0] + V5F_GYRO_KYZ * dv[1];
    h->imu.gyro_bias_dps[0] = bv[0] + V5F_GYRO_KXY * bv[1] + V5F_GYRO_KXZ * bv[2];
    h->imu.gyro_bias_dps[1] = bv[1] + V5F_GYRO_KXY * bv[0] + V5F_GYRO_KYZ * bv[2];
    h->imu.gyro_bias_dps[2] = bv[2] + V5F_GYRO_KXZ * bv[0] + V5F_GYRO_KYZ * bv[1];
    h->imu.corr_flags = (rb != 0u) ? V5F_GYRO_CORR_FLAG_ROLLBACK : 0u;
    h->imu.bias_ok    = s_boot_done;
    h->imu.corr_valid = 1u;
    h->imu.bias_evidence_gran = s_evidence_gran;   /* 供处理函数 2 查阈值表 */

    s_last_tick   = tick;
    s_prev_static = is_static;
    return V5F_PROC_OK;
}
