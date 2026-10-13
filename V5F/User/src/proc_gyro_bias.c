/* 处理函数 1：陀螺三轴零偏校正
 *
 *   out[i] = gyro_lsb[i] - bias[i]        （counts 域，dps 换算放调用端统一做）
 *
 *   启动初值：直接取实测常量（GB_BIAS0_LSB_*），上电即在零点附近，
 *              不需要运行时取均值；否则启动偏置会被动静判定当成运动，
 *              导致牵引被永久冻结。
 *   牵引    ：静止时一阶牵引 bias += (raw - bias) * dt/tau，分两段：
 *              启动段 tau_boot = 1 s —— 上电初值残差按 e^(-t/tau) 衰减
 *              工作段 tau_work = 12 s —— 有效追踪时长满 3*tau_boot = 3 s 后切入
 *              两段按"已累计静止时长"切换，不用上电墙上时间。
 *   收敛判定：有效追踪时长 >= 3*tau_boot -> s_boot_done 锁存 = g_proc.bias_ok
 *              ★ 回退只扣减被回退的粒度数，不清零：否则开机头 3 s 内只要动过
 *                一次就永远锁不上，零偏牵引会永远停在 tau_boot。
 *   回溯记录：内部每 20 ms 存一份 bias 快照，环形 64 份 = 1.28 s；
 *              静止->运动边沿时按门控给的粒度数回退，并钳位到"本段静止已累计
 *              的粒度数"，绝不回退到上一段运动期间的快照。
 *
 *   全部状态为文件静态，仅被 V5F 处理主循环单线程调用，无需加锁。
 *   时间基准统一用 10 ns 计数，与调用次数无关。 */
#include "proc.h"
#include "proc_tune.h"

#define GB_EVIDENCE_MAX  ((uint16_t)(DET_THR_NBUCK << DET_THR_SHIFT))

static float    s_bias[3] = { GB_BIAS0_LSB_X, GB_BIAS0_LSB_Y, GB_BIAS0_LSB_Z };
static float    s_hist[3][GB_HIST_DEPTH];
static uint32_t s_head;
static uint64_t s_last_tick;
static uint64_t s_next_snap;
static uint64_t s_eff_ticks;        /* 有效追踪时长（10 ns），回退时按粒度扣减 */
static uint8_t  s_boot_done;
static uint8_t  s_prev_static;
static uint16_t s_static_gran;
static uint8_t  s_hist_init;

static void gb_hist_init(void)
{
    uint32_t i, j;

    for (i = 0u; i < 3u; i++)
        for (j = 0u; j < GB_HIST_DEPTH; j++)
            s_hist[i][j] = s_bias[i];
    s_hist_init = 1u;
}

/* 回退 granules 个 20 ms 粒度：bias 取该粒度的快照，并覆盖当前槽 */
static void gb_rollback(uint32_t granules)
{
    uint32_t i, back, idx;
    uint64_t loss;

    back = (granules > GB_HIST_DEPTH) ? GB_HIST_DEPTH : granules;
    idx  = (s_head + GB_HIST_DEPTH - back) % GB_HIST_DEPTH;
    for (i = 0u; i < 3u; i++)
    {
        s_bias[i]         = s_hist[i][idx];
        s_hist[i][s_head] = s_bias[i];
    }

    /* ★ 有效追踪时长只扣掉被回退的那部分，不清零 */
    loss        = (uint64_t)back * GB_SNAP_TICKS;
    s_eff_ticks = (s_eff_ticks > loss) ? (s_eff_ticks - loss) : 0u;
    g_proc.bias_evidence_gran =
        (g_proc.bias_evidence_gran > back) ?
        (uint16_t)(g_proc.bias_evidence_gran - back) : 0u;
}

void proc_gyro_bias_init(void)
{
    gb_hist_init();
    s_head        = 0u;
    s_last_tick   = 0u;
    s_next_snap   = 0u;
    s_eff_ticks   = 0u;
    s_boot_done   = 0u;
    s_prev_static = 1u;
    s_static_gran = 0u;
    g_proc.bias_evidence_gran = 0u;
    g_proc.bias_ok = 0u;
}

uint8_t proc_gyro_bias_step(void)
{
    uint64_t tick, dt;
    uint32_t i;
    uint8_t  is_static;

    tick = g_proc.tick;
    if (s_next_snap == 0u)
    {
        s_last_tick = tick;
        s_next_snap = tick + GB_SNAP_TICKS;
    }
    if (s_hist_init == 0u) gb_hist_init();
    if (tick < s_last_tick) return 1u;      /* 时间戳回退：本帧不动内部状态 */

    dt = tick - s_last_tick;
    if (dt > GB_DT_MAX_TICKS) dt = GB_DT_MAX_TICKS;
    is_static = g_gate.is_static;

    /* 静止->运动边沿：按门控给的粒度数回退，并钳位到本段静止已有的粒度数 */
    if (s_prev_static != 0u && is_static == 0u && s_eff_ticks > 0u)
    {
        uint32_t back = g_gate.rollback_granules;
        if (back > s_static_gran) back = s_static_gran;
        if (back != 0u) gb_rollback(back);
    }

    /* 静止：一阶牵引；运动：冻结（于是只有静止时间计入） */
    if (is_static != 0u)
    {
        float alpha = (float)dt * ((s_boot_done != 0u) ? GB_ALPHA_WORK_TICK
                                                       : GB_ALPHA_BOOT_TICK);
        for (i = 0u; i < 3u; i++)
            s_bias[i] += (g_proc.gyro_lsb[i] - s_bias[i]) * alpha;

        s_eff_ticks += dt;
        if (s_eff_ticks >= GB_BOOT_TICKS) s_boot_done = 1u;
    }

    /* 20 ms 粒度快照 */
    if (tick >= s_next_snap)
    {
        s_head = (s_head + 1u) % GB_HIST_DEPTH;
        for (i = 0u; i < 3u; i++) s_hist[i][s_head] = s_bias[i];

        if (is_static != 0u)
        {
            if (s_static_gran < GB_HIST_DEPTH) s_static_gran++;
            if (g_proc.bias_evidence_gran < GB_EVIDENCE_MAX)
                g_proc.bias_evidence_gran++;
        }
        else
        {
            s_static_gran = 0u;     /* 证据不因运动清零：它是 bias 的累积依据 */
        }
        s_next_snap += GB_SNAP_TICKS;
        while (s_next_snap <= tick) s_next_snap += GB_SNAP_TICKS;
    }

    /* 输出：校正后 counts + 零偏估计（dps） */
    for (i = 0u; i < 3u; i++)
    {
        g_proc.gyro_dps[i] = (g_proc.gyro_lsb[i] - s_bias[i]) * gyro_dps_per_lsb((int)i);
        g_proc.bias_dps[i] = s_bias[i] * gyro_dps_per_lsb((int)i);
    }
    g_proc.bias_ok = s_boot_done;

    s_last_tick   = tick;
    s_prev_static = is_static;
    return 0u;
}


#ifdef PC_PROBE
/* ★ PC 侧注入（2026-10-13）: 从录制帧的 bias_dps 还原 s_bias(LSB), 使复刻与板载同起点。
 * 不注入则追踪器从 GB_BIAS0_LSB=(-5,-2,+3) 重新收敛 ⟹ 复刻的 gyro_dps 与板载不同
 * ⟹ 姿态从第一帧就偏（实测 resid 1.147° vs 板载 0.161°），后续全部结论作废。 */
void proc_gyro_bias_pc_inject(const float *bias_lsb)
{
    int i;
    for (i = 0; i < 3; i++) s_bias[i] = bias_lsb[i];
}
#endif