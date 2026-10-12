/* 处理链主循环：把 6 个处理函数按序串起来，并按 100 Hz 上报姿态
 *
 * 每 IMU 帧（7.68 kHz，由 shm->imu.hdr.cnt 变化触发）：
 *   ① 读原始 IMU（陀螺 counts + 加计 g）
 *   ② proc_mag_step       有新磁拍就更新净场，同时推进卡死计数
 *   ③ proc_static_detect  吃上一帧 gyro_dps        -> is_static
 *   ④ proc_gyro_bias      吃 is_static             -> gyro_dps / bias_dps
 *   ⑤ proc_attitude_prop  用 gyro_dps 积分 dt      -> q
 *   ⑥ proc_gate_step      推进协方差标量           -> entry_B / K
 *   ⑦ proc_mag_corr       重力环 + 按入口修正      -> q
 *   ⑧ 每 77 帧发一帧 TYPE7 ATT（约 100 Hz）
 *
 * ★ 顺序不能变：③ 用 ④ 的上一帧输出，④ 用 ③ 的判定，这是后验链。
 * ★ 磁修正每 IMU 帧施加一次，不是每 MMC 拍一次 —— 实测若按 MMC 周期（约 1 ms）
 *   施加，残差 = 漂移率 × 1 ms 的量级会累积到十几度。
 * ★ 上报走 CDC 的 TYPE 包（含 CRC 与序号），与原始值直通共用同一条上行链。
 *
 * 全部状态为文件静态，仅被 V5F 处理主线程单线程调用，无需加锁。 */
#include "proc.h"
#include "proc_tune.h"
#include "mipc_shm.h"

#include "sys_clk.h"

proc_state_t g_proc;
proc_gate_t  g_gate;

static uint32_t s_last_imu_cnt;
static uint8_t  s_aligned;


/* ---------------- EKF 迭代计时（单位 10 ns 计数）----------------
 * ekf_time_add()  每次迭代后累加
 * ekf_time_take() 取走本秒统计并清零
 * 全部为本文件静态，单线程调用，无需加锁。 */
static uint32_t s_ekt_n;
static uint32_t s_ekt_sum;
static uint32_t s_ekt_max;
static uint32_t s_ekt_min;
static uint32_t s_ekt_last;
static float    s_ekt_hz;

/* 上报用的计时快照（us） */
static float s_rep_last_us, s_rep_max_us, s_rep_avg_us, s_rep_hz;

static void ekf_time_reset(void)
{
    s_ekt_n = 0u; s_ekt_sum = 0u; s_ekt_max = 0u; s_ekt_last = 0u;
    s_ekt_min = 0xFFFFFFFFu;
}

static void ekf_time_add(uint32_t d10ns)
{
    s_ekt_last = d10ns;
    if (s_ekt_n < 0xFFFFFFFFu) s_ekt_n++;
    if (s_ekt_sum <= (0xFFFFFFFFu - d10ns)) s_ekt_sum += d10ns;
    if (d10ns > s_ekt_max) s_ekt_max = d10ns;
    if (d10ns < s_ekt_min) s_ekt_min = d10ns;
}

/* 上报已由 main.c 统一用 justfloat 裸帧做；本文件的计时统计仅作内部度量，
 * 上位机不再需要，故只保留累加器，取走统计的接口一并删除。 */

void proc_chain_init(void)
{
    uint32_t i;

    for (i = 0u; i < 3u; i++)
    {
        g_proc.gyro_lsb[i] = 0.0f;
        g_proc.gyro_dps[i] = 0.0f;
        g_proc.bias_dps[i] = 0.0f;
    }
    g_proc.tick        = 0u;
    g_proc.q_valid     = 0u;
    g_proc.mag_corr_en = 0u;      /* 本链路：地磁修正关闭（纯陀螺积分） */
    g_proc.bias_ok     = 0u;
    g_proc.is_static   = 1u;
    g_proc.level_dps   = 0.0f;
    g_proc.mag_fix     = MAG_FIX_OFF;
    g_proc.mag_valid   = 0u;
    g_proc.mag_stall   = 0u;
    g_proc.mag_bad_acc = 0u;

    g_gate.is_static         = 1u;
    g_gate.rollback_granules = DET_ROLLBACK_GRAN;

    s_last_imu_cnt = 0u;
    s_aligned      = 0u;

    proc_mag_init();
    proc_static_detect_init();
    proc_gyro_bias_init();
    proc_attitude_init();
    proc_mag_corr_init();
    proc_eskf_init();
    ekf_time_reset();
}

void proc_chain_step(const volatile shared_mem_t *shm)
{
    uint32_t cnt;
    uint64_t tick, t_ekf0;
    float    dt_s;

    /* ---- ② 磁：每帧都检查有没有新拍（同时推进卡死计数） ---- */
    proc_mag_step(shm);

    /* ---- ① 新 IMU 帧？ ---- */
    cnt = shm->imu.hdr.cnt;
    if (cnt == s_last_imu_cnt) return;
    __sync_synchronize();
    s_last_imu_cnt = cnt;

    tick = shm_chan_ts_read(&shm->imu.hdr);
    if (tick < g_proc.tick) return;                 /* 时戳回退：本帧跳过 */
    /* ★ 首帧播种：g_proc.tick 初值为 0，若不先播种，第一帧的
     *   dt = tick×1e-8 会是"上电至今的全部时间"，必然触发下面的 dt 门。 */
    if (g_proc.tick == 0u)
    {
        g_proc.tick = tick;
        g_proc.gyro_lsb[0] = (float)shm->imu.gx;
        g_proc.gyro_lsb[1] = (float)shm->imu.gy;
        g_proc.gyro_lsb[2] = (float)shm->imu.gz;
        g_proc.gyro_dps[0] = g_proc.gyro_lsb[0] * GYRO_DPS_PER_LSB;
        g_proc.gyro_dps[1] = g_proc.gyro_lsb[1] * GYRO_DPS_PER_LSB;
        g_proc.gyro_dps[2] = g_proc.gyro_lsb[2] * GYRO_DPS_PER_LSB;
        return;
    }

    dt_s = (float)(tick - g_proc.tick) * 1e-8f;
    g_proc.tick = tick;

    /* ★★ dt_s 合理性门（必须）：上电瞬间共享区计数器会一次性暴跳
     *   （实测 imu_cnt 141 → 2131 → 439957、mmc_cnt 34 → 304 → 59664），
     *   若把这个巨大间隔当 dt 用，陀螺预测 q ← q ⊗ exp(ω·dt) 会被放大到
     *   转过巨大角度，姿态被瞬间打飞 —— 这就是上位机看到的"大跳变"
     *   （实测 t=0.01278 s 处 |dQ|=1.985，俯仰/偏航同时跳到 170° 量级）。
     *   ★ 判据：一帧的合理间隔是 1/7670 ≈ 130 µs；若超过 100 ms
     *     （= 767 个采样周期，正常情况绝不会出现），说明计数器发生了
     *     一次性推进，本帧的 dt 不可信 ⟹ 按 0 处理（不预测、不观测），
     *     让链在下一次计数正常变化时自然恢复。
     *   ★ 100 ms 同时也是 proc_attitude_prop 内部原有的上限（dt_s > 0.2f 直接
     *     return），这里取更严的 100 ms，两者不冲突。 */
    if (dt_s < 0.0f || dt_s > 0.1f)
    {
        if (dt_s < 0.0f) { /* 时间戳回退：已在上面 return */ }
        dt_s = 0.0f;
        /* 本帧只更新原始读数与零偏，不做预测/观测 */
        g_proc.gyro_lsb[0] = (float)shm->imu.gx;
        g_proc.gyro_lsb[1] = (float)shm->imu.gy;
        g_proc.gyro_lsb[2] = (float)shm->imu.gz;
        g_proc.gyro_dps[0] = g_proc.gyro_lsb[0] * GYRO_DPS_PER_LSB;
        g_proc.gyro_dps[1] = g_proc.gyro_lsb[1] * GYRO_DPS_PER_LSB;
        g_proc.gyro_dps[2] = g_proc.gyro_lsb[2] * GYRO_DPS_PER_LSB;
        return;
    }

    /* 读原始值 */
    g_proc.gyro_lsb[0] = (float)shm->imu.gx;
    g_proc.gyro_lsb[1] = (float)shm->imu.gy;
    g_proc.gyro_lsb[2] = (float)shm->imu.gz;

    /* ---- 首次：用加计定初始姿态 ---- */
    if (s_aligned == 0u)
    {
        float av[3];
        av[0] = (float)shm->imu.ax * ACCEL_G_PER_LSB;
        av[1] = (float)shm->imu.ay * ACCEL_G_PER_LSB;
        av[2] = (float)shm->imu.az * ACCEL_G_PER_LSB;
        proc_attitude_align(av);
        if (g_proc.q_valid != 0u) s_aligned = 1u;
        g_proc.gyro_dps[0] = g_proc.gyro_lsb[0] * GYRO_DPS_PER_LSB;
        g_proc.gyro_dps[1] = g_proc.gyro_lsb[1] * GYRO_DPS_PER_LSB;
        g_proc.gyro_dps[2] = g_proc.gyro_lsb[2] * GYRO_DPS_PER_LSB;
        return;
    }

    t_ekf0 = GetTime64_10Ns();      /* ★ EKF 迭代计时起点 */

    /* ---- ③ 动静判定（吃上一帧 gyro_dps） ---- */
    proc_static_detect_step();

    /* ---- ④ 零偏追踪（吃 is_static） ---- */
    if (proc_gyro_bias_step() != 0u) { ekf_time_add(0u); return; }

    /* ---- ⑤ 姿态传播 ----
     * ★ ESKF 接管后【不再需要】这一步：ESKF 自己的预测步就是 q ← q ⊗ exp(ωdt)。
     *   两者同时做会把同一帧角速度积分两次，且互补滤波是"世界系左乘"、
     *   ESKF 是"体坐标右乘"，在同一 g_proc.q 上互相污染。
     *   仅在 ESKF 尚未接管（启动标定窗口内）时作为兜底。 */
    if (proc_eskf_ready() == 0u)
        proc_attitude_prop(dt_s);

    /* ---- ⑤b 三传感器 ESKF ----
     * ★ 唯一的姿态融合器：陀螺预测 + 加计观测（三条件门控）+ 地磁观测。
     *   自带的启动标定窗口（0.5~2.5 s）结束后才接管姿态：
     *   窗口内 ESKF 自己也在按陀螺推进（保持角速率连续性），
     *   窗口结束时用窗口内加计/地磁均值【一次性】构造绝对姿态与参考矢量。 */
    {
        float av[3], mu[3];
        av[0] = (float)shm->imu.ax * ACCEL_G_PER_LSB;
        av[1] = (float)shm->imu.ay * ACCEL_G_PER_LSB;
        av[2] = (float)shm->imu.az * ACCEL_G_PER_LSB;
        mu[0] = g_proc.mag_u[0];
        mu[1] = g_proc.mag_u[1];
        mu[2] = g_proc.mag_u[2];

        proc_eskf_step(dt_s, g_proc.gyro_dps, g_proc.gyro_lsb, av, mu,
                       g_proc.mag_valid, proc_mag_is_new(), tick);

        /* ESKF 的姿态即权威姿态（窗口内也已按陀螺推进） */
        {
            float qq[4];
            proc_eskf_get_q(qq);
            g_proc.q[0] = qq[0]; g_proc.q[1] = qq[1];
            g_proc.q[2] = qq[2]; g_proc.q[3] = qq[3];
            g_proc.q_valid = 1u;
            g_proc.grav_err_deg = 0.0f;
            g_proc.grav_applied = proc_eskf_grav_open();   /* 本帧重力倾斜修正是否施加 */
        }
    }

    /* ---- ⑥ 门控（互补滤波路径的协方差标量；ESKF 接管后仅作诊断） ---- */
    proc_gate_step(dt_s);

    /* ---- ⑦ 互补滤波的重力环 + 地磁修正 ----
     * ★ ESKF 接管后【让行】：两者都改 q 会重复注入同一观测量。
     *   仅在 ESKF 尚未接管时（启动标定窗口内 / 标定无效）作为兜底。 */
    if (proc_eskf_ready() == 0u)
        proc_mag_corr_step(shm, dt_s);

    /* ★ EKF 迭代计时终点（10 ns 计数） */
    {
        uint64_t d = GetTime64_10Ns() - t_ekf0;
        if (d > 0xFFFFFFFFULL) d = 0xFFFFFFFFULL;
        ekf_time_add((uint32_t)d);
    }

    /* 上报由 main.c 统一做（justfloat 裸帧）；本文件【不发任何 CDC 包】 */
}
