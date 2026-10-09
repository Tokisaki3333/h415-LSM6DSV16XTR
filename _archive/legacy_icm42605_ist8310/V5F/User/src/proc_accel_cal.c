#include "v5f_proc.h"          /* 处理链契约 + 门控类型 + 常数 */
#include "SPI_rx.h"            /* v5f_hold_t */

/* =====================================================================
 * 处理函数 4：加速度计标定（零偏 + 逐轴标度）+ 加速度零偏牵引
 *
 *   输入    h->imu.accel_lsb[]   DMA 中断解析出的原始 LSB（±16 g，标称 2048 LSB/g）
 *   输出    h->imu.accel_g[3] / acc_valid / accel_bias_g[3] / acc_trust / acc_traction_flags
 *   不写 up_ref：导航系已由处理函数 3 用重力定成竖直，up_ref 恒为 (0,0,1)。
 *
 *   标度 S 是常量（来源与两法互验见 v5f_proc.h 的"传感器定标：加速度计"）；
 *   零偏 b 不是常量：初值来自离线标定，之后由**牵引**在线修正。
 *
 *   ---- 牵引模型：把"被积的量"自己拉向 0 ----
 *     陀螺：  omega_corr = omega_raw - b_g,  静止时 db_g/dt = omega_corr / tau_g
 *             => 静止段内 omega_corr 以 tau_g 衰减 => 角度停止漂
 *     加速度：a_lin = accel_g - R(q)^T * up_ref,  可信时 db_a/dt = a_lin / tau_a
 *             => 可信段内 a_lin 以 tau_a 衰减 => 速度停止线性增长
 *   ★ 驱动量必须是**加速度残差本身**，不是速度。用速度驱动会变成二阶保守环
 *     （两个积分器串联、无阻尼），速度只等幅振荡、不收敛；用残差自己驱动才是一阶
 *     衰减、指数收敛。这与陀螺严格对应 —— 陀螺的驱动量也是**角速度残差**，不是角度。
 *
 *   ---- 与陀螺逐条对应的四个机制（这是本文件的结构骨架）----
 *     1) 硬门控 0/1（不是连续权重）：滞回 + 防抖给出"可信"状态
 *     2) 可信段内一阶牵引；不可信段**冻结**（所以"速度停止增长"只在可信段成立）
 *     3) tau 分两段：启动段 tau_boot，累计可信满 3*tau_boot 后切 tau_work
 *     4) 可信 -> 不可信边沿**回滚**：按 20 ms 粒度快照回退 N 格
 *
 *   ---- 门控判据为什么可以宽 ----
 *     陀螺用"角速度统计量小"判静止；这里用"比力模长接近 1 g 且转速不大"。
 *     能放宽到 D_ON/D_OFF = 0.15/0.30 g，是因为拉的是**零偏（常量）**而不是速度：
 *     判错只是把零偏拉偏一点，而且有回滚兜底，真实速度不会被抹掉。
 *   【盲点】|a|-1g 对**水平**加速度只有二阶敏感（a_h = 1 m/s^2 时仅 0.48 mg），
 *     所以门控分不开"倾斜"和"水平机动"。缓解：tau_a 取大 + 转速降权 +
 *     外部降权接口（上位控制器水平机动时把 g_v5f_acc_traction_wext 写 0）。
 *
 *   本牵引**不需要速度状态**：a_lin 是三分量，三个零偏分量都能被它收敛（含水平）。
 *
 *   每帧代价：标定 3 减 3 除 + 一次 3x3 转置乘；可信段多 3 次乘加。
 *   状态为文件静态，仅被 DMA1_Channel2_IRQHandler 主任务区单线程调用，无需加锁。
 * ===================================================================== */

const float g_v5f_accel_lsb_per_g[3] = {
    V5F_ACCEL_LSB_PER_G_X,
    V5F_ACCEL_LSB_PER_G_Y,
    V5F_ACCEL_LSB_PER_G_Z,
};

/* 零偏：初值来自离线标定，之后由牵引在线修正（所以不是 const） */
/* 离线标定的零偏常量（= s_accel_bias_lsb 的初值）。牵引钳位围着它转，
 * 不再用"绝对 ±60 mg"那种与初值无关的限幅。 */
static const float at_bias0_lsb[3] = {
    V5F_ACCEL_BIAS_LSB_X,
    V5F_ACCEL_BIAS_LSB_Y,
    V5F_ACCEL_BIAS_LSB_Z,
};

static float s_accel_bias_lsb[3] = {
    V5F_ACCEL_BIAS_LSB_X,
    V5F_ACCEL_BIAS_LSB_Y,
    V5F_ACCEL_BIAS_LSB_Z,
};

/* 外部降权因子：上位控制器在水平机动期间写 0（见 v5f_proc.h）。
 * 本文件只读、不改；初值 1.0 = 不降权。 */
volatile float g_v5f_acc_traction_wext = 1.0f;


/* ---------------- 门控 + 快照状态 ---------------- */
static float     s_bias_rate_lsb[3];                 /* PI 积分状态 z（LSB/s） */
static float     s_hist[3][V5F_ACC_TRACTION_HIST];      /* 20 ms 粒度的零偏快照 */
static float     s_hist_rate[3][V5F_ACC_TRACTION_HIST]; /* 同一粒度的积分状态快照 */
static uint32_t  s_head;                             /* 最近一份快照的下标 */
static uint8_t   s_trust       = 1u;                 /* 当前门控状态（= gate->acc_traction） */
static uint8_t   s_boot_done;                        /* 累计可信满 3*tau_boot 后锁存 */
static uint8_t   s_hist_init;                        /* 快照历史是否已预置 */
static uint64_t  s_last_tick;                        /* 上一帧 DRDY 时刻（10 ns 计数） */
static uint64_t  s_next_snap;                        /* 下一个快照边界时刻 */
static uint64_t  s_trust_ticks;                      /* 自上次回退起累计的可信时长 */
static uint16_t  s_trust_gran;                       /* 本段可信已累计的粒度数（回退钳位用） */


/* 写一份快照（在本帧牵引之后调用） */
static void at_snapshot(uint8_t trust)
{
    uint32_t i;

    s_head = (s_head + 1u) % V5F_ACC_TRACTION_HIST;
    for (i = 0u; i < 3u; i++) {
        s_hist[i][s_head]      = s_accel_bias_lsb[i];
        s_hist_rate[i][s_head] = s_bias_rate_lsb[i];
    }

    if (trust != 0u) {
        if (s_trust_gran < V5F_ACC_TRACTION_HIST) s_trust_gran++;
    } else {
        s_trust_gran = 0u;
    }
}

/* 回退 granules 个 20 ms 粒度：零偏取该粒度的快照，并覆盖当前槽使历史与现值一致 */
static void at_rollback(uint32_t granules)
{
    uint32_t i, back, idx;

    back = (granules > V5F_ACC_TRACTION_HIST) ? V5F_ACC_TRACTION_HIST : granules;
    idx  = (s_head + V5F_ACC_TRACTION_HIST - back) % V5F_ACC_TRACTION_HIST;
    for (i = 0u; i < 3u; i++) {
        /* 两个状态必须一起退：PI 环里 b 和 z 是同一个二阶系统的一组状态，
         * 只退 b 不退 z，回退后 z 会拿旧斜率继续推 b，等于回退没生效。 */
        s_accel_bias_lsb[i]    = s_hist[i][idx];
        s_bias_rate_lsb[i]     = s_hist_rate[i][idx];
        s_hist[i][s_head]      = s_accel_bias_lsb[i];
        s_hist_rate[i][s_head] = s_bias_rate_lsb[i];
    }
    s_trust_ticks = 0u;      /* 本段可信时长作废（启动段 tau_boot 判据随之重来；
                              * s_boot_done 是锁存，不受回退影响） */
}

/* 快照历史预置：源数组是零初始化的，若不预置，开机 64 格（1.28 s）内的回退会把
 * 零偏退成全 0。预置成"离线标定初值"，语义上等于"开机前一直就是这个值"。 */
static void at_hist_init(void)
{
    uint32_t i, j;

    for (i = 0u; i < 3u; i++) {
        s_bias_rate_lsb[i] = 0.0f;        /* 积分状态初值 0：开机没有斜率信息 */
        for (j = 0u; j < V5F_ACC_TRACTION_HIST; j++) {
            s_hist[i][j]      = s_accel_bias_lsb[i];
            s_hist_rate[i][j] = 0.0f;
        }
    }
    s_hist_init = 1u;
}


uint8_t v5f_proc_accel_cal(volatile v5f_hold_t *h,
                            const volatile v5f_proc_gate_t *gate)
{
    uint8_t  i;
    uint8_t  rb = 0u;
    uint32_t back;
    uint64_t tick, dt;
    float    alpha, ai;
    float    qw, qx, qy, qz, ux, uy, uz, gx, gy, gz, al;

    /* ---- 1) 标定（用当前零偏）---- */
    for (i = 0u; i < 3u; i++) {
        h->imu.accel_g[i] = ((float)h->imu.accel_lsb[i] - s_accel_bias_lsb[i])
                            / g_v5f_accel_lsb_per_g[i];
    }
    h->imu.acc_valid = 1u;

    /* ---- 2) up_ref：世界竖直在导航系里的表示 ----
     * 导航系已由处理函数 3 用重力初始化成竖直（z = 上），所以 up_ref **恒为 (0,0,1)**，
     * 由处理函数 3 在使能那一帧直接置好。
     * 这里**不再**用加速度计去锁存它：那样锁的是"锁存那一刻的比力方向"，带着当时的
     * 加速度计零偏，而且一旦锁死就不再更新 —— 于是姿态环与零偏牵引环会一起去追一个
     * 错的竖直。改成常量后，竖直基准完全由导航系定义、不随零偏漂移。
     */

    /* ---- 3) 零偏牵引 ----
     * 牵引要 up_ref（去重力）与有效姿态；两者都没就绪时本帧不牵引，直接发布。 */
    if (h->att.up_ref_ok == 0u || h->att.valid == 0u) {
        for (i = 0u; i < 3u; i++) {
            h->imu.accel_bias_g[i] = s_accel_bias_lsb[i] / g_v5f_accel_lsb_per_g[i];
            h->imu.a_lin[i]        = 0.0f;   /* 没有 up_ref 就没法去重力 */
        }
        h->imu.acc_trust = s_trust;
        h->imu.acc_traction_flags = 0u;
        return V5F_PROC_OK;
    }

    if (s_hist_init == 0u) {
        at_hist_init();
        s_last_tick = h->imu.fresh.drdy_tick;
        s_next_snap = s_last_tick + V5F_ACC_TRACTION_SNAP_TICKS;
    }

    tick = h->imu.fresh.drdy_tick;
    /* 时间戳回退：只跳过牵引，标定结果本身与时间无关，故 acc_valid 仍为 1。
     * （与处理函数 1 不同 —— 那里的校正结果依赖时间基准，所以返回错误时置 corr_valid = 0） */
    if (tick < s_last_tick) return V5F_PROC_ERR_TICK;
    dt = tick - s_last_tick;

    /* ---- 2b) 牵引驱动量 a_lin = accel_g - R(q)^T * up_ref（g）----
     * 无条件算、无条件发布：它是"没被消掉的残余"，也是漂移的直接来源。
     * 放在门控之前算，这样即使牵引冻结，上位机也能看到残余有多大。 */
    qw = h->att.q[0]; qx = h->att.q[1]; qy = h->att.q[2]; qz = h->att.q[3];
    ux = h->att.up_ref[0]; uy = h->att.up_ref[1]; uz = h->att.up_ref[2];
    gx = (1.0f - 2.0f*(qy*qy + qz*qz))*ux + 2.0f*(qx*qy + qw*qz)*uy + 2.0f*(qx*qz - qw*qy)*uz;
    gy = 2.0f*(qx*qy - qw*qz)*ux + (1.0f - 2.0f*(qx*qx + qz*qz))*uy + 2.0f*(qy*qz + qw*qx)*uz;
    gz = 2.0f*(qx*qz + qw*qy)*ux + 2.0f*(qy*qz + qw*qx)*uy + (1.0f - 2.0f*(qx*qx + qy*qy))*uz;
    for (i = 0u; i < 3u; i++) {
        h->imu.a_lin[i] = h->imu.accel_g[i] - ((i == 0u) ? gx : ((i == 1u) ? gy : gz));
    }

    /* --- 3a) 门控：**外部门控**（处理函数 5 写入，本帧读的是它上一帧的判定）---
     * 判据不在这里算，也不许用 a_lin / up_ref（都依赖本函数正在更新的 b_a，自引用）。
     * 见 proc_acc_gate.c 与 v5f_proc.h 的 V5F_ACG_*。 */

    /* --- 3b) 牵引 -> 冻结边沿：回滚（钳位到本段牵引已有的粒度数）--- */
    if (s_trust != 0u && gate->acc_traction == 0u && s_trust_ticks > 0u) {
        back = gate->acc_rollback_granules;
        if (back > s_trust_gran) back = s_trust_gran;
        if (back != 0u) {
            at_rollback(back);
            rb = 1u;
        }
    }
    s_trust = gate->acc_traction;      /* 本帧的牵引/冻结由外部门控决定 */

    /* --- 3c) 牵引：门控为牵引时用 PI（二阶）环跟踪零偏，冻结时不更新 ---
     *   e      = a_lin                        （g）
     *   z     += e * S * ALPHA_I              （LSB/s，积分状态）
     *   b     += e * S * ALPHA_P + z * dt_s   （LSB）
     * 一阶->二阶的理由见 v5f_proc.h 的 PI 说明。 */
    if (s_trust != 0u) {
        float w = g_v5f_acc_traction_wext;            /* 外部降权（上位控制器机动信号） */

        alpha  = (float)dt * ((s_boot_done != 0u) ? V5F_ACC_TRACTION_ALPHA_WORK
                                                  : V5F_ACC_TRACTION_ALPHA_BOOT);
        ai     = (float)dt * ((s_boot_done != 0u) ? V5F_ACC_TRACTION_ALPHA_I_WORK
                                                  : V5F_ACC_TRACTION_ALPHA_I_BOOT);
        alpha *= w;
        ai    *= w;

        if (alpha > 0.0f || ai > 0.0f) {
            /* a_lin 已在 2b) 无条件算好，这里直接用。
             * a_lin 乘 S[i] 换算成 LSB 才能与零偏同单位。
             * 积分状态先用上一帧的 z 参与零偏更新（半隐式欧拉，对双积分器更稳）。 */
            for (i = 0u; i < 3u; i++) {
                float s_i = g_v5f_accel_lsb_per_g[i];

                al = h->imu.a_lin[i];

                /* 积分状态先推进，零偏更新用推进后的 z（半隐式欧拉，对双积分器更稳） */
                s_bias_rate_lsb[i] += al * s_i * ai;
                if (s_bias_rate_lsb[i] >  V5F_ACC_TRACTION_RATE_LIM_LSB) s_bias_rate_lsb[i] =  V5F_ACC_TRACTION_RATE_LIM_LSB;
                if (s_bias_rate_lsb[i] < -V5F_ACC_TRACTION_RATE_LIM_LSB) s_bias_rate_lsb[i] = -V5F_ACC_TRACTION_RATE_LIM_LSB;

                s_accel_bias_lsb[i] += al * s_i * alpha
                                     + s_bias_rate_lsb[i] * (float)dt * V5F_ACC_TRACTION_DT_SCALE;
                /* 钳位**相对离线标定初值**，且必须小于门的 ON 阈值（见 v5f_proc.h
                 * 的锁死条件）：门要能重新打开，真静止时 |a_lin| 就得能回到 ON 以下。 */
                {
                    float lim = V5F_ACC_TRACTION_BIAS_LIM_MG * 1e-3f * s_i;
                    if (s_accel_bias_lsb[i] > at_bias0_lsb[i] + lim) s_accel_bias_lsb[i] = at_bias0_lsb[i] + lim;
                    if (s_accel_bias_lsb[i] < at_bias0_lsb[i] - lim) s_accel_bias_lsb[i] = at_bias0_lsb[i] - lim;
                }
            }
            s_trust_ticks += dt;
            if (s_trust_ticks >= V5F_ACC_TRACTION_BOOT_TICKS) s_boot_done = 1u;
        }
    }

    /* --- 3d) 20 ms 粒度快照 --- */
    if (tick >= s_next_snap) {
        at_snapshot(s_trust);
        s_next_snap += V5F_ACC_TRACTION_SNAP_TICKS;
    }

    /* ---- 4) 用牵引后的零偏重算输出（与陀螺"先牵引后出结果"同序）---- */
    for (i = 0u; i < 3u; i++) {
        h->imu.accel_g[i] = ((float)h->imu.accel_lsb[i] - s_accel_bias_lsb[i])
                            / g_v5f_accel_lsb_per_g[i];
        h->imu.accel_bias_g[i] = s_accel_bias_lsb[i] / g_v5f_accel_lsb_per_g[i];
    }
    h->imu.acc_trust          = s_trust;
    h->imu.acc_traction_flags = (rb != 0u) ? V5F_ACC_TRACTION_FLAG_ROLLBACK : 0u;

    s_last_tick = tick;
    return V5F_PROC_OK;
}
