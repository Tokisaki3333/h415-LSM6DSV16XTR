#include "v5f_proc.h"
#include "SPI_rx.h"
#include <math.h>               /* sqrtf：-Ofast 下一条 fsqrt.s */

/* =====================================================================
 * 处理函数 5：世界系（导航系）速度 —— 比力积分 + 时刻向 0 衰减（漏积分器）
 *
 *   输入   h->imu.accel_g[]（处理函数 4 的比力，单位 g）
 *          h->att.q[]      （处理函数 3 的姿态）
 *          h->att.up_ref[] （世界竖直在固件参考系里的表示，处理函数 4 锁存）
 *   递推   v    *= (1 - dt / tau_v)                         <-- 第 0 步，每帧无条件
 *          a_nav = G * ( R(q) * accel_g  -  up_ref )        单位 m/s^2
 *          v    += a_nav * dt                              <-- 仅姿态有效帧
 *   输出   h->vel.v_nav[3] / h->vel.valid / h->vel.age_s
 *
 *   ---- 为什么衰减必须是函数的第一条语句 ----
 *   衰减挂在冻结门（或任何 dt 早退）之后时，门一挡就**完全不衰减**，速度保持
 *   在上一次的值上 —— 那还是"有门控的衰减"，表现就是速度挂在某个值上不下来。
 *   本函数把衰减放在最前面，用**自己独立的时间基准** s_dec_tick，于是：
 *     · 每一帧都恰好衰减一次，没有任何 0/1 门能跳过它；
 *     · 即使积分路径被冻结门 / dt 基准重建 / 时间戳回退跳过，衰减仍按真实
 *       经过的时间推进（s_dec_tick 每帧刷新，与积分基准 s_last_tick 无关）。
 *   先衰减后积分只是让"本帧新增的速度"当帧不被衰减；一阶精度下与顺序无关。
 *
 *   冻结   姿态未出（处理函数 3 冻结在单位四元数，即 bias_ok = 0）时不积分：
 *          输出恒 0、valid = 0、age_s = 0，且不建立 dt 基准（姿态出来的第一帧
 *          只标定时间基准）。理由同姿态：速度是 R(q) 的函数，姿态参考系没定
 *          下来，"世界系速度"没有意义。
 *          ★ 但冻结期间**照样衰减**（见上），所以恢复后不会带着冻结窗口里
 *            攒下来的旧速度。
 *
 *   ---- 为什么没有速度归零门（ZUPT）----
 *   原设计用"陀螺判静 AND 加速度门可信 -> v = 0"把速度环闭合。但那个门要
 *   stat.is_static（陀螺统计量 < 0.065~0.195 dps），本机环境 level 中位
 *   0.6~0.9 dps，远超门槛 —— 实测 20260914_050043 的 t=20~28 s 里 is_static
 *   全程 0%，而 acc_traction 100%、|a_lin| 只有 6~18 mg（零偏/倾斜环已把残余
 *   压到近 0），于是速度冻在 0.055~0.073 m/s 不动、直到 t=28.5 is_static 终于
 *   成立才被清掉。表现就是"低速移动一段后停止，速度漂到一个值"。
 *   换成连续衰减后速度偏置的上界由 tau_v 直接给出：|v|_稳态 <= a_bias * tau_v，
 *   且不依赖任何门、不会出现"门开一次把真实速度整段抹掉"的跳变。
 *
 *   ---- 去重力为什么长这样 ----
 *   accel_g 是**比力**（specific force）：静止时大小 1 g、方向指向"上"。
 *   R(q) * accel_g 就是比力在导航系里的表示；up_ref 是世界竖直在导航系里的表示，
 *   重力沿 -up_ref，所以再减一个 up_ref 就完成去重力。静止时 R(q)*accel_g == up_ref，
 *   于是 a_nav == 0，自洽。
 *
 *   ---- 为什么只出速度、不出位置 ----
 *   位置是速度的二次积分，误差跨锚点累积（实测同一个 17.8 s 空档，速度残差
 *   0.47 m/s，位置误差 1.4 m）。所以本固件不出位置。
 *
 *   ---- 诚实标出可信窗口 ----
 *   纯惯性下"平滑匀速平动"和"零速"不可分（等效原理），衰减把这一点变成显式代价：
 *   真实匀速平动也按 e^(-t/tau_v) 衰减（tau_v = 30 s 时 1 m/s 巡航 3 s 掉 10%、
 *   10 s 掉 28%）。故除了 v 还输出 age_s（距上次速度基本归零的秒数），
 *   上层据此判断速度还能信多久。
 *
 *   每帧代价：一次 3x3 乘 + 3 乘加 + 3 乘。状态为文件静态，单线程调用，无需加锁。
 * ===================================================================== */

static float    s_v[3];
static uint64_t s_last_tick;    /* 积分时间基准 */
static uint64_t s_dec_tick;     /* 衰减时间基准：独立于积分，保证每帧都衰减 */
static uint64_t s_age_tick;     /* 距上次 |v| 基本归零的累计时间（10 ns 计数） */
static uint8_t  s_started;      /* 1 = 速度环启动过（姿态至少有效过一帧） */
static uint8_t  s_dt_ok;        /* 1 = dt 基准已建立（s_last_tick 有效） */
static uint8_t  s_dec_ok;       /* 1 = 衰减基准已建立（s_dec_tick 有效） */

/* 发布：v_nav / age_s 由状态唯一决定，每条返回路径都调它一次，于是 age 每帧
 * 恰好推进一次 dtd（dtd 是本帧真实经过的时间；被门跳过的帧也算，因为时间过了）。 */
static void vel_emit(volatile v5f_hold_t *h, uint64_t dtd, uint8_t valid)
{
    uint8_t i;

    if ((s_v[0]*s_v[0] + s_v[1]*s_v[1] + s_v[2]*s_v[2]) < 1e-8f) {
        s_age_tick = 0u;                        /* 基本归零：age 重新计时 */
    } else if (s_age_tick < 0xFFFFFFFFFFFFFF00ULL) {
        s_age_tick += dtd;
    }

    for (i = 0u; i < 3u; i++) h->vel.v_nav[i] = s_v[i];
    h->vel.age_s = (float)s_age_tick * 1e-8f;   /* 10 ns 计数 -> 秒（精确，不用标称帧率） */
    h->vel.valid = valid;
}

uint8_t v5f_proc_velocity(volatile v5f_hold_t *h,
                            const volatile v5f_proc_gate_t *gate)
{
    uint64_t tick, dt, dtd;
    float    vdt, hs, dec, rate, lam, drop, vm, f, qw, qx, qy, qz, ax, ay, az, wx, wy, wz;

    tick = h->imu.fresh.drdy_tick;

    /* ==== 0) 时刻向 0 衰减：函数的第一件事，无条件、每帧一次 ====
     * 先于冻结门与一切 dt 早退，所以没有任何 0/1 门能把它跳过去。
     * 用自己的时间基准 s_dec_tick，与积分的 s_last_tick 完全独立。
     * 衰减律是**乘性 + 常速率两级叠加**（各管大/小速度），理由与实测事故见
     * v5f_tune.h 的 F 组 —— 只用常速率时大速度刹不住（线性斜坡）。 */
    dtd = 0u;
    if (s_dec_ok == 0u) {
        s_dec_ok   = 1u;
        s_dec_tick = tick;                      /* 首帧：只立基准 */
    } else if (tick > s_dec_tick) {
        dtd        = tick - s_dec_tick;
        s_dec_tick = tick;
    } else if (tick < s_dec_tick) {
        s_dec_tick = tick;                      /* 时间戳回退：重建基准，本帧不衰减 */
    }

    if (dtd != 0u) {
        /* 两级衰减，各管一段（理由与实测事故见 v5f_tune.h 的 F 组）：
         *   ① 软 ZUPT v *= (1 - hs*rate)，rate = lam/tau_fast + (1-lam)/tau_slow
         *             —— 连续自适应速率，见 v5f_tune.h 的 F 组（含三种衰减律的实测对比）。
         *   ② 常速率 |v| -= K*hs  管**小**速度 —— |v| <= K*hs 时一帧减完精确归零，
         *             ①的指数尾被它兜底，稳态才是 0 而不是 a_bias*tau。 */
        hs = (float)dtd * V5F_VEL_DECAY_TICK_S;              /* 本帧时长（s） */

        /* ① 软 ZUPT：衰减**时刻都在跑**，lam 只调速率（不是 0/1 门，关不掉它）。
         *    lam 由处理函数 5 的"门 C"给出：陀螺静 + 比力模长接近 1 g，再按
         *    "距上次高置信锚点的时间"抬高下限。 */
        lam  = (float)gate->vel_soft * (1.0f / 255.0f);
        rate = lam * V5F_VEL_ZUPT_RATE_FAST + (1.0f - lam) * V5F_VEL_ZUPT_RATE_SLOW;

        dec = 1.0f - hs * rate;
        if (dec < 0.0f) dec = 0.0f;                         /* 长停顿(tick 跳变)不反号 */
        s_v[0] *= dec;
        s_v[1] *= dec;
        s_v[2] *= dec;

        drop = V5F_VEL_DECAY_K * hs;                         /* m/s */
        if (drop > 0.0f) {
            vm = sqrtf(s_v[0]*s_v[0] + s_v[1]*s_v[1] + s_v[2]*s_v[2]);
            if (vm > 0.0f) {
                if (drop >= vm) {
                    s_v[0] = 0.0f; s_v[1] = 0.0f; s_v[2] = 0.0f;  /* 一帧减完：精确归零 */
                } else {
                    f = (vm - drop) / vm;                          /* 保方向、缩短模长 */
                    s_v[0] *= f; s_v[1] *= f; s_v[2] *= f;
                }
            }
        }
    }

    /* ==== 1) 上电窗口：姿态从未有效过之前，速度恒 0 ==== */
    if (s_started == 0u) {
        s_v[0] = 0.0f; s_v[1] = 0.0f; s_v[2] = 0.0f;
    }

    /* ==== 2) 冻结门：姿态没定下来就不积分 ====
     * 上游必需：att.valid（处理函数 3 未收敛时冻结在单位四元数、valid = 0）
     *           + att.up_ref_ok（处理函数 4 锁存的世界竖直，否则去重力没基准）。
     * 冻结 = 不积分、不发 valid，但**衰减照做**（已在第 0 步完成），所以恢复后
     * 不会带着冻结窗口里攒下来的旧速度。
     * 作废 dt 基准（s_dt_ok = 0），于是姿态恢复后的第一帧只重新标定时间基准、
     * 不积分 —— 否则那一帧的 dt 会横跨整个冻结窗口，把一段几十秒的假积分塞进速度里。 */
    if (h->att.valid == 0u || h->att.up_ref_ok == 0u) {
        s_dt_ok     = 0u;
        s_last_tick = 0u;
        vel_emit(h, dtd, 0u);
        return V5F_PROC_OK;                     /* valid = 0 */
    }

    s_started = 1u;

    /* ==== 3) 积分用的 dt 基准 ==== */
    if (s_dt_ok == 0u) {                        /* 本帧只标定基准，不积分 */
        s_dt_ok     = 1u;
        s_last_tick = tick;
        vel_emit(h, dtd, 0u);
        return V5F_PROC_OK;
    }
    if (tick < s_last_tick) {                   /* 时间戳回退：下一帧重建基准 */
        s_dt_ok = 0u;
        vel_emit(h, dtd, 0u);
        return V5F_PROC_ERR_TICK;
    }
    dt = tick - s_last_tick;
    s_last_tick = tick;

    if (dt == 0u) {
        vel_emit(h, dtd, 0u);
        return V5F_PROC_OK;
    }

    /* ==== 4) R(q) * accel_g：比力换到导航系 ==== */
    qw = h->att.q[0]; qx = h->att.q[1]; qy = h->att.q[2]; qz = h->att.q[3];
    ax = h->imu.accel_g[0]; ay = h->imu.accel_g[1]; az = h->imu.accel_g[2];
    wx = (1.0f - 2.0f*(qy*qy + qz*qz))*ax + 2.0f*(qx*qy - qw*qz)*ay + 2.0f*(qx*qz + qw*qy)*az;
    wy = 2.0f*(qx*qy + qw*qz)*ax + (1.0f - 2.0f*(qx*qx + qz*qz))*ay + 2.0f*(qy*qz - qw*qx)*az;
    wz = 2.0f*(qx*qz - qw*qy)*ax + 2.0f*(qy*qz + qw*qx)*ay + (1.0f - 2.0f*(qx*qx + qy*qy))*az;

    /* a_nav = G * (R*accel_g - up_ref)；dt 单位是 10 ns 计数。
     * ★ accel_g 单位是 g，必须乘 V5F_VEL_G_LOCAL 才是 m/s^2，漏乘会小 9.8 倍。 */
    vdt = V5F_VEL_G_LOCAL * (float)dt * 1e-8f;
    s_v[0] += vdt * (wx - h->att.up_ref[0]);
    s_v[1] += vdt * (wy - h->att.up_ref[1]);
    s_v[2] += vdt * (wz - h->att.up_ref[2]);

    vel_emit(h, dtd, 1u);
    return V5F_PROC_OK;
}
