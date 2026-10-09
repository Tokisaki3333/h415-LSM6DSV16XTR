#include "v5f_proc.h"
#include "SPI_rx.h"
#include <math.h>               /* sqrtf / fabsf：门 C 的置信度 */

/* =====================================================================
 * 处理函数 5：加速度侧的两个**外部门控**（各自独立判据，供两个不同的环用）
 *
 *   输入   h->imu.accel_lsb[] —— 原始 LSB（本帧）
 *          h->att.q[] / h->att.up_ref[] —— 姿态与竖直参考（本帧）
 *          [20260914 起] 门B 判据 = 256 ms 窗 |off| 均值 ∈ 1 g +-3%，且同窗离线常量校正后净 |w| < 2 dps；不再读 is_static（见函数上方注释）
 *   输出   gate->acc_traction          门 A：加速度零偏牵引  1 = 牵引，0 = 冻结
 *          gate->acc_rollback_granules 门 A：牵引->冻结边沿回退的 20 ms 粒度数
 *          gate->att_tilt             门 B：重力/倾斜融合    1 = 融合，0 = 冻结
 *   ★ 两个门**判据不同、字段不同**：门 A 用 |w|（对姿态歪敏感），门 B 用 |off|
 *     （对姿态歪免疫）。绝不能互相借用 —— 见 v5f_proc.h 的"铁律"。
 *
 *   判据量 w = (accel_lsb - 离线标定零偏) / 标度  -  R(q)^T * up_ref
 *       即"**用离线标定常量**去掉零偏、再去掉重力分量之后的残余"（单位 g）。
 *
 *   ---- 为什么必须用离线标定常量，不能用 h->imu.a_lin ----
 *   a_lin = w - db，db 是牵引自己正在更新的零偏增量。拿它当门就是自引用：
 *   db 一旦饱和（到钳位），|a_lin| ≈ |db| 就永远大于 ON，门再也打不开、
 *   牵引永久停摆 —— 实测 20260914_044158 就是这样：t>=18 s 设备真静止
 *   （is_static=100%、level=0.03 dps），而 |a_lin| 卡在 35 mg、acc_traction=0%，
 *   速度按 0.35 m/s^2 一路线性发散。
 *   用离线标定常量算出来的 w **不含 db**，门就不可能被牵引状态锁死。
 *
 *   ---- 与陀螺门同构 ----
 *   陀螺门问"零偏校正后的角速度是不是接近 0"；这里问"去掉零偏与重力分量之后的
 *   加速度是不是接近 0"。两者都是拿"被跟踪的那个残差本身"当门 —— 只是零偏必须
 *   取**离线标定值**而不是在线估计值，否则自引用。
 *
 *   ---- 顺序语义（与陀螺门一致）----
 *   快冻结、慢恢复：连续 8 帧（1 ms）超过 OFF 就冻结；恢复要连续 800 帧（100 ms）
 *   都在 ON 以下。
 * ===================================================================== */

/* 离线标定的零偏常量（与 proc_accel_cal.c 里 s_accel_bias_lsb 的初值同一组）。 */
static const float ag_bias0_lsb[3] = {
    V5F_ACCEL_BIAS_LSB_X, V5F_ACCEL_BIAS_LSB_Y, V5F_ACCEL_BIAS_LSB_Z
};

static uint16_t s_on_cnt;      /* 连续满足"可牵引"计数 */
static uint16_t s_off_cnt;     /* 连续满足"应冻结"计数 */

/* ---- 门B 判据统计：两级均值（档内 32 ms，8 档 = 256 ms 总窗）----
 * 两个条件同时成立才开门：
 *   1) 256 ms 窗内 |off| 均值落在 1 g +-3%  -> 没有线加速度（|off| 与姿态无关）
 *   2) 同一窗内**离线常量校正后**角速度的净均值 |w| < 2 dps -> 没有真转动
 * 为什么需要条件 2：只用 |off| 会被"往复运动中的瞬时带内"骗到 ——
 *   20260914_063706（桌面转正方形 -> 抬起 10 cm 再转）占空比 3.9% -> 69%，
 *   姿态被多拉偏 0.67 deg，末位置 |p| 0.0445 -> 0.1304 m（变差 3 倍）。
 * 为什么用长窗而不是 is_static：旧门用 **16 ms** 窗判静止，手抖即失效（占空比 10%）；
 *   净转动在 256 ms 窗里手抖零均值（~0.2 dps）、真转动几十 dps，天然分得开。
 * 为什么两级均值：只存每档均值即可，RAM 约 130 B（直存 2048 点要 32 KB）。
 * 窗口只统计**离线标定常量**算出的量，不借在线 gb_bias / accel_bias，无自环。 */
static const float gb_bias0_lsb[3] = {
    V5F_GYRO_BIAS_LSB_X, V5F_GYRO_BIAS_LSB_Y, V5F_GYRO_BIAS_LSB_Z
};
static float    s_g_am;                            /* 当前档内 |off| 累加 */
static float    s_g_w[3];                          /* 当前档内角速度累加 */
static uint16_t s_g_cnt;                           /* 当前档内样本数 */
static float    s_ring_am[V5F_ATT_TILT_NGRAN];     /* 已满档的 |off| 均值 */
static float    s_ring_w[V5F_ATT_TILT_NGRAN][3];   /* 已满档的角速度均值 */
static uint8_t  s_ring_head;
static uint8_t  s_ring_fill;

uint8_t v5f_proc_acc_gate(volatile v5f_hold_t *h, volatile v5f_proc_gate_t *gate)
{
    float    w[3], off[3], g[3], am2, om2, am, am_lp, am2_lp,
             zw[3], lev_lp, inv_den, qw, qx, qy, qz, ux, uy, uz;
    uint32_t den, j;
    uint8_t  i;
    uint8_t  run = gate->acc_traction;

    /* 重力方向在机体系的表示 R(q)^T * up_ref */
    qw = h->att.q[0]; qx = h->att.q[1]; qy = h->att.q[2]; qz = h->att.q[3];
    ux = h->att.up_ref[0]; uy = h->att.up_ref[1]; uz = h->att.up_ref[2];
    g[0] = (1.0f - 2.0f*(qy*qy + qz*qz))*ux + 2.0f*(qx*qy + qw*qz)*uy + 2.0f*(qx*qz - qw*qy)*uz;
    g[1] = 2.0f*(qx*qy - qw*qz)*ux + (1.0f - 2.0f*(qx*qx + qz*qz))*uy + 2.0f*(qy*qz + qw*qx)*uz;
    g[2] = 2.0f*(qx*qz + qw*qy)*ux + 2.0f*(qy*qz + qw*qx)*uy + (1.0f - 2.0f*(qx*qx + qy*qy))*uz;

    /* off = 用**离线标定常量**换算的比力（g）—— 不含姿态、不含牵引状态 db。
     * w   = off - g            <- 门 A（加速度零偏牵引）判据：对姿态歪敏感，挡线加速度
     * om2 = |off|^2            <- 门 B（倾斜融合）判据：**对姿态歪完全免疫**，只挡线加速度 */
    /* [20260914 改判据] 本行以上的 om2 注释中"门B 倾斜融合判据"已作废：
     * 门B 现用 am2_lp（128 ms 窗 |off| 均值的平方），见函数上方说明。 */

    am2 = 0.0f;
    om2 = 0.0f;
    for (i = 0u; i < 3u; i++) {
        off[i] = ((float)h->imu.accel_lsb[i] - ag_bias0_lsb[i]) / g_v5f_accel_lsb_per_g[i];
        w[i]   = off[i] - g[i];
        am2   += w[i] * w[i];
        om2   += off[i] * off[i];
    }

    /* ---- 门B 判据统计（档内累加，满档后压入环形）---- */
    am = sqrtf(om2);
    s_g_am += am;
    for (i = 0u; i < 3u; i++) {
        s_g_w[i] += ((float)h->imu.gyro_lsb[i] - gb_bias0_lsb[i])
                    / g_v5f_gyro_lsb_per_dps[i];
    }
    s_g_cnt++;
    if (s_g_cnt >= V5F_ATT_TILT_GRAN) {
        s_ring_head = (uint8_t)((s_ring_head + 1u) % V5F_ATT_TILT_NGRAN);
        s_ring_am[s_ring_head] = s_g_am / (float)s_g_cnt;
        for (i = 0u; i < 3u; i++) {
            s_ring_w[s_ring_head][i] = s_g_w[i] / (float)s_g_cnt;
        }
        if (s_ring_fill < V5F_ATT_TILT_NGRAN) s_ring_fill++;
        s_g_am = 0.0f; s_g_w[0] = 0.0f; s_g_w[1] = 0.0f; s_g_w[2] = 0.0f; s_g_cnt = 0u;
    }
    /* 组合均值：分子 = 当前未满档累加 + 各满档均值 * 档长；分母 = 样本数 */
    am_lp = s_g_am;
    zw[0] = s_g_w[0]; zw[1] = s_g_w[1]; zw[2] = s_g_w[2];
    for (j = 0u; j < (uint32_t)s_ring_fill; j++) {
        am_lp += s_ring_am[j] * (float)V5F_ATT_TILT_GRAN;
        zw[0] += s_ring_w[j][0] * (float)V5F_ATT_TILT_GRAN;
        zw[1] += s_ring_w[j][1] * (float)V5F_ATT_TILT_GRAN;
        zw[2] += s_ring_w[j][2] * (float)V5F_ATT_TILT_GRAN;
    }
    den = (uint32_t)s_g_cnt + (uint32_t)s_ring_fill * V5F_ATT_TILT_GRAN;
    inv_den = (den != 0u) ? (1.0f / (float)den) : 0.0f;
    am_lp  *= inv_den;
    zw[0]  *= inv_den; zw[1] *= inv_den; zw[2] *= inv_den;
    am2_lp = am_lp * am_lp;
    lev_lp = fabsf(zw[0]);
    if (fabsf(zw[1]) > lev_lp) lev_lp = fabsf(zw[1]);
    if (fabsf(zw[2]) > lev_lp) lev_lp = fabsf(zw[2]);

    /* ---- 门 A：加速度零偏牵引（判据 |w|）---- */
    if (run != 0u) {                                  /* 牵引中 -> 找冻结（快） */
        if (am2 > V5F_ACG_ALIN_OFF2) {
            if (++s_off_cnt >= V5F_ACG_DEB_OFF) { run = 0u; s_off_cnt = 0u; }
        } else {
            s_off_cnt = 0u;
        }
    } else {                                          /* 冻结中 -> 找牵引（慢） */
        if (am2 < V5F_ACG_ALIN_ON2) {
            if (++s_on_cnt >= V5F_ACG_DEB_ON) { run = 1u; s_on_cnt = 0u; }
        } else {
            s_on_cnt = 0u;
        }
    }

    gate->acc_traction          = run;
    gate->acc_rollback_granules = V5F_ACG_ROLLBACK_GRANULES;

    /* [20260914 改判据] 门B 不再用 om2（瞬时），改用 am2_lp = (128 ms 窗 |off| 均值)^2。
     * 旧注释里"为什么需要 is_static"一段已作废，保留仅为记录历史推理。 */


    /* ---- 门 B：重力/倾斜融合（判据 |off|，**独立于门 A**）----
     * 为什么不能复用门 A：门 A 的判据 w = off - g 含姿态误差项，姿态一歪 w 就大、
     *   门 A 就关；而倾斜融合恰恰是唯一能把姿态转回来的环 —— 用它做门就是交叉门控
     *   死锁（实测 061955：姿态歪 6.725 deg -> |w|=107 mg -> 门 A 关 -> 倾斜融合
     *   一起被关 -> 姿态永远回不来）。|off| 只反映比力模长，姿态歪不改变模长，
     *   所以这个门不会被姿态误差关掉。
     * 为什么还要 is_static：|off|-1g 对**水平**线加速度只有二阶敏感（a_h = 1 m/s^2
     *   时仅 0.48 mg），光看模长挡不住水平平动，必须叠陀螺判静。
     * 顺序语义与门 A 不同：门 B 不加去抖 —— 它的两个条件本来就是慢变量（判静有
     *   自己的防抖），再加去抖只会让姿态在机动后多等一段。 */
    /* 判据：256 ms 窗 |off| 均值 ∈ 1 g +-3%  且  同窗净 |w| < 2 dps。
     * 上界 1.03 g 对应水平线加速度 2.4 m/s^2（方向偏差 atan(2.4/g)=13.8 deg）；
     * 净 |w| 条件挡住真转动；20 g 冲击会同时顶出带外 + 抬高净 |w|，双保险关门。 */
    gate->att_tilt = ((am2_lp > V5F_ATT_TILT_ALIM_LO2) &&
                      (am2_lp < V5F_ATT_TILT_ALIM_HI2) &&
                      (lev_lp < V5F_ATT_TILT_LEV_DPS)) ? 1u : 0u;

    /* ---- 门 C：速度的**软 ZUPT 置信度**（0..255，连续值，不是 0/1）----
     * 两个证据相乘：陀螺静（level 小）+ 比力模长接近 1 g。
     * 然后按"距上次高置信锚点的时间"抬高下限 —— 对应文献里"损失因子随距上次
     * 零速观测的时间衰减"的自适应阈值：太久没有锚点就强制快衰减，幽灵速度不可能
     * 长期存在；有锚点时又不至于滥加。 */
    {
        float cg, ca, lam;

        cg = 1.0f - h->stat.level_dps / V5F_VEL_ZUPT_LEV_DPS;
        if (cg < 0.0f) cg = 0.0f; else if (cg > 1.0f) cg = 1.0f;

        ca = 1.0f - fabsf(sqrtf(om2) - 1.0f) / V5F_VEL_ZUPT_AMAG_G;
        if (ca < 0.0f) ca = 0.0f; else if (ca > 1.0f) ca = 1.0f;

        /* [20260914 删除] 这里原先还有"无锚点 5 s 就强制把 lam 顶到 1"的下限。
         * 实测 20260914_063706：抬到 +0.10 m 停住时 Z 只积到 +0.0541（真值 +0.10），
         * 因为正常飞行本来就常年没有"准静止锚点"，该下限会把真实速度按 tau=0.2 s
         * 压成 0。删掉后 Z@5.6 = +0.1051、末位置 p_z 由 +0.0342 回到 -0.0016，
         * |p| 0.0436 -> 0.0391；070300 上几乎无损（0.2695 -> 0.2856）。 */
        lam = cg * ca;

        gate->vel_soft = (uint8_t)(lam * 255.0f + 0.5f);
    }

    return V5F_PROC_OK;
}
