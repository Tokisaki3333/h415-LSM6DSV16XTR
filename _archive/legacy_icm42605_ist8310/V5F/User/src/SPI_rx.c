#include "SPI_rx.h"
#include "mipc_shm.h"               /* g_shm：双核共享区（V3F 小核定义） */
#include "hardware.h"               /* ch32h417.h + debug.h：外设寄存器 / 中断 / Core_ID_V5F */
#include "spi_hw.h"                 /* SPI1_DMA_Rx_Setup */
#include "sys_clk.h"                /* GetTime64_Us */
#include "v5f_proc.h"               /* 处理链：处理函数原型 + 门控类型 */
#include "usbd_compatibility_hid.h" /* hid_up_enqueue：JustFloat 帧入 CDC 上行环 */
#include <string.h>                 /* memcpy：float 按字节直拷 */
#include <math.h>                   /* sqrtf：-Ofast 下一条 fsqrt.s，不引入 libm 调用 */

/* ==================== 内部私有变量 ==================== */
static volatile uint8_t rxfifo[512];

/* DMA1_Channel2 中断单次执行耗时历史最劣值 us（备用，暂无消费者） */
/* 本 ISR 的耗时（us）。注意 justfloat_report() 在 ISR 里跑的位置早于计时结束那行，
 * 所以上报的是**上一帧**的耗时（一帧滞后，对性能监控无影响）。 */
static volatile uint32_t s_dma1_irq_us = 0;
/* VER=117 compute-load probe: ISR only accumulates; printed from main loop */
static volatile uint32_t s_pr_n = 0u, s_pr_sum = 0u, s_pr_max = 0u, s_pr_ovr = 0u;

/* ==================== VER=97 陀螺削顶统计 ====================
 * 判定与统计都在 justfloat_report 里每帧做（此时 imu.gyro_lsb[] 已是本帧原始值），
 * 与日志 flags bit12~15 同源，不会不同步。 */
static uint16_t s_clip_win_cnt;      /* 本窗削顶帧数 */
static uint16_t s_clip_win_n;        /* 本窗总帧数 */
static uint8_t  s_clip_win_ax;       /* 本窗削顶过的轴 bit0..2 */
static uint16_t s_clip_pm;           /* 上一窗削顶千分比 */
static uint8_t  s_clip_ax;           /* 上一窗轴位 */
static uint32_t s_clip_total;        /* 上电以来削顶帧数 */

/* 上报用：当帧 DRDY 间隔（通道 27）。上报在 ISR 内、与处理链同帧，故这里的时间戳
 * 就是本帧 fresh.drdy_tick；差值乘 1e-2 得 us（1 tick = 10 ns）。 */
static uint64_t s_rep_last_tick = 0;
static uint8_t  s_rep_tick_ok   = 0;

/* ==================== 气压计软件 OSR 环形缓冲 ====================
 * BMP388 按用户要求保持 OSR x1 + 硬件 IIR 关闭（0x1C=0x00、0x1F 从不写），
 * 因为只有这样转换时间才短到能与 IST8310 **流水线**读取。
 * 噪声在这里平掉：实测单样本 5.0 Pa，W=32 (0.165 s) 后 1.0~1.5 Pa。
 * 用 O(1) 滑动和；W 取 1 时等价于不平均。 */
#if (V5F_BARO_AVG_W > 1u)
static float    s_bp_ring[V5F_BARO_AVG_W];
static float    s_bp_sum = 0.0f;
static uint16_t s_bp_head = 0u;
static uint16_t s_bp_n = 0u;
#endif

/* ==================== 磁力计标定常量实体化 ====================
 * 数值在 v5f_tune.h 的 H 组（放那儿是因为它随装机变）。这里只做一次实体化，
 * 避免那张表被 v5f_proc.h 广泛 include 时在每个编译单元都生成一份没人用的数组。
 * 用 #if 包住也是为了让 V5F_MAG_CAL_EN=0 时不留未使用的 static 变量（-Wunused）。 */
#if (V5F_MAG_CAL_EN != 0u)
static const float s_mag_A[3][3] = V5F_MAG_A_INIT;
static const float s_mag_C[3]    = V5F_MAG_C_INIT;
#endif

/* ==================== 数据保持器实例 ==================== */
volatile v5f_hold_t g_v5f_hold = { 0 };

/* ==================== 处理链门控实例 ====================
 * 由处理函数 2（动静判定）每帧写入，处理函数 1 下一帧读取；
 * 初值：按静止、回退 24 格（处理函数 2 第一帧即接管）。 */
volatile v5f_proc_gate_t g_v5f_proc_gate = {
    V5F_PROC_GATE_INIT_STATIC,   /* is_static            */
    V5F_PROC_ROLLBACK_GRANULES,  /* rollback_granules    */
    V5F_PROC_GATE_INIT_STATIC,   /* acc_traction         */
    V5F_ACG_ROLLBACK_GRANULES,   /* acc_rollback_granules*/
    V5F_PROC_GATE_INIT_STATIC,   /* att_tilt             */
    255u };                      /* vel_soft = 1.0 -> 上电先按"静止"快衰减 */

/* ==================== 处理链错误码锁存（0 = 未出错） ==================== */
static volatile uint8_t s_proc_err_gyro_bias = V5F_PROC_OK;
static volatile uint8_t s_proc_err_static    = V5F_PROC_OK;
static volatile uint8_t s_proc_err_attitude  = V5F_PROC_OK;
static volatile uint8_t s_proc_err_accel_cal = V5F_PROC_OK;
static volatile uint8_t s_proc_err_acc_gate  = V5F_PROC_OK;
static volatile uint8_t s_proc_err_velocity  = V5F_PROC_OK;
static volatile uint8_t s_proc_err_ekf       = V5F_PROC_OK;
static volatile uint8_t s_proc_err_ekf_gate  = V5F_PROC_OK;

/* ==================== 各共享通道 cnt 快照（变化检测用） ==================== */
static uint32_t s_last_gyro_cnt = 0, s_last_ist_cnt = 0, s_last_bmp_cnt = 0;
static uint32_t s_last_gps_rmc_cnt = 0, s_last_gps_gga_cnt = 0, s_last_gps_gsa_cnt = 0;
static uint32_t s_last_gps_gsv_cnt = 0;

/* ==================== 保持器维护 ====================
 * 每通道都是"检测到 cnt 变化 → __sync_synchronize() → 读 ts/数据/flags"（与写端 release 配对；
 * RISC-V 弱内存序下 load 可重排，缺此屏障会读到上一帧）。低频文本由主循环直接读 g_shm->log。 */
static void hold_poll(void)
{
    uint32_t c;

    /* 陀螺通道：只有 DRDY 时间戳（六轴原始值走 SPI DMA 帧） */
    c = g_shm->gyro.hdr.cnt;
    if (c != s_last_gyro_cnt) {
        s_last_gyro_cnt = c;
        __sync_synchronize();
        g_v5f_hold.imu.fresh.drdy_tick = shm_chan_ts_read(&g_shm->gyro.hdr);
    }

    /* IST8310（磁力计）：原始 LSB 直通 */
    c = g_shm->ist.hdr.cnt;
    if (c != s_last_ist_cnt) {
        s_last_ist_cnt = c;
        __sync_synchronize();
        g_v5f_hold.mag.fresh.drdy_tick = shm_chan_ts_read(&g_shm->ist.hdr);
        g_v5f_hold.mag.lsb[0] = g_shm->ist.mx;
        g_v5f_hold.mag.lsb[1] = g_shm->ist.my;
        g_v5f_hold.mag.lsb[2] = g_shm->ist.mz;

        /* 线性标定 + 污染门（参数与依据见 v5f_tune.h 的 H 组）。
         * y = A*raw + C，A 里已含 z 取反。raw **不做任何裁剪** —— 越界样本靠 y 的
         * 模长门判掉（mag.ok=0），而不是靠截断掩盖成"看起来正常"。
         * f 统一归一化成单位矢量，消费方不必再除模长；mag_norm 保留归一化前的模长
         * 作为门的观测量。这段只在磁样本到达时跑（约 187 Hz），不在每帧的路径上。 */
        {
            float yx, yy, yz, n, s;
#if (V5F_MAG_CAL_EN != 0u)
            yx = s_mag_A[0][0] * (float)g_v5f_hold.mag.lsb[0]
               + s_mag_A[0][1] * (float)g_v5f_hold.mag.lsb[1]
               + s_mag_A[0][2] * (float)g_v5f_hold.mag.lsb[2] + s_mag_C[0];
            yy = s_mag_A[1][0] * (float)g_v5f_hold.mag.lsb[0]
               + s_mag_A[1][1] * (float)g_v5f_hold.mag.lsb[1]
               + s_mag_A[1][2] * (float)g_v5f_hold.mag.lsb[2] + s_mag_C[1];
            yz = s_mag_A[2][0] * (float)g_v5f_hold.mag.lsb[0]
               + s_mag_A[2][1] * (float)g_v5f_hold.mag.lsb[1]
               + s_mag_A[2][2] * (float)g_v5f_hold.mag.lsb[2] + s_mag_C[2];
#else
            yx = (float)g_v5f_hold.mag.lsb[0];   /* 直通对照 */
            yy = (float)g_v5f_hold.mag.lsb[1];
            yz = (float)g_v5f_hold.mag.lsb[2];
#endif
            n = sqrtf(yx * yx + yy * yy + yz * yz);
            s = (n > 1e-6f) ? (1.0f / n) : 0.0f;
            g_v5f_hold.mag.f[0] = yx * s;
            g_v5f_hold.mag.f[1] = yy * s;
            g_v5f_hold.mag.f[2] = yz * s;
            g_v5f_hold.mag.mag_norm = n;
#if (V5F_MAG_CAL_EN != 0u)
            g_v5f_hold.mag.ok = (uint8_t)((n > (1.0f - V5F_MAG_ERR_LIM)) &&
                                          (n < (1.0f + V5F_MAG_ERR_LIM)));
#else
            g_v5f_hold.mag.ok = 1u;             /* 直通模式下模长无意义，不做门 */
#endif
        }

        /* ---- 输出“磁力计自己认为的维度”（偏航）—— 只读，不参与修正 ----
         * 用的是**上一帧**的姿态（hold_poll 在处理链之前）：滞后 124.7 us，
         * 2000 dps 下也只有 0.07 度，可忽略。放在这里而不是每帧算，是因为
         * mag.f 只有 ~187 Hz 更新，每帧用新 q 去转它会产生假跳变。
         * ★ 不写回 att.q：磁力计不闭环。 */
        if ((g_v5f_hold.att.valid != 0u) && (g_v5f_hold.att.up_ref_ok != 0u)) {
            float qw = g_v5f_hold.att.q[0], qx = g_v5f_hold.att.q[1];
            float qy = g_v5f_hold.att.q[2], qz = g_v5f_hold.att.q[3];
            float bx, by, bz, bn;
            float ax, ay, az;
            bx = (1.0f - 2.0f*(qy*qy + qz*qz))*g_v5f_hold.mag.f[0]
               + 2.0f*(qx*qy - qw*qz)*g_v5f_hold.mag.f[1]
               + 2.0f*(qx*qz + qw*qy)*g_v5f_hold.mag.f[2];
            by = 2.0f*(qx*qy + qw*qz)*g_v5f_hold.mag.f[0]
               + (1.0f - 2.0f*(qx*qx + qz*qz))*g_v5f_hold.mag.f[1]
               + 2.0f*(qy*qz - qw*qx)*g_v5f_hold.mag.f[2];
            bz = 2.0f*(qx*qz - qw*qy)*g_v5f_hold.mag.f[0]
               + 2.0f*(qy*qz + qw*qx)*g_v5f_hold.mag.f[1]
               + (1.0f - 2.0f*(qx*qx + qy*qy))*g_v5f_hold.mag.f[2];
            bn = sqrtf(bx*bx + by*by + bz*bz);
            if (bn > 1e-6f) {
                float bs = 1.0f/bn;
                bx *= bs; by *= bs; bz *= bs;
            } else {
                bx = 0.0f; by = 0.0f; bz = 1.0f;
            }
            g_v5f_hold.mag.Bw[0] = bx;
            g_v5f_hold.mag.Bw[1] = by;
            g_v5f_hold.mag.Bw[2] = bz;
            g_v5f_hold.mag.psi_deg      = atan2f(bx, by) * 57.2957795f;
            g_v5f_hold.mag.psi_true_deg = g_v5f_hold.mag.psi_deg + V5F_MAG_DECL_DEG;
            ax = g_v5f_hold.imu.a_lin[0];
            ay = g_v5f_hold.imu.a_lin[1];
            az = g_v5f_hold.imu.a_lin[2];
            g_v5f_hold.mag.trust = (uint8_t)((g_v5f_hold.mag.ok != 0u) &&
                ((ax*ax + ay*ay + az*az) < (V5F_MAG_ALIN_LIM * V5F_MAG_ALIN_LIM)));
        } else {
            /* 姿态还没有效（上电初期）：置不可信，不把垃圾喂给 EKF */
            g_v5f_hold.mag.trust = 0u;
        }

        g_v5f_hold.mag.fresh.new_data = 1;      /* 数据写完后才置新数据信号 */
    }

    /* BMP388（气压计）：float 直通 */
    c = g_shm->bmp.hdr.cnt;
    if (c != s_last_bmp_cnt) {
        s_last_bmp_cnt = c;
        __sync_synchronize();
        g_v5f_hold.baro.fresh.drdy_tick = shm_chan_ts_read(&g_shm->bmp.hdr);
        g_v5f_hold.baro.temp_celsius  = g_shm->bmp.temp_celsius;   /* 共享区已是 float，无需换算 */
        g_v5f_hold.baro.press_pascal  = g_shm->bmp.press_pascal;
        /* 软件 OSR（见上方环形缓冲说明） */
#if (V5F_BARO_AVG_W > 1u)
        {
            float pv = g_v5f_hold.baro.press_pascal;
            if (s_bp_n < V5F_BARO_AVG_W) {
                s_bp_ring[s_bp_n++] = pv;
                s_bp_sum += pv;
            } else {
                s_bp_sum += pv - s_bp_ring[s_bp_head];
                s_bp_ring[s_bp_head] = pv;
                s_bp_head = (uint16_t)((s_bp_head + 1u) % V5F_BARO_AVG_W);
            }
            g_v5f_hold.baro.press_avg_pascal = s_bp_sum / (float)s_bp_n;
        }
#else
        g_v5f_hold.baro.press_avg_pascal = g_v5f_hold.baro.press_pascal;
#endif
        g_v5f_hold.baro.fresh.new_data  = 1;
    }

    /* GPS RMC：经纬度保持共享区的 10^-7 ° 定标整数（float 精度不够），速度换算成 m/s */
    c = g_shm->gps_rmc.hdr.cnt;
    if (c != s_last_gps_rmc_cnt) {
        s_last_gps_rmc_cnt = c;
        __sync_synchronize();  
        g_v5f_hold.gps_rmc.fresh.drdy_tick = shm_chan_ts_read(&g_shm->gps_rmc.hdr);
        g_v5f_hold.gps_rmc.fresh.flags     = g_shm->gps_rmc.flags;
        g_v5f_hold.gps_rmc.lat_e7        = g_shm->gps_rmc.lat_e7;
        g_v5f_hold.gps_rmc.lon_e7        = g_shm->gps_rmc.lon_e7;
        g_v5f_hold.gps_rmc.speed_mps     = g_shm->gps_rmc.speed_mps;
        g_v5f_hold.gps_rmc.status         = g_shm->gps_rmc.status;
        g_v5f_hold.gps_rmc.date_ddmmyy    = g_shm->gps_rmc.date_ddmmyy;
        g_v5f_hold.gps_rmc.course_deg     = g_shm->gps_rmc.course_deg;
        g_v5f_hold.gps_rmc.magvar_deg     = g_shm->gps_rmc.magvar_deg;
        g_v5f_hold.gps_rmc.fresh.new_data = 1;
    }

    /* GPS GGA：高度 cm→m、HDOP ×100→无量纲 float，定位质量/星数本就是小整数 */
    c = g_shm->gps_gga.hdr.cnt;
    if (c != s_last_gps_gga_cnt) {
        s_last_gps_gga_cnt = c;
        __sync_synchronize();  
        g_v5f_hold.gps_gga.fresh.drdy_tick = shm_chan_ts_read(&g_shm->gps_gga.hdr);
        g_v5f_hold.gps_gga.fresh.flags     = g_shm->gps_gga.flags;
        g_v5f_hold.gps_gga.alt_m        = (float)g_shm->gps_gga.alt_cm * 1e-2f;
        g_v5f_hold.gps_gga.fix_quality  = g_shm->gps_gga.quality;
        g_v5f_hold.gps_gga.sat_num      = g_shm->gps_gga.sv;
        g_v5f_hold.gps_gga.hdop         = (float)g_shm->gps_gga.hdop_x100 * 1e-2f;
        g_v5f_hold.gps_gga.fresh.new_data = 1;
    }

    /* GPS GSA：PDOP/VDOP（共享区 ×100 定标整数 → 无量纲 float） */
    c = g_shm->gps_gsa.hdr.cnt;
    if (c != s_last_gps_gsa_cnt) {
        s_last_gps_gsa_cnt = c;
        __sync_synchronize();               
        g_v5f_hold.gps_gsa.fresh.drdy_tick = shm_chan_ts_read(&g_shm->gps_gsa.hdr);
        g_v5f_hold.gps_gsa.fresh.flags     = g_shm->gps_gsa.flags;
        g_v5f_hold.gps_gsa.pdop = (float)g_shm->gps_gsa.pdop_x100 * 1e-2f;
        g_v5f_hold.gps_gsa.vdop = (float)g_shm->gps_gsa.vdop_x100 * 1e-2f;
        g_v5f_hold.gps_gsa.fresh.new_data = 1;
    }

    /* GPS GSV：C/N0 统计（每个 GSV 轮次发一次） */
    c = g_shm->gps_gsv.hdr.cnt;
    if (c != s_last_gps_gsv_cnt) {
        s_last_gps_gsv_cnt = c;
        __sync_synchronize();
        g_v5f_hold.gps_gsv.fresh.drdy_tick = shm_chan_ts_read(&g_shm->gps_gsv.hdr);
        g_v5f_hold.gps_gsv.fresh.flags     = g_shm->gps_gsv.flags;
        g_v5f_hold.gps_gsv.sats_view    = g_shm->gps_gsv.sats_view;
        g_v5f_hold.gps_gsv.snr_n        = g_shm->gps_gsv.snr_n;
        g_v5f_hold.gps_gsv.snr_min      = g_shm->gps_gsv.snr_min;
        g_v5f_hold.gps_gsv.talkers      = g_shm->gps_gsv.talkers;
        g_v5f_hold.gps_gsv.snr_avg_dbhz = (float)g_shm->gps_gsv.snr_avg_x10 * 1e-1f;
        g_v5f_hold.gps_gsv.fresh.new_data = 1;
    }
}

/* ==================== JustFloat 上报帧组装（DMA-RX 内） ====================
 * 帧头 A5 5A + 帧长 2B + 80 路 float 载荷 320 B + 帧尾 5A A5 = 326 字节：
 *   1~4   姿态四元数 w, x, y, z（机体 -> 导航，右手系，见处理函数 3）
 *   5~7   世界系（导航系）速度 m/s，见处理函数 6
 *   8     判静统计量 stat.level_dps —— 陀螺侧门控的输入
 *   9     陀螺静动判定 stat.is_static —— 陀螺零偏追踪门
 *   10    加速度零偏追踪门 gate.acc_traction（1 = 牵引，0 = 冻结）
 *   11~13 牵引驱动量 a_lin[3]（g）—— 没被消掉的残余，速度漂移的直接来源
 *   14~16 加速度零偏估计 accel_bias_g[3]（g）—— 看牵引是否在收敛/跑飞
 *   17    打包标志位：bit0 bias_ok  bit1 up_ref_ok  bit2 att.valid  bit3 vel.valid
 *         bit4 corr_valid  bit5 acc_valid  bit6 stat.valid  bit7 stat.changed
 *         bit8 陀螺本帧回退  bit9 加速度本帧回退  bit10 判静启动阶段旁路门(stat.ac_bypass)
 *   77~78 处理函数 2 的启动阶段旁路判据
 *        77  stat.e_ac      三轴交流能量 dps^2（W 帧滑窗方差之和）
 *        78  stat.ac_bypass 旁路门（只在 bias_ok 锁存前参与；只做"或"）
 *   79    本轮 GSV 的 talker 数（轮次聚合是否正常）
 *   80    构建指纹 fw_tag = (VER<<16)|(通道数<<8)|开关位；刷完先核对它
 *   18    上一帧 DMA1 ISR 耗时（us）—— 性能监控，可看逐帧分布与峰值。
 *         当帧值拿不到：上报发生在 ISR 里、早于该帧计时结束那一行，所以发上一帧的。
 *
 *   ---- 19 起：原始量 + 时间基 + 竖直参考，供 PC 端离线重建整条处理链
 *   19~21 陀螺原始 LSB imu.gyro_lsb[3]（16.4 LSB/(deg/s)，±32768 = ±2000 deg/s）。
 *         ★ 陀螺是整条链的主传感器 —— 姿态由它积分得出，故原始量排在最前。
 *         有了它，PC 才能从原始值重跑 处理函数 1（零偏/逐轴标度/交叉耦合）与
 *         处理函数 2 的判静统计量，而不是只能看已经算好的 gyro_dps
 *         （那是被零偏追踪环改过的导出量，无法用来独立复核追踪环本身）。
 *   22~24 加速度原始 LSB imu.accel_lsb[3]（2048 LSB/g，±32768 = ±16 g）。
 *         与 20~22 合起来，整条处理链（校正/标定/牵引/门控/姿态/速度）都能在 PC 上重放。
 *   25    陀螺温度 imu.temp_celsius（degC）—— 零偏随温度漂移，是零偏建模的解释变量。
 *   26    当帧 DRDY 间隔 dt（us）—— 固件积分用的就是它。
 *         ★ 必须上报：PC 端不能假设帧率恒定。1 tick = 10 ns，故 dt_us = tick 差 * 1e-2，
 *         正常约 124.66 us；明显偏大 = 上游丢帧，等于 0 = 本帧没拿到基准或时间戳回退。
 *
 *   27 起：各处理函数的中间结果 + 内部状态，供 PC 端逐环复核 ----
 *   27~29 imu.gyro_dps[3] —— 处理函数 1 的零偏校正输出（deg/s，含逐轴标度与对称交叉）。
 *         这是处理函数 2（判静统计量）与处理函数 3（四元数积分）**真正的输入**。
 *   30~32 imu.gyro_bias_dps[3] —— 处理函数 1 内部的零偏估计本身（deg/s）。
 *         ★ 唯一的**内部状态**，从任何其它通道都推不出来。陀螺零偏牵引到底有没有在
 *         收敛、收敛到多少，只能靠它；没有它就只能拿 a_lin 去反推，而那是会推错的
 *         （曾据此误判"牵引没跑"，实际它跑了 65.9% 的帧、残余 1.23 deg/h）。
 *   33~35 imu.accel_g[3] —— 处理函数 4 用**牵引后**零偏重新算出的比力（g）。
 *         ★ 注意它与 a_lin（12~14）不同源：a_lin 用的是牵引**前**的零偏（处理函数 4
 *         的 2b 步，为了不带自引用），两者正好差一步 PI 更新。上报它才能把这一步
 *         差看清楚，否则 a_lin 的逐帧预算总差一格。
 *   36    imu.bias_evidence_gran —— 累积静止证据（20 ms 粒度数）。
 *         它是处理函数 2 查判静阈值表的索引，也是回退时被扣减的量：
 *         is_static 为什么开/关，用它 + level_dps 就能完整解释。
 *   37    gate.att_tilt —— **重力/倾斜融合门**（处理函数 3 的倾斜环用）。
 *         ★ 与 11 的 gate.acc_traction 是**两个不同的门**：11 用 |w|（对姿态歪敏感），
 *         37 用 |off| 在 1 g ±5%（对姿态歪免疫）。曾经把两者混用，结果是交叉门控
 *         死锁（061955：姿态歪 6.7 deg -> 11 关 -> 倾斜融合被一起关掉 -> 姿态回不来）。
 *
 *
 *   41 路 196 B = 1.57 MB/s（追加地磁原始 3 路）；49 路 200 B = 1.60 MB/s
 *   （再追加磁样本序号）；54 路 220 B = 1.77 MB/s（再追加校准后地磁 5 路）；
 *   51 路 244 B = 1.96 MB/s（再追加磁力计偏航维度输出 6 路）；
 *   65 路 316 B = 2.53 MB/s（再追加气压计+GPS 18 路）；
 *   70 路 340 B = 2.73 MB/s（再追加经纬度的精确形式 4 路）；
 *   ★ 92 路 372 B = 2.98 MB/s（再追加 C/N0 统计 5 路 + 气压平均 1 路
 *     + 判静启动阶段旁路 2 路：交流能量 e_ac + 旁路门 ac_bypass）。
 *   上行是 USBHS CDC bulk EP2（HS 512 B/包，8 KB 环形缓冲 = 3.2 ms 余量）。
 *   HS bulk 理论上限 53 MB/s，2.53 MB/s 占用 4.8%，余量充足。
 *   注：GPS 只有 1~10 Hz 却按 8 kHz 重复发，冗余约 99.9%——这 18 路里的
 *   0.57 MB/s 几乎全是重复。单格式 JustFloat 下无法避免；
 *   靠序号识别新样本即可，带宽也还在余量内。
 *
 * ★ 为什么要发这些：链里有多个 0/1 门（陀螺零偏追踪门 / 加速度零偏追踪门），
 *   只发姿态+速度时无法区分"门没开"和"门开了但没效果"，排查只能靠猜。
 *
 * 为什么发姿态与速度：它们正是飞控闭环需要的两个量 —— 姿态环（陀螺积分 + 竖直持续修正
 * + 静止牵引）与速度环（比力积分 + 自行向 0 衰减）。位置是速度的二次积分、
 * 误差跨锚点累积，本固件不出位置，所以也不上报位置。
 *
 * 帧尾是上位机(VOFA+ JustFloat)的切帧标志，必须在；全 float 结构，
 * 不加包头/长度/校验等任何非 float 字段，否则上位机解析不了。
 * RISC-V 小端，float 按字节直拷即 IEEE754 小端，无字节序换算。
 * ★ 改通道数必须同步改上位机解析（tools/calib/jf_load.py 按帧长自动识别，不用改）。 */
/* ---- 46~48 追加：IST8310 地磁原始 LSB（0.3 uT/LSB，16 位补码）----
 * 为什么要上报：磁是唯一能给**绝对航向**的传感器，而机上硬铁/软铁必须在 PC 端用
 *   "多姿态静态点拟合椭球"来标定（工具链已有同款做法：tools/calib/accel_ellipse_cal.py
 *   就是拿原始 LSB 拟合椭圆心/半轴）。没有原始 LSB 就没法标，只能看到已被污染的航向。
 * 更新率：IST8310 是**单次测量**（i2c_soft.c 写 0x0A=0x01），由 V3F 主循环每次触发一次，
 *   约 180~200 Hz —— 所以 8 kHz 上报里相邻帧重复属正常，不是丢数据。
 * 磁不参与任何牵引/融合环，所以没有"牵引门"；但它有**样本有效性门**（见 54）。
 *   42    IST8310 样本序号（= 共享区 ist 通道的 cnt，单调递增）。
 *         ★ 磁只有 ~180 Hz，且**只有变化沿有效** —— 中间约 45 帧是同一值的重复。
 *         只报 46~48 的话，"相邻两拍数值恰好相同"会被误判成没有新样本：实测
 *         1 LSB 分辨率 / 0.77 LSB 噪声下会漏掉约 2% 的沿，把 180 Hz 读成 176.4 Hz。
 *         报序号就永不漏沿，而且序号跳 >1 直接说明共享区那侧丢了样本。
 *         注：float32 精确表示 < 2^24 的整数，180 Hz 下约 25.8 小时后才会按 2 递增。
 * ★ 1~48 列号一个都不动，新量只追加在尾部。
 *
 * 43~46 追加：校准后的地磁单位方向 + 污染门（v5f_tune.h H 组的标定输出）----
 *   43~45 mag.f[3] —— y = V5F_MAG_A * raw + V5F_MAG_C，**已归一化**。
 *         机体坐标系，指向磁北且向下；导航系 x=右 y=前 z=上。
 *   ★ 这是**唯一能给绝对航向的量**：PC 端 atan2(f[0], f[1]) 即磁航向，
 *     不必自己重跑标定；同时它也是"固件里的标定到底有没有生效"的唯一证据
 *     （对照 V5F_MAG_CAL_EN=0 的直通录制即可 A/B）。
 *   46    mag.mag_norm —— 归一化**前**的模长，理论 1.0。
 *         ★ 它同时是**标定质量的独立读数**：地磁强度是地点常数，标定对了它就
 *         恒为 1，离散度直接反映剩余畘变。实测干净会话离散 2.0~4.8%，桌面污染
 *         会话 6.1~6.7%，整段被外部磁源覆盖时可达 1.4~2.9。
 *         它也把"是磁力计坏了还是环境坏了"分开：|y| 偏但方向对 = 环境；
 *         方向也乱 = 磁力计或标定。 */
#define JF_CH_NUM     162u   /* **运行时**实际写入的列数。不是估的：tools/calib/count_cols.py
                              * 逐行数 ch[c++] 得到静态 114，减掉 DRDY 间隔那条 if/else
                              * 链的 2 个未执行分支 = 112。ch[] 是栈上数组，多写一格就是
                              * 栈踩踏而编译器一个字都不会说 —— 改列必须用那个脚本复核。*/

/* 构建指纹，编码见 v5f_tune.h 的 K 组。改固件必须 bump V5F_FW_VER。 */
#define JF_FW_TAG  ((float)(((uint32_t)V5F_FW_VER << 16) | ((uint32_t)JF_CH_NUM << 8) \
                  | ((uint32_t)V5F_EKF_EN << 0) | ((uint32_t)V5F_MAG_CAL_EN << 1) \
                  | ((uint32_t)V5F_DET_AC_EN << 2)))
#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* VER=83: 602 = 帧头2+帧长2+载荷596+帧尾2 */


/* 载荷里出现过非有限 float 的累计次数（粘滞：发生即置位，经 flags bit11 上报）。
 * 见 justfloat_report 末尾的按位保护。 */
static uint16_t s_bad_f32;

static void justfloat_report(void)
{
    /* ★ static，不是局部：本函数在 DMA1 中断里，是整条链最深的一层，
     *   而链接脚本给的栈只有 2 KB（Link_v5f.ld 的 __stack_size）—— 光这两个数组
     *   就 902 B，加上处理链（尤其 EKF 的 16x16 中间量）把栈踩穿了。
     *   实测症状：每秒一两次、单帧的 ekf 上报块被清零（|q| -> 0.003、
     *   sigma_yaw -> 185 度、press_avg -> 0），以及 0.013% 的坏帧。
     *   只有这一个中断会调用本函数（单线程），所以 static 安全、且是零代价的。 */
    static float   ch[JF_CH_NUM];
    static uint8_t buf[JF_FRAME_LEN];
    uint32_t i;   /* 列数已达 111：计数器类型必须 >= 上界，见 uint8_t 死循环那次事故 */
    uint8_t clip_any = 0u, clip_ax = 0u;   /* VER=97 本帧削顶：任意轴 / 轴位 */
    uint8_t c = 0u;     /* 通道自增计数器：顺序即列号，增删通道不必手改索引（列号见上方说明） */

extern volatile float g_v5f_ekf_mb_deg;   /* VER=132 */

#if (V5F_CDC_QUAT_ONLY != 0u)
    /* ---- VER=100: CDC(EP2 bulk IN) 上报改为 EKF 四元数, JustFloat 标准格式 ----
     * 载荷 = 5 x float32(小端): q[4] + 地磁入口 + 帧尾 00 00 80 7F  => 24 字节/帧, 每个 IMU 帧一帧(8 kHz)。
     * 只报 EKF 姿态; 不报 att.q、不报其它通道。帧内无版本指纹(JustFloat 无标签位)。
     * 切回旧 162 通道日志: v5f_tune.h 里 V5F_CDC_QUAT_ONLY 改 0u。 */
    {
        static uint8_t qbuf[28];        /* VER=132: 6 float + 4 B 尾 */        /* VER=124: 5 float(四元数+地磁入口) + 4 B 帧尾; static 避免占中断栈 */
        float qv[4], mm;
        qv[0] = g_v5f_hold.ekf.q[0];
        qv[1] = g_v5f_hold.ekf.q[1];
        qv[2] = g_v5f_hold.ekf.q[2];
        qv[3] = g_v5f_hold.ekf.q[3];
        memcpy(qbuf, qv, 16u);
        /* VER=124 第 5 路：当前地磁入口（0=未修正 1=投影(只姿态角) 2=矢量）。
         * 放在帧尾**之前** —— JustFloat 要求 00 00 80 7F 是最后 4 字节。 */
        mm = g_v5f_hold.ekf.mag_mode;
        memcpy(qbuf + 16u, &mm, 4u);
        {   /* VER=132 第 6 路：地磁偏航偏置 b_m（deg） */
            float bm = g_v5f_ekf_mb_deg;
            memcpy(qbuf + 20u, &bm, 4u);
        }
        qbuf[24] = 0x00u; qbuf[25] = 0x00u; qbuf[26] = 0x80u; qbuf[27] = 0x7Fu;
        (void)hid_up_enqueue(qbuf, 28u);
    }
    return;
#endif
    /* ---- VER=101 调试模式抽帧 ----
     * 654B/帧 × 8.08kHz = 5.3MB/s，USB FS 与上位机都吃不下；
     * 按 V5F_CDC_DEBUG_DIV 抽帧(默认 24 -> 337Hz)，放最前面省 CPU。
     * 注意：clip 统计窗口(1000 帧)随之从 125ms 变成 ~3s；
     *       报帧的 tick 列(tk - s_rep_last_tick)也变为跨 24 帧的间隔。 */
#if (V5F_CDC_DEBUG_DIV > 1u)
    {
        static uint32_t s_div_n;
        if (++s_div_n < (uint32_t)V5F_CDC_DEBUG_DIV) return;
        s_div_n = 0u;
    }
#endif

    for (i = 0u; i < 4u; i++) ch[c++]     = g_v5f_hold.att.q[i];
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.vel.v_nav[i];

    ch[c++]  = g_v5f_hold.stat.level_dps;            /* 陀螺侧门控的输入 */
    ch[c++]  = (float)g_v5f_hold.stat.is_static;     /* 速度归零门 + 陀螺零偏追踪门 */
    ch[c++] = (float)g_v5f_proc_gate.acc_traction;  /* 加速度零偏追踪门 */
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.imu.a_lin[i];        /* 牵引驱动量 */
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.imu.accel_bias_g[i]; /* 牵引零偏估计 */
    /* ---- VER=97 陀螺削顶判定（每帧，直接看原始 LSB；满量程 ±2000 dps = ±32768）---- */
    clip_any = 0u; clip_ax = 0u;
    {
        int32_t cx = (int32_t)g_v5f_hold.imu.gyro_lsb[0];
        int32_t cy = (int32_t)g_v5f_hold.imu.gyro_lsb[1];
        int32_t cz = (int32_t)g_v5f_hold.imu.gyro_lsb[2];
        if (cx >= V5F_CLIP_LSB_ABS || cx <= -V5F_CLIP_LSB_ABS) clip_ax |= 1u;
        if (cy >= V5F_CLIP_LSB_ABS || cy <= -V5F_CLIP_LSB_ABS) clip_ax |= 2u;
        if (cz >= V5F_CLIP_LSB_ABS || cz <= -V5F_CLIP_LSB_ABS) clip_ax |= 4u;
        clip_any = (clip_ax != 0u) ? 1u : 0u;
        if (clip_any != 0u) {
            if (s_clip_total < 0xFFFFFFFFu) s_clip_total++;
            if (s_clip_win_cnt < 0xFFFFu) s_clip_win_cnt++;
            s_clip_win_ax |= clip_ax;
        }
        if (s_clip_win_n < 0xFFFFu) s_clip_win_n++;
        if (s_clip_win_n >= V5F_CLIP_WIN_N) {   /* 窗满：发布千分比 + 轴位，清零重来 */
            s_clip_pm = (uint16_t)((1000u * (uint32_t)s_clip_win_cnt) / (uint32_t)s_clip_win_n);
            s_clip_ax = s_clip_win_ax;
            s_clip_win_cnt = 0u; s_clip_win_n = 0u; s_clip_win_ax = 0u;
        }
    }


    /* 打包标志位（bit 序号即权重；float 精确表示 < 2^24） */
    {
        uint32_t f = 0u;
        if (g_v5f_hold.imu.bias_ok    != 0u) f |= 1u << 0;
        if (g_v5f_hold.att.up_ref_ok  != 0u) f |= 1u << 1;
        if (g_v5f_hold.att.valid      != 0u) f |= 1u << 2;
        if (g_v5f_hold.vel.valid      != 0u) f |= 1u << 3;
        if (g_v5f_hold.imu.corr_valid != 0u) f |= 1u << 4;
        if (g_v5f_hold.imu.acc_valid  != 0u) f |= 1u << 5;
        if (g_v5f_hold.stat.valid     != 0u) f |= 1u << 6;
        if (g_v5f_hold.stat.changed   != 0u) f |= 1u << 7;
        if ((g_v5f_hold.imu.corr_flags & V5F_GYRO_CORR_FLAG_ROLLBACK) != 0u) f |= 1u << 8;
        if ((g_v5f_hold.imu.acc_traction_flags & V5F_ACC_TRACTION_FLAG_ROLLBACK) != 0u) f |= 1u << 9;
        if (g_v5f_hold.stat.ac_bypass != 0u) f |= 1u << 10;   /* 判静启动阶段旁路门 */
        if (s_bad_f32 != 0u) f |= 1u << 11;   /* 出现过非有限载荷值 */
        if (clip_any != 0u)       f |= 1u << 12;   /* VER=97 本帧有轴削顶 */
        if ((clip_ax & 1u) != 0u) f |= 1u << 13;   /* VER=97 X 轴削顶 */
        if ((clip_ax & 2u) != 0u) f |= 1u << 14;   /* VER=97 Y 轴削顶 */
        if ((clip_ax & 4u) != 0u) f |= 1u << 15;   /* VER=97 Z 轴削顶 */

        ch[c++] = (float)f;
    }

    ch[c++] = (float)s_dma1_irq_us;       /* 上一帧 DMA1 ISR 耗时 us（见上方说明） */

    /* 20~22 陀螺原始 LSB（主传感器，排在最前） */
    for (i = 0u; i < 3u; i++) ch[c++] = (float)g_v5f_hold.imu.gyro_lsb[i];
    /* 23~25 加速度原始 LSB */
    for (i = 0u; i < 3u; i++) ch[c++] = (float)g_v5f_hold.imu.accel_lsb[i];
    /* 26 陀螺温度 */
    ch[c++] = g_v5f_hold.imu.temp_celsius;
    /* 27 当帧 DRDY 间隔（us）：1 tick = 10 ns，故 * 1e-2 得 us */
    {
        uint64_t tk = g_v5f_hold.imu.fresh.drdy_tick;
        if (s_rep_tick_ok == 0u) {
            ch[c++] = 0.0f;                       /* 首帧：还没有基准 */
            s_rep_tick_ok = 1u;
        } else if (tk >= s_rep_last_tick) {
            ch[c++] = (float)(tk - s_rep_last_tick) * 1e-2f;
        } else {
            ch[c++] = 0.0f;                       /* 时间戳回退 */
        }
        s_rep_last_tick = tk;
    }

    /* 32~34 零偏校正后的角速度（处理函数 2/3 真正的输入） */
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.imu.gyro_dps[i];
    /* 35~37 陀螺零偏估计本身（内部状态，别处推不出来） */
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.imu.gyro_bias_dps[i];
    /* 38~40 用牵引后零偏重算的比力（与 a_lin 差一步 PI 更新） */
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.imu.accel_g[i];
    /* 41 累积静止证据（= 判静阈值表的索引） */
    ch[c++] = (float)g_v5f_hold.imu.bias_evidence_gran;
    /* 42 重力/倾斜融合门（处理函数 3 用；与 11 的 acc_traction 是两个不同的门） */
    ch[c++] = (float)g_v5f_proc_gate.att_tilt;
    /* 45 速度的软 ZUPT 置信度（0..255；除以 255 就是 lam） */
    ch[c++] = (float)g_v5f_proc_gate.vel_soft;
    /* 46~48 IST8310 地磁原始 LSB（0.3 uT/LSB，16 位补码；~200 Hz 更新，
     * 故相邻帧重复属正常）。供 PC 端做硬铁/软铁标定与航向复核。 */
    for (i = 0u; i < 3u; i++) ch[c++] = (float)g_v5f_hold.mag.lsb[i];
    /* 49 IST8310 样本序号（共享区 ist 通道 cnt）—— 见上方说明：磁只有 ~180 Hz，
     * 只有变化沿有效，只报数值会漏沿。序号变了即新样本；跳 >1 = 丢了样本。 */
    ch[c++] = (float)g_shm->ist.hdr.cnt;

    /* 50~52 校准后的地磁单位方向（机体坐标系，指向磁北且向下；导航系 右/前/上）。
     * 倾斜补偿后 atan2(f[0], f[1]) 直接就是磁航向，再加磁偏角即真航向。
     * ★ 已是单位矢量（hold_poll 里归一化过），消费方不必再除模长。 */
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.mag.f[i];
    /* 53 归一化**前**的模长（理论 1.0）—— 污染门的观测量，
     *    也是标定质量的独立读数：地磁强度是常数，标定对了它就恒为 1。 */
    ch[c++] = g_v5f_hold.mag.mag_norm;

    /* 55~60 磁力计“自己认为的维度”（偏航）输出 —— 给后面 EKF 融合用，
     * 本模块不做任何修正（偏航环不闭环）。详见 SPI_rx.h 的 v5f_mag_t 注释。
     *   55~57 mag.Bw[3]       导航系地磁单位方向（EKF 矢量新息）
     *   58    mag.psi_deg     导航系相对磁北的偏航角（度）
     *   59    mag.psi_true_deg 真航向（加了 WMM 磁偏角 D）
     *   60    mag.trust       综合可信度（mag.ok 且倾斜残差小）
     * ★ 偏航方向的噪声底（实测录制 221703）：静止/准静态下段内 std 0.64~0.82 度，
     *   而同一段记录里姿态的偏航真实误差可达 12.4 度 -> 信噪比约 18 倍。
     *   EKF 的 R_mag 可直接取 0.7 度的平方（再按 trust 与 53 路的 |y| 调权）。
     *   ★ mag.trust 为 0 时，上面三个量仍有值但**不应信**；上层若要自己算权重，
     *   可同时读 11~13 路的 a_lin 与 53 路的 |y| 。
     * ★ 1~54 列号一个都不动，新量只追加在尾部。 */
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.mag.Bw[i];
    ch[c++] = g_v5f_hold.mag.psi_true_deg;
    ch[c++] = (float)g_v5f_hold.mag.trust;

    /* 61~78 气压计 + GPS —— 这些量 hold_poll 早就搬到 g_v5f_hold 了，但一直没上报，
     * 以至于固件里存在、日志里查不到。它们是 EKF 的绝对高度/
     * 水平位置/对地速度与定位质量门的唯一来源。
     *  61~63 气压计：press_pascal(Pa) / temp_celsius(℃) / 样本序号
     *  64~69 RMC：status('A'=65 有效 / 'V'=86 无效) / lat(度) / lon(度) /
     *        speed_mps / flags(SHM_RMC_*) / 样本序号
     *  70~75 GGA：alt_m / fix_quality(0无 1单点 2差分 4RTK固定 5RTK浮点) /
     *        sat_num / hdop / flags(SHM_GGA_*) / 样本序号
     *  76~78 GSA：pdop / vdop / 样本序号
     *
     * ★ 为什么要报"样本序号"（= 共享区 hdr.cnt）：GPS 只有 1~10 Hz、
     *   气压计约 180 Hz，而上报是 8 kHz —— 相邻几千帧是同一个值。
     *   只报数值分不清"没更新"还是"更新了但值没变"，也数不了丢包。
     *   序号变了即新样本，跳 >1 = 上游丢了。这与 49 路（磁样本序号）同一套做法。
     *   （float32 精确表示 < 2^24 的整数：GPS 1 Hz 下约 194 天、气压计 180 Hz 下约 25.8 小时）
     * ★ 经纬度精度：上报是 float32，36.23 度在 float32 下量化约 2^-18 度 ≈ 0.42 m，
     *   相对 GPS 自身 ~1 m 噪声只增加约 8%，离线核验够用。
     *   ★ EKF 内部读的是 g_v5f_hold.gps_rmc.lat_e7（int32 原值，无精度损失），不受此影响。
     * ★ 带宽代价：GPS 是 1~10 Hz 却按 8 kHz 重复发，冗余 99.9%。这是单格式
     *   JustFloat 的固有代价（只能靠序号识别新样本）。
     * ★ 1~60 列号一个都不动，新量只追加在尾部。 */
    ch[c++] = g_v5f_hold.baro.press_pascal;
    ch[c++] = g_v5f_hold.baro.temp_celsius;
    ch[c++] = (float)g_shm->bmp.hdr.cnt;

    ch[c++] = g_v5f_hold.gps_rmc.speed_mps;
    ch[c++] = (float)g_v5f_hold.gps_rmc.fresh.flags;
    ch[c++] = (float)g_shm->gps_rmc.hdr.cnt;

    ch[c++] = g_v5f_hold.gps_gga.alt_m;
    ch[c++] = (float)g_v5f_hold.gps_gga.fix_quality;
    ch[c++] = (float)g_v5f_hold.gps_gga.sat_num;
    ch[c++] = g_v5f_hold.gps_gga.hdop;
    ch[c++] = (float)g_v5f_hold.gps_gga.fresh.flags;
    ch[c++] = (float)g_shm->gps_gga.hdr.cnt;

    ch[c++] = g_v5f_hold.gps_gsa.vdop;
    ch[c++] = (float)g_shm->gps_gsa.hdr.cnt;

    /* 79~80 追加（与 RMC 同一条语句，故共用 68 路的 RMC 样本序号，不单独再报）：
     *   79  gps_rmc.course_deg  对地航向（真北，0~360 度）
     *       ★ EKF 要的是速度**矢量**：v_north = speed*cos(course)，
     *         v_east = speed*sin(course)。只有 67 路的标量速度时，
     *         速度只能约束一个方向，这是实质缺口。
     *   80  gps_rmc.magvar_deg  接收机自报的磁偏角（东正西负）
     *       ★ 这是**接收机内部 WMM** 的值，可与 59 路（用离线 WMM 常量
     *         -7.53 度）互相校验；两者不一致就说明离线常量该换。
     *       注意别和 79 混：course 是航向，magvar 是磁北相对真北的偏角。
     * ★ 1~78 列号一个都不动，新量只追加在尾部。 */
    ch[c++] = g_v5f_hold.gps_rmc.course_deg;

    /* 81~84 经纬度的**精确**形式（追加，64/65 的含义不变）。
     * 为什么要这么报：64/65 直接报度值，而 float32 装 36.2293 这种值的
     * 量化步长约 3.8e-6 度 ≈ **0.42 m** —— 实测一份 2.3 s 静止录制里 gps_lon 只有
     * **2 个不同取值**、gps_lat 10 个，在图上看起来就像"没上报"或"卡住的常数"。
     *   81 lat_int  = lat_e7 / 10000000      （C 整除，向零截断）整数度，|.| <= 90
     *   82 lat_frac = lat_e7 - lat_int*1e7   小数部分 x1e7，|.| < 1e7 < 2^24
     *   83 lon_int  / 84 lon_frac            同上
     * 两部分在 float32 里都**精确可表示**，所以 PC 端
     *   lat_deg = lat_int + lat_frac * 1e-7
     * 是**无损重建**。负纬度（南球）也成立：C 整除向零截断，例如
     * lat_e7 = -5000000 -> int 0, frac -5000000 -> 0 + (-0.5) = -0.5 度。
     * ★ 为什么不把 lat_e7 直接当 float 报：3.623e8 需 29 位，float32 在该量级上
     * 步长 32 -> 3.2e-6 度 ≈ 0.36 m，一样有损。
     * ★ 4 列换来精确重建，值：PC 端做 EKF 原型时不能带着 0.4 m 量化误差。
     * ★ 1~80 列号一个都不动，新量只追加在尾部。 */
    {
        int32_t la = g_v5f_hold.gps_rmc.lat_e7;
        int32_t lo = g_v5f_hold.gps_rmc.lon_e7;
        int32_t la_i = la / 10000000;
        int32_t lo_i = lo / 10000000;
        ch[c++] = (float)la_i;
        ch[c++] = (float)(la - la_i * 10000000);
        ch[c++] = (float)lo_i;
        ch[c++] = (float)(lo - lo_i * 10000000);
    }

    /* 85~89 GPS GSV 的 C/N0（载噪比）统计 —— 把"俥号弱"和"多径"分开的唯一手段。
     *   85  sats_view    视野内卫星数（与 71 路的"参与定位卫星数"对比，差多少即剔除多少）
     *   86  snr_avg_dbhz 平均 C/N0，dBHz —— <35 偏弱，>40 良好
     *   87  snr_min      最低 C/N0，dBHz —— <25 说明有弱星拖后腿
     *   88  snr_n        参与统计的卫星数
     *   89  gsv 样本序号     （识别新轮次 / 数丢包）
     * ★ 静止实测：HDOP 1.00 + DGPS + 21 星仍有 3.95 m 误差（UERE ~3.9 m）。
     *   平均 C/N0 低 -> 查天线/馈线/遮挡/干扰；C/N0 正常却仍差 -> 多径。
     * ★ 1~84 列号一个都不动，新量只追加在尾部。 */
    ch[c++] = (float)g_v5f_hold.gps_gsv.sats_view;
    ch[c++] = g_v5f_hold.gps_gsv.snr_avg_dbhz;
    ch[c++] = (float)g_v5f_hold.gps_gsv.snr_min;
    ch[c++] = (float)g_v5f_hold.gps_gsv.snr_n;
    ch[c++] = (float)g_shm->gps_gsv.hdr.cnt;
    ch[c++] = (float)g_v5f_hold.gps_gsv.talkers;   /* 本轮 talker 数 */
    ch[c++] = JF_FW_TAG;                            /* 构建指纹：每次刷完先核对它 */

    /* 81~83 下行（主机→设备）验证：发命令后看这三列
     *   'T' <f32> 写 cmd_echo -> 上报原样回显，且 cmd_cnt 递增，即"下行通 + 上行看得见" */
    ch[c++] = g_cmd_echo;
    ch[c++] = (float)g_cmd_cnt;
    ch[c++] = (float)g_cmd_last;

    /* 90 气压软件平均后的压力（V5F_BARO_AVG_W 个样本的滑动均值）
     * ★ 气压高度 = 44330*(1-(p/101325)^0.190263)；平均后噪声 0.09~0.13 m，
     *   比 GPS 高度（2.5~5.5 m）好约 40 倍，所以高度主观测用它。
     *   62 路仍为单样本值，两者一起上报便于离线对比平均效果。 */
    ch[c++] = g_v5f_hold.baro.press_avg_pascal;

    /* 91~92 处理函数 2 的**启动阶段旁路**判据（见 v5f_tune.h 的 V5F_DET_AC_*）
     *   91  stat.e_ac      三轴交流能量 dps^2（= W 帧滑窗方差之和）。它**对直流零偏不可见**，
     *                      所以主路被零偏残差顶住时，它仍然认得出"静止"。
     *   92  stat.ac_bypass 旁路门：1 = 交流/能量判据成立。只在 imu.bias_ok 锁存前参与判定，
     *                      且只做"或"—— 只能把主路的"运动"改判成"静止"，绝不反向。
     * ★ 旁路为何存在：实测 20260914_233341 上电零偏残差 0.528 dps > 阈值表上限 0.3246 dps
     *   -> 主路恒判运动 -> 证据恒 0 -> 阈值永不衰减 -> q 恒 (1,0,0,0)、v_nav/a_lin/up_ref
     *   全 0、mag_trust 0，整条 V5F 链永久锁死。而同一时刻 e_ac = 0.0545，与正常静止的
     *   0.0538 只差 1.3% —— 交流量根本没有被零偏影响。
     * ★ 1~90 列号一个都不动，新量只追加在尾部。 */
    ch[c++] = g_v5f_hold.stat.e_ac;
    ch[c++] = (float)g_v5f_hold.stat.ac_bypass;

    /* 85~111 导航 EKF（处理函数 7/8）—— 影子模式：与旧链并行跑、全部记录，
     * 达标前不影响任何旧输出。设计见仓库根 ekf_design.md，常量见 v5f_tune.h J 组。
     * ★ 下面一律 **0 基**列号，与 tools/calib/count_cols.py（逐行核列数）一致：
     *     0 基 25 = 当帧 dt_us，0 基 76 = fw_tag，77~79 = 下行验证三列，
     *     80 = 气压软件平均，81/82 = e_ac / ac_bypass。
     *   ★ 为什么曾经写成 78：静态数 ch[c++] 会把"当帧 DRDY 间隔"那条 if/else 的
     *     三个分支都算成列，于是它之后的列号整体 +2。运行时只有一个分支。
     *  83..85   ekf.p[3]        导航系位置 E/N/U，m（原点 = ENU 对齐点，见 gate_bits bit10）
     *  86..88   ekf.v[3]        导航系速度 m/s
     *  89..92   ekf.q[4]        重力系世界四元数（机体->导航）
     *  93..95   ekf.a_nav[3]    导航系线性加速度 m/s^2（已去重力与零偏；"合加速度"的 EKF 版）
     *  96..98   ekf.ba[3]       加计零偏残差 m/s^2
     *  99..101  ekf.bg[3]       陀螺零偏残差 dps
     * 102       ekf.b_baro      气压高度零偏 m（气压高度 = p_z + b_baro）
     * 103       ekf.gate_bits   V5F_EKF_GB_* 位：bit0 GPS位置 bit1 GPS高度 bit2 气压
     *                          bit3 GPS速度 bit4 ZUPT bit5 重力/倾斜 bit6 磁偏航(闭环)
     *                          bit7 已对齐 bit8 本周期有EKF步 bit9 本周期有观测被chi2剔除
     *                          bit10 ENU原点已建立
     * 104       sigma_yaw_deg   偏航 1sigma（度）—— 门开被 M7 收紧、门关按 Q_bg 增长，
     *                          "航向现在能不能信"的直接读数（失效可观测，不是静默失效）
     * 105       sigma_pos_h     水平位置 1sigma m
     * 106       sigma_tilt_deg  倾角(水平两轴合成) 1sigma（度）—— 剧烈运动里
     *                          r_yaw = 真实偏航误差 + tan(I)*倾角误差，倾角误差
     *                          只能从这里读；也是验证 M7 自适应 R 的唯一途径
     * 107       sigma_vel_h     水平速度 1sigma m/s
     * 108..112  ekf.nis[5]      归一化新息平方：位置/速度/气压/重力/磁偏航。
     *                          应分别趋近 2/2/1/3/1；远大于维数 -> R 给小了。
     *                          应分别趋近 2/2/1/3/1；远大于维数 -> R 给小了。
     * ★ 0~106 列号一个都不动，107 起为新增/顺移（共 113 列）。 */
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.p[i];
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.v[i];
    for (i = 0u; i < 4u; i++) ch[c++] = g_v5f_hold.ekf.q[i];
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.a_nav[i];
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.ba[i];
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.bg[i];
    ch[c++] = g_v5f_hold.ekf.b_baro;
    ch[c++] = (float)g_v5f_hold.ekf.gate_bits;
    ch[c++] = g_v5f_hold.ekf.sigma_yaw_deg;
    ch[c++] = g_v5f_hold.ekf.sigma_pos_h;
    ch[c++] = g_v5f_hold.ekf.sigma_vel_h;
    ch[c++] = g_v5f_hold.ekf.sigma_tilt_deg;   /* 倾角 1sigma（度）*/
    for (i = 0u; i < 5u; i++) ch[c++] = g_v5f_hold.ekf.nis[i];
    ch[c++] = g_v5f_hold.ekf.pzz;                  /* 垂直位置方差 m^2（诊断）*/
    ch[c++] = g_v5f_hold.ekf.pbb;                  /* b_baro 方差 m^2（诊断）*/
    ch[c++] = g_v5f_hold.ekf.p_p_zz;               /* s_P[2][2]（P0 自检）*/
    ch[c++] = g_v5f_hold.ekf.p_p_bb;               /* s_P[15][15]（P0 自检）*/
    /* 磁偏航（M7）专项诊断：门/死点/新息/增益一次看全 */
    ch[c++] = (float)g_v5f_hold.ekf.mag_gate;
    ch[c++] = g_v5f_hold.ekf.mag_bh;
    ch[c++] = g_v5f_hold.ekf.mag_r_deg;
    ch[c++] = (float)g_v5f_hold.ekf.mag_used;
    ch[c++] = g_v5f_hold.ekf.p_yy;
     ch[c++] = (float)g_v5f_hold.ekf.prop_ok;   /* VER=44 prop_ok */
     ch[c++] = (float)g_v5f_hold.ekf.f_ok;   /* VER=44 f_ok */
     ch[c++] = (float)g_v5f_hold.ekf.prop_row;   /* VER=44 prop_row */
     ch[c++] = (float)g_v5f_hold.ekf.stage;   /* VER=44 stage */
     ch[c++] = (float)g_v5f_hold.ekf.mag_rej;   /* VER=44 mag_rej */
      ch[c++] = g_v5f_hold.ekf.mag_fhb;   /* VER=45 机体系水平占比 */
       ch[c++] = g_v5f_hold.ekf.mag_rx;   /* VER=46 */
       ch[c++] = g_v5f_hold.ekf.mag_ry;   /* VER=46 */
        ch[c++] = g_v5f_hold.ekf.mag_vx;   /* VER=67 */
        ch[c++] = g_v5f_hold.ekf.mag_vy;   /* VER=67 */
        ch[c++] = g_v5f_hold.ekf.mag_v0x;   /* VER=67 */
        ch[c++] = g_v5f_hold.ekf.mag_v0y;   /* VER=67 */
        ch[c++] = g_v5f_hold.ekf.mag_yawpre;   /* VER=67 */
       ch[c++] = g_v5f_hold.ekf.mag_dqx;   /* VER=46 */
       ch[c++] = g_v5f_hold.ekf.mag_dqy;   /* VER=46 */
       ch[c++] = g_v5f_hold.ekf.mag_dqz;   /* VER=46 */
        ch[c++] = g_v5f_hold.ekf.tilt_dqx;   /* VER=50 */
        ch[c++] = g_v5f_hold.ekf.tilt_dqy;   /* VER=50 */
        ch[c++] = g_v5f_hold.ekf.tilt_dqz;   /* VER=50 */
        ch[c++] = g_v5f_hold.ekf.tilt_prx;   /* VER=50 */
        ch[c++] = g_v5f_hold.ekf.tilt_pry;   /* VER=50 */
        ch[c++] = g_v5f_hold.ekf.tilt_prz;   /* VER=50 */
        ch[c++] = g_v5f_hold.ekf.mag_cmp_thm;   /* ★VER=78 罗盘：实测航向(度) */
        ch[c++] = g_v5f_hold.ekf.mag_cmp_thp;   /* ★VER=78 罗盘：预测航向(度) */
        ch[c++] = g_v5f_hold.ekf.mag_cmp_amn;   /* ★VER=78 罗盘：比力模长(g) */
        ch[c++] = g_v5f_hold.ekf.mag_cmp_mhn;   /* ★VER=78 罗盘：法平面内磁场模长 */
        ch[c++] = g_v5f_hold.ekf.mag_age_ms;
        ch[c++] = g_v5f_hold.ekf.mag_rs;           /* VER=123 上报：重力矢量误差估计(deg)，<2 用投影入口 */
        ch[c++] = g_v5f_hold.ekf.tilt_inv_ms;      /* ★VER=84 倾角参考失效时长(ms) */
        ch[c++] = (float)g_v5f_hold.ekf.mag_hold;  /* ★VER=84 磁被暂停 */
        ch[c++] = (float)g_v5f_hold.ekf.grav_ok;   /* ★VER=84 重力观测门 */
        ch[c++] = g_v5f_hold.ekf.grav_nis;

        /* ★VER=95 上报补齐（PC 复演用；追加在末尾 -> 既有列号全部不变，NCH 154 -> 159）
         *   为什么必须补：仿真此前**自己重算门**，而它没有编译上游 proc，门输入与固件不同，
         *   实测 gate==1 恒为 0.998、mag_used 恒 50%，"门关 vs 门开"根本没法对照。
         *   记录下面这几个量后，仿真可直接沿用固件的门，复演才有意义。 */
        ch[c++] = (float)g_v5f_hold.mag.ok;             /* 154 样本有效性 |mag_norm-1| < V5F_MAG_ERR_LIM */
        ch[c++] = (float)g_v5f_proc_gate.ekf_mag_yaw;   /* 155 磁牵引外门（输出） */
        ch[c++] = (float)g_v5f_proc_gate.ekf_tilt;      /* 156 倾角门（输出；VER=95 起不再直接管磁） */
        ch[c++] = (float)g_v5f_hold.ekf.mag_gate;       /* 157 EKF 记录的磁门 */
        ch[c++] = (float)g_v5f_hold.ekf.mag_used;       /* 158 本周期 M7 是否真的执行了更新 */

        /* ★VER=96 追加：DRDY 时间戳 + 逐帧 XOR 校验和（PC 复演必需）
         *   159/160 单位 1us(GetTime64_10Ns()/100), 取低 24 位(16.7s 回绕), 上位机按时戳还原无歧义 -> 可自算
         *   磁样本年龄 = (159-160), 不再依赖被 ~196Hz 锁存的 mag_age_ms。
         *   161 对"前 NCH-1 列载荷的全部字节"异或(种子 0x5A5A)：坏帧(载荷整体位移,
         *   实测约 1~3%)靠它逐帧判废, 不必再用 |q|≈1 / 幅值 之类的启发式。 */
        {
            uint64_t tk10 = GetTime64_10Ns() / 100ULL;  /* tk 在此作用域不可见; 与 DRDY 同一时钟 */
            uint64_t md10 = g_v5f_hold.mag.fresh.drdy_tick / 100ULL;
            uint16_t xk = 0x5A5Au;
            const uint8_t *pb = (const uint8_t *)ch;
            uint32_t bc;
            ch[c++] = (float)(uint32_t)(tk10 & 0xFFFFFFULL);
            ch[c++] = (float)(uint32_t)(md10 & 0xFFFFFFULL);
            ch[c++] = 0.0f;                                  /* 占位, 下面回填 */
            for (bc = 0u; bc < (uint32_t)(JF_CH_NUM - 1u) * 4u; bc++) xk ^= pb[bc];
            ch[JF_CH_NUM - 1u] = (float)xk;
        }         /* ★VER=84 重力观测 NIS */   /* ★VER=83 磁数据年龄(ms) */


    /* ---- 载荷必须是有限 float（数据有效性；与切帧无关，见下）----
     * ★ 不能写 x != x 或 isfinite()：本工程 -Ofast 隐含 -ffinite-math-only，
     *   编译器会假定没有 inf/NaN 而把这类判断优化掉。必须按位判：指数域全 1。 */
    for (i = 0u; i < JF_CH_NUM; i++) {
        uint32_t u;
        memcpy(&u, &ch[i], 4u);
        if ((u & 0x7F800000u) == 0x7F800000u) {
            ch[i] = 0.0f;
            if (s_bad_f32 < 0xFFFFu) s_bad_f32++;
        }
    }

    /* ---- 帧结构：帧头 2B + 帧长 2B + 载荷 + 帧尾 2B ----
     *   帧头 A5 5A | 帧长(小端 u16 = 载荷字节数) | 载荷 JF_CH_NUM*4 B | 帧尾 5A A5
     * 放弃 JustFloat 的"帧尾 00 00 80 7F 唯一"强假设：那种格式下该组合一旦出现在载荷里，
     * 主机就会误切、之后整段错位（实测 2.6% 的帧因此被旋转）。现在帧长字段直接给出边界，
     * **载荷里出现任何字节组合都不会造成错位**，所以此处不再需要字节级不变量扫描。 */
    buf[0] = 0xA5u;
    buf[1] = 0x5Au;
    buf[2] = (uint8_t)((uint32_t)sizeof(ch) & 0xFFu);
    buf[3] = (uint8_t)(((uint32_t)sizeof(ch) >> 8) & 0xFFu);
    memcpy(&buf[4], ch, sizeof(ch));
    buf[JF_FRAME_LEN - 2u] = 0x5Au;
    buf[JF_FRAME_LEN - 1u] = 0xA5u;

    hid_up_enqueue(buf, JF_FRAME_LEN);   /* 空间不足时整帧丢弃，不会写出半帧 */
}

/* ==================== DMA 中断服务函数：只负责调度分配 ==================== */
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1, DMA1_IT_TC2)) {
        DMA_ClearITPendingBit(DMA1, DMA1_IT_TC2);
        if (g_shm) {
            uint64_t isr_t0 = GetTime64_Us();   /* ISR 计时起点 */

            /* SPI 帧解析（温度 + 六轴）→ 保持器（DRDY 时刻由 hold_poll 搬入） */
            g_v5f_hold.imu.temp_celsius = (int16_t)((rxfifo[1] << 8) | rxfifo[2]) / 132.48f + 25.0f;
            g_v5f_hold.imu.gyro_lsb[0]  = (int16_t)((rxfifo[9]  << 8) | rxfifo[10]);
            g_v5f_hold.imu.gyro_lsb[1]  = (int16_t)((rxfifo[11] << 8) | rxfifo[12]);
            g_v5f_hold.imu.gyro_lsb[2]  = (int16_t)((rxfifo[13] << 8) | rxfifo[14]);
            g_v5f_hold.imu.accel_lsb[0] = (int16_t)((rxfifo[3]  << 8) | rxfifo[4]);
            g_v5f_hold.imu.accel_lsb[1] = (int16_t)((rxfifo[5]  << 8) | rxfifo[6]);
            g_v5f_hold.imu.accel_lsb[2] = (int16_t)((rxfifo[7]  << 8) | rxfifo[8]);
            hold_poll();
            g_v5f_hold.imu.fresh.new_data = 1;  /* 所有字段写完后才置新数据信号 */

            /* 主任务区开始 */

            /* 处理链：依次调用，每个处理函数把结果直接写回 g_v5f_hold；
             * 返回非 0 只锁存错误码，不打断后续处理函数（结果字段的有效性由各函数自己置）。 */
            {
                uint8_t st = v5f_proc_gyro_bias(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_gyro_bias = st;

                /* 第二步：动静判定 —— 吃本帧的校正输出，写下一帧的门控 */
                st = v5f_proc_static_detect(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_static = st;

                /* 第三步：姿态四元数 —— 吃本帧的补偿输出（零偏 + 逐轴标度已载入） */
                st = v5f_proc_attitude(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_attitude = st;

                /* 第四步：加速度计标定 —— 牵引/冻结由**外部门控**决定（读上一帧的判定） */
                st = v5f_proc_accel_cal(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_accel_cal = st;

                /* 第五步：加速度牵引的外部门控 —— 用本帧零偏校准后的原始值判别
                 *        牵引/冻结，写入 gate，作用于**下一帧**的第 4 步
                 *        （与第 2 步写 -> 第 1 步下一帧读 完全同构） */
                st = v5f_proc_acc_gate(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_acc_gate = st;

                /* 第六步：世界系速度 —— 吃第 3 步的姿态与第 4 步的比力 */
                st = v5f_proc_velocity(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_velocity = st;

                /* 第七步：导航 EKF —— 读**上一帧**第 8 步写下的七道门 */
                st = v5f_proc_ekf(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_ekf = st;

                /* 第八步：EKF 各观测的外部门控 —— 写本帧的门，第 7 步下一帧读
                 *        （与"第 2 步写 -> 第 1 步下一帧读"完全同构） */
                st = v5f_proc_ekf_gate(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_ekf_gate = st;
            }

            /* 上报：组 1 帧 JustFloat 交给 CDC 上行环，主循环负责灌 EP2 */
            justfloat_report();

            /* 主任务区结束 */

            /* -------- 重新触发 DMA 接收 -------- */
            GPIO_SetBits(GPIOF, GPIO_Pin_4);
            DMA_Cmd(DMA1_Channel2, DISABLE);
            DMA1_Channel2->CNTR = 15;
            DMA_Cmd(DMA1_Channel2, ENABLE);

            /* 记录本次 ISR 用时的历史最劣(最大)值，us */
            uint32_t d = (uint32_t)(GetTime64_Us() - isr_t0);
            s_dma1_irq_us = d;           /* 下一帧上报它 */
            s_pr_n++; s_pr_sum += d; if (d > s_pr_max) s_pr_max = d;
            if (d > 125u) s_pr_ovr++;
        }
    }
}

/* ==================== DMA 初始化函数 ==================== */
void SPI_DMA_Init(void)
{
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    SPI1_DMA_Rx_Setup((uint8_t*)rxfifo, 15);

    NVIC_SetAllocateIRQ(DMA1_Channel2_IRQn, Core_ID_V5F);
    NVIC_SetPriority(DMA1_Channel2_IRQn, 0<<5);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}


/* ==================== VER=97 陀螺削顶统计接口（OLED / 上位机） ==================== */
uint16_t v5f_imu_clip_pm(void)    { return s_clip_pm; }
uint8_t  v5f_imu_clip_axis(void)  { return s_clip_ax; }
uint32_t v5f_imu_clip_total(void) { return s_clip_total; }


/* VER=117: compute-load probe. Call from main loop; returns 1 once per second
 * with the last second stats: frames, average ISR time (us), worst ISR time (us),
 * over-budget frames (ISR longer than the 124.66us nominal period).
 * NOTE: accumulators are read without masking interrupts - a one-frame race is harmless. */
uint8_t v5f_isr_probe_1s(uint32_t *n, uint32_t *avg_us, uint32_t *max_us, uint32_t *ovr)
{
    static uint64_t s_t0 = 0ULL;
    uint64_t t = GetTime64_Us();
    if (s_t0 == 0ULL) { s_t0 = t; return 0u; }
    if (t - s_t0 < 1000000ULL) return 0u;
    *n      = s_pr_n;
    *avg_us = (s_pr_n != 0u) ? (s_pr_sum / s_pr_n) : 0u;
    *max_us = s_pr_max;
    *ovr    = s_pr_ovr;
    s_t0 = t; s_pr_n = 0u; s_pr_sum = 0u; s_pr_max = 0u; s_pr_ovr = 0u;
    return 1u;
}
