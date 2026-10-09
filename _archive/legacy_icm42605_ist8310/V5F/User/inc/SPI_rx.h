#ifndef __SPI_RX_H
#define __SPI_RX_H

#include <stdint.h>
#include "mipc_shm.h"   /* 共享区 flags 位定义（SHM_RMC_ / SHM_GGA_ / SHM_GSA_ 前缀）与布局 */

/* ==================== 数据保持器（原 observer 模块并入本文件） ====================
 * 集中保存各传感器最新数据 + 新鲜度 + 有效性，供消费者读取；不做任何解算。
 *   - IMU（陀螺/加速度/温度）：SPI DMA 帧，DMA1_Channel2 中断解析后写入；
 *   - 磁力计 / 气压计 / GPS：共享区，中断里按 cnt 变化搬运（见 SPI_rx.c 的 hold_poll）。
 *
 * v5f_fresh_t 三项：
 *   drdy_tick —— DRDY 时刻，单位 10 ns 计数（与共享区同基，可直接与 GetTime64_10Ns() 比较）
 *   new_data  —— 新数据信号，中断置 1，消费者处理完自行清 0
 *   flags     —— 共享区 flags 原样拷贝；★ 判空查位、勿用值判空；陀螺/磁力计/气压计无 flags，恒 0
 *
 * GPS 三态：无 SHM_RMC_STATUS → 未启动/无语句；status=='V' → 有信号无定位；'A' → 定位有效。
 * 各单位见字段注释；唯一必须定点的是经纬度（×1e7 度）。 */

/* 通道新鲜度 + 有效性 */
typedef struct {
    uint64_t         drdy_tick;  /* DRDY 时刻，10 ns 计数 */
    volatile uint8_t new_data;   /* 新数据信号：1 = 有尚未处理的新数据 */
    uint8_t          flags;      /* 共享区 flags 原样；无 flags 的通道恒 0 */
} v5f_fresh_t;

/* ---- IMU：ICM-42605（SPI DMA 帧，DMA 中断更新；drdy_tick 来自共享区陀螺通道） ----
 * 前半段是 DMA 中断解析出的原始量，后半段是处理链（v5f_proc.h）在本帧写回的结果；
 * 处理链未跑或该帧报错时，结果字段按 v5f_proc.h 的约定置无效（corr_valid=0）。 */
typedef struct {
    v5f_fresh_t fresh;
    int16_t     gyro_lsb[3];     /* 角速度原始 LSB：16.4 LSB/(°/s)，满量程 ±2000 °/s 对应 ±32768 */
    int16_t     accel_lsb[3];    /* 加速度原始 LSB：2048 LSB/g（±16 g 档，见 spi_hw.c），满量程对应 ±32768 */
    float       temp_celsius;    /* 陀螺温度 ℃（132.48 LSB/℃，25 ℃ 零点） */

    /* ↓ 处理结果：陀螺零偏校正（v5f_proc_gyro_bias 写） */
    float            gyro_dps[3];      /* 零偏校正后角速度 °/s */
    float            gyro_bias_dps[3]; /* 当前零偏估计 °/s（便于观察/记录，非校正结果本身） */
    volatile uint8_t corr_valid;       /* 1 = 本帧校正结果有效 */
    volatile uint8_t bias_ok;          /* 1 = 零偏已收敛可信（累计静止 ≥ 3τ_boot = 3 s，锁存）
                                        * ★ 只作观察标志上报，**不再是**处理函数 3 的使能门：
                                        * 姿态的使能条件是"拿到可信重力方向"，与零偏无关。 */
    uint8_t          corr_flags;       /* V5F_GYRO_CORR_FLAG_* 位标志 */
    uint16_t         bias_evidence_gran; /* 累积静止证据（20 ms 粒度数，回退时扣减）
                                          * 处理函数 2 用它查判静阈值表 */

    /* ↓ 处理结果：加速度计标定（v5f_proc_accel_cal 写） */
    float            accel_g[3];       /* 比力，单位 g（静止时 |accel_g| = 1，方向为 -重力） */
    volatile uint8_t acc_valid;        /* 1 = 本帧标定结果有效 */
    uint8_t          _rsv_acc[3];

    /* ↓ 处理结果：加速度零偏牵引（v5f_proc_accel_cal 写） */
    float            accel_bias_g[3];     /* 在线零偏估计，单位 g（便于观察/记录） */
    volatile uint8_t acc_trust;           /* 1 = 本帧门控可信（正在牵引） */
    uint8_t          acc_traction_flags;  /* bit0 = 本帧发生了零偏回退 */
    float            a_lin[3];            /* 牵引驱动量 = accel_g - R(q)^T*up_ref（g）。
                                           * 这就是"没被消掉的残余"，速度漂移的直接来源；
                                           * 发出来才能看出牵引到底有没有在收敛。
                                           * 由 v5f_proc_accel_cal 每帧无条件写入。 */
} v5f_imu_t;

/* ---- 处理结果：动静判定（v5f_proc_static_detect 写；门控也由它产出） ---- */
typedef struct {
    float            level_dps;   /* 主路判静统计量 max_axis |W 帧滑窗均值|，dps（直流判据） */
    volatile uint8_t is_static;   /* 1 = 本帧判为静止（主路 或 启动阶段旁路） */
    volatile uint8_t valid;       /* 1 = 本帧判定有效（滑窗未满时为 0） */
    uint8_t          changed;     /* 1 = 本帧发生静止<->运动翻转 */
    volatile uint8_t ac_bypass;   /* 启动阶段旁路门：1 = 交流/能量判据成立。
                                   * 只在 imu.bias_ok 锁存前参与判定；它只做"或"，
                                   * 把主路的"运动"改判成"静止"，反向永不允许。
                                   * 见 v5f_tune.h 的 V5F_DET_AC_* 与 proc_static_detect.c。 */
    uint8_t          _rsv[2];
    float            e_ac;        /* 三轴交流能量（= W 帧滑窗方差之和）dps^2 —— 旁路的判据量。
                                   * 对直流零偏不可见，所以零偏残差顶住主路时它仍认得出静止。 */
} v5f_static_t;

/* ---- 处理结果：姿态四元数（v5f_proc_attitude 写） ----
 * 输入是处理函数 1 的补偿输出（零偏 + 逐轴标度系数已载入），本项只做积分。 */
typedef struct {
    float            q[4];        /* 机体→导航 旋转：q[0]=w q[1]=x q[2]=y q[3]=z */
    volatile uint8_t valid;       /* 1 = 本帧四元数有效（上游补偿结果有效时） */
    uint8_t          _rsv[3];

    /* ↓ 世界竖直在导航系里的表示（不是重力方向！重力沿 -up_ref）。
     * 导航系由处理函数 3 用重力初始化成竖直（z = 上），所以这里**恒为 (0,0,1)**。
     * 保留字段只是让去重力的算式不依赖"导航系恰好竖直"这一假设。
     * 由处理函数 3 在使能那一帧置好，不再由处理函数 4 锁存。 */
    float            up_ref[3];
    volatile uint8_t up_ref_ok;
    uint8_t          _rsv_up[3];
} v5f_att_t;

/* ---- IST8310 磁力计（共享区搬运 + 线性标定 + 污染门） ---- */
typedef struct {
    v5f_fresh_t fresh;
    int16_t     lsb[3];          /* 磁场原始 LSB（16 位补码）：0.3 μT/LSB */

    /* ↓ 处理结果：线性标定（hold_poll 里用 v5f_tune.h H 组的 V5F_MAG_A / V5F_MAG_C 算）。
     * 校准式 y = A*raw + C，A 里已含"z 轴取反"（IST8310 原始三轴是左手系）。 */
    float            f[3];        /* 机体坐标系下地磁**单位方向**（指向磁北且向下）。
                                   * 与姿态 q 同导航系约定：x = 右，y = 前，z = 上。
                                   * ★ 已归一化，消费方不必再除模长；倾斜补偿后
                                   *   atan2(f[0], f[1]) 直接就是磁航向。 */
    float            mag_norm;    /* 归一化**前**的模长，理论 1.0（标定残差 2 ~ 5%）。
                                   * 既是污染门的观测量，也是标定质量的独立读数 ——
                                   * 地磁强度是常数，标定对了它就恒为 1。 */
    volatile uint8_t ok;          /* 1 = 本样本可信（|mag_norm - 1| < V5F_MAG_ERR_LIM）；
                                   * 0 = 附近有外部磁源/异常，本样本应丢弃。
                                   * 注意这是**样本有效性门**，不是牵引门（磁不参与
                                   * 任何融合环），语义与 gate 里那几个 0/1 门不同。 */
    uint8_t          _rsv_mag[3];

    /* ↓ 输出：磁力计**自己认为的维度**（偏航）—— **只读，不参与修正**。
     * 偏航环不闭环：磁力计只把“它唯一有资格发言的那个维度”
     * 交给上层 EKF。倾斜不在此列：重力对倾斜有绝对参考，
     * 磁力计对倾斜没有增量信息。
     * 由 hold_poll 在磁样本到达时（~187 Hz）计算，用的是上一帧的姿态。 */
    float            Bw[3];         /* R(q)*f，导航系地磁单位方向（EKF 矢量新息用） */
    float            psi_deg;       /* 导航系相对**磁北**的偏航角（度）= atan2(Bw_x, Bw_y) */
    float            psi_true_deg;  /* psi_deg + V5F_MAG_DECL_DEG，真航向 */
    volatile uint8_t trust;         /* 1 = mag.ok 且倾斜残差小，上面三个量可信 */
    uint8_t          _rsv_mag2[3];
} v5f_mag_t;

/* ---- BMP388 气压计（共享区搬运，float 直通） ---- */
typedef struct {
    v5f_fresh_t fresh;
    float       temp_celsius;    /* ℃ */
    float       press_pascal;    /* Pa，单样本 */
    float       press_avg_pascal;/* Pa，**软件 OSR**：最近 V5F_BARO_AVG_W 个样本的滑动均值。
                                  * 单样本噪声 5.0 Pa，W=32 后 1.0~1.5 Pa（0.09~0.13 m）。
                                  * ★ 不改 OSR/IIR 是因为要与 IST8310 流水线读取。 */
} v5f_baro_t;

/* ---- GPS RMC：定位 / 速度 / 航向 / 磁偏角 / 日期
 * （共享区搬运；flags: SHM_RMC_STATUS/POS/SPEED/DATE/COURSE/MAGVAR） ---- */
typedef struct {
    v5f_fresh_t fresh;
    uint8_t     status;          /* NMEA 原字符：'A'=有效 / 'V'=无效 */
    int32_t     lat_e7;          /* 纬度 ×1e7 度（1 LSB = 10^-7°，北正南负） */
    int32_t     lon_e7;          /* 经度 ×1e7 度（东正西负） */
    float       speed_mps;       /* 对地速度 m/s（共享区同为 float m/s） */
    uint32_t    date_ddmmyy;     /* UTC 日期 ddmmyy（十进制字面量） */
    float       course_deg;      /* 对地航向（真北，0~360 度）：EKF 靠 speed + course 组成速度矢量 */
    float       magvar_deg;      /* 接收机自报的磁偏角（东正西负），可与离线 WMM 值互校 */
} v5f_gps_rmc_t;

/* ---- GPS GGA：定位质量 / 高度（共享区搬运；flags: SHM_GGA_QUALITY/SV/HDOP/ALT） ---- */
typedef struct {
    v5f_fresh_t fresh;
    float       alt_m;           /* 海拔 m（共享区为 cm 定标整数） */
    uint8_t     fix_quality;     /* 0=无效 1=单点 2=差分 4=RTK 固定 5=RTK 浮点 */
    uint8_t     sat_num;         /* 卫星数 */
    float       hdop;            /* 水平精度因子，无量纲（共享区为 ×100 定标整数） */
} v5f_gps_gga_t;

/* ---- GPS GSA：PDOP / VDOP（共享区搬运；flags: SHM_GSA_DOP） ---- */
typedef struct {
    v5f_fresh_t fresh;
    float       pdop;            /* 位置精度因子（共享区为 ×100 定标整数） */
    float       vdop;            /* 垂直精度因子（同上） */
} v5f_gps_gsa_t;


/* ---- GPS GSV：C/N0（载噪比）统计（共享区搬运）----
 * ★ 用途：把"信号弱"和"多径"分开。
 *   静止实测 HDOP 1.00 + DGPS + 21 星仍有 3.95 m 误差（UERE ~3.9 m，
 *   健康 DGPS 应 0.5~1 m）。C/N0 平均 <35 dBHz 或最低 <25 -> 信号弱；
 *   >40 却仍差 -> 多径。 */
typedef struct {
    v5f_fresh_t fresh;
    uint8_t     sats_view;       /* 视野内卫星数（GSV 字段 3） */
    uint8_t     snr_n;           /* 有效 C/N0 的卫星数 */
    uint8_t     snr_min;         /* 最小 C/N0，dBHz（0 = 无） */
    uint8_t     talkers;         /* 本轮 talker 数（轮次聚合是否正常的直接读数） */
    float       snr_avg_dbhz;    /* 平均 C/N0，dBHz */
} v5f_gps_gsv_t;

/* ---- 处理结果：世界系速度（v5f_proc_velocity 写） ----
 * age_s = 距上次 ZUPT 归零的秒数。纯惯性下"平滑匀速平动"与"静止"不可分（等效原理），
 * 判静一定有误判，所以速度的可信度随时间衰减：误差上界约 2 mg * age_s。
 * 上层据此判断这个速度还能信多久，而不是无条件采信。 */
typedef struct {
    float            v_nav[3];    /* 导航系速度 m/s（右/前/上，与 q 的导航系同序） */
    volatile uint8_t valid;       /* 1 = 本帧有效 */
    uint8_t          _rsv[3];
    float            age_s;       /* 距上次静止归零的秒数 */
} v5f_vel_t;

/* ---- 处理结果：导航 EKF（v5f_proc_ekf 写；门由 v5f_proc_ekf_gate 写） ----
 * 16 维误差状态 ESKF，标称量就是下面这些。内部一律 SI + 弧度，只有上报换成度/dps。
 * 导航系 = **真 ENU**（x 东, y 北, z 上），由对齐那一帧用磁力计一次性把 x 定到真东。
 * ★ 偏航闭环在**滤波器内部**（观测 M7，z = 磁偏角 D 这个已知常数）；
 *   旧 proc_attitude 链仍然不闭环，两者分工不混。 */
typedef struct {
    float            p[3];            /* 导航系位置 E/N/U，m；原点 = ENU 对齐点 */
    float            v[3];            /* 导航系速度 m/s */
    float            q[4];            /* 重力系世界四元数（机体->导航） */
    float            a_nav[3];        /* 导航系线性加速度 m/s^2（已去重力与零偏） */
    float            ba[3];           /* 加计零偏残差 m/s^2（在离线常量之上） */
    float            bg[3];           /* 陀螺零偏残差 dps（内部是 rad/s，发布时换算） */
    float            b_baro;          /* 气压高度零偏 m；气压高度 = p_z + b_baro */
    float            pzz;             /* P[2][2]：垂直位置的方差 m^2（诊断气压观测的分配）*/
    float            pbb;             /* P[15][15]：b_baro 的方差 m^2（同上）*/
    float            p_p_zz;          /* s_P[2][2]（上周期末）—— 与 pzz 一起看可判定
                                       * P0 是否落地、传播是否搬运 */
    float            p_p_bb;          /* s_P[15][15]（同上）*/
    /* ↓ 磁偏航（M7）专项诊断：回答"地磁为什么没有牵引" */
    volatile uint8_t mag_gate;        /* 第 8 步算出的外部门状态（1 = 稳健模长门通过）*/
    uint8_t          mag_used;        /* 1 = 本周期 M7 真的执行了更新 */
    uint8_t          _rsv_m7[2];
    float            mag_bh;          /* |B 水平分量| / |B|（死点判据量，正常 0.59）*/
    float            mag_r_deg;       /* M7 新息（度）：恒 0 说明观测没形成 */
    float            p_yy;            /* P[8][8]：偏航方差，与 R 一比可估 K */
     uint8_t      prop_ok;   /* VER=44 EKF 内部状态：传播是否完成 */
     uint8_t      f_ok;   /* VER=44 EKF 内部状态：传播是否完成 */
     uint8_t      prop_row;   /* VER=44 EKF 内部状态：传播是否完成 */
     uint8_t      stage;   /* VER=44 EKF 内部状态：传播是否完成 */
     uint16_t     mag_rej;   /* VER=44 EKF 内部状态：传播是否完成 */
      float     mag_fhb;   /* VER=45 机体系水平占比（向量有效性） */
       float    mag_rx;   /* VER=46 */
       float    mag_ry;   /* VER=46 */
        float    mag_vx;   /* VER=67 */
        float    mag_vy;   /* VER=67 */
        float    mag_v0x;   /* VER=67 */
        float    mag_v0y;   /* VER=67 */
        float    mag_yawpre;   /* VER=67 */
       float    mag_dqx;   /* VER=46 */
       float    mag_dqy;   /* VER=46 */
       float    mag_dqz;   /* VER=46 */
        float    tilt_dqx;   /* VER=50 */
        float    tilt_dqy;   /* VER=50 */
        float    tilt_dqz;   /* VER=50 */
        float    tilt_prx;   /* VER=50 */
        float    tilt_pry;   /* VER=50 */
        float    tilt_prz;   /* VER=50 */
        float    mag_cmp_thm;   /* ★VER=78 罗盘：重力法平面内实测航向(度) */
        float    mag_cmp_thp;   /* ★VER=78 罗盘：同一平面内预测航向(度) */
        float    mag_cmp_amn;   /* ★VER=78 罗盘：比力模长(g)，1.0=加计即重力 */
        float    mag_age_ms;
        float    mag_rs;          /* ★VER=84 磁 R 放大倍数 */
        float    tilt_inv_ms;     /* ★VER=84 倾角参考失效时长(ms) */
        uint8_t  mag_hold;        /* ★VER=84 磁被暂停 */
        uint8_t  grav_ok;         /* ★VER=84 重力观测门 */
        float    grav_nis;        /* ★VER=84 重力观测 NIS */    /* ★VER=83 磁数据年龄(ms) = IMU tick - mag DRDY tick */
        float    mag_cmp_mhn;   /* ★VER=78 罗盘：法平面内磁场模长(0~1) */
    float            sigma_yaw_deg;   /* 偏航 1sigma（度）—— "航向现在能不能信"的直接读数：
                                       * 门开被 M7 收紧、门关按 Q_bg 增长。 */
    float            sigma_tilt_deg;  /* 倾角(水平两轴合成) 1sigma，度。
                                       * 剧烈运动里 r_yaw = 真实偏航误差 + tan(I)*倾角误差，
                                       * 倾角误差只能从这里读 —— 没有它就无法验证
                                       * M7 的自适应 R 是否诚实。 */
    float            sigma_pos_h;     /* 水平位置 1sigma，m */
    float            sigma_vel_h;     /* 水平速度 1sigma，m/s */
    float            nis[5];          /* 归一化新息平方：位置/速度/气压/重力/磁偏航。
                                       * 应分别趋近 2/2/1/3/1；>> 说明 R 给小了。 */
    uint16_t         gate_bits;       /* V5F_EKF_GB_* 位（v5f_tune.h J 组） */
    volatile uint8_t aligned;         /* 1 = 已完成重力 + 磁力计对齐 */
    volatile uint8_t origin_ok;       /* 1 = ENU 原点已建立 */
    volatile uint8_t valid;           /* 1 = 本帧状态有效 */
    float            mag_mode;        /* VER=124 本观测帧实际生效的地磁入口
                                       *   0 = 未修正(门控/采样去重/剔除/更新失败)
                                       *   1 = 入口A 重力法平面投影(只修正姿态角)
                                       *   2 = 入口B 矢量方式(e1 偏航+e2 倾斜) */
    uint8_t          _rsv_ekf[1];
} v5f_ekf_t;

/* ---- 保持器总成 ---- */
typedef struct {
    v5f_imu_t     imu;
    v5f_static_t  stat;          /* 动静判定结果（处理函数 2） */
    v5f_att_t     att;           /* 姿态四元数（处理函数 3） */
    v5f_vel_t     vel;           /* 世界系速度（处理函数 5） */
    v5f_mag_t     mag;
    v5f_baro_t    baro;
    v5f_gps_rmc_t gps_rmc;
    v5f_gps_gga_t gps_gga;
    v5f_gps_gsa_t gps_gsa;
    v5f_gps_gsv_t gps_gsv;
    v5f_ekf_t     ekf;       /* 导航 EKF（处理函数 7/8） */
} v5f_hold_t;

extern volatile v5f_hold_t g_v5f_hold;

/* ---------------- 函数原型 ---------------- */
/* ---- VER=97 陀螺削顶统计（日志 flags bit12~15 的镜像，供 OLED / 上位机）---- */
uint16_t v5f_imu_clip_pm(void);      /* 最近一个统计窗的削顶千分比 0~1000 */
uint8_t  v5f_imu_clip_axis(void);    /* 该窗内削顶过的轴 bit0=X bit1=Y bit2=Z */
uint32_t v5f_imu_clip_total(void);   /* 上电以来削顶帧总数 */
void SPI_DMA_Init(void);

#endif /* __SPI_RX_H */

/* VER=117: compute-load probe (see SPI_rx.c) */
uint8_t v5f_isr_probe_1s(uint32_t *n, uint32_t *avg_us, uint32_t *max_us, uint32_t *ovr);
