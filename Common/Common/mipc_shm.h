#ifndef MIPC_SHM_H
#define MIPC_SHM_H

#include <stdint.h>

/* =====================================================================
 * 双核共享区定义（V3F 小核定义实例，V5F 大核经 IPC 基址映射同一地址）
 * 两端必须 include 同一份本头文件，保证结构体布局完全一致。
 *
 * 发布-订阅协议（每通道独立）：
 *   写端：写数据/flags → 写 ts 高 32 → 写 ts 低 32 → __sync_synchronize() → cnt++
 *   读端：cur = cnt; if (cur != last) { __sync_synchronize(); last = cur; 读 ts 低→高; 读数据; }
 *   ts 为 DRDY（数据就绪）时刻，不是读完时刻；单位统一 10 ns 计数（GetTime64_10Ns()）。
 *
 * flags：每通道一个 uint8_t 位掩码，标明本帧哪些字段有数据（位 0 = 缺失、数值同时清零）。
 *   ★ 0 值本身可能合法（速度 0、经度 0、HDOP 0、海拔 0），判空必须查位，不要用值判空。
 *
 * 数值表示：能用 float 精确表达的物理量直接传 float（两核均按 -mabi=ilp32f 编译，硬浮点）；
 *   只有 float 精度不够时才保留定点，并逐字段注明定标（本文件的 ×1e7 度 / cm / ×100
 *   都是十进制定标整数，不是 Qm.n 二进制定点）。
 * ===================================================================== */

/* 通道头：12 B，4 字节对齐（全结构体因此无 8 字节对齐空洞） */
typedef struct {
    volatile uint32_t cnt;      /* 帧序号（发布标记，最后写） */
    volatile uint32_t ts_lo;    /* 时间戳低 32 位（最后写 / 最先读） */
    volatile uint32_t ts_hi;    /* 时间戳高 32 位 */
} chan_hdr_t;

/* ---------------- flags 位定义（每通道一个 uint8_t） ---------------- */
#define SHM_RMC_STATUS   0x01u   /* RMC 状态字段存在 */
#define SHM_RMC_POS      0x02u   /* 经纬度存在 */
#define SHM_RMC_SPEED    0x04u   /* 对地速度存在 */
#define SHM_RMC_DATE     0x08u   /* 日期存在 */
#define SHM_RMC_COURSE   0x10u   /* 对地航向（RMC 字段 8）存在 */
#define SHM_RMC_MAGVAR   0x20u   /* 磁偏角（RMC 字段 10/11，接收机自带 WMM）存在 */
#define SHM_GGA_QUALITY  0x01u   /* 定位质量存在 */
#define SHM_GGA_SV       0x02u   /* 卫星数存在 */
#define SHM_GGA_HDOP     0x04u   /* HDOP 存在 */
#define SHM_GGA_ALT      0x08u   /* 海拔存在 */
#define SHM_GSA_DOP      0x01u   /* PDOP/VDOP 存在 */
#define SHM_GSV_SNR      0x01u   /* C/N0 统计存在（GSV） */

/* ---- 陀螺仪 LSM6DSV16X：六轴 + 温度原始值 ----
 * 六轴原始值由 SPI2 RX DMA 直接落进中转缓冲，本通道发布解析结果。
 * 全部是原始 LSB，换算放消费端：
 *   陀螺     0.140 dps/LSB（±4000 dps 档）
 *   加速度计 0.061 mg/LSB （±2g 档，CTRL8 未配，复位默认）
 *   温度     raw/256 + 25 ℃（256 LSB/℃，0 LSB 约 25℃）
 * ts 为 DRDY（数据就绪）时刻，不是读完时刻。 */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile int16_t temp_raw;
    volatile int16_t gx, gy, gz;
    volatile int16_t ax, ay, az;
} imu_chan_t;                           /* 28 B */

/* ---- 地磁 MMC5983MA：18 位原始值 ----
 * 每轴 18 位放进 int32（范围 0..262143，零点 131072，16384 counts/G）。
 * 刻意不在这里换算成高斯：椭球标定（硬铁/软铁）参数还没定，
 * 原始值直通可以随时重算，换算了就丢信息。 */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile int32_t mx, my, mz;        /* 18 位原始值 */
} mmc_chan_t;                           /* 24 B */

/* ---- BMP388 气压计：float 直通 ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile float temp_celsius;        /* ℃ */
    volatile float press_pascal;        /* Pa */
    volatile uint32_t wait_us;          /* 进入 DRDY 轮询后等了多久（us）
                                         * 0/1 说明轮询开始得太晚，时序有误 */
} bmp_chan_t;                           /* 24 B */

/* ---- GPS RMC：定位 / 速度 / 航向 / 磁偏角 / 日期 ----
 * 三态判读：无 SHM_RMC_STATUS → 未启动/无语句；status=='V' → 有信号无定位；'A' → 定位有效。 */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint8_t  flags;
    volatile uint8_t  status;           /* NMEA 原字符：'A'=有效 / 'V'=无效 */
    volatile uint16_t _rsv0;            /* 对齐填充 */
    volatile float    speed_mps;        /* 对地速度 m/s（解析自 RMC 的节值 ×0.5144444） */
    volatile int32_t  lat_e7;           /* 纬度 ×1e7 度（1 LSB = 10^-7°，南纬为负）；float 精度不够，必须定点 */
    volatile int32_t  lon_e7;           /* 经度 ×1e7 度（西经为负） */
    volatile uint32_t date_ddmmyy;      /* UTC 日期 ddmmyy（十进制字面量） */
    volatile float    course_deg;       /* 对地航向（真北，0~360 度），RMC 字段 8。
                                         * ★ EKF 要的是速度**矢量**，靠 speed_mps + course_deg 才组得出来；
                                         * 只给标量速度只能约束一个方向。无此语句时值为 0 且无 SHM_RMC_COURSE。 */
    volatile float    magvar_deg;       /* 接收机自报的磁偏角（东正西负），RMC 字段 10/11。
                                         * ★ 这是**接收机内部 WMM** 的值，可与我们离线查的 WMM 互校
                                         * （本机离线值 -7.53 度）。不要与 course 混用：
                                         * course 是航向（相对真北），magvar 是磁北相对真北的偏角。 */
} gps_rmc_chan_t;                       /* 40 B，无空洞 */

/* ---- GPS GGA：定位质量 / 卫星数 / HDOP / 海拔 ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint16_t hdop_x100;        /* HDOP ×100（1 LSB = 0.01） */
    volatile uint8_t  flags;
    volatile uint8_t  quality;          /* 0=无效 1=单点 2=差分 4=RTK固定 5=RTK浮点 */
    volatile int32_t  alt_cm;           /* 海拔 cm */
    volatile uint8_t  sv;               /* 卫星数 */
    volatile uint8_t  _rsv[3];          /* 对齐填充 */
} gps_gga_chan_t;                       /* 24 B */

/* ---- GPS GSA：PDOP / VDOP（各系统 GSA 独立计数，最新一条覆盖） ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint8_t  flags;
    volatile uint8_t  _rsv0;
    volatile uint16_t pdop_x100;        /* ×100 */
    volatile uint16_t vdop_x100;        /* ×100 */
    volatile uint16_t _rsv1;
} gps_gsa_chan_t;                       /* 20 B */

/* ---- GPS GSV：C/N0（载噪比）统计 ----
 * ★ 为什么要它：静止实测 HDOP 1.00 + DGPS + 21 星，水平误差仍 3.95 m
 *   （反推 UERE 3.9 m，健康 DGPS 应 0.5~1 m）。只剩两种可能：
 *   ① 信号弱（天线/馈线/遮挡/干扰）② 伪距被多径污染。C/N0 一眼可分：
 *   平均 <35 dBHz 或最低 <25 -> 信号弱；>40 却仍差 -> 多径。
 * ★ 每个 GSV 轮次（多条消息）只在**最后一条**发布一次，所以统计是整轮的。 */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint8_t  flags;
    volatile uint8_t  sats_view;        /* 视野内卫星数（GSV 字段 3） */
    volatile uint8_t  snr_n;            /* 有有效 C/N0 的卫星数 */
    volatile uint8_t  snr_min;          /* 最小 C/N0，dBHz（0 = 无） */
    volatile uint16_t snr_avg_x10;      /* 平均 C/N0 ×10，dBHz */
    volatile uint8_t  talkers;          /* 本轮出现的 talker 数（1 = 只有合并报文）：
                                         * 用来验证轮次聚合是否正常。实测一个轮次
                                         * 有 4 个 talker（sats_view 取自 {2,3,10,11}） */
    volatile uint8_t  _rsv0;
} gps_gsv_chan_t;                       /* 20 B */                       /* 20 B */

/* ---- 低频文本通道（mipc_v3_printf，每秒一帧） ---- */
typedef struct {
    volatile uint32_t cnt;
    volatile uint8_t  data_buf[256];
} log_chan_t;                           /* 260 B */

/* ---- 整体共享区 ---- */
typedef struct {
    volatile uint32_t v5f_progress;     /* V5F 握手进度 1..6 */
    volatile uint32_t v3f_cfg_done;     /* IMU 配置完成信号 */
    volatile uint32_t imu_start_cnt;    /* 陀螺 DRDY 次数（V3F 写、V5F 读）。
                                         * 与 imu.hdr.cnt（V5F 解析后发布）对比即知丢帧：
                                         * start_cnt - cnt = 未走完 DMA 的帧数。 */
    volatile uint32_t v5f_dma_ready;    /* V5F 的 RX DMA 通道已配好、可以接收陀螺数据。
                                         * V3F 必须等它置 1 才启动采集 —— 否则首帧 DMA
                                         * 落空，陀螺 INT 锁存后就再无上升沿。 */
    volatile uint32_t mmc_start_cnt;    /* 地磁 DRDY 次数（V3F 写、V5F 读）。
                                         * 地磁不走 DMA，这个计数就是它的实际采样率。 */
    log_chan_t        log;
    imu_chan_t        imu;              /* LSM6DSV16X 六轴 + 温度 */
    mmc_chan_t        mmc;              /* MMC5983MA 三轴 18 位原始值 */
    bmp_chan_t        bmp;
    gps_rmc_chan_t    gps_rmc;
    gps_gga_chan_t    gps_gga;
    gps_gsa_chan_t    gps_gsa;
    gps_gsv_chan_t    gps_gsv;
    /* OLED 显存：128×64/8 = 1024 B，页主序 8 页 × 128 列；双核可读写，aligned(4) 便于双核/DMA 访问 */
    volatile uint8_t  oled_fb[1024] __attribute__((aligned(4)));
} shared_mem_t;

/* 共享区指针：V3F 定义 storage 并由 g_shm 指向（地址经 IPC_MSG0 传给 V5F）；V5F 同样以 g_shm 命名 */
extern volatile shared_mem_t *g_shm;

/* ---------------- 写端（小核）---------------- */

/* 写通道时间戳：高 32 位先写、低 32 位最后写，屏障保证数据/ts 先于 cnt++ 可见 */
static inline void shm_chan_ts_write(volatile chan_hdr_t *h, uint64_t ts)
{
    h->ts_hi = (uint32_t)(ts >> 32);
    h->ts_lo = (uint32_t)ts;
    __sync_synchronize();
}

static inline void shm_publish_imu(uint64_t ts_drdy,
                                   int16_t temp_raw,
                                   int16_t gx, int16_t gy, int16_t gz,
                                   int16_t ax, int16_t ay, int16_t az)
{
    g_shm->imu.temp_raw = temp_raw;
    g_shm->imu.gx = gx; g_shm->imu.gy = gy; g_shm->imu.gz = gz;
    g_shm->imu.ax = ax; g_shm->imu.ay = ay; g_shm->imu.az = az;
    shm_chan_ts_write(&g_shm->imu.hdr, ts_drdy);
    g_shm->imu.hdr.cnt++;
}

/* MMC5983MA：18 位原始值直通，不做换算（标定参数未定） */
static inline void shm_publish_mmc(uint64_t ts_drdy, int32_t mx, int32_t my, int32_t mz)
{
    g_shm->mmc.mx = mx;
    g_shm->mmc.my = my;
    g_shm->mmc.mz = mz;
    shm_chan_ts_write(&g_shm->mmc.hdr, ts_drdy);
    g_shm->mmc.hdr.cnt++;
}

/* BMP388：温度/气压直接以 float 传递（无定点往返） */
static inline void shm_publish_bmp(uint64_t ts_drdy, float temp_celsius,
                                    float press_pascal, uint32_t wait_us)
{
    g_shm->bmp.temp_celsius = temp_celsius;
    g_shm->bmp.press_pascal = press_pascal;
    g_shm->bmp.wait_us = wait_us;
    shm_chan_ts_write(&g_shm->bmp.hdr, ts_drdy);
    g_shm->bmp.hdr.cnt++;
}

/* ---------------- 读端（大核）---------------- */

/* 读通道时间戳：低 32 位先读、再读高 32 位；必须在"检测到 cnt 变化 + 屏障"之后调用 */
static inline uint64_t shm_chan_ts_read(const volatile chan_hdr_t *h)
{
    uint64_t lo = h->ts_lo;
    uint64_t hi = h->ts_hi;
    return (hi << 32) | lo;
}

#endif /* MIPC_SHM_H */
