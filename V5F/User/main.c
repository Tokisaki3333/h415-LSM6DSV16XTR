/*******************************************************************************
 * main.c —— V5F 大核
 *
 * 职责：
 *   ① 从 IPC_MSG0 取共享区地址（V3F 启动时写入）
 *   ② 等 V3F 置 v3f_cfg_done（V3F 停止 printf、开始连续采集）
 *   ③ 配好陀螺 SPI2 的 RX DMA 通道（指向本核的 rxfifo）
 *   ④ 每秒打印一次：陀螺 + 地磁 + 时间戳
 *
 * 数据来源：
 *   陀螺 —— 本核的 DMA1_Channel4 完成中断解析 rxfifo 后发布到共享区
 *   地磁 —— V3F 在自己的 DRDY 中断里发布到共享区（本核只读）
 *
 * 不碰 SPI 的初始化与片选（V3F 负责），也不碰 SDMMC / 气压 I2C / GPS。
 *******************************************************************************/
#include "debug.h"
#include "ch32h417.h"
#include "sys_clk.h"
#include "mipc_v5.h"
#include "board_io.h"      /* BOARD_IMU_Init / BOARD_IMU_Enable */
#include "mipc_shm.h"
#include "sensor_dma.h"
#include "sdmmc_sd.h"      /* SD_Init / SD_ReadDisk / SD_WriteDisk */
#include "USBCDC.h"
#include "cdc_report.h"

/*==============================================================================
 * 全速采集 -> 写 TF 卡 -> 读回 -> 串口转发
 *
 * 为什么经卡中转：采集侧带宽约 199 KB/s（IMU 7680Hz + MMC 1024Hz + BMP 42Hz），
 * 而串口只有 86 KB/s。先写卡（实测 15 MB/s），测完再回放。
 *
 * 卡当无文件结构用：直接按扇区读写，不走 FatFS。
 * 帧格式（自同步，PC 端可断流重同步）：
 *   A5 5A | TYPE | LEN | CNT(4) | 净荷 | CHK(XOR)
 *   TYPE 1=IMU 2=MMC 3=BMP 4=EOF
 */
#define SD_LOG_START_SECTOR     4096u       // 2MB 偏移
#define SD_LOG_PAD_BYTES        512u        // 前 512B 作校验暂存，同时保证 16B 对齐
#define SD_LOG_MAX_SECTORS      512u        // 单次最多 256KB
#define SD_FLUSH_WATERMARK      (224u * 1024u)
#define SD_CAPTURE_US           (15ULL * 1000000ULL)

/* SD 日志（采集->写卡->回放）开关。USB CDC 使能后 USART1 让位，故默认关闭；
 * 置 1 时需在 main 里重新调用 capture_loop() + replay_to_uart()。 */
#define V5F_SD_LOG_EN           0u

#define REC_HEAD_SZ             8u          // SYNC(2) TYPE(1) LEN(1) CNT(4)
#define IMU_PAYLOAD_SZ          18u         // ts_lo(4) + 7 * int16
#define MMC_PAYLOAD_SZ          16u         // ts_lo(4) + 3 * int32
#define BMP_PAYLOAD_SZ          20u         // ts_lo(4) + 2 * float + wait_us

enum { REC_IMU = 1, REC_MMC = 2, REC_BMP = 3, REC_EOF = 4 };

// 帧缓冲：
//   [0, 512)                 : 校验用暂存（读回一个扇区到这，与写出内容比对）
//   [SD_LOG_PAD_BYTES, ...)  : 记录填充区 / 落卡时的扇区头+数据区
static uint8_t  s_buf[SD_LOG_MAX_SECTORS * 512u + SD_LOG_PAD_BYTES]
                __attribute__((aligned(32)));
static uint32_t s_fill;                     // 已填充字节数
static uint32_t s_log_sectors;              // 已落卡扇区数
static uint32_t s_records;                  // 已封装的记录数

// 小端写入辅助
static void put_u16(uint32_t off, uint16_t v)
{
    s_buf[off + 0u] = (uint8_t)v;
    s_buf[off + 1u] = (uint8_t)(v >> 8);
}

static void put_u32(uint32_t off, uint32_t v)
{
    s_buf[off + 0u] = (uint8_t)v;
    s_buf[off + 1u] = (uint8_t)(v >> 8);
    s_buf[off + 2u] = (uint8_t)(v >> 16);
    s_buf[off + 3u] = (uint8_t)(v >> 24);
}

// 从回读的扇区头里取字段
static uint16_t rd_u16(uint32_t off)
{
    return (uint16_t)(s_buf[off] | ((uint16_t)s_buf[off + 1u] << 8));
}

static uint32_t rd_u32(uint32_t off)
{
    return (uint32_t)s_buf[off]
         | ((uint32_t)s_buf[off + 1u] << 8)
         | ((uint32_t)s_buf[off + 2u] << 16)
         | ((uint32_t)s_buf[off + 3u] << 24);
}

// 开一条记录：写帧头（CNT 用共享区的发布计数，便于 PC 查“相邻帧错同”）
static void rec_begin(uint8_t type, uint32_t cnt, uint8_t payload_len)
{
    s_buf[s_fill + 0u] = 0xA5u;
    s_buf[s_fill + 1u] = 0x5Au;
    s_buf[s_fill + 2u] = type;
    s_buf[s_fill + 3u] = payload_len;
    put_u32(s_fill + 4u, cnt);
}

// 收尾：算 XOR 校验、推进写指针
static void rec_end(void)
{
    uint32_t len = REC_HEAD_SZ + s_buf[s_fill + 3u];
    uint8_t  x = 0u;
    uint32_t i;
    for (i = 2u; i < len; i++) x ^= s_buf[s_fill + i];
    s_buf[s_fill + len] = x;
    s_fill += len + 1u;
    s_records++;
}

// ---------------------------------------------------------------------------
// 日志设备格式：每个扇区 = 16 字节头 + 496 字节数据
//   头: MAGIC(4) | SEQ(4) | LEN(2) | XOR(2) | RSV(4)
//   LEN  = 本扇区有效数据字节数（0..496）
// 这样回放时能逐扇区判定：是否有效、序号是否连续、有没有丢失/重复扇区。
// 回放时把每个扇区的 LEN 字节数据取出拼接，PC 端拿到的就是纯记录流。
// ---------------------------------------------------------------------------
#define SD_SEC_MAGIC    0x354E4C53u     /* "SLN5" */
#define SD_SEC_HDR      16u
#define SD_SEC_DATA     (512u - SD_SEC_HDR)

static uint32_t s_sec_seq;              /* 已分配的扇区序号 */
static uint32_t s_sec_nbad;             /* 写后读回校验失败次数 */
static uint32_t s_lo;                   /* 本批次原始记录字节的起点（上界用 s_fill） */
static uint32_t s_pend;                 /* 本次待写入的扇区数 */

#if (V5F_SD_LOG_EN)
// 打包：把 s_lo..s_fill 的原始记录字节切成 496B 一段，每段前面加 16B 头。
// 从后往前处理，这样目标位置永远不小于源位置，不会覆盖未处理的数据。
static void sd_pack_all(void)
{
    uint32_t raw = s_fill - s_lo;           /* 原始字节数（上界就是 s_fill） */
    uint32_t nsec = (raw + SD_SEC_DATA - 1u) / SD_SEC_DATA;
    uint32_t k;

    for (k = nsec; k > 0u; k--)
    {
        uint32_t idx = k - 1u;
        uint32_t src = s_lo + idx * SD_SEC_DATA;          /* 源起点 */
        uint32_t dst = SD_LOG_PAD_BYTES + idx * 512u;     /* 目的扇区起点 */
        uint32_t len = raw - idx * SD_SEC_DATA;
        uint32_t i;
        uint16_t x = 0u;

        if (len > SD_SEC_DATA) len = SD_SEC_DATA;

        for (i = len; i > 0u; i--)
            s_buf[dst + SD_SEC_HDR + i - 1u] = s_buf[src + i - 1u];

        put_u32(dst + 0u, SD_SEC_MAGIC);
        put_u32(dst + 4u, s_sec_seq + idx);
        put_u16(dst + 8u, (uint16_t)len);
        for (i = 0u; i < len; i++) x ^= s_buf[dst + SD_SEC_HDR + i];
        put_u16(dst + 10u, x);
        put_u32(dst + 12u, 0u);
    }
    s_sec_seq  += nsec;
    s_pend = nsec;
}

// 落卡 + 写后读回校验（逐扇区）
static void sd_flush(void)
{
    uint32_t raw = s_fill - s_lo;
    uint32_t tail, i, k;

    if (raw == 0u) return;

    sd_pack_all();

    // ---- 批量写 ----
    k = 0u;
    while (k < s_pend)
    {
        uint32_t batch = s_pend - k;
        if (batch > 255u) batch = 255u;
        if (SD_WriteDisk(s_buf + SD_LOG_PAD_BYTES + k * 512u,
                         SD_LOG_START_SECTOR + s_log_sectors + k,
                         (uint8_t)batch) != SD_OK)
        {
            s_sec_nbad++;                   /* 写失败也计入坏扇区 */
            return;
        }
        k += batch;
    }

    // ---- 写后读回校验：逐扇区读回，与刚写出的内容逐字节比 ----
    for (k = 0u; k < s_pend; k++)
    {
        const uint8_t *wr = s_buf + SD_LOG_PAD_BYTES + k * 512u;
        if (SD_ReadDisk(s_buf, SD_LOG_START_SECTOR + s_log_sectors + k, 1u) != SD_OK)
        {
            s_sec_nbad++;
            continue;
        }
        for (i = 0u; i < 512u; i++)
        {
            if (s_buf[i] != wr[i]) { s_sec_nbad++; break; }
        }
    }

    s_log_sectors += s_pend;

    /* 诊断：每次落卡都报一次，定位"写作是否真的发生" */
    printf("FLUSH,raw=%u,sec=%u,log=%u,bad=%u\r\n",
           (unsigned)raw, (unsigned)s_pend,
           (unsigned)s_log_sectors, (unsigned)s_sec_nbad);

    // ---- 把不足 SD_SEC_DATA 的尾部搬到 pad 之后，下一轮继续累积 ----
    tail = raw % SD_SEC_DATA;
    if (tail > 0u)
    {
        uint32_t src = s_lo + raw - tail;
        uint32_t dst = SD_LOG_PAD_BYTES;
        for (i = 0u; i < tail; i++) s_buf[dst + i] = s_buf[src + i];
    }
    s_lo   = SD_LOG_PAD_BYTES;
    s_fill = SD_LOG_PAD_BYTES + tail;
    s_pend = 0u;
}


// 采集：从共享区取三路数据，按帧封装，水位到就落卡
static void capture_loop(uint32_t dur_us)
{
    uint64_t t_end = GetTime64_Us() + dur_us;
    uint32_t lu = 0u, lm = 0u, lb = 0u;   // 上次见过的发布计数

    while ((int64_t)(t_end - GetTime64_Us()) > 0)
    {
        // ---- IMU：tp,gx,gy,gz,ax,ay,az，全部 int16 原始值 ----
        if (g_shm->imu.hdr.cnt != lu)
        {
            lu = g_shm->imu.hdr.cnt;
            if (s_fill + REC_HEAD_SZ + IMU_PAYLOAD_SZ + 1u < sizeof(s_buf))
            {
                rec_begin(REC_IMU, lu, IMU_PAYLOAD_SZ);
                put_u32(s_fill + REC_HEAD_SZ, g_shm->imu.hdr.ts_lo);
                put_u16(s_fill + REC_HEAD_SZ + 4u,  (uint16_t)g_shm->imu.temp_raw);
                put_u16(s_fill + REC_HEAD_SZ + 6u,  (uint16_t)g_shm->imu.gx);
                put_u16(s_fill + REC_HEAD_SZ + 8u,  (uint16_t)g_shm->imu.gy);
                put_u16(s_fill + REC_HEAD_SZ + 10u, (uint16_t)g_shm->imu.gz);
                put_u16(s_fill + REC_HEAD_SZ + 12u, (uint16_t)g_shm->imu.ax);
                put_u16(s_fill + REC_HEAD_SZ + 14u, (uint16_t)g_shm->imu.ay);
                put_u16(s_fill + REC_HEAD_SZ + 16u, (uint16_t)g_shm->imu.az);
                rec_end();
            }
        }
        // ---- 地磁：mx,my,mz（18 位原始值放在 int32）----
        if (g_shm->mmc.hdr.cnt != lm)
        {
            lm = g_shm->mmc.hdr.cnt;
            if (s_fill + REC_HEAD_SZ + MMC_PAYLOAD_SZ + 1u < sizeof(s_buf))
            {
                rec_begin(REC_MMC, lm, MMC_PAYLOAD_SZ);
                put_u32(s_fill + REC_HEAD_SZ, g_shm->mmc.hdr.ts_lo);
                put_u32(s_fill + REC_HEAD_SZ + 4u,  (uint32_t)g_shm->mmc.mx);
                put_u32(s_fill + REC_HEAD_SZ + 8u,  (uint32_t)g_shm->mmc.my);
                put_u32(s_fill + REC_HEAD_SZ + 12u, (uint32_t)g_shm->mmc.mz);
                rec_end();
            }
        }
        // ---- 气压：float t/p，直接按位搬 ----
        if (g_shm->bmp.hdr.cnt != lb)
        {
            lb = g_shm->bmp.hdr.cnt;
            if (s_fill + REC_HEAD_SZ + BMP_PAYLOAD_SZ + 1u < sizeof(s_buf))
            {
                union { float f; uint32_t u; } cv;
                rec_begin(REC_BMP, lb, BMP_PAYLOAD_SZ);
                put_u32(s_fill + REC_HEAD_SZ, g_shm->bmp.hdr.ts_lo);
                cv.f = g_shm->bmp.temp_celsius;  put_u32(s_fill + REC_HEAD_SZ + 4u,  cv.u);
                cv.f = g_shm->bmp.press_pascal;  put_u32(s_fill + REC_HEAD_SZ + 8u,  cv.u);
                put_u32(s_fill + REC_HEAD_SZ + 12u, g_shm->bmp.wait_us);
                rec_end();
            }
        }

        if (s_fill >= SD_FLUSH_WATERMARK) sd_flush();
    }

    // 收尾：补一条 EOF（带总扇区数与总记录数），再落卡
    if (s_fill + REC_HEAD_SZ + 8u + 1u < sizeof(s_buf))
    {
        rec_begin(REC_EOF, 0u, 8u);
        put_u32(s_fill + REC_HEAD_SZ, s_log_sectors);
        put_u32(s_fill + REC_HEAD_SZ + 4u, s_records);
        rec_end();
    }
    sd_flush();
}

// 回放：逐扇区校验头，只把有效数据段发给 PC（PC 收到的是纯记录流）
static void replay_to_uart(void)
{
    uint32_t sec;
    for (sec = 0u; sec < s_log_sectors; sec++)
    {
        const uint8_t *h;
        uint32_t len, i;

        if (SD_ReadDisk(s_buf, SD_LOG_START_SECTOR + sec, 1u) != SD_OK)
        {
            printf("REPLAY_FAIL,sector=%u\r\n", (unsigned)(SD_LOG_START_SECTOR + sec));
            return;
        }
        h = s_buf;

        if (rd_u32(0u) != SD_SEC_MAGIC)     /* 扇区无效 -> 只报，不发 */
        {
            printf("BADSEC,seq=%u\r\n", (unsigned)sec);
            continue;
        }
        len = rd_u16(8u);
        if (len > SD_SEC_DATA) len = SD_SEC_DATA;
        for (i = 0u; i < len; i++)
        {
            while ((USART1->STATR & 0x80u) == 0u) { }
            USART1->DATAR = s_buf[SD_SEC_HDR + i];
        }
    }
    while ((USART1->STATR & 0x40u) == 0u) { }   // 等最后一位发完
}


#endif /* V5F_SD_LOG_EN */

int main(void)
{
    uint32_t last_imuc = 0u;
    uint32_t last_imustart = 0u;
    uint32_t last_mmcc = 0u;
    uint32_t last_pubmc = 0u;
    uint32_t last_bmpc = 0u;
    uint64_t t_next;
    uint64_t t_cap0 = 0u;

    SystemAndCoreClockUpdate();
    Delay_Init();

    printf("\r\n[V5F] start\r\n");

    /* 取共享区地址。必须在 RX DMA 使能之前完成 —— 否则 g_shm 还是 NULL，
     * 完成中断里的发布会被 if (g_shm) 静默丢弃。 */
    (void)mipc_v5_get_shared();

    /* 等 V3F 配好器件并切到"只发布不打印" */
    while (g_shm->v3f_cfg_done == 0u) Delay_Ms(1);
    printf("[V5F] shm=%08X, v3f_cfg_done=%u\r\n",
           (unsigned)(uintptr_t)g_shm, (unsigned)g_shm->v3f_cfg_done);

    /* 配 RX DMA（目标 = 本核 rxfifo）并开完成中断 */
    SPI_DMA_Init();

    /*--------------------------------------------------------------------------
     * 陀螺由本核（消费方）自己使能
     *
     * BOARD_IMU_Init/Enable 在板级配置库里，不依赖任何全局/静态量，
     * 两核都能调用。由消费方使能是为了保证顺序：
     *   ① 先把自己的 RX DMA 武装好（SPI_DMA_Init）
     *   ② 再配置并启动陀螺
     * 若反过来，陀螺产生 DRDY 时 RX 还没就绪 -> INT 锁存为高 ->
     * 上升沿再也不来（实测：IMU,hz=2,pub=0）。
     *------------------------------------------------------------------------*/
    printf("LSM6,init_rc=%d\r\n", BOARD_IMU_Init());
    printf("LSM6,enable_rc=%d\r\n", BOARD_IMU_Enable());
    printf("[V5F] Ready.\r\n");

    /*--------------------------------------------------------------------------
     * USBHS CDC 上报（使能后 USART1 让位，数据走 CDC 上行）
     *------------------------------------------------------------------------*/
    cdc_reset_seq();
    usbhs_hid_enable();
    /*--------------------------------------------------------------------------
     * 主循环：每秒打印一次
     *------------------------------------------------------------------------*/
    t_next = GetTime64_Us() + 1000000ULL;

    for (;;)
    {
        usbhs_hid_poll();
        cdc_rx_poll();

        if ((int64_t)(t_next - GetTime64_Us()) <= 0)
        {
            uint32_t ic = g_shm->imu.hdr.cnt;
            uint32_t mc = g_shm->mmc.hdr.cnt;
            uint32_t ms = g_shm->mmc_start_cnt;
            uint32_t bc = g_shm->bmp.hdr.cnt;

            printf("IMU,hz=%u,pub=%u,tim=%u,g=%d,%d,%d,a=%d,%d,%d,tp=%d\r\n",
                   (unsigned)(g_shm->imu_start_cnt - last_imustart),
                   (unsigned)(ic - last_imuc),
                   (unsigned)(g_shm->imu.hdr.ts_lo / 100000ULL),
                   (int)g_shm->imu.gx, (int)g_shm->imu.gy, (int)g_shm->imu.gz,
                   (int)g_shm->imu.ax, (int)g_shm->imu.ay, (int)g_shm->imu.az,
                   (int)g_shm->imu.temp_raw);

            /* 气压计：只读共享区，不在采集路径上报 */
            printf("BARO,pub=%u,tim=%u,t=%.2f,p=%.1f,wait_us=%u\r\n",
                   (unsigned)(bc - last_bmpc),
                   (unsigned)(g_shm->bmp.hdr.ts_lo / 100000ULL),
                   (double)g_shm->bmp.temp_celsius,
                   (double)g_shm->bmp.press_pascal,
                   (unsigned)g_shm->bmp.wait_us);

            /* 双核同步标记：pg = v5f_progress, cd = v3f_cfg_done, dr = v5f_dma_ready */
            printf("SYNC,cd=%u,dr=%u,pg=%u\r\n",
                   (unsigned)g_shm->v3f_cfg_done,
                   (unsigned)g_shm->v5f_dma_ready,
                   (unsigned)g_shm->v5f_progress);

            printf("MMC,hz=%u,pub=%u,tim=%u,raw=%d,%d,%d\r\n",
                   (unsigned)(ms - last_mmcc),
                   (unsigned)(mc - last_pubmc),
                   (unsigned)(g_shm->mmc.hdr.ts_lo / 100000ULL),
                   (int)g_shm->mmc.mx, (int)g_shm->mmc.my, (int)g_shm->mmc.mz);

            last_imustart = g_shm->imu_start_cnt;
            last_imuc = ic;
            last_pubmc = mc;
            last_mmcc = ms;
            last_bmpc = bc;

            /* 负载 = 本秒 ISR 总耗时(ns)/1e9*100%，存成 %×100：avg_ns*n/1e7 */
            {
                uint32_t n = 0u, avg = 0u, mx = 0u, ovr = 0u;
                if (dma_rx_probe_1s(&n, &avg, &mx, &ovr))
                {
                    uint32_t load = (uint32_t)(((uint64_t)avg * (uint64_t)n) / 10000000ULL);
                    printf("PRF,isr_hz=%u,avg_ns=%u,max_ns=%u,ovr130=%u,load=%u.%02u%%\r\n",
                           (unsigned)n, (unsigned)avg, (unsigned)mx, (unsigned)ovr,
                           (unsigned)(load / 100u), (unsigned)(load % 100u));
                }
            }

            {
                uint8_t pl[PL_STAT];
                uint32_t v;
                v = g_cdc_tx_pkts;
                pl[0]=(uint8_t)v; pl[1]=(uint8_t)(v>>8); pl[2]=(uint8_t)(v>>16); pl[3]=(uint8_t)(v>>24);
                v = g_cdc_tx_drop;
                pl[4]=(uint8_t)v; pl[5]=(uint8_t)(v>>8); pl[6]=(uint8_t)(v>>16); pl[7]=(uint8_t)(v>>24);
                v = g_shm->imu_start_cnt;
                pl[8]=(uint8_t)v; pl[9]=(uint8_t)(v>>8); pl[10]=(uint8_t)(v>>16); pl[11]=(uint8_t)(v>>24);
                v = g_shm->mmc_start_cnt;
                pl[12]=(uint8_t)v; pl[13]=(uint8_t)(v>>8); pl[14]=(uint8_t)(v>>16); pl[15]=(uint8_t)(v>>24);
                (void)cdc_send(PKT_TYPE_STAT, PKT_FLAG_TSOK, GetTime64_10Ns(), pl, PL_STAT);
            }

            t_next = GetTime64_Us() + 1000000ULL;
        }
    }
}
