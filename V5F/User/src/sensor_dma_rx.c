/*==============================================================================
 * sensor_dma_rx.c —— V5F（大核）侧：陀螺 SPI2 的 RX DMA
 *
 * 照 v2 的分工：
 *   V3F：填 TX 缓冲、拉低 CS、启动 TX DMA、立即返回（SPI_ReadMulti2）
 *   V5F：持有 RX 缓冲 rxfifo，把它配给 RX DMA 通道，在完成中断里解析
 *
 * RX 缓冲是【本核自己的内存】，DMA 直接往里写，不存在跨核传地址的问题。
 *============================================================================*/
#include "sensor_dma.h"
#include "board_io.h"
#include "mipc_shm.h"
#include "sys_clk.h"

#define SPI_DMA_RX_CH    DMA1_Channel4
#define SPI_DMA_RX_MUX   DMA_MuxChannel4
#define SPI_DMA_RX_REQ   66u                 /* SPI2_RX */

/* RX 缓冲布局见 sensor_dma.h。Normal 模式下传完一帧 CNTR=0、EN 自动清，
 * 每帧由本文件的完成中断重装。 */
static volatile uint8_t rxfifo[SENSOR_IMU_RX_LEN] __attribute__((aligned(32)));

/*------------------------------------------------------------------------------
 * ISR 性能统计（照 v2 的设计）
 *
 * ISR 只累加，主循环每秒读取并打印。三个量：
 *   n/sum/max  —— 本秒帧数、ISR 总耗时、单次最劣耗时（us）
 *   ovr        —— 超过一个中断预算（陀螺周期 130us）的帧数
 * 负载 = avg_ns * n / 1e7 即得百分比×100（avg_ns×n = 本秒总耗时 ns）。
 *------------------------------------------------------------------------------*/
static volatile uint32_t s_pr_n   = 0u;
static volatile uint32_t s_pr_sum = 0u;
static volatile uint32_t s_pr_max = 0u;
static volatile uint32_t s_pr_ovr = 0u;

/* 本秒统计快照；由 dma_rx_probe_1s() 在每秒边界取走并清零累加器 */
uint8_t dma_rx_probe_1s(uint32_t *n, uint32_t *avg_ns, uint32_t *max_ns, uint32_t *ovr)
{
    static uint64_t s_t0 = 0ULL;
    uint64_t t = GetTime64_Us();

    if (s_t0 == 0ULL) { s_t0 = t; return 0u; }
    if ((t - s_t0) < 1000000ULL) return 0u;

    *n      = s_pr_n;
    *avg_ns = (s_pr_n != 0u) ? ((s_pr_sum / s_pr_n) * 10u) : 0u;   /* 10ns -> ns */
    *max_ns = s_pr_max * 10u;
    *ovr    = s_pr_ovr;

    s_t0 = t; s_pr_n = 0u; s_pr_sum = 0u; s_pr_max = 0u; s_pr_ovr = 0u;
    return 1u;
}

void SPI2_DMA_Rx_Setup(volatile uint8_t *target, uint16_t len)
{
    DMA_InitTypeDef d = {0};

    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    DMA_DeInit(SPI_DMA_RX_CH);
    d.DMA_PeripheralBaseAddr = (uint32_t)(&SPI2->DATAR);
    d.DMA_Memory0BaseAddr    = (uint32_t)target;
    d.DMA_DIR                = DMA_DIR_PeripheralSRC;       /* 外设 -> 内存 */
    d.DMA_BufferSize         = len;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    d.DMA_Mode               = DMA_Mode_Normal;
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(SPI_DMA_RX_CH, &d);

    DMA_MuxChannelConfig(SPI_DMA_RX_MUX, SPI_DMA_RX_REQ);
    SPI_DMA_RX_CH->CFGR |= DMA_IT_TC;       /* 传输完成中断 */
    DMA_Cmd(SPI_DMA_RX_CH, ENABLE);         /* 使能，等 RXNE */
}

void SPI_DMA_Init(void)
{
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    SPI2_DMA_Rx_Setup(rxfifo, SENSOR_IMU_RX_LEN);

    NVIC_SetAllocateIRQ(DMA1_Channel4_IRQn, Core_ID_V5F);
    NVIC_SetPriority(DMA1_Channel4_IRQn, (uint8_t)(1u << 5));
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    /* 通知 V3F：RX 已就绪，可以启动陀螺采集。
     * 必须先于 V3F 的首次 TX，否则首帧 DMA 落空 -> 陀螺 INT 锁存 -> 再无上升沿。 */
    if (g_shm != 0)
        g_shm->v5f_dma_ready = 1u;
}

/*==============================================================================
 * RX 完成中断：解析 -> 发布共享区 -> 抬 CS -> 重装 RX
 *
 * 抬 CS 放在即将退出处：CS 上升沿必须晚于最后一个时钟（t_h(CS)），
 * 前面几步的耗时就当自然延时。
 *============================================================================*/
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1, DMA1_IT_TC4) != RESET)
    {
        uint64_t isr_t0 = GetTime64_10Ns();   /* 10ns 刻度；用 us 会把亚 us 的 ISR 整成 0 */
        uint32_t isr_d10;

        DMA_ClearITPendingBit(DMA1, DMA1_IT_TC4);

        if (g_shm != 0)
        {
            /* ★★ 在中断【入口处】取 DRDY 时刻（2026-10-13）。
             * 该时刻由 V3F 在 DRDY 中断里写进共享区（board_io.c 的 EXTI15_8_IRQHandler），
             * 本处只做拾取 —— 这是跨核唯一可行的通路。
             * ★ 不能在解析之后取：原实现（GetTime64_10Ns()）把发布时间拖到
             *   DRDY + SPI传输(~12us) + 中断延迟 + 解析耗时，后三项都抖动
             *   ⟹ 实测逐帧 Δt 散布 ±20~30%（真实帧间隔仅 130.25us）。
             *   dt 是每次角速度积分的乘数，抖动直接注入姿态误差。
             * ★ 也不能直接引用 g_imu_drdy_ts：那属于 V3F 镜像，V5F 读到的是自己的初值 0
             *   （board_io.c 在 Common/Common，两个核各编译一份）。
             * 先读、后解析，使本时刻与 DmaBuf 里这一帧的配对窗口最小。 */
            uint64_t ts_drdy = shm_chan_ts_read(&g_shm->imu_drdy);
            /* 布局见 sensor_dma.h：[1..2]温度 [3..8]陀螺 [9..14]加计，低字节在前 */
            int16_t tp = (int16_t)(((uint16_t)rxfifo[2]  << 8) | rxfifo[1]);
            int16_t gx = (int16_t)(((uint16_t)rxfifo[4]  << 8) | rxfifo[3]);
            int16_t gy = (int16_t)(((uint16_t)rxfifo[6]  << 8) | rxfifo[5]);
            int16_t gz = (int16_t)(((uint16_t)rxfifo[8]  << 8) | rxfifo[7]);
            int16_t ax = (int16_t)(((uint16_t)rxfifo[10] << 8) | rxfifo[9]);
            int16_t ay = (int16_t)(((uint16_t)rxfifo[12] << 8) | rxfifo[11]);
            int16_t az = (int16_t)(((uint16_t)rxfifo[14] << 8) | rxfifo[13]);

            /* ★ 发布 DRDY 时刻（不是发布时刻）。ts_drdy 已在中断入口处从共享区取好。
             *   若为 0（V3F 尚未写过）则退回本核当前时刻，避免 dt_s=0 让链路停摆。 */
            shm_publish_imu((ts_drdy != 0u) ? ts_drdy : GetTime64_10Ns(),
                            tp, gx, gy, gz, ax, ay, az);

            {
            }
        }

        IMU_CS_HIGH();

        /* 重装 RX 准备下一帧（先清完成位再重装，避免重复处理） */
        SPI_DMA_RX_CH->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
        SPI_DMA_RX_CH->CNTR  = SENSOR_IMU_RX_LEN;
        SPI_DMA_RX_CH->CFGR |= DMA_CFGR1_EN;

        /* 耗时保持 10ns 计数、不换算（亚 us 会被整除成 0）。预算 = 13000 个计数。
         * 超预算的帧说明主任务区（将来的解算链）放不进这个周期。 */
        isr_d10 = (uint32_t)(GetTime64_10Ns() - isr_t0);
        s_pr_n++;
        s_pr_sum += isr_d10;
        if (isr_d10 > s_pr_max) s_pr_max = isr_d10;
        if (isr_d10 > SENSOR_ISR_BUDGET_10NS) s_pr_ovr++;
    }
}
