/*==============================================================================
 * sensor_dma.c —— V3F（小核）侧：初始化 + 两个 DRDY 中断处理体
 *
 * 【本文件只放小核的东西】
 *   大核的两个 DMA 完成中断在 V5F/User/src/sensor_dma_rx.c。
 *   为什么必须分开：V5F 已有一份同名的 DMA1_Channel2_IRQHandler（旧 SPI_rx.c），
 *   若把完成中断也放这里，V5F 链接时会重复定义。
 *   而且 V5F 的构建清单里根本没有 sensor_spi.c，本文件依赖它。
 *
 * 【本文件做四件事】
 *   ① 定义 DMA 缓冲（放共享内存，大核读同一份）
 *   ② 初始化 DMA 通道 + Mux 绑定 + 打开 SPI 的 DMA 请求
 *   ③ 分配两个 DMA 完成中断给大核 V5F，并给它们设优先级
 *   ④ 提供两个 DRDY 处理体：记时间戳 -> 拉 CS -> 启动 TX DMA
 *
 * 详见 sensor_dma.h 的架构说明。
 *============================================================================*/
#include "sensor_dma.h"
#include "board_io.h"
#include "sys_clk.h"
#include "mipc_shm.h"
#include "sensor_spi.h"

/*==============================================================================
 * 1. DMA 缓冲定义 —— 小核定义，大核 extern 引用
 *
 * TX 缓冲：[0] = 读命令，[1..len] = dummy（0xFF）。
 *          dummy 的作用是产生时钟；器件在时钟上把数据吐回来，RX 通道同步收。
 * RX 缓冲：长度 = 命令 + 数据。收到的 [0] 是发命令时同时收到的无用字节，
 *          真正数据从 [1] 开始。
 *============================================================================*/
uint8_t g_mmc_tx[SENSOR_MMC_FRAME_LEN] __attribute__((aligned(32)));
uint8_t g_mmc_rx[SENSOR_MMC_FRAME_LEN] __attribute__((aligned(32)));
uint8_t g_imu_tx[SENSOR_IMU_FRAME_LEN] __attribute__((aligned(32)));
uint8_t g_imu_rx[SENSOR_IMU_FRAME_LEN] __attribute__((aligned(32)));

volatile uint64_t g_mmc_drdy_ts = 0u;
volatile uint64_t g_imu_drdy_ts = 0u;

/* 帧计数（小核自己数启动次数，用于确认 DRDY 真的在来）*/
static volatile uint32_t s_mmc_start = 0u;
static volatile uint32_t s_imu_start = 0u;

/*------------------------------------------------------------------------------
 * 计时定位用：传感器启动时刻与首次进入 DRDY 中断的时刻
 *
 * 【为什么要这三个时间戳】
 * "启动转换 -> 首次进 DRDY 中断 -> 大核进 DMA-RX 中断"是一条完整链条，
 * 任何一环断开，用三个时刻的差值就能一眼定位断点：
 *   启动后一直没有首次中断   -> 器件没产数据，或 EXTI 没收到
 *   有首次中断但大核一直没进  -> DMA 没完成（TX 没发 / RX 没配好）
 * 比任何"兜底"或"探针"都直接。
 *---------------------------------------------------------------------------*/
volatile uint64_t g_mmc_enable_ts = 0u;     /* MMC 启动连续转换的时刻 */
volatile uint64_t g_imu_enable_ts = 0u;     /* 陀螺写入 ODR 的时刻 */
volatile uint64_t g_mmc_first_isr_ts = 0u;  /* 首次进入地磁 DRDY 中断 */
volatile uint64_t g_imu_first_isr_ts = 0u;  /* 首次进入陀螺 DRDY 中断 */

/*==============================================================================
 * 2. 小核侧：DRDY 中断的处理体
 *
 * 只做四件事，全部整数、不碰浮点：
 *   ① 记 DRDY 时刻（首次另存一份，供计时定位）
 *   ② 拉低 CS
 *   ③ 重装并启动 TX DMA（命令 + dummy，产生时钟）
 *   ④ 返回
 * RX 通道在初始化时已使能，不需要在这里动。
 *
 * 【CS 归属】小核只拉低；抬起由大核的 DMA 完成中断做 ——
 * 因为只有大核知道传输什么时候真正结束。
 *============================================================================*/

void SENSOR_MMC_DrdyIsr(void)
{
    g_mmc_drdy_ts = GetTime64_10Ns();
    if (s_mmc_start == 0u)
        g_mmc_first_isr_ts = g_mmc_drdy_ts;     /* 首次进入，记下供定位 */
    s_mmc_start++;

    MMC_CS_LOW();

    /* 重装并启动 TX DMA。
     * DMA 的 CFGR 是 16 位寄存器，操作必须按 16 位来，
     * 用 uint16_t 强转避免把高 16 位误清零。 */
    DMA1_Channel3->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
    DMA1_Channel3->MADDR = (uint32_t)g_mmc_tx;
    DMA1_Channel3->CNTR  = SENSOR_MMC_FRAME_LEN;
    DMA1_Channel3->CFGR |= DMA_CFGR1_EN;
}

void SENSOR_IMU_DrdyIsr(void)
{
    g_imu_drdy_ts = GetTime64_10Ns();
    if (s_imu_start == 0u)
        g_imu_first_isr_ts = g_imu_drdy_ts;
    s_imu_start++;

    IMU_CS_LOW();

    DMA1_Channel5->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
    DMA1_Channel5->MADDR = (uint32_t)g_imu_tx;
    DMA1_Channel5->CNTR  = SENSOR_IMU_FRAME_LEN;
    DMA1_Channel5->CFGR |= DMA_CFGR1_EN;
}


/*==============================================================================
 * 3. DMA 通道初始化
 *============================================================================*/

static void dma_tx_init(DMA_Channel_TypeDef *ch, SPI_TypeDef *spi,
                        const uint8_t *buf, uint16_t len)
{
    DMA_InitTypeDef d = {0};

    DMA_DeInit(ch);
    d.DMA_PeripheralBaseAddr = (uint32_t)(&spi->DATAR);
    d.DMA_Memory0BaseAddr    = (uint32_t)buf;
    d.DMA_DIR                = DMA_DIR_PeripheralDST;       /* 内存 -> 外设 */
    d.DMA_BufferSize         = len;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    d.DMA_Mode               = DMA_Mode_Normal;
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(ch, &d);
}

static void dma_rx_init(DMA_Channel_TypeDef *ch, SPI_TypeDef *spi,
                        const uint8_t *buf, uint16_t len, uint32_t prio)
{
    DMA_InitTypeDef d = {0};

    DMA_DeInit(ch);
    d.DMA_PeripheralBaseAddr = (uint32_t)(&spi->DATAR);
    d.DMA_Memory0BaseAddr    = (uint32_t)buf;
    d.DMA_DIR                = DMA_DIR_PeripheralSRC;       /* 外设 -> 内存 */
    d.DMA_BufferSize         = len;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    d.DMA_Mode               = DMA_Mode_Normal;
    d.DMA_Priority           = prio;
    d.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(ch, &d);
}

static void dma_buffers_fill(void)
{
    uint16_t i;

    g_mmc_tx[0] = SENSOR_MMC_READ_CMD;
    for (i = 1u; i < SENSOR_MMC_FRAME_LEN; i++) g_mmc_tx[i] = 0xFFu;

    g_imu_tx[0] = SENSOR_IMU_READ_CMD;
    for (i = 1u; i < SENSOR_IMU_FRAME_LEN; i++) g_imu_tx[i] = 0xFFu;
}

void SENSOR_DMA_Init(void)
{
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    dma_buffers_fill();

    /* --- Mux 绑定：两条 SPI 用不同的 DMA 通道，见 board_io.c 9.11 ---
     * SPI1 RX/TX = Mux2/Mux3（DMA1_Ch2/Ch3），SPI2 = Mux4/Mux5（DMA1_Ch4/Ch5）。
     * 旧架构里两条 SPI 共用 Ch2/Ch3，DMA 只能二选一；本架构要求同时跑。 */
    BOARD_SPI1_DMA_MuxInit();
    BOARD_SPI2_DMA_MuxInit();

    /* --- TX 通道：命令 + dummy，由 DRDY 中断按需启动 --- */
    dma_tx_init(DMA1_Channel3, SPI1, g_mmc_tx, SENSOR_MMC_FRAME_LEN);
    dma_tx_init(DMA1_Channel5, SPI2, g_imu_tx, SENSOR_IMU_FRAME_LEN);

    /* --- RX 通道 ---
     * 优先级：陀螺 VeryHigh、地磁 High。
     * 方向与中断优先级【相反】，因为保护的资源不同：
     *   中断级保护"谁截止时间更紧" -> 地磁最高（它会被陀螺反复侵入数十 us）
     *   DMA 通道级保护"谁更不能溢出" -> 陀螺最高（连续流，被延迟会丢字节）
     * 优先级必须在 DMA_Init 时给定，不能事后直写 CFGR 的 PL 位。 */
    dma_rx_init(DMA1_Channel2, SPI1, g_mmc_rx, SENSOR_MMC_FRAME_LEN,
                DMA_Priority_High);
    dma_rx_init(DMA1_Channel4, SPI2, g_imu_rx, SENSOR_IMU_FRAME_LEN,
                DMA_Priority_VeryHigh);

    /* --- 使能 RX（TX 由 DRDY 中断启动）；开 DMA 传输完成中断 --- */
    DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    /* --- 打开 SPI 的 DMA 收发请求（初始化时一次，之后不再动）---
     * 若不开，DMA 通道使能了但外设不发请求，永远等不到完成中断。
     *
     * 为什么可以一直开着：TX 通道只在 DRDY 中断里重新使能，所以不会有
     * "旧数据被提前发出"的问题 —— 通道使能时就必须立刻发。
     * 这与旧工程 spi_hw.c 的做法一致（它在 SPI1_Init 里 DMACmd 一次，
     * 之后只在 SPI_ReadMulti 里操作通道的 EN 位）。 */
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);

    /* --- 两个 DMA 完成中断都归【大核 V5F】---
     * IRQn 都 > 31（Ch2=38, Ch4=40），所以 NVIC_SetAllocateIRQ 有效；
     * 若不满足 >31 的条件，该函数什么都不做，会静默归错核。 */
    NVIC_SetAllocateIRQ(DMA1_Channel2_IRQn, Core_ID_V5F);
    NVIC_SetAllocateIRQ(DMA1_Channel4_IRQn, Core_ID_V5F);

    /* 优先级方向：地磁【高于】陀螺。
     * 理由：地磁是短中断（解析 6 字节），陀螺会实打实侵入它数十 us。
     * 反过来让陀螺更高，地磁就会被反复打断。
     * 编码（core_riscv.h）：V5F 在 4 位子优先级模式下 bit[7:5]=抢占、bit[4]=子，
     * 所以 (preempt << 5) 的形式。 */
    NVIC_SetPriority(DMA1_Channel2_IRQn, (uint8_t)(0u << 5));   /* 地磁：最高 */
    NVIC_SetPriority(DMA1_Channel4_IRQn, (uint8_t)(1u << 5));   /* 陀螺：次之 */

    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/*==============================================================================
 * 使能 DRDY 的 EXTI 中断
 *
 * 【为什么单独一个函数、而且必须由调用方在"清锁存之后"调用】
 * EXTI15_8 是分组向量，服务 PB10(复位键)、PE1(地磁 DRDY)、PD11(陀螺 DRDY)。
 *
 * 三个要求互相冲突，必须定一个明确顺序：
 *   ① 地磁 INT 是【高有效 + 电平锁存】的 —— 上电后只要有过一次测量完成
 *      就一直为高。而 EXTI 配的是【上升沿触发】：引脚已经是高，
 *      就永远等不到 0→1 边沿，中断一次都不会进。
 *      实测现象正是"PE1 恒高 + mmc_cnt=0 + first_isr=0"。
 *   ② 所以必须【先清锁存让引脚回低】，再使能 EXTI。
 *   ③ 使能之后才启动转换，第一次测量完成才有真正的上升沿。
 *
 * 正确顺序（调用方保证）：
 *   MMC_Init()（配置，不测量）
 *     -> MMC_ClearLatch()（清锁存，PE1 回低）
 *     -> SENSOR_DMA_ArmDrdy()（本函数，使能 EXTI）
 *     -> MMC_Enable()（启动连续转换）
 *============================================================================*/
void SENSOR_DMA_ArmDrdy(void)
{
    /* EXTI15_8_IRQn = 33 > 31，NVIC_SetAllocateIRQ 有效。
     * 若不满足 >31 的条件该函数什么都不做，会静默归错核（默认 V5F），
     * 那样 V3F 永远收不到中断，所以必须显式分配。 */
    NVIC_SetAllocateIRQ(EXTI15_8_IRQn, Core_ID_V3F);
    NVIC_SetPriority(EXTI15_8_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_8_IRQn);

    printf("ARM,exti15_8_enabled,pe1=%u\r\n", (unsigned)MMC_DRDY_READ());
}

/*==============================================================================
 * 6. 中断链路诊断（把寄存器读回来，不靠推理）
 *
 * 【为什么必须读回】
 * "计数为 0" 有三种互斥原因，处理方向完全不同：
 *   ① 器件没拉高引脚            -> 器件侧（DRDY 路由/使能没生效）
 *   ② 引脚跳变但 EXTI 没置挂起  -> EXTI 线映射错（AFIO 选择子写错）
 *   ③ EXTI 挂起但 NVIC 没使能   -> 分配/使能没生效
 * 读回这几组寄存器即可一次区分。
 *
 * NVIC 寄存器布局（core_riscv.h）：
 *   NVIC->IALLOCR[n]  中断分配给哪个核（bit0：1=V5F, 0=V3F）
 *   NVIC->IENR[n]     使能位
 *   NVIC->IPRIOR[n]   优先级
 * 注意：只有 IRQn > 31 才有 IALLOCR 项，EXTI15_8_IRQn = 33 属于该区间。
 *============================================================================*/
void SENSOR_DMA_Diag(void)
{
    uint32_t mmc_edges = 0u, imu_edges = 0u, k = 0u;
    uint8_t  mp, ip;
    uint64_t t0;
    uint32_t start_m = s_mmc_start, start_i = s_imu_start;

    /* --- 0. 帧计数：确认 DRDY 处理体到底进过没有 --- */
    printf("DIAG,starts,mmc=%u,imu=%u\r\n",
           (unsigned)s_mmc_start, (unsigned)s_imu_start);

    /* --- 1. EXTI 三条线（PE1=1, PB10=10, PD11=11）的使能与触发沿 ---
     * 这是最该核实的一组：我"以为"设了上升沿，但必须读回来确认。 */
    printf("DIAG,exti,intenr=0x%08X,rtenr=0x%08X,ftenr=0x%08X,intfr=0x%08X\r\n",
           (unsigned)EXTI->INTENR, (unsigned)EXTI->RTENR,
           (unsigned)EXTI->FTENR, (unsigned)EXTI->INTFR);
    printf("DIAG,exti_l1,inten=%u,ris=%u,fal=%u\r\n",
           (unsigned)((EXTI->INTENR >> 1) & 1u),
           (unsigned)((EXTI->RTENR >> 1) & 1u),
           (unsigned)((EXTI->FTENR >> 1) & 1u));
    printf("DIAG,exti_l11,inten=%u,ris=%u,fal=%u\r\n",
           (unsigned)((EXTI->INTENR >> 11) & 1u),
           (unsigned)((EXTI->RTENR >> 11) & 1u),
           (unsigned)((EXTI->FTENR >> 11) & 1u));

    /* --- 2. NVIC：向量 33 归哪个核、是否使能、优先级 --- */
    printf("DIAG,nvic,alloc33=%u,ienr33=%u,prio33=%u\r\n",
           (unsigned)(NVIC->IALLOCR[EXTI15_8_IRQn] & 1u),
           (unsigned)((NVIC->IENR[EXTI15_8_IRQn >> 5] >> (EXTI15_8_IRQn & 0x1F)) & 1u),
           (unsigned)NVIC->IPRIOR[EXTI15_8_IRQn]);

    /* --- 3. DMA 通道与 SPI 的 DMA 请求位 --- */
    printf("DIAG,dma,ch2_cfgr=0x%04X,ch2_cntr=%u,ch4_cfgr=0x%04X,ch4_cntr=%u\r\n",
           (unsigned)DMA1_Channel2->CFGR, (unsigned)DMA1_Channel2->CNTR,
           (unsigned)DMA1_Channel4->CFGR, (unsigned)DMA1_Channel4->CNTR);
    printf("DIAG,spi,spi1_ctlr2=0x%04X,spi2_ctlr2=0x%04X\r\n",
           (unsigned)SPI1->CTLR2, (unsigned)SPI2->CTLR2);

    /* --- 4. 两条 DRDY 的当前电平 --- */
    printf("DIAG,pin,mmc=%u,imu=%u\r\n",
           (unsigned)MMC_DRDY_READ(), (unsigned)IMU_INT1_READ());

    /* --- 5. 200ms 内统计两条 DRDY 的【上升沿】次数与中断触发次数 ---
     * 判据与 EXTI_Trigger_Rising 一致，所以 edges 可直接与 *_start 对比：
     *   edges > 0 而 start 不变  -> 引脚有信号但中断没进（EXTI/NVIC 侧问题）
     *   edges = 0                -> 器件侧根本没拉引脚 */
    mp = (uint8_t)MMC_DRDY_READ();
    ip = (uint8_t)IMU_INT1_READ();
    t0 = GetTime64_Us();
    while ((GetTime64_Us() - t0) < 200000ULL)
    {
        uint8_t m = (uint8_t)MMC_DRDY_READ();
        uint8_t i = (uint8_t)IMU_INT1_READ();

        if (m != 0u && mp == 0u) mmc_edges++;
        if (i != 0u && ip == 0u) imu_edges++;
        mp = m; ip = i; k++;
    }

    printf("DIAG,edges,mmc=%u,imu=%u,loops=%u,isr_mmc=%u,isr_imu=%u\r\n",
           (unsigned)mmc_edges, (unsigned)imu_edges, (unsigned)k,
           (unsigned)(s_mmc_start - start_m), (unsigned)(s_imu_start - start_i));
}

/*==============================================================================
 * 4. 统计
 *
 * 这里报的是"小核启动了多少次 DMA"，即 DRDY 来了多少次。
 * 与共享区里 gyro/mmc 通道的 cnt 对比，就能看出有没有丢帧
 * （启动次数 > 发布次数 => 有帧没走完）。
 *============================================================================*/
void SENSOR_DMA_Stats(uint32_t *mmc_cnt, uint32_t *imu_cnt)
{
    if (mmc_cnt) *mmc_cnt = s_mmc_start;
    if (imu_cnt) *imu_cnt = s_imu_start;
}

