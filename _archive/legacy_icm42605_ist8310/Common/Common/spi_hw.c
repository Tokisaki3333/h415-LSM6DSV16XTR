/********************************** (C) COPYRIGHT  *******************************
* File Name          : spi_hw.c
* Description        : 硬件 SPI1 驱动 + ICM42605 陀螺仪（替换 board.c 软件 SPI）
*                      SPI1: SCK=PF5(AF5)  MOSI=PD7(AF5)  MISO=PF3(AF5)  CS=PF4(GPIO 软件片选)
*                      SPI mode 0（CPOL=0, CPHA=0），与旧软件 SPI 时序一致
*                      42605 协议：读命令 reg|0x80，写命令 reg&0x7F
*                      burst 连续读：TX DMA 发 命令+dummy 产生时钟，RX DMA 同步收
*                      （DMA 结构照抄 EVT EXAM/SPI/SPI_DMA 例程）
* 依赖: spi_hw.h / ch32h417.h / ch32h417_spi.h / debug.h
*********************************************************************************/
#include "spi_hw.h"
#include "ch32h417.h"
#include "debug.h"

/* ================= SPI1 引脚与配置 ================= */
#define SPI1_CS_PORT   GPIOF
#define SPI1_CS_PIN    GPIO_Pin_4
#define SPI1_SCK_SRC   GPIO_PinSource5
#define SPI1_SCK_PIN   GPIO_Pin_5
#define SPI1_MOSI_SRC  GPIO_PinSource7
#define SPI1_MOSI_PIN  GPIO_Pin_7
#define SPI1_MISO_SRC  GPIO_PinSource3
#define SPI1_MISO_PIN  GPIO_Pin_3
#define SPI1_AF        GPIO_AF5

#define SPI1_TIMEOUT   100000u

/* ================= SPI1 DMA（照抄 EVT SPI_DMA 例程：TX+RX 双通道） ================= */
#define SPI_DMA_MAXLEN  16u                  /* burst 读最大长度（六轴 12 字节 + 余量）*/
#define SPI_DMA_TX_CH   DMA1_Channel3
#define SPI_DMA_RX_CH   DMA1_Channel2
#define SPI_DMA_TX_MUX  DMA_MuxChannel3
#define SPI_DMA_RX_MUX  DMA_MuxChannel2
#define SPI_DMA_TX_REQ  63u                  /* SPI1_TX（例程 SPI2_TX=65，SPI1=63）*/
#define SPI_DMA_RX_REQ  64u                  /* SPI1_RX（用户提供）*/
#define SPI_DMA_TX_TC   DMA1_FLAG_TC3
#define SPI_DMA_RX_TC   DMA1_FLAG_TC2

/* TX 缓冲：[0]=读命令, [1..len]=dummy（RX 通道由大核全权管理，小核不碰 RX）*/
static uint8_t  SPI1_TxBuf[SPI_DMA_MAXLEN + 1];

/* ================= 底层原语 ================= */

/* TX DMA 配置（SPI_ReadMulti 使用）：命令 + dummy 由 DMA 发出产生时钟 */
static void SPI1_DMA_Tx_Init(uint16_t bufsize)
{
    DMA_InitTypeDef d = {0};

    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    DMA_DeInit(SPI_DMA_TX_CH);
    d.DMA_PeripheralBaseAddr = (u32)(&SPI1->DATAR);
    d.DMA_Memory0BaseAddr    = (u32)SPI1_TxBuf;
    d.DMA_DIR                = DMA_DIR_PeripheralDST;
    d.DMA_BufferSize         = bufsize;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    d.DMA_Mode               = DMA_Mode_Normal;
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(SPI_DMA_TX_CH, &d);

    DMA_MuxChannelConfig(SPI_DMA_TX_MUX, SPI_DMA_TX_REQ);   /* 63 = SPI1_TX */
}

/* 初始化 SPI1：PF5=SCK(AF5)、PD7=MOSI(AF5)、PF3=MISO(AF5)、PF4=CS(GPIO 输出) */
void SPI1_Init(void)
{
    GPIO_InitTypeDef g = {0};
    SPI_InitTypeDef  s = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOF | RCC_HB2Periph_GPIOD | RCC_HB2Periph_AFIO | RCC_HB2Periph_SPI1, ENABLE);

    /* CS PF4：GPIO 推挽输出，空闲高；读时按 42605 时序拉低/拉高 */
    g.GPIO_Pin   = SPI1_CS_PIN;
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOF, &g);
    GPIO_SetBits(SPI1_CS_PORT, SPI1_CS_PIN);   /* CS 空闲高 */

    /* SCK PF5(AF5) */
    GPIO_PinAFConfig(GPIOF, SPI1_SCK_SRC, SPI1_AF);
    g.GPIO_Pin   = SPI1_SCK_PIN;
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOF, &g);

    /* MOSI PD7(AF5) */
    GPIO_PinAFConfig(GPIOD, SPI1_MOSI_SRC, SPI1_AF);
    g.GPIO_Pin   = SPI1_MOSI_PIN;
    GPIO_Init(GPIOD, &g);

    /* MISO PF3(AF5)，复用输入 */
    GPIO_PinAFConfig(GPIOF, SPI1_MISO_SRC, SPI1_AF);
    g.GPIO_Pin   = SPI1_MISO_PIN;
    GPIO_Init(GPIOF, &g);

    s.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    s.SPI_Mode              = SPI_Mode_Master;
    s.SPI_DataSize          = SPI_DataSize_8b;
    s.SPI_CPOL              = SPI_CPOL_Low;              /* mode 0：SCK 空闲低 */
    s.SPI_CPHA              = SPI_CPHA_1Edge;            /* 上升沿采样 */
    s.SPI_NSS               = SPI_NSS_Soft;
    s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode3;   /* BR=011：高速模式 ÷5 = 100M/5 ≈ 20MHz */
    s.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &s);

    SPI1->HSCR |= SPI_HSCR_HSRXEN;   /* 高速模式：HSRXEN=1，BR=001 → SCK=FHCLK/3≈33.3MHz */

    SPI1_DMA_Tx_Init(SPI_DMA_MAXLEN + 1);   /* TX DMA 通道（SPI_ReadMulti 用）*/

    /* SPI 外设的 TX/RX DMA 请求使能（双核共享外设寄存器，由大核统一使能，小核不碰）*/
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);

    SPI_Cmd(SPI1, ENABLE);
}

/* RX DMA 通道配置（大核 V5F 调用，激活小核前）：目标 = 调用方指定（rxfifo），含 TCIE 并使能 EN 等待 RXNE
 * Normal 模式一帧后 CNTR=0、EN 自动清，每帧由大核 DMA 中断重装 */
void SPI1_DMA_Rx_Setup(volatile uint8_t *target, uint16_t len)
{
    DMA_InitTypeDef d = {0};

    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    DMA_DeInit(SPI_DMA_RX_CH);
    d.DMA_PeripheralBaseAddr = (u32)(&SPI1->DATAR);
    d.DMA_Memory0BaseAddr    = (u32)target;
    d.DMA_DIR                = DMA_DIR_PeripheralSRC;
    d.DMA_BufferSize         = len;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    d.DMA_Mode               = DMA_Mode_Normal;
    d.DMA_Priority           = DMA_Priority_High;
    d.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(SPI_DMA_RX_CH, &d);

    DMA_MuxChannelConfig(SPI_DMA_RX_MUX, SPI_DMA_RX_REQ);   /* 64 = SPI1_RX */
    SPI_DMA_RX_CH->CFGR |= DMA_IT_TC;                       /* TCIE：DMA 外设中断源 */
    DMA_Cmd(SPI_DMA_RX_CH, ENABLE);                         /* EN，等待 RXNE */
}

/* 纯 CPU 循环短延时（~微秒级）：不碰 SysTick——避免与主循环 Delay_Ms（同用 SysTick0）冲突 */
static void spi_busy_wait(void)
{
    volatile uint32_t i = 400;   /* ~几 us @400MHz */
    while (i--) ;
}

/* 轮询收发一字节（写命令/写数据用）*/
static uint8_t SPI1_RW(uint8_t byte)
{
    uint32_t t;

    t = SPI1_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
        if (--t == 0) { printf("[SPI1] TXE timeout\r\n"); return 0xFF; }

    SPI_I2S_SendData(SPI1, byte);

    t = SPI1_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
        if (--t == 0) { printf("[SPI1] RXNE timeout\r\n"); return 0xFF; }

    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}

/* ================= 42605 寄存器读写 ================= */

/* burst 连续读（快速版）：小核只启动 TX（CS 低 + 重装 TX 通道 + EN），产生 SPI 时钟
 * RX 通道完全由大核管理（配置/使能/每帧重装在 DMA 中断里），小核不碰 RX 寄存器
 * 不等 BSY/不抬 CS：CS 高由大核 DMA 中断 Delay 后做
 * 返回 0=成功 */
uint8_t SPI_ReadMulti(uint8_t reg, uint8_t len)
{
    uint8_t i;

    if (len == 0 || len > SPI_DMA_MAXLEN) { printf("[SPI1] len=%d err\r\n", len); return 0xFF; }

    SPI1_TxBuf[0] = reg | 0x80;                       /* 读命令：bit7=1 */
    for (i = 1; i <= len; i++) SPI1_TxBuf[i] = 0xFF;  /* dummy，产生时钟 */

    GPIO_ResetBits(SPI1_CS_PORT, SPI1_CS_PIN);        /* CS 低（小核负责：TX 前拉低）*/

    /* DMA 通道寄存器级更新（无需 DeInit/Init）：CFGR 失能 → 写 MADDR/CNTR → CFGR 使能 */
    SPI_DMA_TX_CH->CFGR  &= ~DMA_CFGR1_EN;                /* TX 失能 */
    SPI_DMA_TX_CH->MADDR  = (uint32_t)SPI1_TxBuf;         /* TX 数据源 = 本核 TxBuf（跨核 static 各自实例）*/
    SPI_DMA_TX_CH->CNTR   = len + 1;                      /* 只改长度 */
    SPI_DMA_TX_CH->CFGR  |=  DMA_CFGR1_EN;                /* TX 使能 */
    DMA_Cmd(SPI_DMA_TX_CH, ENABLE);

    /* 立即返回：不等传输完成；CS 高由大核 DMA 中断 Delay 后做，数据由大核读 */
    return 0;
}

void SPI_WriteReg(uint8_t reg, uint8_t data)
{
    GPIO_ResetBits(SPI1_CS_PORT, SPI1_CS_PIN);   /* CS 低 */
    spi_busy_wait();                                  /* t_CSN */
    SPI1_RW(reg & 0x7F);                          /* 写命令：bit7=0 */
    SPI1_RW(data);
    GPIO_SetBits(SPI1_CS_PORT, SPI1_CS_PIN);     /* CS 高 */
    spi_busy_wait();                                  /* t_CSN_HIGH */
}

void icm52605_Init_A(void)
{
    GPIO_InitTypeDef g = {0};
    EXTI_InitTypeDef e = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_SPI1, ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    
    /* PE0 = INT1(DRDY) 输入 + EXTI 上升沿（先配引脚；中断使能在等大核就绪后）*/
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOE, ENABLE);
    g.GPIO_Pin   = GPIO_Pin_0;
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOE, &g);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);
    e.EXTI_Line    = EXTI_Line0;
    e.EXTI_Mode    = EXTI_Mode_Interrupt;
    e.EXTI_Trigger = EXTI_Trigger_Rising;
    e.EXTI_LineCmd = ENABLE;
    EXTI_Init(&e);

    /* 初始化序列（手册）：先使能 G+A Low Noise，转换后 200us 不写寄存器，等 gyro 启动 45ms */
    SPI_WriteReg(0x4E, 0x0F);   /* PWR_MGMT0：GYRO_MODE=11(LN) + ACCEL_MODE=11(LN) */
    return;
}
/* icm52605_Init_B：量程 + ODR + DRDY 中断（V3F main 在 Init_A 之后调用）
 *
 * ★ 量程改了必须同步改下游：上位机换算、六面标定解出的标度/零偏都是按下面的
 *   LSB 刻度算的，对不上就是静默失效。
 *   GYRO_CONFIG0  (0x4F) = 0x03 : GYRO_FS_SEL=000 -> ±2000 dps = 16.384 LSB/dps
 *   ACCEL_CONFIG0 (0x50) = 0x03 : ACCEL_FS_SEL=000 -> ±16  g   = 2048  LSB/g
 *                                 （1 LSB = 488.3 ug = 4.7889e-3 m/s^2）
 *   两处 ODR 都是 0011 = 8 kHz。
 *   历史坑：0x4F 曾写成 0x43(=±500dps) 而注释没改；0x50 曾为 0x43(=±4g)。
 *   改量程后 gyro_FS=001(±1000)/010(±500)/011(±250)... 逐档差 2 倍，别手滑。 */
void icm52605_Init_B(void)
{
    SPI_WriteReg(0x4F, 0x03);   /* GYRO_CONFIG0：GYRO_FS_SEL=000(±2000dps) + GYRO_ODR=0011(8kHz) */
    SPI_WriteReg(0x50, 0x03);   /* ACCEL_CONFIG0：ACCEL_FS_SEL=000(±16g)  + ACCEL_ODR=0011(8kHz) */
    SPI_WriteReg(0x14, 0x03);   /* INT_CONFIG：INT1 推挽输出 + 高有效 */
    SPI_WriteReg(0x64, 0x60);   /* INT_CONFIG1：INT_ASYNC_RESET=0 + TPULSE=1(8us, ODR≥4kHz) + TDEASSERT_DISABLE=1 */
    SPI_WriteReg(0x65, 0x08);   /* INT_SOURCE0：UI_DRDY_INT1_EN=1（DRDY 路由 INT1）*/
    return;
}
