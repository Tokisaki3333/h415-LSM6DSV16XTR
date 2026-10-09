#include "sensor_spi.h"
#include "board_io.h"
#include "sys_clk.h"
#include <stdio.h>

/* 单字节收发等待上限（us）。
 * 【为什么必须有超时】原先是 `while (flag == RESET) ;` 死等 ——
 * 一旦 SPI 不产生 TXE/RXNE（外设未使能、时钟被关、引脚配错），
 * 整个核就永久卡死在这里，连串口输出都停了，极难定位。
 * 给上限之后：出错会返回 0xFF 并继续跑，故障就变成可观测的。 */
#define SPI_XFER_TIMEOUT_US     50u

/* SPI 全双工收发一个字节（纯轮询，不用中断/DMA）。
 * 超时返回 0xFF；因为 0xFF 也正是"总线空闲"读到的值，语义一致。 */
static uint8_t spi_xfer(SPI_TypeDef *spi, uint8_t tx)
{
    uint64_t t0 = GetTime64_Us();

    while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET)
    {
        if ((GetTime64_Us() - t0) > SPI_XFER_TIMEOUT_US)
            return 0xFFu;                       /* TXE 一直不来：外设可能没使能 */
    }

    SPI_I2S_SendData(spi, tx);

    t0 = GetTime64_Us();
    while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if ((GetTime64_Us() - t0) > SPI_XFER_TIMEOUT_US)
            return 0xFFu;                       /* RXNE 一直不来：时钟没走起来 */
    }

    return (uint8_t)SPI_I2S_ReceiveData(spi);
}

static void cs_low(GPIO_TypeDef *port, uint16_t pin)
{
    if (pin != 0u) GPIO_ResetBits(port, pin);
}

static void cs_high(GPIO_TypeDef *port, uint16_t pin)
{
    if (pin != 0u) GPIO_SetBits(port, pin);
}

/*==============================================================================
 * CPU 阻塞式访问时的 DMA 冲突处理
 *
 * 【问题】SPI 的 RX DMA 请求一旦打开（DMA 路径初始化时做的），DMA 会与 CPU
 * 争抢同一个 DATAR 寄存器：
 *   发一个字节 -> 器件回一个字节进 DATAR -> RXNE 置位，同时 DMA 请求有效
 *   -> DMA 硬件【立刻】把 DATAR 读走 -> RXNE 被清
 *   -> CPU 侧 spi_xfer 永远等不到 RXNE -> 超时失败
 *
 * 【现象】加 DMA 之前，纯轮询读写一切正常；加 DMA 之后，
 * SPI2 的阻塞式写开始超时（LSM6_Enable 里两次写各 1ms 超时，
 * 表现为"输出静默 2 秒后停住"）。
 *
 * 【为什么之前单独测试正常】那时根本没装 DMA 通道，CPU 独占总线。
 *
 * 【修法】CPU 要做阻塞访问时，临时【关掉 RX DMA 请求位】而不是关通道 ——
 * 这样再打开时无需重装 CNTR/EN，通道状态不受影响。
 * 每次调用只多两次寄存器读改写，代价可忽略。
 *============================================================================*/
static uint16_t spi_dma_rx_suspend(SPI_TypeDef *spi)
{
    uint16_t saved = spi->CTLR2;
    spi->CTLR2 = (uint16_t)(saved & (uint16_t)(~SPI_I2S_DMAReq_Rx));
    return saved;
}

static void spi_dma_rx_resume(SPI_TypeDef *spi, uint16_t saved)
{
    spi->CTLR2 = saved;
}

/* 一帧 16 个时钟：命令字节 + 数据字节 */
int SPI_ReadReg(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                uint8_t reg, uint8_t *out)
{
    uint16_t dma_save;

    if (spi == 0 || out == 0) return -1;

    dma_save = spi_dma_rx_suspend(spi);
    cs_low(cs_port, cs_pin);
    spi_xfer(spi, (uint8_t)((reg & 0x7Fu) | 0x80u));   /* bit7 = 1 读 */
    *out = spi_xfer(spi, 0x00u);
    cs_high(cs_port, cs_pin);
    spi_dma_rx_resume(spi, dma_save);

    return 0;
}

int SPI_WriteReg(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                 uint8_t reg, uint8_t val)
{
    uint16_t dma_save;

    if (spi == 0) return -1;

    dma_save = spi_dma_rx_suspend(spi);
    cs_low(cs_port, cs_pin);
    spi_xfer(spi, (uint8_t)(reg & 0x7Fu));             /* bit7 = 0 写 */
    spi_xfer(spi, val);
    cs_high(cs_port, cs_pin);
    spi_dma_rx_resume(spi, dma_save);

    return 0;
}

/* 连读 len 个连续寄存器，全程同一次片选 —— 不要拆成多次单寄存器读 */
int SPI_ReadRegs(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                 uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint16_t dma_save;

    if (spi == 0 || buf == 0 || len == 0u) return -1;

    dma_save = spi_dma_rx_suspend(spi);
    cs_low(cs_port, cs_pin);
    spi_xfer(spi, (uint8_t)((reg & 0x7Fu) | 0x80u));    /* bit7 = 1 读 */

    for (i = 0u; i < len; i++)
        buf[i] = spi_xfer(spi, 0x00u);                  /* 地址自动递增 */

    cs_high(cs_port, cs_pin);
    spi_dma_rx_resume(spi, dma_save);

    return 0;
}

/* 打印比对结果，返回 0 = 通过。
 * 区分"无器件应答"与"值不符"：前者是接线/模式问题，后者才是协议问题。 */
static int check_id(const char *name, uint8_t got, uint8_t want)
{
    if (got == 0x00u || got == 0xFFu)
    {
        printf("ID,%s,NO_RESPONSE (got=0x%02X)\r\n", name, (unsigned)got);
        return 1;
    }

    printf("ID,%s,got=0x%02X,doc=0x%02X,%s\r\n",
           name, (unsigned)got, (unsigned)want,
           (got == want) ? "PASS" : "FAIL");
    return (got == want) ? 0 : 1;
}

int SPI_ProbeIDs(void)
{
    uint8_t v = 0u;
    int fail = 0;

    /* 磁强计 MMC5983MA @ SPI1，软件片选 PE0（网表 PE0 ; U1.63 U11.4） */
    if (SPI_ReadReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN, MMC_PRODID_REG, &v) != 0)
        printf("ID,MMC5983MA,param_err\r\n");
    else
        fail += check_id("MMC5983MA", v, MMC_PRODID_VAL);

    /* IMU LSM6DSV16X @ SPI2，片选 PD10（网表 PD10 ; U1.33 U4.12） */
    v = 0u;
    if (SPI_ReadReg(IMU_SPI, IMU_CS_PORT, IMU_CS_PIN, IMU_WHOAMI_REG, &v) != 0)
        printf("ID,LSM6DSV16X,param_err\r\n");
    else
        fail += check_id("LSM6DSV16X", v, IMU_WHOAMI_VAL);

    return fail;
}
