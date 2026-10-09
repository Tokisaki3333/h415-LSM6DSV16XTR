#include "i2c_soft.h"
#include "hardware.h"
#include "debug.h"
#include "sys_clk.h"
#include <stdio.h>

/* ==================== 位操作 ==================== */
#define I2C_SCL_HIGH()   GPIO_SetBits(SOFTWARE_I2C_PORT, I2C_SCL_PIN)
#define I2C_SCL_LOW()    GPIO_ResetBits(SOFTWARE_I2C_PORT, I2C_SCL_PIN)
#define I2C_SDA_HIGH()   GPIO_SetBits(SOFTWARE_I2C_PORT, I2C_SDA_PIN)
#define I2C_SDA_LOW()    GPIO_ResetBits(SOFTWARE_I2C_PORT, I2C_SDA_PIN)
#define I2C_READ_SDA()   GPIO_ReadInputDataBit(SOFTWARE_I2C_PORT, I2C_SDA_PIN)

void SOFT_I2C_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOF, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(SOFTWARE_I2C_PORT, &GPIO_InitStructure);

    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
}

/* ==================== 时序 ==================== */
static void I2C_Start(void)
{
    I2C_SDA_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SCL_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SDA_LOW();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SCL_LOW();
    Delay_10Ns(I2C_HALF_10NS);
}

static void I2C_Stop(void)
{
    I2C_SDA_LOW();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SCL_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SDA_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
}

/* 返回 0 = 收到 ACK */
static uint8_t I2C_SendByte(uint8_t data)
{
    uint8_t i, ack;

    I2C_SCL_LOW();
    for (i = 0; i < 8; i++) {
        if (data & 0x80) I2C_SDA_HIGH();
        else             I2C_SDA_LOW();
        data <<= 1;
        Delay_10Ns(I2C_HALF_10NS);      /* 低电平：SDA 在此期间建立 */
        I2C_SCL_HIGH();
        Delay_10Ns(I2C_HALF_10NS);      /* 高电平 */
        I2C_SCL_LOW();
    }

    I2C_SDA_HIGH();
    Delay_10Ns(I2C_ACK_DELAY_10NS);
    I2C_SCL_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    ack = I2C_READ_SDA();
    I2C_SCL_LOW();
    Delay_10Ns(I2C_HALF_10NS);
    return ack;
}

/* ack_en: 1 = 回 ACK, 0 = 回 NACK */
static uint8_t I2C_RecvByte(uint8_t ack_en)
{
    uint8_t i, data = 0;

    I2C_SDA_HIGH();
    for (i = 0; i < 8; i++) {
        data <<= 1;
        I2C_SCL_HIGH();
        Delay_10Ns(I2C_HALF_10NS);
        if (I2C_READ_SDA()) data |= 0x01;
        I2C_SCL_LOW();
        Delay_10Ns(I2C_HALF_10NS);
    }

    if (ack_en) I2C_SDA_LOW();
    else        I2C_SDA_HIGH();
    Delay_10Ns(I2C_SETUP_10NS);
    I2C_SCL_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SCL_LOW();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SDA_HIGH();
    return data;
}

/* ==================== 寄存器操作 ==================== */
uint8_t I2C_ReadReg(uint8_t dev7bit, uint8_t reg)
{
    uint8_t val;

    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x00) != 0) { I2C_Stop(); return 0xFF; }
    if (I2C_SendByte(reg) != 0) { I2C_Stop(); return 0xFF; }
    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x01) != 0) { I2C_Stop(); return 0xFF; }
    val = I2C_RecvByte(0);
    I2C_Stop();
    return val;
}

uint8_t I2C_ReadMulti(uint8_t dev7bit, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if (!buf || len == 0) return 0xFF;

    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x00) != 0) { I2C_Stop(); return 0xFF; }
    if (I2C_SendByte(reg) != 0) { I2C_Stop(); return 0xFF; }
    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x01) != 0) { I2C_Stop(); return 0xFF; }

    for (i = 0; i < len; i++)
        buf[i] = I2C_RecvByte(i + 1 < len);

    I2C_Stop();
    return 0;
}

void I2C_WriteReg(uint8_t dev7bit, uint8_t reg, uint8_t data)
{
    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x00) != 0) { I2C_Stop(); return; }
    if (I2C_SendByte(reg) != 0) { I2C_Stop(); return; }
    I2C_SendByte(data);
    I2C_Stop();
}

/* ==================== BMP388 寄存器表 ====================
 *
 * 0x00 CHIP_ID        固定 0x50
 * 0x02 ERR_REG        bit0 fatal_err | bit1 cmd_err | bit2 conf_err
 * 0x03 STATUS         bit6 drdy_temp | bit5 drdy_press | bit4 cmd_rdy
 * 0x04~0x06           压力 24bit，低字节在前（0x06 为 MSB）
 * 0x07~0x09           温度 24bit，低字节在前（0x09 为 MSB）
 * 0x19 INT_CTRL       bit6 drdy_en | bit2 int_latch | bit1 int_level(1=高) | bit0 int_od
 * 0x1B PWR_CTRL       bit1 temp_en | bit0 press_en | bit[5:4] mode(00睡眠,01/10强制,11正常)
 * 0x1C OSR            bit[2:0] osr_p | bit[5:3] osr_t，编码 0..5 = x1,x2,x4,x8,x16,x32
 * 0x1D ODR            bit[4:0] odr_sel，仅正常模式用
 * 0x1F CONFIG         bit[3:1] IIR 系数(000旁路,001=1,010=3,011=7,100=15,...)
 * 0x31~0x45           标定系数 21 字节
 */

void BMP388_Init(void)
{
    /* CHIP_ID 只在这里校验一次；ReadRaw 每帧不再重读（省一次 I2C 事务） */
    if (BMP388_CheckID(BMP388_DEV7BIT) != 0)
    {
        printf("CHIP_ID_FAIL\r\n");
        return;
    }

    /* drdy_en | int_latch | int_level 高有效 */
    I2C_WriteReg(BMP388_DEV7BIT, 0x19, 0x46);

    /* osr_p = x8(3) | osr_t = x2(1) << 3 */
    I2C_WriteReg(BMP388_DEV7BIT, 0x1C, 0x0B);

    /* IIR 系数 1 */
    I2C_WriteReg(BMP388_DEV7BIT, 0x1F, 0x02);

    /* 标定值必须来自本颗芯片 */
    if (BMP388_ReadCalib() != 0)
        printf("CAL_FAIL\r\n");
    else
        ;
        // BMP388_DumpCalib();
}

void BMP388_TriggerMeasurement(void)
{
    /* temp_en(bit1) | press_en(bit0) | mode[5:4] = 01 强制模式 */
    I2C_WriteReg(BMP388_DEV7BIT, 0x1B, 0x13);
}

uint8_t BMP388_IsDRDY(void)
{
    uint8_t st = I2C_ReadReg(BMP388_DEV7BIT, 0x03);

    if (st == 0xFFu) return 0u;                 /* 读失败，不算就绪 */
    return ((st & 0x60u) == 0x60u) ? 1u : 0u;   /* drdy_temp | drdy_press */
}

uint32_t BMP388_WaitDRDY(uint32_t timeout_us, uint64_t *out_ts_drdy)
{
    uint64_t t0 = GetTime64_Us();
    uint64_t el;

    /* 转换时间下限：此时数据必定已更新，避免拿残留标志当就绪。
     * 纯忙等（只读 TIM9/TIM10），不调 Delay_Us —— Delay_Us 每次要重配
     * SysTick 的 5 个寄存器，25ms 里调用约 25000 次纯属浪费外设带宽。 */
    for (;;)
    {
        el = GetTime64_Us() - t0;
        if (el >= BMP388_CONV_US) break;
        if (el >= timeout_us) return (uint32_t)el;
    }

    /* 轮询就绪标志 */
    for (;;)
    {
        if (BMP388_IsDRDY() != 0u) break;
        el = GetTime64_Us() - t0;
        if (el >= timeout_us) return (uint32_t)el;
    }

    if (out_ts_drdy) *out_ts_drdy = GetTime64_10Ns();
    return (uint32_t)(GetTime64_Us() - t0);
}

uint8_t BMP388_Read(float *temp_c, float *press_pa)
{
    int ret = BMP388_ReadCompensated(BMP388_DEV7BIT, temp_c, press_pa);
    return (ret == 0) ? 0 : 0xFF;
}

/* 完整一次测量，并以 FireWater 协议打印一行 CSV。
 * 返回 0 成功；-1 等待超时；-2 读取/补偿失败 */
int read_baro(void)
{
    float    temp_c = 0.0f;
    float    press_pa = 0.0f;
    uint64_t ts = 0;
    uint64_t t_start, t_drdy, t_end;
    uint32_t waited;
    uint32_t t_conv_us, t_read_us, t_loop_us;

    t_start = GetTime64_Us();

    BMP388_TriggerMeasurement();

    waited = BMP388_WaitDRDY(BMP388_TIMEOUT_US, &ts);
    t_conv_us = waited;

    if (waited >= BMP388_TIMEOUT_US)
    {
        t_loop_us = (uint32_t)(GetTime64_Us() - t_start);
        printf("-1,-1,%u,0,%u\r\n", (unsigned)waited, (unsigned)t_loop_us);
        return -1;
    }

    t_drdy = ts / 100ULL;               /* 10ns 计数 -> us */

    if (BMP388_Read(&temp_c, &press_pa) != 0)
    {
        t_loop_us = (uint32_t)(GetTime64_Us() - t_start);
        printf("-1,-1,%u,0,%u\r\n", (unsigned)t_conv_us, (unsigned)t_loop_us);
        return -2;
    }

    t_end = GetTime64_Us();
    t_read_us = (uint32_t)(t_end - t_drdy);
    t_loop_us = (uint32_t)(t_end - t_start);

    printf("%.2f,%.2f,%u,%u,%u\r\n",
           (double)temp_c, (double)press_pa,
           (unsigned)t_conv_us, (unsigned)t_read_us, (unsigned)t_loop_us);
    return 0;
}
