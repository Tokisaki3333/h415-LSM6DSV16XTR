#include "i2c_soft.h"
#include "board_io.h"
#include "debug.h"
#include "sys_clk.h"
#include <stdio.h>

/* ==================== 浣嶆搷浣?==================== */
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

/* ==================== 鏃跺簭 ==================== */
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

/* 杩斿洖 0 = 鏀跺埌 ACK */
static uint8_t I2C_SendByte(uint8_t data)
{
    uint8_t i, ack;

    I2C_SCL_LOW();
    for (i = 0; i < 8; i++) {
        if (data & 0x80) I2C_SDA_HIGH();
        else             I2C_SDA_LOW();
        data <<= 1;
        Delay_10Ns(I2C_HALF_10NS);      /* 浣庣數骞筹細SDA 鍦ㄦ鏈熼棿寤虹珛 */
        I2C_SCL_HIGH();
        Delay_10Ns(I2C_HALF_10NS);      /* 楂樼數骞?*/
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

/* ack_en: 1 = 鍥?ACK, 0 = 鍥?NACK */
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

/* ==================== 瀵勫瓨鍣ㄦ搷浣?==================== */
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

/* ==================== BMP388 瀵勫瓨鍣ㄨ〃 ====================
 *
 * 0x00 CHIP_ID        鍥哄畾 0x50
 * 0x02 ERR_REG        bit0 fatal_err | bit1 cmd_err | bit2 conf_err
 * 0x03 STATUS         bit6 drdy_temp | bit5 drdy_press | bit4 cmd_rdy
 * 0x04~0x06           鍘嬪姏 24bit锛屼綆瀛楄妭鍦ㄥ墠锛?x06 涓?MSB锛?
 * 0x07~0x09           娓╁害 24bit锛屼綆瀛楄妭鍦ㄥ墠锛?x09 涓?MSB锛?
 * 0x19 INT_CTRL       bit6 drdy_en | bit2 int_latch | bit1 int_level(1=楂? | bit0 int_od
 * 0x1B PWR_CTRL       bit1 temp_en | bit0 press_en | bit[5:4] mode(00鐫＄湢,01/10寮哄埗,11姝ｅ父)
 * 0x1C OSR            bit[2:0] osr_p | bit[5:3] osr_t锛岀紪鐮?0..5 = x1,x2,x4,x8,x16,x32
 * 0x1D ODR            bit[4:0] odr_sel锛屼粎姝ｅ父妯″紡鐢?
 * 0x1F CONFIG         bit[3:1] IIR 绯绘暟(000鏃佽矾,001=1,010=3,011=7,100=15,...)
 * 0x31~0x45           鏍囧畾绯绘暟 21 瀛楄妭
 */

void BMP388_Init(void)
{
    /* CHIP_ID 鍙湪杩欓噷鏍￠獙涓€娆★紱ReadRaw 姣忓抚涓嶅啀閲嶈锛堢渷涓€娆?I2C 浜嬪姟锛?*/
    if (BMP388_CheckID(BMP388_DEV7BIT) != 0)
    {
        printf("CHIP_ID_FAIL\r\n");
        return;
    }

    /* drdy_en | int_latch | int_level 楂樻湁鏁?*/
    I2C_WriteReg(BMP388_DEV7BIT, 0x19, 0x46);

    /* osr_p = x8(3) | osr_t = x2(1) << 3 */
    I2C_WriteReg(BMP388_DEV7BIT, 0x1C, 0x0B);

    /* IIR 绯绘暟 1 */
    I2C_WriteReg(BMP388_DEV7BIT, 0x1F, 0x02);

    /* 鏍囧畾鍊煎繀椤绘潵鑷湰棰楄姱鐗?*/
    if (BMP388_ReadCalib() != 0)
        printf("CAL_FAIL\r\n");
    else
        ;
        // BMP388_DumpCalib();
}

void BMP388_TriggerMeasurement(void)
{
    /* temp_en(bit1) | press_en(bit0) | mode[5:4] = 01 寮哄埗妯″紡 */
    I2C_WriteReg(BMP388_DEV7BIT, 0x1B, 0x13);
}

uint8_t BMP388_IsDRDY(void)
{
    uint8_t st = I2C_ReadReg(BMP388_DEV7BIT, 0x03);

    if (st == 0xFFu) return 0u;                 /* 璇诲け璐ワ紝涓嶇畻灏辩华 */
    return ((st & 0x60u) == 0x60u) ? 1u : 0u;   /* drdy_temp | drdy_press */
}

uint32_t BMP388_WaitDRDY(uint32_t timeout_us, uint64_t *out_ts_drdy)
{
    uint64_t t0 = GetTime64_Us();
    uint64_t el;

    /* 杞崲鏃堕棿涓嬮檺锛氭鏃舵暟鎹繀瀹氬凡鏇存柊锛岄伩鍏嶆嬁娈嬬暀鏍囧織褰撳氨缁€?
     * 绾繖绛夛紙鍙 TIM9/TIM10锛夛紝涓嶈皟 Delay_Us 鈥斺€?Delay_Us 姣忔瑕侀噸閰?
     * SysTick 鐨?5 涓瘎瀛樺櫒锛?5ms 閲岃皟鐢ㄧ害 25000 娆＄函灞炴氮璐瑰璁惧甫瀹姐€?*/
    for (;;)
    {
        el = GetTime64_Us() - t0;
        if (el >= BMP388_CONV_US) break;
        if (el >= timeout_us) return (uint32_t)el;
    }

    /* 杞灏辩华鏍囧織 */
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

/* 瀹屾暣涓€娆℃祴閲忥紝骞朵互 FireWater 鍗忚鎵撳嵃涓€琛?CSV銆?
 * 杩斿洖 0 鎴愬姛锛?1 绛夊緟瓒呮椂锛?2 璇诲彇/琛ュ伩澶辫触 */
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

    t_drdy = ts / 100ULL;               /* 10ns 璁℃暟 -> us */

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
