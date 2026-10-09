#include "bmp388.h"
#include "i2c_soft.h"

/* ==================== 标定参数 ====================
 * 寄存器 0x31~0x45 共 21 字节，位域按数据手册 Table 23 拼装。
 * 每颗芯片出厂标定值不同，必须上电读取。 */
static int16_t s_T1, s_T2;
static int8_t  s_T3;
static int16_t s_P1, s_P2, s_P5, s_P6, s_P9, s_P10, s_P11;
static int8_t  s_P3, s_P4, s_P7, s_P8;

/* 返回 0 成功 */
int BMP388_ReadCalib(void)
{
    uint8_t d[21];
    int16_t t1, t2, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;

    if (I2C_ReadMulti(BMP388_DEV7BIT, 0x31, d, sizeof(d)) != 0)
        return -1;

    t1  = (int16_t)(((uint16_t)d[1] << 8) | d[0]);
    t2  = (int16_t)(((uint16_t)d[3] << 8) | d[2]);
    s_T3 = (int8_t)d[4];

    p1  = (int16_t)(((uint16_t)d[6] << 8) | d[5]);
    p2  = (int16_t)(((uint16_t)d[8] << 8) | d[7]);
    p3  = (int8_t)d[9];
    p4  = (int8_t)d[10];
    p5  = (int16_t)(((uint16_t)d[12] << 8) | d[11]);
    p6  = (int16_t)(((uint16_t)d[14] << 8) | d[13]);
    p7  = (int8_t)d[15];
    p8  = (int8_t)d[16];
    p9  = (int16_t)(((uint16_t)d[18] << 8) | d[17]);
    p10 = (int8_t)d[19];
    p11 = (int8_t)d[20];

    s_T1 = t1;   s_T2 = t2;
    s_P1 = p1;   s_P2 = p2;   s_P3 = p3;   s_P4 = p4;
    s_P5 = p5;   s_P6 = p6;   s_P7 = p7;   s_P8 = p8;
    s_P9 = p9;   s_P10 = p10; s_P11 = p11;

    return 0;
}

/* 打印成一行 CSV：CAL,t1,t2,t3,p1..p11 */
void BMP388_DumpCalib(void)
{
    printf("CAL,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
           (int)s_T1, (int)s_T2, (int)s_T3,
           (int)s_P1, (int)s_P2, (int)s_P3, (int)s_P4,
           (int)s_P5, (int)s_P6, (int)s_P7, (int)s_P8,
           (int)s_P9, (int)s_P10, (int)s_P11);
}

/**
 * @brief BMP388温度补偿
 * @param uncomp_temp 24bit原始ADC温度
 * @return t_lin 线性化温度(℃)
 */
float bmp388_compensate_temp(uint32_t uncomp_temp)
{
    float pd1 = (float)uncomp_temp - ((float)s_T1 * T1_scale);
    float pd2 = pd1 * ((float)s_T2 * T2_scale_inv);
    return pd2 + (pd1 * pd1) * ((float)s_T3 * T3_scale_inv);
}

/**
 * @brief BMP388压力补偿
 * @param uncomp_press 24bit原始ADC压力
 * @param t_lin 由bmp388_compensate_temp返回的线性温度
 * @return 补偿后压力，单位 Pa
 */
float bmp388_compensate_press(uint32_t uncomp_press, float t_lin)
{
    float t2 = t_lin * t_lin;
    float t3 = t2 * t_lin;

    float po1  =  ((float)s_P5 * P5_scale)
                + ((float)s_P6 * P6_scale_inv) * t_lin
                + ((float)s_P7 * P7_scale_inv) * t2
                + ((float)s_P8 * P8_scale_inv) * t3;

    float term_p1 = ((float)s_P1 - BMP388_P1_OFFSET) * P1_scale_inv;
    float term_p2 = ((float)s_P2 - BMP388_P1_OFFSET) * P2_scale_inv;
    float term_p3 = ((float)s_P3 * P3_scale_inv);
    float term_p4 = ((float)s_P4 * P4_scale_inv);

    float po2 = (float)uncomp_press * ( term_p1 + term_p2 * t_lin + term_p3 * t2 + term_p4 * t3 );

    float up2 = (float)uncomp_press * (float)uncomp_press;
    float up3 = up2 * (float)uncomp_press;

    float pdB2 = ((float)s_P9 * P9_scale_inv) + ((float)s_P10 * P10_scale_inv) * t_lin;
    float pdB3 = up2 * pdB2;
    float pdB4 = pdB3 + up3 * ((float)s_P11 * P11_scale_inv);

    return po1 + po2 + pdB4;
}

/* 上电校验一次芯片 ID 即可，不必每帧重读（每帧重读要多花一次 I2C 事务） */
int BMP388_CheckID(uint8_t dev7bit)
{
    return (I2C_ReadReg(dev7bit, 0x00) == 0x50) ? 0 : -1;
}

int BMP388_ReadRaw(uint8_t dev7bit,
                            uint32_t* out_uncomp_press,
                            uint32_t* out_uncomp_temp)
{
    uint8_t d[6];   /* 0x04~0x09：压力+温度各 3 字节，低字节在前 */

    if (I2C_ReadMulti(dev7bit, 0x04, d, 6) != 0)
    {
        return -1;
    }

    *out_uncomp_press  = ((uint32_t)d[2] << 16U) | ((uint32_t)d[1] << 8U) | d[0];
    *out_uncomp_temp   = ((uint32_t)d[5] << 16U) | ((uint32_t)d[4] << 8U) | d[3];

    return 0;
}

int BMP388_ReadCompensated(uint8_t dev7bit, float *out_temp_c, float *out_press_pa)
{
    uint32_t raw_press, raw_temp;
    int ret = BMP388_ReadRaw(dev7bit, &raw_press, &raw_temp);
    if(ret != 0)
    {
        return ret;
    }

    float t_lin = bmp388_compensate_temp(raw_temp);
    float press_pa = bmp388_compensate_press(raw_press, t_lin);

    *out_temp_c = t_lin;
    *out_press_pa = press_pa;

    return 0;
}
