#ifndef BMP388_H
#define BMP388_H

#include "ch32h417.h"

/* 数学补偿系数，datasheet 附录 9.1 固定值 */
#define T1_scale         2.560000000000e+02f
#define T2_scale_inv     9.313225746155e-10f
#define T3_scale_inv     3.552713678801e-15f

#define P1_scale_inv     9.536743164062e-07f
#define P2_scale_inv     1.862645149231e-09f
#define P3_scale_inv     2.328306436539e-10f
#define P4_scale_inv     7.275957614183e-12f

#define P5_scale         8.000000000000e+00f
#define P6_scale_inv     1.562500000000e-02f
#define P7_scale_inv     3.906250000000e-03f
#define P8_scale_inv     3.051757812500e-05f

#define P9_scale_inv     3.552713678801e-15f
#define P10_scale_inv    3.552713678801e-15f
#define P11_scale_inv    2.710505431214e-20f

#define BMP388_P1_OFFSET 16384.0f   /* 2^14 */

/* 标定参数：每颗芯片出厂值不同，上电从 0x31~0x45 读取 */
int BMP388_ReadCalib(void);
void BMP388_DumpCalib(void);

/**
 * @brief 读取原始ADC并自动执行全套补偿
 * @param dev7bit 器件7bit I2C地址
 * @param out_temp_c 输出温度（℃）
 * @param out_press_pa 输出气压（Pa）
 * @retval 0成功，-1芯片ID不符
 */
int BMP388_ReadCompensated(uint8_t dev7bit, float *out_temp_c, float *out_press_pa);

float bmp388_compensate_temp(uint32_t uncomp_temp);
float bmp388_compensate_press(uint32_t uncomp_press, float t_lin);
/* 校验 CHIP_ID(0x00) 是否为 0x50，返回 0 正确 */
int BMP388_CheckID(uint8_t dev7bit);

/* 读 0x04~0x09 的原始 24bit 压力/温度 */
int BMP388_ReadRaw(uint8_t dev7bit,
                   uint32_t* out_uncomp_press,
                   uint32_t* out_uncomp_temp);

#endif
