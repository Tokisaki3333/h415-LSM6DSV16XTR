#ifndef _I2C_SOFT_H
#define _I2C_SOFT_H

#include <stdint.h>

#include "bmp388.h"

/* 软件 I2C 引脚 */
#define SOFTWARE_I2C_PORT    GPIOF
#define I2C_SCL_PIN          GPIO_Pin_0      /* PF0 */
#define I2C_SDA_PIN          GPIO_Pin_1      /* PF1 */

/* 位时序，单位 10ns。目标 600kHz（位周期 1.667us）。
 *
 * 实测（每位 2 次 Delay_10Ns 的结构，SendByte 已并入建立时间）：
 *   HALF=13 -> t_read=119us -> 1.566us/位 (639kHz)
 * 由旧结构两点拟合得每个 HALF 单位实拖约 16.8ns（每位 2 个 HALF 量）：
 *   1.667 = 1.566 + 2 × ΔHALF × 0.0168  ->  ΔHALF ≈ 3
 * 取 HALF=17 居中（按标称 20ns/单位算则应为 18）。
 *
 * 注：Delay_10Ns 单次调用成本约 0.65us，构成每位 1.31us 的固定开销，
 *     故该结构速率上限约 760kHz。
 */
#define I2C_HALF_10NS         17
#define I2C_SETUP_10NS        8      /* 已并入低电平，保留供参考 */
#define I2C_ACK_DELAY_10NS    8      /* 80ns 应答建立 */

/* BMP388，SDO 接 PF2 且外部下拉 -> SDO=0 -> 地址 0x76 */
#define BMP388_DEV7BIT       0x76

/* 一次强制转换的时长（us）。OSR 编码 0..5 对应 x1,x2,x4,x8,x16,x32，
 * t_conv = 2.2ms + osr_t*2.3ms + osr_p*2.3ms。
 * 当前 OSR_P=x8 / OSR_T=x2（0x1C=0x0B）-> 25200us。需与 0x1C 的写入值一致。 */
#define BMP388_CONV_US       25200u

/* 等转换完成的上限（us）。需大于 BMP388_CONV_US，且不超出采样周期。 */
#define BMP388_TIMEOUT_US    30000u

/*----- I2C 基础（BMP388 之外的器件也可用） -----*/
void SOFT_I2C_GPIO_Init(void);
uint8_t I2C_ReadReg(uint8_t dev7bit, uint8_t reg);
uint8_t I2C_ReadMulti(uint8_t dev7bit, uint8_t reg, uint8_t *buf, uint16_t len);
void I2C_WriteReg(uint8_t dev7bit, uint8_t reg, uint8_t data);

/*----- BMP388 -----*/
void BMP388_Init(void);
void BMP388_TriggerMeasurement(void);
uint8_t BMP388_IsDRDY(void);        /* 读 STATUS(0x03)，压力+温度都就绪返回 1 */
uint8_t BMP388_Read(float *temp_c, float *press_pa);   /* 0 成功 */
uint32_t BMP388_WaitDRDY(uint32_t timeout_us, uint64_t *out_ts_drdy);

/* 完整一次测量：触发 -> 等就绪 -> 读取并补偿，
 * 以 FireWater 协议打印一行 CSV：
 *   温度(C), 气压(Pa), 转换等待(us), I2C读取+补偿(us), 整拍(us)
 * 返回 0 成功；-1 等待超时；-2 读取/补偿失败 */
int read_baro(void);

/* 标定参数接口在 bmp388.h（BMP388_ReadCalib / BMP388_DumpCalib） */

#endif
