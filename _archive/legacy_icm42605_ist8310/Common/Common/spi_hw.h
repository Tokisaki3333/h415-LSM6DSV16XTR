/********************************** (C) COPYRIGHT  *******************************
* File Name          : spi_hw.h
* Description        : 硬件 SPI1 驱动（替换 board.c 中的软件 SPI）
*                      SPI1: SCK=PF5(AF5)  MOSI=PD7(AF5)  MISO=PF3(AF5)  CS=PF4(GPIO)
*                      设备: ICM42605 陀螺仪（SPI mode 0，读命令 bit7=1）
*                      本文件不 include 旧的 board.h
*********************************************************************************
* 依赖: ch32h417.h / ch32h417_spi.h / debug.h（printf、Delay_Ms）
*******************************************************************************/
#ifndef __SPI_HW_H
#define __SPI_HW_H

#include "ch32h417.h"

#ifdef __cplusplus
 extern "C" {
#endif

void    SPI1_Init(void);                                   /* 初始化硬件 SPI1 + CS */
uint8_t SPI_ReadMulti(uint8_t reg, uint8_t len); /* burst 连续读：小核启动 TX（0=成功）*/
void     SPI1_DMA_Rx_Setup(volatile uint8_t *target, uint16_t len); /* RX 通道配置（大核接管）*/
void    SPI_WriteReg(uint8_t reg, uint8_t data);           /* 写 42605 寄存器 */

void icm52605_Init_A(void);
void icm52605_Init_B(void);
#ifdef __cplusplus
}
#endif

#endif /* __SPI_HW_H */
