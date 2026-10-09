/*******************************************************************************
 * sensor_dma.h —— 陀螺 SPI2 的 DMA 采集：帧长、缓冲布局、大核侧接口
 *
 * 分工（照 v2 的 spi_hw.c / SPI_rx.c）：
 *   V3F：BOARD_IMU_DrdyStart() 启动 TX（命令 + dummy）、拉低 CS、立即返回。
 *        TX 缓冲是本核私有 static，实现在 board_io.c 9.12。
 *   V5F：自己持有 RX 缓冲并把它配给 RX DMA 通道，在完成中断里解析。
 *        RX 缓冲是【消费者自己的内存】，不存在跨核传地址的问题。
 *
 * 本头文件只放两核都要用的约定；V3F 的 TX 接口声明在 board_io.h。
 *******************************************************************************/

#ifndef SENSOR_DMA_H
#define SENSOR_DMA_H

#include <stdint.h>
#include "ch32h417.h"

/* 陀螺 LSM6DSV16X @ SPI2，CS=PD10，INT1=PD11。
 * 一次片选读 0x20~0x2D 共 14 字节：温度2 + 陀螺6 + 加速度计6。 */
#define SENSOR_IMU_DATA_LEN     14u
#define SENSOR_IMU_FRAME_LEN    (SENSOR_IMU_DATA_LEN + 1u)   /* +1 = 命令字节 */
#define SENSOR_IMU_READ_CMD     0x20u                        /* 读时 |0x80 */
#define SENSOR_IMU_RX_LEN       15u                          /* RX 缓冲长度 */

/* RX 缓冲布局（V5F 侧 rxfifo）：
 *   [0]      命令回读，无用
 *   [1..2]   温度
 *   [3..8]   陀螺 X/Y/Z（低字节在前）
 *   [9..14]  加速度计 X/Y/Z */

/* 单个陀螺周期的时长（7680Hz = 130us = 13000 个 10ns 计数）。 */
#define SENSOR_ISR_BUDGET_10NS  13000u

/* ---------------------------------------------------------------------------
 * V5F 侧（实现在 V5F/User/src/sensor_dma_rx.c）
 * -------------------------------------------------------------------------*/
void SPI2_DMA_Rx_Setup(volatile uint8_t *target, uint16_t len);
void SPI_DMA_Init(void);

/* 每秒取一次 ISR 统计（主循环调用）。返回 1 表示本秒已有数据并已清零累加器。
 *   n      本秒帧数
 *   avg_ns ISR 平均耗时（ns）
 *   max_ns ISR 单次最劣耗时（ns）
 *   ovr    超过 SENSOR_ISR_BUDGET_10NS 的帧数 */
uint8_t dma_rx_probe_1s(uint32_t *n, uint32_t *avg_ns, uint32_t *max_ns, uint32_t *ovr);

#endif /* SENSOR_DMA_H */
