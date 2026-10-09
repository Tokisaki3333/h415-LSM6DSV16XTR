#include "hardware.h"

/* 此文件原为硬件调试入口 Hardware()，已被"中断 + DMA 采集"架构取代。
 * 保留空文件以维持工程编译列表（MRS 的 subdir.mk 里仍列着它）。
 *
 * 旧的 SPI1 驱动（spi_hw.c/.h）与 ICM-42605 整条链路已归档到：
 *   _archive/legacy_icm42605_ist8310/
 * 其中包含旧 8K 中断、DMA 帧解析、v5f_hold_t、以及完整的姿态解算八步链
 * （proc_* 系列），以后做运动解析时可作参考。
 *
 * 新实现：
 *   小核 V3F  Common/Common/sensor_dma.c
 *   大核 V5F  V5F/User/src/sensor_dma_rx.c
 *   共享约定  Common/Common/sensor_dma.h
 */
//GPS-pps PE1
