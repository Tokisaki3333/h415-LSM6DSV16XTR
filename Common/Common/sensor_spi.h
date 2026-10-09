#ifndef _SENSOR_SPI_H
#define _SENSOR_SPI_H

#include <stdint.h>

#include "ch32h417.h"

/* ==================== SPI 传感器寄存器读写（纯轮询） ====================
 *
 * 命令字节格式（两颗芯片相同）：
 *      bit 7   : RW，1 = 读，0 = 写
 *      bit 6:0 : 寄存器地址
 * 读命令 = reg | 0x80，写命令 = reg & 0x7F。
 * 一帧 16 个时钟（2 字节）：先命令字节，再数据字节。
 *
 * 手册依据：
 *   MMC5983MA  MEMSIC MMC5983MA Rev.A "SPI Read and Write protocol"
 *   LSM6DSV16X ST DS13510 Rev 4 §5.1.3 SPI read / SPI write
 * 两者均支持 SPI mode 0（board_io.c 按 mode 0 初始化）。
 * 片选由调用方传端口/引脚；引脚为 0 表示不做片选动作。
 * ====================================================================== */

/* 读/写一个寄存器；返回 0 成功，-1 参数非法 */
int SPI_ReadReg(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                uint8_t reg, uint8_t *out);
int SPI_WriteReg(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                 uint8_t reg, uint8_t val);

/* 连读 len 个连续寄存器：命令字节 + len 个数据字节，全部在【同一次】片选内完成
 * （地址指针自动递增）。不要拆成多次单寄存器读。
 *   MMC5983MA : "address pointer automatically moves to the next byte"
 *   LSM6DSV16X: IF_INC=1 时地址每块递增
 * 返回 0 成功。 */
int SPI_ReadRegs(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin,
                 uint8_t reg, uint8_t *buf, uint16_t len);

/* ==================== ID 寄存器（地址与期望值取自手册）====================
 *   MMC5983MA  Product ID 1   0x2F  复位值 0x30
 *              (Rev.A 寄存器表："Product ID 1  2FH"，"Reset Value 0011 0000")
 *   LSM6DSV16X WHO_AM_I       0x0F  固定值 0x70
 *              (DS13510 Rev4 §9.13："WHO_AM_I (0Fh) ... fixed at 70h")
 * ======================================================================== */
#define MMC_PRODID_REG      0x2Fu
#define MMC_PRODID_VAL      0x30u

#define IMU_WHOAMI_REG      0x0Fu
#define IMU_WHOAMI_VAL      0x70u

/* 读两颗芯片 ID 并与手册值比对后打印。
 * 返回 0 = 两个都对；否则为失败个数。 */
int SPI_ProbeIDs(void);

#endif
