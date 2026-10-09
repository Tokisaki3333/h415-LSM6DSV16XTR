#ifndef SPI_FLASH_W25N_H
#define SPI_FLASH_W25N_H

#include <stdint.h>

/* ============================================================================
 * 外挂 SPI NAND：W25N01GVZEIG（1Gbit = 128MB）
 *   页   ：2048 B 数据区（+64 B OOB，本驱动只操作数据区）
 *   块   ：64 页 = 128 KB；共 1024 块，页号 0~65535（PA = block<<6 + page_in_block）
 *   寿命 ：10 万次擦写/块；ECC 默认开启（透明）
 *   注意 ：同一块内页必须从低地址到高地址顺序编程，禁止乱序
 *
 * 接口：SPI2 —— CS:PB12  SCK:PB13  MISO:PB14  MOSI:PC3（均 AF5）
 * 时序：SPI Mode 0（CPOL_Low / CPHA_1Edge）；状态寄存器读无 dummy（规格书 Figure 7）
 * 状态：BUSY/WEL/E-FAIL/P-FAIL 在 SR3(0xC0)；SR1(0xA0) 为保护寄存器（BP[3:0]/TB）
 *
 * 典型使用（写入流程）：
 *   W25N_Init();            // 上电初始化（不自动解锁）
 *   W25N_Unprotect();       // 需要写/擦时显式解锁（解锁后全片可写）
 *   W25N_EraseBlock(p);     // 1. 先擦目标块
 *   W25N_WritePage(p,..);   // 2. 再写页（块内页顺序编程）
 *   W25N_ReadPage(p,..);    // 3. 读回校验
 * ==========================================================================*/

/* 初始化 SPI2 与 GPIO（Mode 0），并复位器件。上电调用一次；不解锁。 */
void W25N_Init(void);

/* 读 JEDEC ID，返回 0xEFAA21（W25N01GV 的 厂商 EF + 器件 AA21）。
 * 用于确认芯片在位、型号匹配。 */
uint32_t W25N_ReadJEDECID(void);

/* 读一页数据：page 为全局页号 0~65535，buf 至少 len 字节，len ≤ 2048。
 * 内部：0x13 页→内部缓存（BUSY 轮询等待加载完成）→ 0x03 从缓存读。
 * 读无需解锁/擦除。 */
void W25N_ReadPage(uint32_t page, uint8_t *buf, uint32_t len);

/* 写一页数据：page 全局页号，buf 待写数据，len ≤ 2048（不足补 0xFF 满页编程）。
 * 前置：所在块必须先 W25N_EraseBlock；同一块内页必须顺序（低→高）编程。
 * 内部：0x06 写使能（SR3 WEL 确认）→ 0x02 载入缓存（同步）→ 0x10 编程执行（BUSY 轮询）。 */
void W25N_WritePage(uint32_t page, const uint8_t *buf, uint32_t len);

/* 擦除 page 所在 128KB 块（64 页）。前置：W25N_Unprotect。
 * 内部：0x06 写使能 → 0xD8 块擦除（BUSY 轮询等待完成）。 */
void W25N_EraseBlock(uint32_t page);

/* 专用解锁：写 SR1(0xA0)=0x00 清除 BP[3:0]/TB（出厂 0x7C = 全片写保护）。
 * 注意：解锁后全片可写；只在确认需要写时显式调用，勿放 init 自动执行。 */
void W25N_Unprotect(void);

/* 自检：读 ID + 解锁 + 仅块 0 擦写读验证（实验区，其他块不动）。
 * 返回 0=通过，非 0=失败。仅供调试/出厂测试，会擦写块 0。 */
uint8_t W25N_SelfTest(void);

#endif /* SPI_FLASH_W25N_H */
