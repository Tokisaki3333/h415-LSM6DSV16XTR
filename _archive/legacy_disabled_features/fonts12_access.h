/* fonts12_access.h — 字库访问层(双核共享)
 * 用法: 必须先 include "fonts12.h" 再 include 本文件(oled_ssd1306.c 已如此)。
 *   1) 内嵌模式: fonts12.h 为 #if 1, 数组进 .font 段(0x00050000)
 *      -> FONT_DATA_EMBEDDED 已定义, 本文件不定义指针宏, 直接引用数组
 *   2) 绑地址模式: fonts12.h 为 #if 0(空壳, 无宏无数组)
 *      -> 本文件定义绑地址指针, 字库常驻 Flash 0x00050000 直接读取
 */
#ifndef FONTS12_ACCESS_H
#define FONTS12_ACCESS_H

#include <stdint.h>

/* 取字偏移宏(字节) —— 原 fonts12.h, 迁移至此 */
#define ASC8X12_OFFSET(ch)      ((ch - 0x20) * 16)
#define HZK12_OFFSET(hi, lo)   (((hi - 0xA1) * 94 + (lo - 0xA1)) * 24)

#ifndef FONT_DATA_EMBEDDED
    /* 绑地址模式: 字库区基址与 V5F/Ld/Link_v5f.ld FONT 区域一致(0x00050000)
     * 实际布局(已由 nm 验证): hzk12[124080] 在前(偏移 0x0),
     * asc8x12[1536] 在后(偏移 0x1E4B0=124080) */
    #define FONT_BASE_ADDR  0x00050000UL
    #define hzk12_font      ((const unsigned char *)(FONT_BASE_ADDR + 0x0000UL))
    #define asc8x12_font    ((const unsigned char *)(FONT_BASE_ADDR + 0x1E4B0UL))
#endif

#endif /* FONTS12_ACCESS_H */
