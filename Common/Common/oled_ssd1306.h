#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include <stdint.h>
#include "mipc_shm.h"   /* g_shm / oled_fb（双核共享显存，aligned(4)）*/

/* SSD1306 7bit 从机地址（SA0=GND）*/
#define OLED_I2C_ADDR   0x3C

#define OLED_W          128
#define OLED_H          64
#define OLED_FB_SIZE    (OLED_W * OLED_H / 8)   /* 1024，页主序 */

/* 软件 I2C 引脚（用户指定）：SCL=PA14  SDA=PA13 */
#define OLED_SCL_PORT   GPIOA
#define OLED_SCL_PIN    GPIO_Pin_14
#define OLED_SDA_PORT   GPIOA
#define OLED_SDA_PIN    GPIO_Pin_13

/* 初始化（GPIO + SSD1306 寄存器序列 + 清显存刷屏）*/
void oled_ssd1306_init(void);

/* 整屏刷新：共享显存 → SSD1306 GDDRAM。由小核（V3F）周期性调用。
 * max_pages：本次调用最多刷新的页数（1~8，0=不刷）；内部按 start_page 分片轮转。*/
void oled_refresh(uint8_t max_pages);

/* ---- 修改共享显存（供双核调用；访问 fb 保证 4 字节对齐，避免撕裂） ---- */

/* 清屏：整块显存清零（按 uint32_t 对齐批量写，不刷屏）*/
void oled_clear_fb(void);

/* 画实心方块：(x0,y0)-(x1,y1) 内像素置亮(on=1)/清除(on=0)，页主序，不刷屏*/
void oled_fill_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t on);

/* 演示/调试动画 */
void oled_animation(void);
/* 标定方向 */
void oled_check(void);

/* 放字: 画(on=1)/擦(on=0) 一个字符到显存, 左上角 (x, y)
 * 参数: s — 指向字符首字节("你好"传'你'首字节, "abc"传'a')
 * 返回: 0=出错(不画, 指针不动)  1=ASCII(指针应 +1)  2=GB2312(指针应 +2)
 * 典型调用: 单字符绘制; 连续字符串请按返回值推进指针循环调用
 */
uint8_t oled_put_char(const char *s, uint8_t x, uint8_t y, uint8_t on);

/* 从 (x0,y0) 绘制整串, 超宽自动换行; 返回实际绘制字符数, 非法字符替换为 '?' */
uint8_t oled_puts(uint8_t x0, uint8_t y0, const char *s);

/* printf 风格格式化绘制(形如 mipc_v3_printf / printf):
 * 支持 %d/%f/%s 等; \n 换行 \r 忽略, 非法字符替换为 '?';
 * 返回实际绘制字符数 */
uint8_t oled_printf(uint8_t x0, uint8_t y0, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

/* nanofont3x4 3x4 小字库(双核共享): 仅 ASCII 0x20~0x7F, 非法字符替换为 '?'。
 * 3 像素宽, 行距 NANO3X4_LINE_H=5, \n 换行 \r 忽略 */
uint8_t oled_nano_puts(uint8_t x0, uint8_t y0, const char *s);
uint8_t oled_nano_printf(uint8_t x0, uint8_t y0, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

void oled_draw_image(uint8_t x, uint8_t y,
                     const uint8_t *bitmap,
                     uint8_t width, uint8_t height,
                     uint8_t on);

#endif /* OLED_SSD1306_H */
