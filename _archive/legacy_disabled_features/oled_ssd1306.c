#include "oled_ssd1306.h"
#include "debug.h"          /* Delay_10Ns / Delay_Ms */
#include "ch32h417.h"
#include <string.h>
#include <stdarg.h>

#include "fonts12.h"          /* 内嵌模式: 字库数组进 .font 段; 绑地址模式(#if 0): 空壳 */
#include "fonts12_access.h"   /* 取字偏移宏; 绑地址模式时定义绑地址指针 */
#include "fonts3x4.h"         /* nanofont3x4 3x4 小字库(列优先页模式版) */

/* oled_printf 格式化缓冲(字节): 128 够约 10 行 ASCII / 6 行汉字 */
#define OLED_PRINTF_BUF   128

/* ================= 软件 I2C（PA14=SCL, PA13=SDA） =================
 * NOP 精确延时：宏单位为“NOP 循环次数”（1 次循环 ≈ 3~4 指令周期，按主频定标）。
 * 目标 ~1.2M：低于 1.63M 临界留安全余量；对称占空比实测最稳。 */
#define OLED_I2C_HIGH_NOP      15    /* SCL 高电平 NOP 次数 */
#define OLED_I2C_LOW_NOP       15    /* SCL 低电平 NOP 次数 */
#define OLED_I2C_SETUP_NOP     3     /* 数据建立 NOP 次数 */
#define OLED_I2C_ACK_DELAY_NOP 3     /* ACK 采样前释放延时 NOP 次数 */

static uint32_t s_i2c_err = 0;   /* I2C ACK 失败累计（诊断用）*/

/* NOP 延时：不依赖 Delay_10Ns，单位 = NOP 循环次数 */
static inline void oled_delay(uint32_t nop_cnt)
{
    while (nop_cnt--) {
        __NOP();
    }
}

#define OLED_SCL_HIGH()  GPIO_SetBits(OLED_SCL_PORT, OLED_SCL_PIN)
#define OLED_SCL_LOW()   GPIO_ResetBits(OLED_SCL_PORT, OLED_SCL_PIN)
#define OLED_SDA_HIGH()  GPIO_SetBits(OLED_SDA_PORT, OLED_SDA_PIN)
#define OLED_SDA_LOW()   GPIO_ResetBits(OLED_SDA_PORT, OLED_SDA_PIN)
#define OLED_SDA_READ()  GPIO_ReadInputDataBit(OLED_SDA_PORT, OLED_SDA_PIN)

static void oled_i2c_start(void)
{
    OLED_SDA_HIGH();
    oled_delay(OLED_I2C_LOW_NOP);                    /* 空闲准备 */
    OLED_SCL_HIGH();
    oled_delay(OLED_I2C_HIGH_NOP);                   /* SCL 稳定高 */
    OLED_SDA_LOW();                                  /* START：SCL 高时 SDA 下降 */
    oled_delay(OLED_I2C_HIGH_NOP);                   /* START 保持（SCL 高）*/
    OLED_SCL_LOW();
    oled_delay(OLED_I2C_LOW_NOP);                    /* 进入低电平 */
}

static void oled_i2c_stop(void)
{
    OLED_SDA_LOW();
    oled_delay(OLED_I2C_LOW_NOP);                    /* 准备（SCL 低）*/
    OLED_SCL_HIGH();
    oled_delay(OLED_I2C_HIGH_NOP);                   /* SCL 高 */
    OLED_SDA_HIGH();                                 /* STOP：SCL 高时 SDA 上升 */
    oled_delay(OLED_I2C_HIGH_NOP);                   /* STOP 保持（SCL 高）*/
}

/* 发一字节，返回 ACK 电平（0=从机应答，1=失败）*/
static uint8_t oled_i2c_send_byte(uint8_t data)
{
    uint8_t i, ack;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) OLED_SDA_HIGH();
        else             OLED_SDA_LOW();
        data <<= 1;
        oled_delay(OLED_I2C_SETUP_NOP);              /* 数据建立（低电平期内）*/
        OLED_SCL_HIGH();
        oled_delay(OLED_I2C_HIGH_NOP);               /* 高电平保持（短）*/
        OLED_SCL_LOW();
        oled_delay(OLED_I2C_LOW_NOP);                /* 低电平（长，含数据切换裕量）*/
    }
    OLED_SDA_HIGH();                                 /* 释放 SDA 采样 ACK */
    oled_delay(OLED_I2C_ACK_DELAY_NOP);
    OLED_SCL_HIGH();
    oled_delay(OLED_I2C_HIGH_NOP);                   /* 采样窗口（高电平短）*/
    ack = OLED_SDA_READ();
    OLED_SCL_LOW();
    oled_delay(OLED_I2C_LOW_NOP);
    return ack;
}

/* 一次 I2C 事务：地址 + 控制字节(0x00=命令 / 0x40=数据) + 数据；失败累计 s_i2c_err */
static uint8_t oled_i2c_transact(uint8_t ctrl, const uint8_t *data, uint16_t len)
{
    oled_i2c_start();
    if (oled_i2c_send_byte((uint8_t)(OLED_I2C_ADDR << 1)) != 0) { s_i2c_err++; oled_i2c_stop(); return 1; }
    if (oled_i2c_send_byte(ctrl) != 0) { s_i2c_err++; oled_i2c_stop(); return 1; }
    while (len--) {
        if (oled_i2c_send_byte(*data++) != 0) { s_i2c_err++; oled_i2c_stop(); return 1; }
    }
    oled_i2c_stop();
    return 0;
}

static void ssd1306_cmd(uint8_t c)
{
    oled_i2c_transact(0x00, &c, 1);
}

/* ================= 初始化 ================= */
void oled_ssd1306_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;   /* 开漏，板上需有上拉 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    OLED_SCL_HIGH();
    OLED_SDA_HIGH();
    Delay_Us(500);                   /* 等 SSD1306 上电就绪 */

    ssd1306_cmd(0xAE);              /* display off */
    ssd1306_cmd(0x20); ssd1306_cmd(0x00);   /* 水平寻址模式（刷新可一次写完，自动翻页）*/
    ssd1306_cmd(0xB0);              /* 起始页 0 */
    ssd1306_cmd(0xC8);              /* COM 输出扫描方向（正常）*/
    ssd1306_cmd(0x40);              /* 起始行 0 */
    ssd1306_cmd(0x81); ssd1306_cmd(0xFF);   /* 对比度（用户参考 0xFF）*/
    ssd1306_cmd(0xA1);              /* 段重映射（正常）*/
    ssd1306_cmd(0xA6);              /* 正常颜色 */
    ssd1306_cmd(0xA8); ssd1306_cmd(0x3F);   /* 复用比 64 */
    ssd1306_cmd(0xA4);              /* 输出跟随 RAM */
    ssd1306_cmd(0xD3); ssd1306_cmd(0x00);   /* 显示偏移 0 */
    ssd1306_cmd(0xD5); ssd1306_cmd(0xF0);   /* 时钟分频/振荡器 */
    ssd1306_cmd(0xD9); ssd1306_cmd(0x22);   /* 预充电周期 */
    ssd1306_cmd(0xDA); ssd1306_cmd(0x12);   /* COM 引脚硬件配置 */
    ssd1306_cmd(0xDB); ssd1306_cmd(0x20);   /* VCOMH */
    ssd1306_cmd(0x8D); ssd1306_cmd(0x14);   /* DC-DC 电荷泵使能 */
    ssd1306_cmd(0xAF);              /* display on */

    // oled_clear_fb();
    oled_refresh(8);
}

/* ================= 刷屏（小核 V3F 周期调用） ================= */
void oled_refresh(uint8_t max_pages)
{
    static uint8_t start_page = 0;   // 当前刷新的起始页 0..7

    if (max_pages == 0) return;
    uint8_t pages = max_pages;
    if (start_page + pages > 8) pages = 8 - start_page;   // 不跨屏（最多刷到第 8 页）

    // 设置页起始地址（0xB0 ~ 0xB7）
    ssd1306_cmd(0xB0 | start_page);
    // 列地址从 0 开始（低字节 0x00，高字节 0x10）
    ssd1306_cmd(0x00);
    ssd1306_cmd(0x10);

    // 发送当前区域的数据：pages 页 × 128 字节/页
    oled_i2c_transact(0x40,
                      (const uint8_t *)(g_shm->oled_fb + start_page * 128),
                      pages * 128);

    // 更新静态变量，指向下一段区域
    start_page += pages;
    if (start_page >= 8) {   // 共 8 页，循环回第 0 页
        start_page = 0;
    }
}
/* ================= 修改共享显存（双核可用） ================= */

/* 清屏：按 4 字节对齐批量清零（fb 已 aligned(4)，对齐访问避免撕裂）*/
void oled_clear_fb(void)
{
    volatile uint32_t *p = (volatile uint32_t *)g_shm->oled_fb;
    uint16_t i;
    for (i = 0; i < OLED_FB_SIZE / 4; i++) p[i] = 0;
}

/* 画实心方块：矩形内像素置亮(on=1)/清除(on=0)，页主序写入共享显存 */
void oled_fill_rect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t on)
{
    uint8_t x, y;
    if (x0 > x1) { uint8_t t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { uint8_t t = y0; y0 = y1; y1 = t; }
    if (x1 >= OLED_W) x1 = (uint8_t)(OLED_W - 1);
    if (y1 >= OLED_H) y1 = (uint8_t)(OLED_H - 1);
    for (y = y0; y <= y1; y++) {
        uint32_t base = (uint32_t)(y >> 3) * OLED_W;
        uint8_t  bit  = (uint8_t)(1u << (y & 7));
        for (x = x0; x <= x1; x++) {
            uint32_t idx = base + x;
            if (on) g_shm->oled_fb[idx] |= bit;
            else    g_shm->oled_fb[idx] &= (uint8_t)~bit;
        }
    }
}

// 矩形尺寸与移动速度（像素/帧）
#define RECT_W  16
#define RECT_H  16
#define SPEED_X 2
#define SPEED_Y 1

/**
 * @brief 矩形弹跳动画，每次调用更新一帧
 * @note  内部静态计数器递增，用三角波计算位置，实现边界反弹。
 *        先清空显存再绘制新矩形，保证无拖影。
 */
void oled_animation(void)
{
    static uint32_t counter = 0;   // 帧计数器，自增作为时间基准
    counter++;

    // 计算可移动范围（左上角坐标取值范围）
    int max_x = OLED_W - RECT_W;
    int max_y = OLED_H - RECT_H;

    // 使用三角波计算 x 坐标：先取模再折返
    int raw_x = (int)(counter * SPEED_X) % (2 * max_x);
    int x = (raw_x <= max_x) ? raw_x : (2 * max_x - raw_x);

    int raw_y = (int)(counter * SPEED_Y) % (2 * max_y);
    int y = (raw_y <= max_y) ? raw_y : (2 * max_y - raw_y);

    // 清空整个显存（所有像素熄灭）
    memset((void*)g_shm->oled_fb, 0, OLED_FB_SIZE);

    // 绘制实心矩形（亮）
    oled_fill_rect((uint8_t)x, (uint8_t)y,
                   (uint8_t)(x + RECT_W - 1),
                   (uint8_t)(y + RECT_H - 1), 1);
}

/**
 * @brief 静态标定图案，四个固定位置正方形
 * @note  左上最小、右上稍大、右下再大、左下最大，不运动，仅静态绘制
 */
void oled_check(void)
{
    // 四个正方形边长
    #define S1_SIZE 3    // 左上 最小
    #define S2_SIZE 5    // 右上 稍大
    #define S3_SIZE 7    // 右下 再大
    #define S4_SIZE 9    // 左下 最大

    int mid_x = OLED_W / 2;
    int mid_y = OLED_H / 2;

    // 清空显存
    memset((void*)g_shm->oled_fb, 0, OLED_FB_SIZE);

    // 左上(0,0附近) 最小正方形
    oled_fill_rect(0, 0, S1_SIZE - 1, S1_SIZE - 1, 1);
    // 右上(mid_x附近) 稍大正方形
    oled_fill_rect(mid_x, 0, mid_x + S2_SIZE - 1, S2_SIZE - 1, 1);
    // 右下
    oled_fill_rect(mid_x, mid_y, mid_x + S3_SIZE - 1, mid_y + S3_SIZE - 1, 1);
    // 左下
    oled_fill_rect(0, mid_y, S4_SIZE - 1, mid_y + S4_SIZE - 1, 1);
}

/* 写入两段(页0/页1)点阵到显存; 自动按目标页拆分(非对齐 y 跨页安全) */
static void blit_planes(uint8_t y, uint8_t x, const uint8_t *pg0, uint8_t w,
                        uint8_t seg0, const uint8_t *pg1, uint8_t seg1, uint8_t on)
{
    const uint8_t *srcs[2] = { pg0, pg1 };
    uint8_t segs[2] = { seg0, seg1 };
    uint8_t base_row[2] = { 0, 8 };
    uint8_t p;
    for (p = 0; p < 2; p++) {
        uint8_t seg = segs[p];
        uint8_t r = 0;
        if (!seg)
            continue;
        while (r < seg) {
            uint8_t yy = (uint8_t)(y + base_row[p] + r);
            uint8_t page = (uint8_t)(yy >> 3);
            uint8_t bit = (uint8_t)(yy & 7);
            uint8_t cnt = (uint8_t)(8 - bit);            /* 本目标页可写行数 */
            if (cnt > seg - r)
                cnt = (uint8_t)(seg - r);
            {
                uint32_t base = (uint32_t)page * OLED_W;
                uint8_t col;
                if (bit == 0 && cnt == 8) {              /* 整字节独占: 直接覆盖 */
                    for (col = 0; col < w; col++) {
                        if (on) g_shm->oled_fb[base + x + col] = srcs[p][col];
                        else    g_shm->oled_fb[base + x + col] = 0;
                    }
                } else {
                    uint16_t mask = (uint16_t)(((1u << cnt) - 1u) << bit);
                    uint8_t not_mask = (uint8_t)~mask;
                    for (col = 0; col < w; col++) {
                        uint32_t idx = base + x + col;
                        uint8_t val = (uint8_t)(((uint16_t)(srcs[p][col] >> r) << bit) & mask);
                        if (on) g_shm->oled_fb[idx] = (uint8_t)((g_shm->oled_fb[idx] & not_mask) | val);
                        else    g_shm->oled_fb[idx] &= not_mask;
                    }
                }
            }
            r = (uint8_t)(r + cnt);
        }
    }
}

uint8_t oled_put_char(const char *s, uint8_t x, uint8_t y, uint8_t on)
{
    const uint8_t *font;
    uint8_t w, seg0, seg1;

    if (s == NULL)
        return 0;

    {
        uint8_t c0 = (uint8_t)s[0];
        if (c0 >= 0x20 && c0 <= 0x7F) {             /* ASCII */
            font = &asc8x12_font[ASC8X12_OFFSET(c0)];
            w = 8;
        } else if ((c0 >= 0xA1 && c0 <= 0xA9) ||      /* GB2312 符号区 */
                   (c0 >= 0xB0 && c0 <= 0xD7)) {      /* GB2312 一级汉字 */
            uint8_t c1 = (uint8_t)s[1];
            if (c1 < 0xA1)
                return 0;
            font = &hzk12_font[HZK12_OFFSET(c0, c1)];
            w = 12;
        } else {
            return 0;
        }
    }

    if (x >= OLED_W || y >= OLED_H)
        return 0;
    if ((uint16_t)x + w > OLED_W)
        w = (uint8_t)(OLED_W - x);

    /* 可用行高 -> 两段: 页0(行0~7), 页1(行8~11) */
    {
        uint8_t avail = (uint8_t)(OLED_H - y);
        seg0 = (avail > 8) ? 8 : avail;
        seg1 = (avail > 12) ? 4 : (avail > 8 ? (uint8_t)(avail - 8) : 0);
    }

    blit_planes(y, x, font, w, seg0, font + w, seg1, on);

    return ((uint8_t)s[0] < 0x80u) ? 1u : 2u;
}

/* 字符串绘制核心: \r 忽略, \n 换行, 非法字符用 ASCII '?' 替换继续; 超宽自动换行。
 * 每个字符绘制前先涂黑其区域背景(12 行), 防旧内容残留 */
static uint8_t oled_draw_str(uint8_t x0, uint8_t y0, const char *s)
{
    const char *p = s;
    uint8_t x = x0, y = y0, n = 0;
    while (*p) {
        if (*p == '\r') { p++; continue; }
        if (*p == '\n') { x = x0; y += 12; p++; if (y >= OLED_H) break; continue; }
        uint8_t c0 = (uint8_t)*p;
        uint8_t w = 8;                                   /* ASCII 8 宽 */
        if ((c0 >= 0xA1 && c0 <= 0xA9) || (c0 >= 0xB0 && c0 <= 0xD7)) w = 12;  /* GB 12 宽 */
        /* 绘制前涂黑字符区域背景(12 行), 防旧内容残留 */
        if (x < OLED_W && y < OLED_H) {
            uint8_t xe = (uint8_t)(((uint16_t)x + w - 1 < OLED_W) ? (x + w - 1) : (OLED_W - 1));
            uint8_t ye = (uint8_t)(((uint16_t)y + 11 < OLED_H) ? (y + 11) : (OLED_H - 1));
            oled_fill_rect(x, y, xe, ye, 0);
        }
        uint8_t r = oled_put_char(p, x, y, 1);
        if (r == 1)      { x += 8;  p += 1; }   /* ASCII 8 宽 */
        else if (r == 2) { x += 12; p += 2; }   /* GB 12 宽 */
        else {
            /* 非法字符: 用 ASCII '?' 替换显示, 继续绘制 */
            if (x < OLED_W) oled_put_char("?", x, y, 1);
            x += 8;
            p += ((uint8_t)*p >= 0x80 && (uint8_t)p[1] != 0) ? 2 : 1;
        }
        n++;
        if ((uint16_t)x + 12 > OLED_W) { x = x0; y += 12; }  /* 超宽换行 */
        if (y >= OLED_H) break;                             /* 出屏保护 */
    }
    return n;
}

/* 从 (x0,y0) 绘制整串, 超宽自动换行; 返回实际绘制字符数, 非法字符替换为 '?' */
uint8_t oled_puts(uint8_t x0, uint8_t y0, const char *s)
{
    return oled_draw_str(x0, y0, s);
}

/* printf 风格格式化绘制(形如 mipc_v3_printf / printf): 变参 + 格式化,
 * 支持 %d/%f/%s 等; \n 换行 \r 忽略, 非法字符替换为 '?' */
uint8_t oled_printf(uint8_t x0, uint8_t y0, const char *fmt, ...)
{
    char buf[OLED_PRINTF_BUF];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return oled_draw_str(x0, y0, buf);
}

/**
 * 绘制图像（列连续存储，直接调用 blit_planes）
 * @param x         起始列
 * @param y         起始行
 * @param img_data  图像数据（页连续，每页 width 字节）
 * @param width     图像宽度
 * @param height    图像高度
 * @param on        1=绘制，0=擦除
 */
void oled_draw_image(uint8_t x, uint8_t y,
                     const uint8_t *img_data,
                     uint8_t width, uint8_t height,
                     uint8_t on)
{
    if (!img_data || x >= OLED_W || y >= OLED_H) return;

    // 裁剪右边界
    if ((uint16_t)x + width > OLED_W) width = OLED_W - x;
    if (width == 0) return;

    // 裁剪下边界
    if ((uint16_t)y + height > OLED_H) height = OLED_H - y;
    if (height == 0) return;

    uint8_t seg = height;                // 剩余行数
    uint8_t cur_y = y;
    uint8_t page_offset = 0;             // 当前页在 img_data 中的偏移（字节）

    while (seg > 0) {
        uint8_t seg_rows = (seg > 8) ? 8 : seg;
        // 调用 blit_planes：只有 pg0 有效，seg1 = 0
        blit_planes(cur_y, x,
                    &img_data[page_offset * width], width, seg_rows,
                    NULL, 0,
                    on);
        cur_y += seg_rows;
        seg -= seg_rows;
        page_offset++;
    }
}

/* ==================== nanofont3x4 3x4 小字库(双核共享绘图库) ====================
 * 数据: nano3x4_font[96][3] 列优先页模式版(见 fonts3x4.h), 绘制整列移位, 免逐像素。
 * 仅 ASCII 0x20~0x7F; 非法字符(含汉字/控制符)替换为 '?'。 */

/* 3x4 单字符绘制: 处理页内(整列移位一次)与跨两页(本页低段+下页高段)。
 * on=1 时先涂黑字符区域(4 行)背景再置位, 防旧内容残留 */
static void oled_nano_put_char(uint8_t x, uint8_t y, uint8_t c, uint8_t on)
{
    if ((uint16_t)x + NANO3X4_W > OLED_W || (uint16_t)y + NANO3X4_H > OLED_H) return;
    const uint8_t *col = nano3x4_font[c - 0x20];
    uint8_t page = y >> 3, bit = y & 7;
    volatile uint8_t *fb = &g_shm->oled_fb[(uint16_t)page * OLED_W + x];
    uint8_t i;
    if (bit + NANO3X4_H <= 8) {
        uint8_t mask = (uint8_t)(0x0F << bit);        /* 4 行背景掩码 */
        for (i = 0; i < NANO3X4_W; i++) {
            if (on) fb[i] = (uint8_t)((fb[i] & (uint8_t)~mask) | ((uint8_t)(col[i] << bit)));
            else    fb[i] &= (uint8_t)~mask;
        }
    } else {
        /* 跨两页: 本页(bit..7)低段 + 下页(bit0 起)高段 */
        uint8_t seg = (uint8_t)(8 - bit);
        uint8_t mask_lo = (uint8_t)(0x0F << bit);     /* 本页掩码(自动截断) */
        uint8_t mask_hi = (uint8_t)(0x0F >> seg);     /* 下页掩码 */
        volatile uint8_t *fb2 = fb + OLED_W;
        for (i = 0; i < NANO3X4_W; i++) {
            uint8_t lo = (uint8_t)(col[i] << bit);
            uint8_t hi = (uint8_t)(col[i] >> seg);
            if (on) {
                fb[i]  = (uint8_t)((fb[i]  & (uint8_t)~mask_lo) | lo);
                fb2[i] = (uint8_t)((fb2[i] & (uint8_t)~mask_hi) | hi);
            } else {
                fb[i]  &= (uint8_t)~mask_lo;
                fb2[i] &= (uint8_t)~mask_hi;
            }
        }
    }
}

/* 清除 (x, y..y+3) 一列 4 行背景(用于字符间间距列), 防残留 */
static void oled_nano_clear_gap(uint8_t x, uint8_t y)
{
    if (x >= OLED_W || (uint16_t)y + NANO3X4_H > OLED_H) return;
    uint8_t page = y >> 3, bit = y & 7;
    volatile uint8_t *fb = &g_shm->oled_fb[(uint16_t)page * OLED_W + x];
    if (bit + NANO3X4_H <= 8) {
        fb[0] &= (uint8_t)~(0x0F << bit);
    } else {
        fb[0] &= (uint8_t)~(0x0F << bit);
        fb[OLED_W] &= (uint8_t)~(0x0F >> (8 - bit));
    }
}

/* 3x4 字符串绘制: \r 忽略, \n 换行, 非法字符替换为 '?'; 返回实际绘制字符数 */
uint8_t oled_nano_puts(uint8_t x0, uint8_t y0, const char *s)
{
    const char *p = s;
    uint8_t x = x0, y = y0, n = 0;
    while (*p) {
        uint8_t c = (uint8_t)*p;
        if (c == '\r') { p++; continue; }
        if (c == '\n') { x = x0; y += NANO3X4_LINE_H; p++; if (y >= OLED_H) break; continue; }
        if (c >= 0x20 && c <= 0x7F) {
            oled_nano_put_char(x, y, c, 1);
            p++;
        } else {
            oled_nano_put_char(x, y, '?', 1);                 /* 非法字符替换 */
            p += (c >= 0x80 && (uint8_t)p[1] != 0) ? 2 : 1;
        }
        oled_nano_clear_gap((uint8_t)(x + NANO3X4_W), y);     /* 清字符间 1 列间距背景 */
        x += NANO3X4_W + 1;                                   /* 3 宽 + 1 列间距 */
        n++;
        if ((uint16_t)x + NANO3X4_W > OLED_W) { x = x0; y += NANO3X4_LINE_H; }
        if (y >= OLED_H) break;
    }
    return n;
}

/* 3x4 printf 风格格式化绘制(形如 oled_printf): 变参 + 格式化 */
uint8_t oled_nano_printf(uint8_t x0, uint8_t y0, const char *fmt, ...)
{
    char buf[OLED_PRINTF_BUF];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    return oled_nano_puts(x0, y0, buf);
}
