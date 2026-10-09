/**
 * @file lv_port_disp_templ.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include <stdbool.h>
#include "oled_ssd1306.h"   /* OLED_W/OLED_H、g_shm->oled_fb（双核共享显存，页主序）*/
#include "sys_clk.h"        /* GetTime64_Us() */
#include "debug.h"          /* printf 诊断 */

/*********************
 *      DEFINES
 *********************/
#ifndef MY_DISP_HOR_RES
#define MY_DISP_HOR_RES OLED_W      /* SSD1306: 128 */
#endif

#ifndef MY_DISP_VER_RES
#define MY_DISP_VER_RES OLED_H      /* SSD1306: 64 */
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p);
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//        const lv_area_t * fill_area, lv_color_t color);

static lv_disp_drv_t disp_drv; /*Descriptor of a display driver*/

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/

    /**
     * LVGL requires a buffer where it internally draws the widgets.
     * Later this buffer will passed to your display driver's `flush_cb` to copy its content to your display.
     * The buffer has to be greater than 1 display row
     *
     * There are 3 buffering configurations:
     * 1. Create ONE buffer:
     *      LVGL will draw the display's content here and writes it to your display
     *
     * 2. Create TWO buffer:
     *      LVGL will draw the display's content to a buffer and writes it your display.
     *      You should use DMA to write the buffer's content to the display.
     *      It will enable LVGL to draw the next part of the screen to the other buffer while
     *      the data is being sent form the first buffer. It makes rendering and flushing parallel.
     *
     * 3. Double buffering
     *      Set 2 screens sized buffers and set disp_drv.full_refresh = 1.
     *      This way LVGL will always provide the whole rendered screen in `flush_cb`
     *      and you only need to change the frame buffer's address.
     */

    /* Example for 1) */
    static lv_disp_draw_buf_t                                        draw_buf_dsc_1;
    static lv_color_t buf_1[MY_DISP_HOR_RES * MY_DISP_VER_RES]; /*A buffer for 10 rows*/
    lv_disp_draw_buf_init(&draw_buf_dsc_1, buf_1, NULL, MY_DISP_HOR_RES * MY_DISP_VER_RES);                    /*Initialize the display buffer*/

    /* Example for 2) */
    // static lv_disp_draw_buf_t                                        draw_buf_dsc_2;
    // static lv_color_t buf_2_1[MY_DISP_HOR_RES * 100]; /*A buffer for 10 rows*/
    // static lv_color_t buf_2_2[MY_DISP_HOR_RES * 100]; /*An other buffer for 10 rows*/
    // lv_disp_draw_buf_init(&draw_buf_dsc_2, buf_2_1, buf_2_2, MY_DISP_HOR_RES * 100);                 /*Initialize the display buffer*/

    /* Example for 3) also set disp_drv.full_refresh = 1 below*/
    //    static lv_disp_draw_buf_t draw_buf_dsc_3;
    //    static lv_color_t buf_3_1[MY_DISP_HOR_RES * MY_DISP_VER_RES];            /*A screen sized buffer*/
    //    static lv_color_t buf_3_2[MY_DISP_HOR_RES * MY_DISP_VER_RES];            /*Another screen sized buffer*/
    //    lv_disp_draw_buf_init(&draw_buf_dsc_3, buf_3_1, buf_3_2,
    //                          MY_DISP_VER_RES * LV_VER_RES_MAX);   /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    lv_disp_drv_init(&disp_drv); /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = MY_DISP_HOR_RES;
    disp_drv.ver_res = MY_DISP_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc_1;

    /*Required for Example 3)*/
    //disp_drv.full_refresh = 1;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    //disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    /*You code here*/
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/* Flush：LVGL 1-bit 帧缓冲（行主序，1 字节/像素，bit0=颜色）
 *   → SSD1306 共享显存（页主序，8 像素/字节，bit(y%8)=颜色）。
 * 只写共享显存 g_shm->oled_fb，不碰 I2C：屏幕传输由主循环 oled_refresh() 分片完成，保持时序。*/
static void disp_flush(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p)
{
    if (disp_flush_enabled)
    {
        int32_t x, y;
        const int32_t w = area->x2 - area->x1 + 1;
        for (y = area->y1; y <= area->y2; y++) {
            const uint8_t  bit = (uint8_t)(1u << (y & 7));
            const uint32_t off = (uint32_t)(y >> 3) * OLED_W;
            for (x = area->x1; x <= area->x2; x++) {
                const uint32_t idx = (uint32_t)(y - area->y1) * (uint32_t)w + (uint32_t)(x - area->x1);
                if (color_p[idx].full & 1u) g_shm->oled_fb[off + (uint32_t)x] |= bit;
                else                        g_shm->oled_fb[off + (uint32_t)x] &= (uint8_t)~bit;
            }
        }
        lv_disp_flush_ready(disp_drv);
    }
}

/*OPTIONAL: GPU INTERFACE*/

/*If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color*/
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//                    const lv_area_t * fill_area, lv_color_t color)
//{
//    /*It's an example code which should be done by your GPU*/
//    int32_t x, y;
//    dest_buf += dest_width * fill_area->y1; /*Go to the first line*/
//
//    for(y = fill_area->y1; y <= fill_area->y2; y++) {
//        for(x = fill_area->x1; x <= fill_area->x2; x++) {
//            dest_buf[x] = color;
//        }
//        dest_buf+=dest_width;    /*Go to the next line*/
//    }
//}

/* LVGL tick：GetTime64_Us()/1000（lv_conf.h LV_TICK_CUSTOM 使用）*/
uint32_t lv_tick_get_ms(void)
{
    return (uint32_t)(GetTime64_Us() / 1000);
}

/* ===================== 演示 UI =====================
 * 128x64 小屏：用 unscii_8（8x8 等宽位图字体，非抗锯齿，1-bit 色深下渲染干净）
 * 显式坐标布局，不依赖 center/align，避免元素重叠。 */
static lv_obj_t *s_cnt_lbl;

static void ui_cnt_cb(lv_timer_t *tm)
{
    static uint32_t n = 0;
    lv_label_set_text_fmt(s_cnt_lbl, "n=%u", (unsigned)(n++));
}

void lv_demo_create(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    /* 标题：顶部居中（"LVGL OK" 8 字符 × 8px = 64px 宽，x=32 居中）*/
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "LVGL OK");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_unscii_8, 0);
    lv_obj_set_pos(title, 32, 8);

    /* 计数：底部居中 */
    s_cnt_lbl = lv_label_create(scr);
    lv_obj_set_style_text_color(s_cnt_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_cnt_lbl, &lv_font_unscii_8, 0);
    lv_obj_set_pos(s_cnt_lbl, 32, 52);

    lv_timer_create(ui_cnt_cb, 500, NULL);
}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
