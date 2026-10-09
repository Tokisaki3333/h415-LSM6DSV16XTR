/**
 * @file lv_port_disp_templ.h
 *
 */

/*Copy this file as "lv_port_disp.h" and set this value to "1" to enable content*/
#if 1

#ifndef LV_PORT_DISP_A_H
#define LV_PORT_DISP_A_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#if 1// defined(LV_LVGL_H_INCLUDE_SIMPLE)
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
/* Initialize low level display driver */
void lv_port_disp_init(void);

/* LVGL tick（ms）：GetTime64_Us()/1000，供 lv_conf.h LV_TICK_CUSTOM 使用 */
uint32_t lv_tick_get_ms(void);

/* 演示 UI：居中标题 + 底部计数，由主循环 lv_timer_handler() 驱动 */
void lv_demo_create(void);

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void);

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_PORT_DISP_TEMPL_H*/

#endif /*Disable/Enable content*/
