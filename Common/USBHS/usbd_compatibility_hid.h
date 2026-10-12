/********************************** (C) COPYRIGHT *******************************
 * File Name          : usbd_compatibility_hid.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : header file of usbd_compatibility_hid.c
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef USER_USBD_COMPATIBILITY_HID_H_
#define USER_USBD_COMPATIBILITY_HID_H_

#include "usb_desc.h"
#define DEF_UART_BUF_SIZE               8192        /* 上行字节流环形容量 */

extern volatile uint16_t Data_Pack_Max_Len;                         // (预留,CDC下未用)
extern volatile uint16_t Head_Pack_Len;                             // (预留,CDC下未用)

/* ---- USBHS-CDC 上行:裸字节流,严格顺序,只依赖 USBHS 一个高频中断 ----
 *  生产者(IMU 的 DMA-RX 中断) -> hid_up_enqueue()
 *  消费者(单一,主循环高频轮询) -> hid_up_flush()
 *  USBHS_IRQHandler 只做枚举/端点事件;装载统一在主循环完成 */

extern uint16_t hid_up_avail(void);                /* 剩余可写字节数 */
extern uint8_t  hid_up_enqueue(const uint8_t*, uint16_t len); /* 1=入队;0=空间不足放弃 */
extern volatile uint32_t g_ep_busy;   /* 端点忙导致装包失败 */
extern volatile uint32_t g_ep_empty;  /* 环内无字节 */
extern volatile uint32_t g_ep_sent;   /* 实际装包成功 */
extern volatile uint32_t g_ep_bytes;  /* 累计装出字节 */
extern void     hid_up_flush(void);                /* 主循环高频:有字节就把 1 个 CDC bulk 包灌 EP2 */

/* ---- 下行（主机→设备）----
 * 生产者 = USBHS 中断里的 EP2 bulk OUT（只入环，不解析）
 * 消费者 = 主循环里的 hid_cmd_poll()（解析并执行）
 * 整包或丢弃：放不下就整段放弃，保证字节流严格顺序。 */
#define DEF_DOWN_BUF_SIZE               1024        /* 下行环形容量 */

extern void     hid_down_push(const uint8_t*, uint16_t len);  /* 中断里调用：只入环 */
extern void     hid_cmd_poll(void);                           /* 主循环调用：解析并执行 */

extern volatile float    g_cmd_echo;    /* 'T' 写进来的值，原样上报（下行验证用） */
extern volatile uint16_t g_cmd_cnt;     /* 收到的合法命令数 */
extern volatile uint8_t  g_cmd_last;    /* 最近一条命令的 opcode */

#endif /* USER_USBD_COMPATIBILITY_HID_H_ */
