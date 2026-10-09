#ifndef MIPC_V5_H
#define MIPC_V5_H

#include "ch32h417.h"
#include "mipc_shm.h"

/* 共享区指针（V3F 定义共享区，地址经 IPC_MSG0 传入；布局由 mipc_shm.h 统一）
 * 通道：v5f_progress=握手、v3f_cfg_done=IMU 配置完成、log=低频文本、
 *       gyro/ist/bmp=三个传感器（读端协议见 mipc_shm.h 注释）*/
extern volatile shared_mem_t *g_shm;

/* Staged handshake: V5F reports init progress via g_shm->v5f_progress
 * 1=addr got  2=SPI1_Init  3=ICM42605 config  4=RX+DMA irq ready  5=all ready */
uint8_t mipc_v5_get_shared(void);             /* step1: poll IPC for shared addr */
void    mipc_v5_sync(uint8_t step);           /* step2..5: report handshake progress */

#endif
