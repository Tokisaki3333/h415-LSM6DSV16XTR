#ifndef MIPC_V3_H
#define MIPC_V3_H

#include "ch32h417.h"
#include "debug.h"
#include "mipc_shm.h"

/* 共享区指针（V3F 定义 storage，g_shm 指向它；地址经 IPC_MSG0 传给 V5F，两端同名同型）
 * 通道：v5f_progress=握手、v3f_cfg_done=IMU 配置完成、log=低频文本、
 *       gyro/ist/bmp=三个传感器（先写数据→再写 DRDY 时间戳→cnt++ 发布）*/
extern volatile shared_mem_t *g_shm;

uint8_t ipc_comm_init(void);
void    mipc_v3_printf(const char *fmt, ...);  /* printf-like: 写 log 通道 data_buf & cnt++ */

#endif
