#ifndef GPS_H
#define GPS_H

#include <string.h> 
#include "ch32h417.h"
#include "debug.h"

#include "sys_clk.h"

void GPS_USART_Init(void);
void GPS_Check(void);
void GPS_Send_Cmd(const char *cmd);

/* 第一级语句分派 + 第二级解析（增量状态机，支持 DMA 回绕跨段续读）。
 * data     : 本段数据（GPS_Check 传入的连续块）
 * len      : 本段精确长度（段边界由长度控制，不依赖 '\0' 哨兵——
 *            哨兵会被 DMA 覆盖导致误读回绕区旧数据）
 * abs_base : data 在 RxBuffer2 中的绝对起始下标（用于跨段定位）
 * clk      : 本段到达时间戳（微秒）
 */
void parse_gps_data(uint8_t *data, uint32_t len, uint32_t abs_base, uint64_t clk);

#endif
