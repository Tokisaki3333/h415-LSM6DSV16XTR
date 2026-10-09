#ifndef GPS_H
#define GPS_H

#include <string.h>
#include "ch32h417.h"
#include "debug.h"

#include "sys_clk.h"

void GPS_USART_Init(void);

/* 轮询一次：解析 DMA 收到的新数据，返回本次处理的字节数。
 *   max_bytes = 0 -> 不限制，一次处理到底
 *   max_bytes > 0 -> 本次最多处理这么多字节，剩余的留给下次调用
 * 实测解析约 0.45us/字节，调用方可按"允许耗时"反推 max_bytes。 */
uint32_t GPS_Check(uint32_t max_bytes);

/* 上次 GPS_Check 是否已把缓冲处理到底 */
uint8_t  GPS_CaughtUp(void);

void GPS_Send_Cmd(const char *cmd);

/* 第一级语句分派 + 第二级解析（增量状态机，支持 DMA 回绕跨段续读）。
 * data     : 本段数据（GPS_Check 传入的连续块）
 * len      : 本段精确长度（段边界由长度控制，不依赖 '\0' 哨兵——哨兵会被
 *            DMA 覆盖导致误读回绕区旧数据）
 * abs_base : data 在 RxBuffer2 中的绝对起始下标（用于跨段定位）
 * clk      : 本段到达时间戳（10 ns 计数） */
void parse_gps_data(uint8_t *data, uint32_t len, uint32_t abs_base, uint64_t clk);

#endif
