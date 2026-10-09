#ifndef SYS_CLK_H
#define SYS_CLK_H

#include "ch32h417.h"

void SYS_CLK_Init(void);

uint64_t GetTime64_Us(void);
uint64_t GetTime64_10Ns(void);
void HWT_DelayUs(uint64_t us);
void HWT_DelayMs(uint64_t ms);

#endif