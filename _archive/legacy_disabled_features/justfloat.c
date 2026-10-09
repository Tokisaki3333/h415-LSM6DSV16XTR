/*******************************************************************************
 * File Name        : justfloat.c
 * Description      : justfloat（FireWater）二进制上报实现
 *
 * 依赖：debug.h 里的 DEBUG 宏决定用哪个 USART；本工程为 DEBUG_UART1。
 * 用法：与 USART_Printf_Init(baud) 共用同一个串口，不要与 printf 混用。
 *******************************************************************************/

#include "justfloat.h"
#include "debug.h"

/*------------------------------------------------------------------------------
 * 按 DEBUG 宏取出实际使用的 USART
 *----------------------------------------------------------------------------*/
#if (DEBUG == DEBUG_UART1)
#define JF_USART        USART1
#elif (DEBUG == DEBUG_UART8)
#define JF_USART        USART8
#elif (DEBUG == DEBUG_UART6)
#define JF_USART        USART6
#else
#define JF_USART        USART1
#endif

/*------------------------------------------------------------------------------
 * 发送一个字节（轮询 TXE）
 *----------------------------------------------------------------------------*/
void justfloat_putb(uint8_t b)
{
    while (USART_GetFlagStatus(JF_USART, USART_FLAG_TXE) == RESET)
    {
    }
    USART_SendData(JF_USART, (uint16_t)b);
}

/*------------------------------------------------------------------------------
 * 小端发送 4 字节
 *----------------------------------------------------------------------------*/
void justfloat_send_u32(uint32_t v)
{
    justfloat_putb((uint8_t)(v & 0xFFu));
    justfloat_putb((uint8_t)((v >> 8) & 0xFFu));
    justfloat_putb((uint8_t)((v >> 16) & 0xFFu));
    justfloat_putb((uint8_t)((v >> 24) & 0xFFu));
}

void justfloat_send_f32(float v)
{
    union
    {
        float    f;
        uint32_t u;
    } cv;

    cv.f = v;
    justfloat_send_u32(cv.u);
}

/*------------------------------------------------------------------------------
 * 同步头：连写两次，上位机命中一次即可对齐
 *----------------------------------------------------------------------------*/
void justfloat_send_head(void)
{
    justfloat_send_u32(JUSTFLOAT_SYNC);
    justfloat_send_u32(JUSTFLOAT_SYNC);
}

/*------------------------------------------------------------------------------
 * 一帧：同步头 + nch 个 float
 *----------------------------------------------------------------------------*/
void justfloat_send(const float *ch, uint32_t nch)
{
    uint32_t i;

    justfloat_send_head();
    for (i = 0u; i < nch; i++)
        justfloat_send_f32(ch[i]);
}

void justfloat_send1(float a)
{
    justfloat_send_head();
    justfloat_send_f32(a);
}

void justfloat_send2(float a, float b)
{
    justfloat_send_head();
    justfloat_send_f32(a);
    justfloat_send_f32(b);
}

void justfloat_send3(float a, float b, float c)
{
    justfloat_send_head();
    justfloat_send_f32(a);
    justfloat_send_f32(b);
    justfloat_send_f32(c);
}

void justfloat_send4(float a, float b, float c, float d)
{
    justfloat_send_head();
    justfloat_send_f32(a);
    justfloat_send_f32(b);
    justfloat_send_f32(c);
    justfloat_send_f32(d);
}
