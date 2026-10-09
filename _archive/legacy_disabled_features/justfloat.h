/*******************************************************************************
 * File Name        : justfloat.h
 * Description      : justfloat（FireWater）二进制上报格式
 *
 * 帧结构（小端）：
 *      float32  = 0x7F800000   (+Inf 位型)   <- 同步头
 *      float32  = 0x7F800000                  <- 同步头（双写，抗误同步）
 *      float32  ch0
 *      float32  ch1
 *      ...
 *
 * 上位机（如 Synex / VOFA+）逐字节扫描同步头，命中后按通道数取 float，
 * 因此不需要帧长或校验字段，也不会因为中间丢字节而永久错位。
 *
 * 串口开销（921600 波特 = 92160 字节/秒）：
 *      3 通道 -> 20 字节/帧 -> 1kHz 占 21.7%
 *      4 通道 -> 24 字节/帧 -> 1kHz 占 26.0%
 *      8 通道 -> 40 字节/帧 -> 1kHz 占 43.4%
 * 对比 ASCII 逐样本一行（约 34 字节）占 36.9%，且 ASCII 还要付格式化开销。
 *
 * 注意：发送期间不能被中断打断到"帧中间插入其他输出"。
 *       本工程的调试打印与 justfloat 共用同一个串口，二者不要混用。
 *******************************************************************************/

#ifndef __JUSTFLOAT_H
#define __JUSTFLOAT_H

#include <stdint.h>

/* 同步头：+Inf 的 IEEE754 位型 */
#define JUSTFLOAT_SYNC         0x7F800000u

/* 发送一帧：head 为同步头（通常写两次，见 justfloat_send），
 * 后接 nch 个 float。返回发送的通道数。 */
void justfloat_send_head(void);
void justfloat_send_f32(float v);
void justfloat_send_u32(uint32_t v);
void justfloat_send(const float *ch, uint32_t nch);

/* 便于直接传单个通道 */
void justfloat_send1(float a);
void justfloat_send2(float a, float b);
void justfloat_send3(float a, float b, float c);
void justfloat_send4(float a, float b, float c, float d);

/* 发送一个原始字节（供同步头使用） */
void justfloat_putb(uint8_t b);

#endif /* __JUSTFLOAT_H */
