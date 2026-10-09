/*******************************************************************************
 * File Name        : lsm6dsv16x.h
 * Description      : LSM6DSV16XTR 六轴 IMU 驱动（SPI2，片选 PD10）
 *
 * 寄存器定义见 lsm6dsv16x_reg.h —— 本文件只提供接口。
 *
 * 【本板工作点】陀螺仪 HAODR 模式 + 最高 ODR 7.68kHz + ±4000 dps
 *   加速度计同步进 HAODR（手册 §6.5 约束：两者必须一致）
 *
 * 【已核对的器件行为与规格】（DS13510 Rev 4）
 *   1) WHO_AM_I(0Fh) 固定读回 0x70
 *   2) 零速率电平 G_TyOff  ±1 dps（室温）
 *      温漂 G_OffDr        ±0.006 dps/°C
 *      噪声密度 Rn          2.8 mdps/√Hz（高性能模式）
 *   3) 灵敏度 ±4000 dps 时 140 mdps/LSB；灵敏度容差 ±0.3%
 *   4) 数据 16 位二进制补码，L 低字节在前，X/Y/Z 连续（0x22 起 6 字节）
 *   5) 数据就绪：STATUS_REG(1Eh).GDA (bit1)  —— 与
 *      INT1_CTRL(0Dh).INT1_DRDY_G (bit1) 同位，便于先轮询后切中断
 *   6) 选 ±4000 dps 要求陀螺 OIS 链关闭（OIS_G_EN = 0），其复位默认即为 0
 *   7) HAODR 使能/关闭必须在【掉电模式】下进行；与模拟前端/Qvar、
 *      活动-静止检测不兼容（本板不使用这两项）
 *******************************************************************************/

#ifndef __LSM6DSV16X_H
#define __LSM6DSV16X_H

#include <stdint.h>
#include "ch32h417.h"

/*==============================================================================
 * 返回值
 *============================================================================*/
#define LSM6_OK                 0
#define LSM6_ERR_PARAM         (-1)
#define LSM6_ERR_SPI           (-2)
#define LSM6_ERR_NO_ID         (-3)     /* WHO_AM_I 不符 */
#define LSM6_ERR_DRDY_TIMEOUT  (-4)     /* 等 GDA 超时 */

/*==============================================================================
 * 量与数据结构
 *============================================================================*/
typedef struct
{
    int16_t x;              /* 原始 16 位补码 */
    int16_t y;
    int16_t z;
} LSM6_GyroRaw;

typedef struct
{
    float x;                /* dps */
    float y;
    float z;
} LSM6_GyroDps;

/* 一次读回的全部输出（温度 + 六轴），原始值 */
typedef struct
{
    int16_t temp_raw;       /* 温度原始值：256 LSB/°C，0 LSB 约对应 25°C */
    int16_t gx, gy, gz;     /* 陀螺原始值：±4000dps 下 0.140 dps/LSB */
    int16_t ax, ay, az;     /* 加速度计原始值 */
} LSM6_AllData;

/* 初始化等待数据就绪的超时（us）。7.68kHz 下约 130us 一个样本，取 10ms 极宽裕 */
#define LSM6_DRDY_TIMEOUT_US    10000u

/* ±4000 dps 下的换算 */
#define LSM6_GYRO_DPS_PER_LSB   (140.0f / 1000.0f)      /* 0.140 dps/LSB */

/*==============================================================================
 * 接口
 *============================================================================*/

/* 初始化：软复位 -> 校验 WHO_AM_I -> 关 LPF1 -> 开 DRDY_MASK
 * -> 掉电状态下进 HAODR -> 写 HAODR_CFG -> DRDY 路由到 INT1。
 *
 * ★ 本函数【不启动转换】：器件保持掉电（ODR=0000），因此不产生 DRDY。
 *   启动转换必须用 LSM6_Enable()，且要在 DRDY 中断使能之后调用 ——
 *   否则第一个上升沿会丢，而锁存模式下丢了第一个边沿就再也没有第二个。
 * 返回 LSM6_OK 或负错误码。 */
int LSM6_Init(void);

/* 启动第一次转换：写 ODR 从掉电切到 HAODR 最高速，并等滤波器建立（5ms）。
 * 【必须在两个 DRDY 中断使能之后调用】原因见上。 */
int LSM6_Enable(void);

/* 配置回读核对：读回 CTRL1/CTRL2/HAODR_CFG/CTRL6/CTRL7/IF_CFG 并打印比对。
 * 用于判断 SPI 写是否真的生效 —— INT 恒低时这是唯一能定论的检查。
 * 返回 LSM6_OK 表示 ODR 位已非零（器件确实在产数据）。 */
int LSM6_VerifyConfig(void);

/* 等陀螺新样本就绪（轮询 STATUS_REG.GDA，bit1）。
 * DRDY 是状态位而非需清除的中断标志，读完高字节后自动回 0。 */
int LSM6_WaitGyroDrdy(void);

/* 一次性读回整个输出块：0x20 起 14 字节 = 温度2 + 陀螺6 + 加速度计6，
 * 单次片选完成。buf 需至少 LSM6DSV16X_BLOCK_LEN(14) 字节。 */
int LSM6_ReadBlock(uint8_t *buf);

/* 同上，但直接解析成结构体，省去调用方自己算字节偏移 */
int LSM6_ReadAll(LSM6_AllData *out);

/* 温度原始值换算成 °C（raw/256 + 25） */
float LSM6_TempToC(int16_t raw);

/* 读一个【新鲜】陀螺样本：轮询 STATUS_REG.GDA -> 连读 0x22 起 6 字节。
 * 返回 LSM6_OK 或负错误码。 */
int LSM6_ReadGyroRaw(LSM6_GyroRaw *out);

/* 同 LSM6_ReadGyroRaw，但换算成 dps 输出 */
int LSM6_ReadGyroDps(LSM6_GyroDps *out);

/* 读温度原始值（16 位补码） */
int LSM6_ReadTempRaw(int16_t *out);

/* 使能/关闭 INT1 上的陀螺数据就绪中断（PD11）。
 * 当前为轮询测试阶段，此接口备用。 */
int LSM6_Int1GyroDrdy(uint8_t enable);

/* 小批量突发：连续采 count 个样本缓存后统一 ASCII 上报（看波形细节用）*/
int LSM6_TestBurst(uint32_t count);

/* 大数据量压力测试：连续跑 seconds 秒，每秒上报一次统计量。
 * V3F 只有 64KB RAM（Link_v3f.ld: RAM = 64K - 256），无法缓存大数据量，
 * 所以边采边算，每帧只占几十字节，可跑任意长度。
 * 每秒输出五帧：
 *   RPT,<秒>,n=<样本数>,loss=<时间戳缺口us>,fail=,latch=
 *       loss = 本秒实际耗时 - 名义耗时(n/7680)。持续为正说明丢样本，
 *              持续为负说明在重复读同一份数据。
 *   D,  <秒>,n=<间隔数>,min,max,mean,sd,oob=   时间戳间隔均匀性
 *       标称 1/7680 = 130.2us；oob = 落在 [100,200]us 之外的数量
 *   G,  <秒>,minX,maxX,minY,maxY,minZ,maxZ,sat=   陀螺极值/饱和计数
 *   A,  <秒>,minX,maxX,minY,maxY,minZ,maxZ,sat=   加速度计极值/饱和计数
 *   T,  <秒>,min,max,tail                          温度极值/末值
 * 返回 LSM6_OK 或负错误码。 */
int LSM6_StressTest(uint32_t seconds);

#endif /* __LSM6DSV16X_H */
