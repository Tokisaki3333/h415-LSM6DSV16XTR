/*******************************************************************************
 * File Name        : mmc5983.h
 * Description      : MMC5983MA 三轴磁传感器驱动（SPI1，片选 PE0）
 *
 * 寄存器定义见 mmc5983ma_reg.h —— 本文件只提供接口。
 *
 * 【已实测确认的器件行为】（全部由本工程实测得出，不要在别处再猜）
 *
 *   1) 连续模式 CM_Freq=111 + BW=11 下，芯片自身每约 826us 产出一个新样本
 *      （实测 1206 Hz）。速率由芯片决定，CPU 不需要做 1ms 定时对齐。
 *
 *   2) Status(0x08) 的 Meas_M_Done(bit0) 是【窄脉冲】：每约 826us 置起一次，
 *      宽度约一个采样周期。必须在紧贴的循环里轮询才能抓到。
 *
 *   3) 读走数据寄存器【不会】清 Meas_M_Done / INT 引脚。
 *      必须写 1 到 Status(0x08) 的对应位才会清（同时清 INT 引脚）。
 *
 *   4) INT 引脚（PE1）是【锁存】的，置起后不会自己回落（2000 次采样里只变化
 *      过 1 次）。因此【不能】用"等 INT 引脚变高"判断新数据，只能用寄存器判据。
 *
 *   5) 上电后芯片需要先读内部 OTP，期间读回 0xFF（SDO 高阻）。
 *      就绪标志是 Status(0x08) 的 OTP_Read_Done(bit4)。
 *
 *   6) SET/RESET 翻转有效：两个极性的读数差约 9000 counts（0.55G），
 *      而单点噪声仅约 6~7 counts。
 *
 *   7) 翻转后的瞬态：时间常数约 275us，约 800us 后衰减到噪声水平。
 *      两个方向对称。
 *
 *   8) CTRL0/CTRL2 是【只写】寄存器，禁止读-改-写（见 mmc5983ma_reg.h）。
 *      Set/Reset 触发后不要读回判断完成，用固定时延。
 *******************************************************************************/

#ifndef __MMC5983_H
#define __MMC5983_H

#include <stdint.h>

#include "ch32h417.h"

/*==============================================================================
 * 常量
 *============================================================================*/

/* SET/RESET 触发后等待操作结束（实测操作约 13us，取 100us 留足余量）。
 * 不读回 CTRL0 判断 —— 它是只写寄存器，读回值不可信。 */
#define MMC_PULSE_DELAY_US      100u

/* SET/RESET 脉冲之后、触发测量之前的等待（us）。
 *
 * 手册 P15 对 CTRL0.Set / CTRL0.Reset 的说明：
 *   "Writing 1 will cause the chip to do the Set/Reset operation, which will
 *    allow large set/reset current to flow through the sensor coils for 500ns.
 *    This bit is self-cleared at the end of Set/Reset operation."
 * 即：脉冲只需 500ns，且该位【在操作结束时自清】，手册没有额外等待要求。
 * （同页 OTP Read 位同样是自清位，也只说"刷新完自动归零"。）
 *
 * 所以这里按手册 500ns 加 4 倍余量取 2us。
 * 归档 mmc_pulse 里的 MMC_PULSE_DELAY_US=100us 是保守值 ——
 * 它放在中断里会形成 100us 的忙等，把陀螺（7680Hz，周期 130us）拖后，
 * 是实时性的最大负担，故不再使用。 */
#define MMC_RST_PULSE_WAIT_US   2u

/* 自动周期 SET 的间隔档位（CR2 bit6:4 Prd_set）。数值与手册表一致：
 * 每 N 次测量做一次 SET，N = 1/25/75/100/250/500/1000/2000。
 * 每 1 次测量对应编码 000，所以 MMC_PRD_SET_1 就是 0x00。 */
#define MMC_PRD_SET_1       0x00u
#define MMC_PRD_SET_100     0x30u
#define MMC_PRD_SET_1000    0x60u
#define MMC_PRD_SET_EN      0x80u   /* CR2 bit7 En_prd_set：周期 SET 使能 */

/* 等待新鲜样本的超时（us）。芯片约 826us 出一个样本，10ms 足够宽裕。 */
#define MMC_READ_TIMEOUT_US     10000u

/* 零偏指数滤波权重（归档机制的 MMC_OFF_WEIGHT）。
 *
 * 每完成一次跨周期配对 offset = (上一周期RESET + 本周期首个SET)/2，
 * 就按下面的公式更新零偏：
 *     one_over_w = 1 / MMC_OFF_WEIGHT
 *     keep       = 1 - one_over_w
 *     off        = keep * off + one_over_w * pair
 * 即等效于窗口长度 8 的滑动平均。
 *
 * 放在头文件是因为消费端（V3F main / V5F 解析）也要用同一个权重；
 * 原先是 mmc5983.c 的私有宏，消费端看不到。 */
#define MMC_OFF_WEIGHT          8u


/*==============================================================================
 * 返回值
 *============================================================================*/
#define MMC_OK                  0
#define MMC_ERR_PARAM          (-1)
#define MMC_ERR_NO_ID          (-2)     /* Product ID 不符 */
#define MMC_ERR_OTP_TIMEOUT    (-3)     /* 等 OTP 就绪超时 */
#define MMC_ERR_SPI            (-4)
#define MMC_ERR_DRDY_TIMEOUT   (-5)     /* 等 Meas_M_Done 超时 */

/*==============================================================================
 * 数据结构
 *
 *   原始值为 18bit 无符号，每轴标度 16384 counts/G，器件理想零点
 *   131072 counts（手册 Null Field Output 允许 ±0.5G 偏差）。
 *============================================================================*/
typedef struct
{
    uint32_t x;             /* 18bit 无符号原始值 */
    uint32_t y;
    uint32_t z;
} MMC_RawXYZ;

/* 三轴磁场，单位 G */
typedef struct
{
    float x;
    float y;
    float z;
} MMC_Field;

/*==============================================================================
 * 接口
 *============================================================================*/

/* 初始化：等 OTP 就绪 -> 校验 Product ID -> 配置 CTRL0/1/2/3。
 *
 * ★ 本函数【不启动测量】（CMM_EN=0），因此不会产生 INT。
 *   启动连续转换必须用 MMC_Enable()，且要在 DRDY 中断使能之后调用 ——
 *   否则第一个测量完成产生的 INT 上升沿会丢，而 INT 是【锁存】的，
 *   丢了第一个边沿就再也没有第二个。
 * 返回 MMC_OK 或负错误码。 */
int MMC_Init(void);

/* 启动连续转换（置 CTRL2.CMM_EN）。
 * 【必须在 DRDY 中断使能之后调用】原因见上。 */
int MMC_Enable(void);

/* 清除 INT 锁存，让 PE1 回到低电平。
 *
 * 【为什么必须调用，且必须在使能 EXTI 之前】
 * INT 是"高有效 + 电平锁存"的：上电后只要有过一次测量完成就一直为高。
 * 而 EXTI 配的是上升沿触发 —— 引脚已经是高，就再没有 0→1 边沿，
 * 中断永远不会进（实测现象就是"PE1 恒高 + 中断计数为 0"）。
 * 所以必须先清锁存让引脚回低，之后第一次测量完成才有真正的上升沿。
 * 本函数会打印清完之后的引脚电平，用于确认它确实回到 0。 */
int MMC_ClearLatch(void);

/* 读一个【新鲜】样本：轮询 Meas_M_Done -> 连读 7 字节 -> 写 1 清标志。
 * 返回 MMC_OK 或负错误码。 */
int MMC_ReadXYZ(MMC_RawXYZ *out);

/* ---- 单次触发模式（TM_M）----
 * MMC_OneShotConfig 关掉连续转换；之后每次 MMC_TriggerAndRead 触发一次测量
 * 并读出结果。MMC_ContinuousConfig 恢复连续 1000Hz。
 * 单次触发的好处是转换一定发生在置极性之后，不受连续模式自由节拍的影响。 */
int MMC_OneShotConfig(void);
int MMC_ContinuousConfig(void);
int MMC_TriggerAndRead(MMC_RawXYZ *out);

/* 消费在途转换：切到单次触发后芯片可能还有一次在途转换，其转换窗口早于
 * RESET。若不消费掉，下一次 MMC_TriggerAndRead 的轮询会立刻命中它的
 * Meas_M_Done 标志，读到旧极性的数据（实测恒定偏低约 6300 counts）。
 * 轮询窗口 MMC_CONSUME_TIMEOUT_US 不小于一个完整转换周期，因此"没有消费到"
 * 本身就是"确实没有在途转换"的证据，也就不必关心 RESET 落在周期的哪个相位。
 * out 可为 NULL。 */
int MMC_ConsumePending(MMC_RawXYZ *out);

/* 消费在途转换的统计（累计值）：
 *   s_consume_hit    真的消费到（确实有在途转换）
 *   s_consume_miss   窗口内始终无标志（确实没有在途转换，也是干净的）
 *   s_consume_us_max 命中时等待在途转换完成的最长耗时(us)，用于评估窗口余量 */
extern uint32_t s_consume_hit;
extern uint32_t s_consume_miss;
extern uint32_t s_consume_us_max;

/* ---- 自动 SET（正常工作用这个，CPU 不参与日常翻转）----
 * prd_set 取上面的 MMC_PRD_SET_* 档位宏。 */
int MMC_AutoSetOn(uint8_t prd_set);
int MMC_AutoSetOff(void);

/* 手动触发一次 SET / RESET。两者是同一操作的两个方向：线圈通入方向相反的
 * 大电流脉冲，把 AMR 薄膜的磁化矢量对齐到相反方向，从而翻转感知极性。
 * 内部会先关自动 SET、做完再恢复，避免与芯片自动 SET 冲突。
 * 返回固定时延（us），不表示真实操作耗时。 */
uint32_t MMC_Set(void);
uint32_t MMC_Reset(void);

/* ---- 主量测入口 ----
 * 先做开机标定：滑动窗口（MMC_CAL_WIN 对）判定收敛，并检查零偏落在合理范围。
 * 然后跑 ngroups 组，每组 MMC_GROUP_CYC 个周期；每组开始前把运行期零偏重置
 * 为标定值，因此各组的组末零偏可直接横向比较。组末打印 GROUP 标记与消费
 * 统计并静默 MMC_GROUP_IDLE_MS 毫秒，全部跑完后返回，末尾给出 EVAL 评估。
 * 每个 SET 极性样本输出一行 ASCII（S,组,序号,X,Y,Z,门控余量）。
 * 返回负错误码表示标定失败或采集异常。 */
int MMC_StreamJustFloat(uint32_t b, uint32_t ngroups);

/* ---- 长连续测量实验 ----
 * 与 MMC_StreamJustFloat 共用同一个测量周期实现，但不分组、不静默，连续跑
 * max_cycles 个周期后返回。每个 SET 样本发一帧 4 通道 justfloat：
 *   ch0 = X(G)  ch1 = Y(G)  ch2 = Z(G)  ch3 = offset_X(G)
 * 帧长 24 字节（双同步头 8 + 4 个 float），比 ASCII 省约 30% 带宽。
 * 每 status_every 个周期插入一行 ASCII 状态（justfloat 上位机会跳过）：
 *   零偏值、零偏极差 span、周期耗时最大值、看门狗超时次数。
 * 看门狗门限 MMC_WDOG_US；超时说明时序被破坏，会累计计数。
 * 先做一次开机标定，失败返回负错误码。 */
int MMC_LongRun(uint32_t b, uint32_t max_cycles, uint32_t status_every);

#endif /* __MMC5983_H */
