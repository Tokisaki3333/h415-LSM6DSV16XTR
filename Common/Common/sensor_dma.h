/*==============================================================================
 * sensor_dma.h —— 两路传感器（MMC5983MA + LSM6DSV16X）的中断 + DMA 采集路径
 *
 * 【为什么这个头文件要跨越两个核】
 *   缓冲区布局与帧长度是【两个核必须完全一致】的约定：
 *     小核填 TX 缓冲、启动 DMA；
 *     大核在完成中断里按同一布局解析 RX 缓冲。
 *   任何一侧改长度而另一侧没跟上，都会静默解析出错位数据。
 *   所以布局只在本文件定义一次，两侧都 include 它。
 *
 * 【分工】
 *   小核 V3F（100MHz）—— 两个 DRDY 中断，短中断
 *     地磁 DRDY (PE1)  : 记 DRDY 时刻 -> 拉低 CS -> 启动 TX DMA -> 立即返回
 *     陀螺 DRDY (PD11) : 同上
 *     ★ ISR 内【不碰浮点】。RISC-V 下若 ISR 用到 FP 寄存器，编译器要在入口
 *       保存/恢复它们，80ns 的裸进出会变成几百 ns。陀螺 130us 周期里很要紧。
 *
 *   大核 V5F（400MHz）—— 两个 DMA 完成中断，负责收尾
 *     SPI1 RX 完成 (DMA1_Ch2, 地磁) : 解析 -> 发共享区 -> 抬 CS -> 重装 RX
 *     SPI2 RX 完成 (DMA1_Ch4, 陀螺) : 解析 -> 发共享区 -> 抬 CS -> 重装 RX
 *     ★ CS 抬起【不放在中断开头】，而是放在 ISR 即将退出的位置 ——
 *       用解析耗时当自然延时，不浪费任何显式等待（旧工程同样做法）。
 *
 * 【为什么 CS 必须由大核抬】
 *   小核启动 TX 后立即返回，无法知道 SPI 何时真正传完；只有 RX DMA 完成中断
 *   （即最后一个字节收到）才知道此刻可以释放片选。
 *
 * 【时间戳捆绑】
 *   DRDY 中断里读 GetTime64_10Ns() 存入共享变量，DMA 完成中断里取出，
 *   与该帧数据一起发布 —— 时间戳是"数据就绪时刻"，不是"读完时刻"。
 *
 * 【文件归属，注意别放错】
 *   sensor_dma.c    -> V3F 侧（初始化 + 两个 DRDY 处理体），放 Common/Common
 *   sensor_dma_rx.c -> V5F 侧（两个 DMA 完成中断），放 V5F/User/src
 *   两者共用一个头文件，故头文件放 Common/Common。
 *   ★ 不能把 DMA 完成中断也放进 sensor_dma.c：V5F 已有一份同名的
 *     DMA1_Channel2_IRQHandler（旧 SPI_rx.c），会重复定义。
 *============================================================================*/
#ifndef SENSOR_DMA_H
#define SENSOR_DMA_H

#include "ch32h417.h"

/*==============================================================================
 * 1. 通道参数 —— 两个核必须一致
 *============================================================================*/

/* --- 地磁 MMC5983MA @ SPI1，CS=PE0，DRDY=INT=PE1 ---
 * 寄存器 0x00..0x06 连读 7 字节 —— 【是 7 不是 6】。
 * 18 位数据不是每轴独占 3 字节，而是三轴共用中间与末尾字节：
 *   X = [0]<<10 | [1]<<2 | [2]>>6
 *   Y = [2]<<10 | [3]<<2 | [4]>>6      <- [2] 被 X 和 Y 共用
 *   Z = [4]<<10 | [5]<<2 | [6]>>6      <- [4] 被 Y 和 Z 共用，[6] 是三轴共用低位
 * 少读一个字节会同时丢掉三轴低位并造成数组越界。
 * 解析直接用 mmc5983ma_reg.h 的 MMC5983MA_XYZ_X/Y/Z 宏，不要手写位运算。
 *
 * 【INT 引脚行为，手册实证 —— 与旧注释相反，务必按这个来】
 *   - 是【电平锁存】不是脉冲。手册 Status bit0 原文：
 *       "When the measurement is finished, this bit will remain '1' till next measurement."
 *     旧代码注释里的"约 826us 窄脉冲"是错的 —— "826" 在手册全文出现 0 次。
 *   - 极性【固定高有效】，四个 CTRL 寄存器里没有任何极性配置位。
 *   - 使能前是 hi-Z，故必须有上拉（板上 PE1 是上拉输入，符合）。
 *   - 清除方式【只有写 1 到 Status 对应位】，原文：
 *       "Writing 1 into this bit will clear the corresponding interrupt."
 *     ★ 只读 Status 或只读数据寄存器都【不会】清中断。不清就会反复进中断。 */
#define SENSOR_MMC_DATA_LEN     7u
#define SENSOR_MMC_FRAME_LEN    (SENSOR_MMC_DATA_LEN + 1u)      /* +1 = 命令字节 */
#define SENSOR_MMC_READ_CMD     ((uint8_t)((0x00u & 0x7Fu) | 0x80u))    /* 0x00 起读 */

/* --- 陀螺 LSM6DSV16X @ SPI2，CS=PD10，DRDY=PD11 ---
 * 输出块 0x20~0x2D 共 14 字节：温度2 + 陀螺6 + 加速度计6。 */
#define SENSOR_IMU_DATA_LEN     14u
#define SENSOR_IMU_FRAME_LEN    (SENSOR_IMU_DATA_LEN + 1u)      /* +1 = 命令字节 */
#define SENSOR_IMU_READ_CMD     ((uint8_t)((0x20u & 0x7Fu) | 0x80u))    /* 0x20 起读 */

/* 单个 RX 缓冲的最大长度（供 DMA 缓冲声明用）*/
#define SENSOR_RX_BUF_LEN       SENSOR_IMU_FRAME_LEN

/*==============================================================================
 * 2. DMA 缓冲 —— 定义在共享内存里，两个核访问同一份
 *
 * 【为什么必须共享而不是各自的 static】
 *   小核启动 DMA，数据落进缓冲；大核在完成中断里读同一个缓冲。
 *   TX 缓冲由小核填、DMA 读，也要在同一块可被 DMA 访问的内存里。
 *
 *   storage 由【小核】定义（sensor_dma.c），大核只以 extern 引用 ——
 *   与 g_shm 的做法一致：小核定义，大核经 IPC 拿到基址后映射同一地址。
 *   aligned(32) 是 cache line 对齐，避免两核各自的 cache 争用同一行。
 *============================================================================*/
extern uint8_t g_mmc_tx[SENSOR_MMC_FRAME_LEN] __attribute__((aligned(32)));
extern uint8_t g_mmc_rx[SENSOR_MMC_FRAME_LEN] __attribute__((aligned(32)));
extern uint8_t g_imu_tx[SENSOR_IMU_FRAME_LEN] __attribute__((aligned(32)));
extern uint8_t g_imu_rx[SENSOR_IMU_FRAME_LEN] __attribute__((aligned(32)));

/* DRDY 时刻（10ns 单位）：DRDY 中断（小核）写，DMA 完成中断（大核）读。
 * 单独用变量而不是放进通道结构，是因为要在 DMA 完成中断里与数据一起发布，
 * 而不是等消费者来取。 */
/* g_mmc_drdy_ts / g_imu_drdy_ts 两个时间戳的声明都已移到 board_io.h
 * （两个 DRDY 的 ISR 都在那个模块里），避免重复声明。
 * 定义仍在本文件 —— 这里只放 extern 的话，ISR 那边看不到。 */

/*------------------------------------------------------------------------------
 * 计时定位用时刻（全部为 10ns 单位，由小核写、两核都可读）
 *
 * 这是一条完整链条上的三个点，任何一环断开都能靠差值一眼定位：
 *   g_*_enable_ts     启动转换（写 CMM_EN / ODR）的时刻
 *   g_*_first_isr_ts  首次进入 DRDY 中断的时刻
 *   大核 DMA-RX 中断   在 sensor_dma_rx.c 里打印
 *
 * 判读：first_isr 一直为 0 -> 器件没产数据或 EXTI 没收到；
 *       first_isr 有而大核没进  -> DMA 没完成（TX 没发 / RX 没配好）
 *----------------------------------------------------------------------------*/
extern volatile uint64_t g_mmc_enable_ts;
extern volatile uint64_t g_imu_enable_ts;
extern volatile uint64_t g_mmc_first_isr_ts;
extern volatile uint64_t g_imu_first_isr_ts;

/*==============================================================================
 * 3. 小核 V3F 接口
 *============================================================================*/

/* 初始化：DMA 通道 + Mux 绑定 + 缓冲填充 + 中断分配。
 * 必须在 SPI 与两个器件都初始化之后调用。 */
void SENSOR_DMA_Init(void);

/* 使能 DRDY 的 EXTI 中断。
 *
 * ★ 必须在"清掉 INT 锁存、引脚回到低电平"之后调用：
 *   MMC5983MA 的 INT 是【高有效 + 电平锁存】，而 EXTI 配的是【上升沿】。
 *   若调用时引脚已经是高（上电后有过一次测量完成就会如此），
 *   就永远等不到 0→1 边沿，中断一次都不会进。
 * 正确顺序：Init（不测量）-> ClearLatch -> ArmDrdy -> Enable（启动转换）。 */
void SENSOR_DMA_ArmDrdy(void);

/* 小核 DRDY 中断处理体 —— 由 board_io.c 的 EXTI15_8 ISR 按线调用。
 * 短中断：不碰浮点、不做多余调用。 */
void SENSOR_MMC_DrdyIsr(void);      /* 地磁 DRDY (PE1)  */
void SENSOR_IMU_DrdyIsr(void);      /* 陀螺 DRDY (PD11) */

/* 统计：两路各自的帧计数（即 DRDY 触发次数）*/
void SENSOR_DMA_Stats(uint32_t *mmc_cnt, uint32_t *imu_cnt);

/* 通路诊断（核心排障手段）：把 EXTI/NVIC/DMA/SPI 的配置寄存器读回来，
 * 并在 200ms 窗口内统计两条 DRDY 的上升沿数与中断实际进入次数。
 *
 * 判读表：
 *   armed=0            -> SENSOR_DMA_Init 没走完，处理体被闸住
 *   edges>0 且 isr=0   -> 引脚有信号但中断没进（EXTI 触发沿 / NVIC 分配）
 *   edges=0            -> 器件侧没拉引脚（DRDY 路由 / 器件没使能）
 * 阻塞约 200ms，只在排障时调用。 */
void SENSOR_DMA_Diag(void);

#endif /* SENSOR_DMA_H */
