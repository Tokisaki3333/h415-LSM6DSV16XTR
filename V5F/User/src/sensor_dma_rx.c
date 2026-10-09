/*==============================================================================
 * sensor_dma_rx.c —— V5F（大核）侧：两个 DMA 完成中断
 *
 * 【本文件与 sensor_dma.c 的分工】
 *   sensor_dma.c（Common/Common，V3F 编）  : 初始化 + 两个 DRDY 处理体
 *   本文件      （V5F/User/src，V5F 编）   : 两个 DMA 完成中断
 *   两者共用 Common/Common/sensor_dma.h 里的缓冲布局与帧长度约定。
 *
 * 【为什么必须拆成两个文件】
 *   V5F 的构建清单里没有 sensor_spi.c，而 sensor_dma.c 依赖它；
 *   而且 V5F 已有一份同名的 DMA1_Channel2_IRQHandler（旧 SPI_rx.c），
 *   不拆就会重复定义。
 *   ★ 因此旧的 SPI_rx.c 必须从 V5F 构建里排除（MRS 的 excludeResources），
 *     或按本文件重写。两者的 DMA1_Channel2_IRQHandler 是同一个向量。
 *
 * 【中断处理顺序，每一步都有理由】
 *   ① 清 DMA 挂起位
 *   ② 解析 RX 缓冲
 *   ③ 发共享区（数据 + DRDY 时刻）
 *   ④ 清器件中断（仅地磁需要）
 *   ⑤ 抬 CS        <- 放在即将退出处，用前面几步的耗时当自然延时
 *   ⑥ 重装 RX 通道 <- 必须在器件中断清掉之后
 *
 * 【为什么 CS 抬在最后而不是最前】
 *   CS 上升沿必须晚于最后一个时钟（LSM6DSV16X 要求 t_h(CS)，mode0 下 20ns）。
 *   把抬 CS 放在解析之后，解析天然消耗的微秒正好满足，不需要显式等待。
 *   旧工程（SPI_rx.c）也是这个位置。
 *============================================================================*/
#include "sensor_dma.h"
#include "board_io.h"
#include "mipc_shm.h"
#include "sensor_spi.h"
#include "mmc5983ma_reg.h"
#include "sys_clk.h"        /* GetTime64_10Ns：本文件要打印计时定位信息 */

/* 首次进入 DMA-RX 中断只打印一次，避免 8kHz/1kHz 刷屏 */
static volatile uint8_t s_dma_rx_seen = 0u;

/*==============================================================================
 * 1. SPI1 RX 完成 —— 地磁 MMC5983MA（DMA1_Channel2）
 *
 * 与陀螺那路的区别：地磁的 INT 是【电平锁存且只能靠写 1 清除】，
 * 所以这里必须多一步"清器件中断"，否则 INT 一直高、EXTI 反复触发。
 *============================================================================*/
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1, DMA1_IT_TC2) != RESET)
    {
        DMA_ClearITPendingBit(DMA1, DMA1_IT_TC2);

        /*--------------------------------------------------------------------------
         * ★ 计时定位：大核进入 DMA-RX 中断的时刻
         *
         * 与另外两个时刻构成完整链条（都是 10ns 计数）：
         *   g_mmc_enable_ts    小核启动转换
         *   g_mmc_first_isr_ts 小核首次进 DRDY 中断
         *   dma_rx             大核首次进本中断（本行）
         *
         * 本行是否出现就是"通路是否成立"的判据：
         *   出现   -> DMA 真的完成了，整条路径成立
         *   不出现 -> DMA 没完成（TX 没发出 / RX 没配好 / 通道没使能）
         *------------------------------------------------------------------------*/
        if (s_dma_rx_seen == 0u)
        {
            uint64_t t = GetTime64_10Ns();
            s_dma_rx_seen = 1u;
            printf("TIMING,dma_rx,now_us=%u,enable_us=%u,first_isr_us=%u,"
                   "dly_en2rx_us=%d,dly_isr2rx_us=%d\r\n",
                   (unsigned)(t / 100ULL),
                   (unsigned)(g_mmc_enable_ts / 100ULL),
                   (unsigned)(g_mmc_first_isr_ts / 100ULL),
                   (int)((g_mmc_enable_ts != 0u)
                             ? (int32_t)((t - g_mmc_enable_ts) / 100ULL) : -1),
                   (int)((g_mmc_first_isr_ts != 0u)
                             ? (int32_t)((t - g_mmc_first_isr_ts) / 100ULL) : -1));
        }

        /* --- 解析 ---
         * MMC5983MA 的 18 位数据是【三轴共用字节】的 7 字节结构：
         *   X = [0]<<10 | [1]<<2 | [2]>>6
         *   Y = [2]<<10 | [3]<<2 | [4]>>6
         *   Z = [4]<<10 | [5]<<2 | [6]>>6
         * 这里直接用 mmc5983ma_reg.h 的宏，不手写位运算 —— 手写极易把共用字节
         * 搞错（先前就写成"每轴 3 字节"，少读 1 字节并越界访问到 g_mmc_rx[9]）。
         * 零点 131072（=2^17），16384 counts/G。
         * 只搬原始值，换算与椭球标定放消费端 —— 中断里省时间，且标定参数未定。 */
        if (g_shm)
        {
            /* +1 是因为 g_mmc_rx[0] 是命令字节，数据从 [1] 起 */
            int32_t x = (int32_t)MMC5983MA_XYZ_X(g_mmc_rx[1], g_mmc_rx[2], g_mmc_rx[7]);
            int32_t y = (int32_t)MMC5983MA_XYZ_Y(g_mmc_rx[3], g_mmc_rx[4], g_mmc_rx[7]);
            int32_t z = (int32_t)MMC5983MA_XYZ_Z(g_mmc_rx[5], g_mmc_rx[6], g_mmc_rx[7]);

            shm_publish_mmc(g_mmc_drdy_ts, x, y, z);
        }

        /* --- 清器件中断（必须，且只能是"写 1"）---
         * INT 是高有效【电平锁存】，只读 Status 或只读数据寄存器都不会解断言。
         * 不清 -> EXTI 立刻再触发 -> 反复进中断 -> 系统被淹没。
         *
         * 写 0x05 = 清 Meas_M_Done(bit0) + Meas_T_Done(bit2)。
         * 是【写常量】不是读-改-写；Status(0x08) 可读可写。
         *
         * 此刻 CS 仍低（DRDY 中断里拉低后一直保持），所以"读数据"与"清中断"
         * 在同一次片选内完成。这里显式传 (MMC_SPI, 0, 0) ——
         * cs_pin=0 让 SPI_WriteReg 不做片选动作（片选已在低）。
         *
         * 注意：不能用 mmc5983.c 里的 MMC_SPI_HANDLE()，那是该文件的私有宏，
         * 本文件（V5F 工程）看不到。直接用 board_io.h 的公开宏 MMC_SPI。 */
        (void)SPI_WriteReg(MMC_SPI, 0, 0u, MMC5983MA_REG_STATUS,
                           (uint8_t)(MMC5983MA_ST_MEAS_M_DONE |
                                     MMC5983MA_ST_MEAS_T_DONE));

        /* --- 抬 CS（放在即将退出处，前面几步的耗时就当延时）--- */
        MMC_CS_HIGH();

        /* --- 重装 RX 通道，准备下一帧 ---
         * 必须在清中断之后：否则 RX 一使能就可能立刻完成，
         * 而在中断未清时再触发一次，造成重复处理。 */
        DMA1_Channel2->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
        DMA1_Channel2->CNTR  = SENSOR_MMC_FRAME_LEN;
        DMA1_Channel2->CFGR |= DMA_CFGR1_EN;
    }
}

/*==============================================================================
 * 2. SPI2 RX 完成 —— 陀螺 LSM6DSV16X（DMA1_Channel4）
 *
 * 优先级低于地磁（见 sensor_dma.c 的 NVIC 配置）：地磁是短中断，
 * 陀螺会实打实侵入它数十 us，所以让地磁压陀螺一头。
 *============================================================================*/
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1, DMA1_IT_TC4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1, DMA1_IT_TC4);

        if (g_shm)
        {
            /* RX 缓冲布局：rx[0] = 发命令时收到的无用字节，
             *              rx[1..14] = 输出块 0x20~0x2D
             * LSM6DSV16X 输出寄存器是【低字节在前】的 16 位补码：
             *   rx[1..2]   温度      rx[3..4]  陀螺 X    rx[5..6]  陀螺 Y
             *   rx[7..8]   陀螺 Z    rx[9..10] 加速度 X  rx[11..12] 加速度 Y
             *   rx[13..14] 加速度 Z */
            int16_t tp = (int16_t)(((uint16_t)g_imu_rx[2]  << 8) | g_imu_rx[1]);
            int16_t gx = (int16_t)(((uint16_t)g_imu_rx[4]  << 8) | g_imu_rx[3]);
            int16_t gy = (int16_t)(((uint16_t)g_imu_rx[6]  << 8) | g_imu_rx[5]);
            int16_t gz = (int16_t)(((uint16_t)g_imu_rx[8]  << 8) | g_imu_rx[7]);
            int16_t ax = (int16_t)(((uint16_t)g_imu_rx[10] << 8) | g_imu_rx[9]);
            int16_t ay = (int16_t)(((uint16_t)g_imu_rx[12] << 8) | g_imu_rx[11]);
            int16_t az = (int16_t)(((uint16_t)g_imu_rx[14] << 8) | g_imu_rx[13]);

            shm_publish_imu(g_imu_drdy_ts, tp, gx, gy, gz, ax, ay, az);

            /* --- 主运动解析的挂载点 ---
             * 会长成"任务抢占调度"的那部分（旧工程 proc_gyro_bias /
             * proc_static_detect / proc_attitude / ... / proc_ekf_gate
             * 八步链的等价物）。当前不实现，只留这个明确位置。
             *
             * 注意：这里在 130us 周期内运行，放长任务时必须保证可被
             * 地磁 DMA 中断（更高抢占级）打断 —— 这正是"长中断本质是
             * 任务抢占调度"的含义。 */
        }

        /* --- 抬 CS（放在即将退出处）--- */
        IMU_CS_HIGH();

        /* --- 重装 RX 通道，准备下一帧 --- */
        DMA1_Channel4->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
        DMA1_Channel4->CNTR  = SENSOR_IMU_FRAME_LEN;
        DMA1_Channel4->CFGR |= DMA_CFGR1_EN;
    }
}
