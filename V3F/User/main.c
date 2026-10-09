/*******************************************************************************
* File Name          : main.c
* Description        : 最小实验 —— 只验证"能否进入 EXTI 中断"
*
* 【本版本的唯一目的】
* 之前的版本把太多东西叠在一起（DMA + 时间戳 + 双核 + 共享区 + 锁存时序），
* 任何一环出错都表现为"中断计数为 0"，无法定位。
*
* 现在只留一条路径：
*     MMC5983MA 连续测量 -> 测量完成拉高 PE1 -> EXTI 上升沿 -> 进中断
*
* 【故意去掉的东西】
*   - 不装 DMA        （不碰 DMA1_Channel2/3、不碰 Mux）
*   - 不记时间戳      （不调 GetTime64_10Ns）
*   - 不用共享区      （不读不写 g_shm）
*   - 不等 V5F 握手   （V5F 与本次实验完全无关）
*   - 中断里只做两件事：清 EXTI 挂起位 + 计数
*
* 【判据】
*   计数 > 0  -> EXTI 通路成立，问题在 DMA 侧
*   计数 = 0  -> EXTI 通路不成立，问题在 EXTI/GPIO/器件侧
*******************************************************************************/

/*==============================================================================
 * 本板使用的外设开关 —— ★ 必须放在【所有 include 之前】
 *
 * board_io.h 里这些宏都有 #ifndef 保护，但 board_io.h 会被 debug.h /
 * hardware.h 间接提前包含。若把定义放在 include 之后，那时 #ifndef 已经
 * 生效、默认值已经定下，于是产生 "redefined" 警告且覆盖不掉默认值。
 * 放到最前面才能保证先生效。
 *============================================================================*/
#define BOARD_EN_SDMMC          0
#define BOARD_EN_SPI1_MMC       1       /* 地磁 MMC5983MA @ SPI1，CS=PE0，DRDY=PE1 */
#define BOARD_EN_SPI2_IMU       0       /* 本实验不用陀螺 */
#define BOARD_EN_I2C_BMP388     0       /* 本实验不用气压计 */
#define BOARD_EN_USART2_GNSS    0       /* 本实验不用 GPS */
#define BOARD_EN_CAN1           0
#define BOARD_EN_EXTI           1       /* 地磁 DRDY 需要 EXTI */

#include "debug.h"
#include "hardware.h"
#include "board_io.h"
#include "sys_clk.h"
#include "sensor_spi.h"
#include "mmc5983.h"
#include "mmc5983ma_reg.h"
#include "lsm6dsv16x.h"      /* LSM6 六轴陀螺：初始化与整块读 */
#include "ch32h417.h"

int main(void)
{
    int rc;

    SystemInit();
    SystemAndCoreClockUpdate();
    Delay_Init();
    SYS_CLK_Init();
    USART_Printf_Init(921600);

    printf("\r\n\r\nREBOOTED,minimal-exti-test\r\n");

    /* SPI1 + GPIO + EXTI 线映射（不装 DMA）*/
    BOARD_Init();

    printf("BOARD,init_done\r\n");

    /*--------------------------------------------------------------------------
     * 1. 配置地磁器件（此时【不】启动测量）
     *------------------------------------------------------------------------*/
    rc = MMC_Init();
    printf("MMC,init_rc=%d\r\n", rc);

    /*--------------------------------------------------------------------------
     * 1b. 配置陀螺 LSM6DSV16X（配置 + 启动转换）
     *
     * ★ 必须放在 EXTI 使能【之前】：
     *   LSM6_Enable() 内部有一段固定次数的 CPU 忙等（约 5~10ms），
     *   用来等 HAODR 滤波器建立。若那时地磁 EXTI 已经使能，
     *   这 5~10ms 会把地磁的节拍整个堵住。
     *
     * ★ LSM6_Init() 只做配置、不启动转换；启动必须单独用 LSM6_Enable()，
     *   且要在 DRDY 中断使能之后 —— 但这里的 EXTI 使能紧随其后，
     *   中间只隔几条指令，第一次 DRDY 边沿不会丢
     *   （器件建立需要约 1.86ms，远晚于 EXTI 使能完成）。
     *------------------------------------------------------------------------*/
    rc = LSM6_Init();
    printf("LSM6,init_rc=%d\r\n", rc);

    rc = LSM6_Enable();
    printf("LSM6,enable_rc=%d\r\n", rc);

    /*--------------------------------------------------------------------------
     * 2. 中断归核与抢占优先级
     *
     * ★ 分组归属（按 EXTI 线号，与引脚端口无关）：
     *     EXTI_Line1  = PE1  -> EXTI7_0_IRQn  (IRQn=36)  地磁 DRDY
     *     EXTI_Line11 = PD11 -> EXTI15_8_IRQn (IRQn=33)  陀螺 INT1
     *     EXTI_Line10 = PB10 -> EXTI15_8_IRQn (IRQn=33)  复位键
     *   两个 IRQn 都 > 31，NVIC_SetAllocateIRQ 有效，必须显式分给 V3F。
     *
     * ★ 抢占级：陀螺(EXTI15_8) = 0 最高，地磁(EXTI7_0) = 1。
     *
     * 【为什么陀螺必须高于地磁】
     *   地磁 ISR 现在做完整一帧：读 7 字节 + 清锁存 + 极性处理 + 触发下一拍，
     *   最长约 640us（含 RESET 脉冲等待 100us）。
     *   陀螺周期只有 130us（7680 Hz）。若地磁优先级更高，陀螺会被堵住
     *   4 个周期 -> 直接丢帧。
     *   反向影响可忽略：地磁周期 1000us，被抢断几十 us 不影响自身节拍。
     *
     * 优先级编码：V3F 用 2 位抢占级（4 档），写 IPRIOR 时左移到 bit7:6。
     *------------------------------------------------------------------------*/
    NVIC_SetAllocateIRQ(EXTI15_8_IRQn, Core_ID_V3F);   /* 陀螺 INT1 + 复位键 */
    NVIC_SetPriority(EXTI15_8_IRQn, (uint8_t)(0u << 6));
    NVIC_EnableIRQ(EXTI15_8_IRQn);

    NVIC_SetAllocateIRQ(EXTI7_0_IRQn, Core_ID_V3F);    /* 地磁 DRDY（线1）*/
    NVIC_SetPriority(EXTI7_0_IRQn, (uint8_t)(1u << 6));
    NVIC_EnableIRQ(EXTI7_0_IRQn);

    printf("ARM,exti7_0_and_15_8_enabled,pe1=%u\r\n", (unsigned)MMC_DRDY_READ());

    /*--------------------------------------------------------------------------
     * 3. 清 INT 锁存，让 PE1 回低
     *------------------------------------------------------------------------*/
    (void)MMC_ClearLatch();
    printf("CLR,pin=%u\r\n", (unsigned)MMC_DRDY_READ());

    /*==========================================================================
     * 采样时序 —— 严格照归档 mmc_cycle（_newfunc.c 第 28-80 行）
     *
     *   1) for k in 0..b-1: MMC_ReadXYZ()   连续模式 + 自动 SET，读 b 个 SET 样本
     *   2) mmc_cfg_ex(0,0,0)                切单次触发（关连续、关自动 SET）
     *   3) mmc_pulse(CTRL0_RESET)           发 RESET 脉冲
     *   4) MMC_ConsumePending(0)            ★ 消费在途转换（窗口 1400us）
     *   5) MMC_TriggerAndRead()             触发并读真正的 RESET 样本
     *   6) mmc_auto_set_every()             恢复自动 SET，进下一周期
     *   7) 配对：本周期 RESET + 下一周期首个 SET（约 623us，最短窗口）
     *
     * 【本工程 b=1】SET 段只取一个样本，随后切 RESET 段。
     *   这一整套动作在 EXTI7_0_IRQHandler 里按极性状态机执行：
     *     读到 SET 样本  -> 切单次 -> 发 RESET -> ConsumePending -> 触发 RESET 拍
     *     读到 RESET 样本 -> 暂存供下周期配对 -> 恢复自动 SET -> 触发 SET 拍
     *
     * 【中断内纪律】不碰浮点、不 printf（理由见 board_io.c）。
     *   上报由主循环每秒做一次。
     *==========================================================================*/
    printf("RUNNING,isr_mmc_cycle\r\n");

    /* 第一拍用 SET 极性：走归档实测跑通的 MMC_Enable()。
     *
     * ★ 不要手写 CTRL0/CTRL2 —— 我手写过一次，写的是
     *     CTRL2 = CMM_EN | CM_FREQ_1000HZ          = 0x0F
     *   漏了 PRD_SET_EN(0x80)，实测【器件完全不测量、引脚零翻转】
     *   （这个坑 mmc5983.c 的 MMC_Enable 注释里就记着）。
     *   归档跑通的那一套是 auto_set=1：
     *     CTRL0 = INT_MEAS_EN | AUTO_SR_EN              = 0x24
     *     CTRL2 = CM_FREQ_1000HZ|CMM_EN|PRD_SET_EN|PRD  = 0x8F
     *   直接调用它，避免再手写出错。
     *
     * 之后每个 DRDY 中断读出样本，由 board_io.c 的状态机决定下一拍。 */
    rc = MMC_Enable();
    printf("MMC,enable_rc=%d\r\n", rc);


    /*--------------------------------------------------------------------------
     * 主循环：每秒一次上报
     *------------------------------------------------------------------------*/
    {
        uint64_t t_report = GetTime64_Us();
        uint32_t last_cnt  = 0u;
        uint32_t last_icnt = 0u;
        double   off_x = 0.0, off_y = 0.0, off_z = 0.0;   /* 零偏滑动值 */

        for (;;)
        {
            uint64_t now = GetTime64_Us();

            if ((now - t_report) >= 1000000ULL)      /* 每 1 秒 */
            {
                uint32_t cnt = g_mmc_isr_cnt;
                int32_t  x = g_mmc_raw[0];
                int32_t  y = g_mmc_raw[1];
                int32_t  z = g_mmc_raw[2];
                uint8_t  pol = g_mmc_polarity;
                uint32_t n   = g_mmc_pair_n;

                /* 取走中断内累计的配对均值，并把累加器清零。
                 * 中断可能在这一瞬间又加了一笔，误差只有一对，可接受。 */
                int64_t sx = g_mmc_pair_sum[0];
                int64_t sy = g_mmc_pair_sum[1];
                int64_t sz = g_mmc_pair_sum[2];
                g_mmc_pair_sum[0] = 0;
                g_mmc_pair_sum[1] = 0;
                g_mmc_pair_sum[2] = 0;
                g_mmc_pair_n = 0u;

                /* 零偏指数滤波：off = keep*off + (1/W)*本秒配对均值
                 * 配对由中断用"不相邻两拍"完成，是物理时间上最近的一对。 */
                if (n > 0u)
                {
                    double px = (double)sx / (double)n;
                    double py = (double)sy / (double)n;
                    double pz = (double)sz / (double)n;
                    double w  = 1.0 / (double)MMC_OFF_WEIGHT;
                    double k  = 1.0 - w;

                    off_x = k * off_x + w * px;
                    off_y = k * off_y + w * py;
                    off_z = k * off_z + w * pz;
                }

                printf("SEC,mmc_hz=%u,pairs=%u,pol=%u,raw=%d,%d,%d,off=%.0f,%.0f,%.0f,"
                       "mmc_err=%u\r\n",
                       (unsigned)(cnt - last_cnt), (unsigned)n, (unsigned)pol,
                       (int)x, (int)y, (int)z,
                       off_x, off_y, off_z,
                       (unsigned)g_mmc_spi_err);

                /* 陀螺：每秒中断次数 + 最新一帧原始值。
                 * 0.140 dps/LSB（±4000dps 档），换算成 dps 一并给出。 */
                {
                    uint32_t icnt = g_imu_isr_cnt;
                    int16_t  gx = g_imu_raw[0];
                    int16_t  gy = g_imu_raw[1];
                    int16_t  gz = g_imu_raw[2];

                    printf("IMU,hz=%u,g=%d,%d,%d,dps=%.1f,%.1f,%.1f,err=%u\r\n",
                           (unsigned)(icnt - last_icnt),
                           (int)gx, (int)gy, (int)gz,
                           (double)gx * 0.140, (double)gy * 0.140, (double)gz * 0.140,
                           (unsigned)g_imu_spi_err);
                    last_icnt = icnt;
                }

                last_cnt = cnt;
                t_report = now;
            }
        }
    }
}
