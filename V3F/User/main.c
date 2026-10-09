/*******************************************************************************
 * File Name    : main.c
 * Description  : V3F 小核入口 —— 地磁 + 陀螺双 DRDY 中断采集
 *
 * 【分工】
 *   EXTI7_0_IRQHandler  (抢占级 1)  地磁 MMC5983MA，1024 Hz
 *   EXTI15_8_IRQHandler (抢占级 0)  陀螺 LSM6DSV16X，7680 Hz
 *   本文件只做：上电初始化、中断使能、主循环每秒上报一次。
 *******************************************************************************/

/*==============================================================================
 * 外设开关 —— 必须放在所有 include 之前
 *
 * board_io.h 里这些宏有 #ifndef 保护，而 board_io.h 会被 debug.h /
 * hardware.h 间接提前包含；定义放在 include 之后会被默认值抢先，产生
 * "redefined" 警告且覆盖不掉。
 *============================================================================*/
#define BOARD_EN_SDMMC          0
#define BOARD_EN_SPI1_MMC       1       /* 地磁 MMC5983MA @ SPI1，CS=PE0，DRDY=PE1 */
#define BOARD_EN_SPI2_IMU       0       /* SPI2 由 LSM6 驱动自行初始化（见 lsm6dsv16x.c） */
#define BOARD_EN_I2C_BMP388     0       /* 气压计暂不启用 */
#define BOARD_EN_USART2_GNSS    0       /* GPS 暂不启用 */
#define BOARD_EN_CAN1           0
#define BOARD_EN_EXTI           1       /* 两个 DRDY 都要 EXTI */

#include "debug.h"
#include "hardware.h"
#include "board_io.h"
#include "sys_clk.h"
#include "sensor_spi.h"
#include "mmc5983.h"
#include "mmc5983ma_reg.h"
#include "lsm6dsv16x.h"
#include "ch32h417.h"

int main(void)
{
    int rc;

    SystemInit();
    SystemAndCoreClockUpdate();
    Delay_Init();
    SYS_CLK_Init();
    USART_Printf_Init(921600);

    printf("\r\n\r\nREBOOTED\r\n");

    BOARD_Init();
    printf("BOARD,init_done\r\n");

    /*--------------------------------------------------------------------------
     * 1. 器件配置（都【不】启动转换）
     *
     * LSM6 必须排在地磁之后、EXTI 使能之前：
     *   LSM6_Enable() 内部有约 5~10ms 的 CPU 忙等（等 HAODR 滤波器建立），
     *   若地磁 EXTI 已使能，这段时间会把地磁节拍整个堵住。
     *   LSM6_Init() 只配置不启动，启动必须单独调 LSM6_Enable()。
     *------------------------------------------------------------------------*/
    rc = MMC_Init();
    printf("MMC,init_rc=%d\r\n", rc);

    rc = LSM6_Init();
    printf("LSM6,init_rc=%d\r\n", rc);

    rc = LSM6_Enable();
    printf("LSM6,enable_rc=%d\r\n", rc);

    /*--------------------------------------------------------------------------
     * 2. 中断归核与抢占优先级
     *
     * 分组按 EXTI 线号，与端口无关：
     *     EXTI_Line1  = PE1  -> EXTI7_0_IRQn  (36)  地磁 DRDY
     *     EXTI_Line11 = PD11 -> EXTI15_8_IRQn (33)  陀螺 INT1
     *     EXTI_Line10 = PB10 -> EXTI15_8_IRQn (33)  复位键
     * 两个 IRQn 都 > 31，必须显式 NVIC_SetAllocateIRQ 分给 V3F。
     *
     * 抢占级：陀螺 0（最高）、地磁 1。理由：地磁 ISR 单次阻塞约 35us，
     * 陀螺周期只有 130us，地磁若优先会把陀螺堵住 4 个周期直接丢帧；
     * 反向影响可忽略（地磁周期 977us，被抢断几十 us 无所谓）。
     *
     * V3F 用 2 位抢占级，写 IPRIOR 时左移到 bit7:6。
     *------------------------------------------------------------------------*/
    NVIC_SetAllocateIRQ(EXTI15_8_IRQn, Core_ID_V3F);
    NVIC_SetPriority(EXTI15_8_IRQn, (uint8_t)(0u << 6));
    NVIC_EnableIRQ(EXTI15_8_IRQn);

    NVIC_SetAllocateIRQ(EXTI7_0_IRQn, Core_ID_V3F);
    NVIC_SetPriority(EXTI7_0_IRQn, (uint8_t)(1u << 6));
    NVIC_EnableIRQ(EXTI7_0_IRQn);

    printf("ARM,pe1=%u\r\n", (unsigned)MMC_DRDY_READ());

    /*--------------------------------------------------------------------------
     * 3. 清地磁 INT 锁存，让 PE1 回低
     *
     * 器件 INT 是【高有效 + 电平锁存】，只会被"写 1 到 Status"清除。
     * 不清的话上电第一次测量完成就把它拉高并一直保持，之后没有上升沿。
     *------------------------------------------------------------------------*/
    (void)MMC_ClearLatch();
    printf("CLR,pin=%u\r\n", (unsigned)MMC_DRDY_READ());

    /*--------------------------------------------------------------------------
     * 4. 启动首拍
     *
     * 用 MMC_Enable()（归档验证过的 auto_set=1 配置）：
     *     CTRL0 = INT_MEAS_EN | AUTO_SR_EN             = 0x24
     *     CTRL2 = CM_FREQ_1000HZ|CMM_EN|PRD_SET_EN|PRD = 0x8F
     * 不要手写这两个寄存器 —— 漏掉 PRD_SET_EN(0x80) 会让器件完全不测量。
     *
     * 之后每拍都由 EXTI7_0_IRQHandler 里的极性状态机接力。
     *------------------------------------------------------------------------*/
    rc = MMC_Enable();
    printf("MMC,enable_rc=%d\r\n", rc);

    printf("RUNNING\r\n");

    /*--------------------------------------------------------------------------
     * 主循环：每秒上报一次
     *------------------------------------------------------------------------*/
    {
        uint64_t t_report = GetTime64_Us();
        uint32_t last_cnt  = 0u;
        uint32_t last_icnt = 0u;
        double   off_x = 0.0, off_y = 0.0, off_z = 0.0;

        for (;;)
        {
            uint64_t now = GetTime64_Us();

            if ((now - t_report) >= 1000000ULL)
            {
                uint32_t cnt = g_mmc_isr_cnt;
                int32_t  x = g_mmc_raw[0];
                int32_t  y = g_mmc_raw[1];
                int32_t  z = g_mmc_raw[2];
                uint8_t  pol = g_mmc_polarity;
                uint32_t n   = g_mmc_pair_n;

                /* 取走中断累计的配对均值并清零 */
                int64_t sx = g_mmc_pair_sum[0];
                int64_t sy = g_mmc_pair_sum[1];
                int64_t sz = g_mmc_pair_sum[2];
                g_mmc_pair_sum[0] = 0;
                g_mmc_pair_sum[1] = 0;
                g_mmc_pair_sum[2] = 0;
                g_mmc_pair_n = 0u;

                /* 零偏指数滤波：off = keep*off + (1/W)*本秒配对均值 */
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
