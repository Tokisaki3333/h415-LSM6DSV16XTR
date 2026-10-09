/*******************************************************************************
 * main.c —— V3F 小核入口
 * 地磁 MMC5983MA（EXTI7_0，抢占级 1）；陀螺 LSM6DSV16X（EXTI15_8，抢占级 0）
 * 本核只启动地磁采集，并把陀螺的 TX DMA 配好；陀螺的初始化与使能由大核自己做。
 *******************************************************************************/

/* 外设开关必须在所有 include 之前：board_io.h 被 debug.h 间接包含，
 * 晚定义会被其默认值抢先。
 * 置 0 的模块（气压计 / GPS / CAN / SD 卡 / 显示 / SPI Flash）已归档到
 * _archive/legacy_disabled_features/，board_io.c 里对应的初始化被
 * #if 门控，因此不会产生未定义符号。 */
#define BOARD_EN_SDMMC          0       /* 已归档 */
#define BOARD_EN_SPI1_MMC       1       /* 地磁 @ SPI1，CS=PE0，DRDY=PE1 */
#define BOARD_EN_SPI2_IMU       0       /* SPI2 由大核的 BOARD_IMU_Init 配置 */

/* 气压转换期（us）。手册 OSR x8/x2 的最大值是 25.2ms，但本片实测
 * trigger 到 DRDY 约 23.3ms、采样周期约 23.8ms —— 手册给的是最坏情况，
 * 用最大值会让保护带被吃掉（轮询等到就绪点才开始）。故取实测值。 */
#define BARO_CONV_US            23500u

/* 进入 DRDY 轮询前的保护带（us）。一次 BMP388_IsDRDY 实测 60.6us，
 * 留 2ms 覆盖调度抖动，保证轮询在数据就绪前就开始。 */
#define BARO_DRDY_GUARD_US      2000u

/* 每次 GPS 处理的字节上限。实测 0.45us/字节 -> 150 字节约 68us。 */
#define GPS_CHUNK_BYTES         150u
#define BOARD_EN_I2C_BMP388     0       /* 气压计暂停轮询 */
#define BOARD_EN_USART2_GNSS    0       /* GPS 由 GPS_USART_Init 自配 */
#define BOARD_EN_CAN1           0       /* 未实现 */
#define BOARD_EN_EXTI           1       /* 两个 DRDY 都要 EXTI */

#include "debug.h"
#include "board_io.h"
#include "sys_clk.h"
#include "sensor_spi.h"
#include "mmc5983.h"
#include "mmc5983ma_reg.h"
#include "lsm6dsv16x.h"
#include "GPS.h"            /* GPS_USART_Init / GPS_Check */
#include "i2c_soft.h"      /* SOFT_I2C_GPIO_Init / BMP388_Init / Trigger / IsDRDY / Read */

#include "mipc_v3.h"         /* ipc_comm_init：写共享区地址并唤醒 V5F */
#include "mipc_shm.h"        /* g_shm + 通道结构 */
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

    /*------------------------------------------------------------------
     * 1. 本核只负责【地磁】的初始化与使能
     *
     * 陀螺的初始化与使能由消费方（大核）自己做 —— 它必须先武装好自己的
     * RX DMA 再启动陀螺，否则陀螺产生 DRDY 时无人读取输出寄存器，
     * INT 会锁存为高，上升沿再也不来（实测：IMU,hz=2,pub=0）。
     * 那两个函数在板级配置库里，两核都能调用，本核不需要碰。
     *----------------------------------------------------------------*/
    rc = MMC_Init();                                /* 配置地磁，不启动测量 */
    printf("MMC,init_rc=%d\r\n", rc);

    /* 陀螺的 TX 侧：本核只配 TX 通道（命令 + dummy），
     * RX 通道由大核配到它自己的缓冲。必须在 SPI2 之后、EXTI 之前。 */
    BOARD_IMU_DmaInit();

    /* 共享区：写地址到 IPC_MSG0 并唤醒 V5F（大核据此映射同一块内存）。
     * 随即置 v3f_cfg_done 通知大核"共享区已可用、可以开始它的初始化"。 */
    (void)ipc_comm_init();
    if (g_shm != 0)
        g_shm->v3f_cfg_done = 1u;

    /*------------------------------------------------------------------
     * 2. NVIC 归核与抢占优先级
     * EXTI 按【线号】分组，与端口无关：
     *   Line1(PE1)=地磁 -> EXTI7_0_IRQn(36)；
     *   Line11(PD11)=陀螺、Line10(PB10)=复位键 -> EXTI15_8_IRQn(33)。
     * 两个 IRQn 都 > 31，必须显式 NVIC_SetAllocateIRQ 分给 V3F。
     * 抢占级陀螺 0 > 地磁 1：地磁 ISR 约 35us，若它优先会把陀螺
     * （周期 130us）推迟约 1/4 周期；反向无影响（地磁周期 977us）。
     * V3F 有 2 位抢占级，写 IPRIOR 时左移到 bit7:6。
     *----------------------------------------------------------------*/
    NVIC_SetAllocateIRQ(EXTI15_8_IRQn, Core_ID_V3F);        /* 陀螺 + 复位键 */
    NVIC_SetPriority(EXTI15_8_IRQn, (uint8_t)(0u << 6));    /* 最高 */
    NVIC_EnableIRQ(EXTI15_8_IRQn);

    NVIC_SetAllocateIRQ(EXTI7_0_IRQn, Core_ID_V3F);         /* 地磁 */
    NVIC_SetPriority(EXTI7_0_IRQn, (uint8_t)(1u << 6));
    NVIC_EnableIRQ(EXTI7_0_IRQn);

    printf("ARM,pe1=%u\r\n", (unsigned)MMC_DRDY_READ());

    /* 3. 清 INT 锁存必须在启动转换之前：器件 INT 高有效且电平锁存，
     *    顺序颠倒会让第一次完成的上升沿被写入抹掉，之后不会再有。
     *    后续每拍的清除由 ISR 负责。 */
    (void)MMC_ClearLatch();
    printf("CLR,pin=%u\r\n", (unsigned)MMC_DRDY_READ());

    /* 4. 等地磁就绪。地磁的 INT 锁存已在第 3 步清掉，可以直接启动。 */
    rc = MMC_Enable();
    printf("MMC,enable_rc=%d\r\n", rc);

    /* 气压计：显式初始化软件 I2C 引脚（基线就是这么调的），再配置器件 */
    SOFT_I2C_GPIO_Init();
    BMP388_Init();

    /* GPS：初始化串口 + DMA，只做解析与发布，本核不上报 */
    GPS_USART_Init();

    /* 冷启动：器件上电后自己跑过一次转换，STATUS 的 0x60 两位已挂着。
     * 必须先把这拍数据读掉，否则第一次轮询会立刻命中旧标志，
     * 读到的是"上电自测那一次"的数据，且此后每轮都落后一拍。 */
    {
        float t0, p0;
        (void)BMP388_WaitDRDY(BMP388_TIMEOUT_US, 0);
        (void)BMP388_Read(&t0, &p0);
    }

    /* 主循环：触发转换 -> 用 GPS 填转换期 -> 轮询就绪 -> 读回发布。
     * 一轮 = 一次转换。wait_us 从 trigger 起算，即"触发到数据就绪"的真实间隔。 */
    for (;;)
    {
        uint64_t t_trig;
        uint8_t  ok;
        float    temp_c = 0.0f;
        float    press_pa = 0.0f;
        uint32_t wait_us;

        BMP388_TriggerMeasurement();
        t_trig = GetTime64_Us();

        /* 转换期：有 GPS 数据就处理一块（限 150 字节 ~ 68us），否则原地重试 */
        while ((int64_t)((t_trig + BARO_CONV_US - BARO_DRDY_GUARD_US) - GetTime64_Us()) > 0)
        {
            if (GPS_Check(GPS_CHUNK_BYTES) == 0u)
                continue;              /* 没新数据，继续等转换，不要退出 */
        }

        /* 保护带：专心轮询就绪 */
        for (;;)
        {
            ok = BMP388_IsDRDY();
            if (ok != 0u) break;
            if ((int64_t)((t_trig + BARO_CONV_US + BMP388_TIMEOUT_US) - GetTime64_Us()) <= 0)
                break;                 /* 超时放弃，不发布 */
        }

        wait_us = (uint32_t)(GetTime64_Us() - t_trig);

        if (ok != 0u && BMP388_Read(&temp_c, &press_pa) == 0)
            shm_publish_bmp(GetTime64_10Ns(), temp_c, press_pa, wait_us);
    }
}
