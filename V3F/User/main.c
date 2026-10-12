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

/*==============================================================================
 * 地磁失效恢复
 *
 * 失效机理（已在多份录制上复现，g_mmc_isr_cnt 冻结在 10378 / 418844 且此后永不更新）：
 *   ISR 末尾那次"写 STATUS 清输出锁存"若没生效，INT 保持高（高有效 + 电平锁存），
 *   而 EXTI 是上升沿触发 ⟹ 之后【再无上升沿】⟹ ISR 永不进入
 *   ⟹ g_mmc_isr_cnt 冻结，三轴与 pol 一起冻结（它们与计数在同一个 ISR 里发布）
 *   ⟹ 软件侧唯一的清除点在 ISR 内，而 ISR 已经不跑 ⟹ 只能整板断电
 *
 * 本恢复路径补上 ISR 之外的入口：
 *   第 1 级（几十 us）: 主动写一次器件 STATUS(0x08) 清输出锁存，2ms 内看计数是否恢复
 *   第 2 级（~12 ms）: 仍未恢复则写【器件 CTRL1 的 SW_RST 位】让地磁自身复位，
 *                      再重跑 MMC_Init/ClearLatch/Enable
 *
 * ★ 两级都只操作 MMC5983MA 自己的寄存器 —— MCU 不复位、不外设复位、不重入 main()。
 *   连续录制因此不会中断，只是地磁在这段时间不产生样本。
 *
 * 检测判据用 g_mmc_isr_cnt：它写在 ISR 最开头，早于任何 SPI 操作，
 * 所以"计数不动"等价于"ISR 没进"，不受 ISR 内 SPI 失败干扰。
 * 地磁周期约 977us，"停滞 2ms"已错过两拍，不会误触发。
 *
 * 调用位置：气压计 23.5ms 转换窗口内的 GPS 轮询里 —— 那里不阻塞气压计时序，
 * 且单次开销（几次 SPI 读 ~几 us）远小于 GPS 分块（150 字节约 68us）。
 *============================================================================*/
#define MMC_STALL_US            2000u   /* 连续这么久计数不动 = 停滞 */
#define MMC_RECOVER_WAIT_US     2000u   /* 清锁存后等这么久看是否恢复 */

static uint32_t s_mmc_last_cnt    = 0u;      /* 上次见到的 DRDY 计数 */
static uint64_t s_mmc_last_chg_us = 0u;      /* 计数上次变化的时刻 */
static uint8_t  s_mmc_in_recover  = 0u;      /* 防重入 */

/* 恢复计数只在本核累加，随后写进共享区供 V5F 上报（V5F 看不到本核的全局符号，
 * 做成跨核 extern 会链接失败 —— 所以走共享区，见 mipc_shm.h 的同名字段）。 */
static uint32_t s_mmc_recover_cnt    = 0u;   /* 第 1 级(清锁存)触发次数 */
static uint32_t s_mmc_reinit_cnt     = 0u;   /* 第 2 级(软件复位)触发次数 */
static uint32_t s_mmc_last_reinit_rc = 0u;   /* 最近一次 MMC_Init 返回码 */

static void MMC_RecoverPublish(void)
{
    if (g_shm == 0)
        return;
    g_shm->mmc_recover_cnt    = s_mmc_recover_cnt;
    g_shm->mmc_reinit_cnt     = s_mmc_reinit_cnt;
    g_shm->mmc_last_reinit_rc = s_mmc_last_reinit_rc;
}

static void MMC_RecoverPoll(void)
{
    uint64_t now;
    uint32_t cnt;

    if (s_mmc_in_recover != 0u)
        return;

    cnt = g_mmc_isr_cnt;
    now = GetTime64_Us();

    /* 计数前进：刷新时间戳 */
    if (cnt != s_mmc_last_cnt)
    {
        s_mmc_last_cnt    = cnt;
        s_mmc_last_chg_us = now;
        return;
    }

    /* 计数不动，且还没到停滞门限 */
    if ((now - s_mmc_last_chg_us) < (uint64_t)MMC_STALL_US)
        return;

    /*--- 判定停滞，开始恢复（此时 now 即停滞判定时刻） ---*/
    s_mmc_in_recover = 1u;
    s_mmc_recover_cnt++;
    MMC_RecoverPublish();

    /* 第 1 级：主动清一次输出锁存（不依赖 ISR） */
    (void)MMC_ClearLatch();

    {
        uint64_t t_end = now + (uint64_t)MMC_RECOVER_WAIT_US;
        do
        {
            if (g_mmc_isr_cnt != s_mmc_last_cnt)
            {
                /* 恢复了：锁存已清，下一次测量完成会产生真正的上升沿 */
                s_mmc_last_cnt    = g_mmc_isr_cnt;
                s_mmc_last_chg_us = GetTime64_Us();
                s_mmc_in_recover  = 0u;
                return;
            }
        } while ((int64_t)(t_end - GetTime64_Us()) > 0);
    }

    /* 第 2 级：清锁存无效 ⟹ 器件测量状态机也需复位。
     * 只写【器件 CTRL1 的 SW_RST 位】让地磁自身复位（MCU 不动），
     * 再重跑初始化。MMC_Init 内部即为"写 SW_RST → 等 OTP → 清标志 → 配寄存器"。 */
    s_mmc_reinit_cnt++;
    {
        int rc = MMC_Init();
        s_mmc_last_reinit_rc = (uint32_t)(rc & 0xFF);

        if (rc == MMC_OK)
        {
            (void)MMC_ClearLatch();     /* 必须在开测量前清，否则第一个上升沿会丢 */
            rc = MMC_Enable();
            if (rc != MMC_OK)
                s_mmc_last_reinit_rc = 0x80u | (uint32_t)(rc & 0x7Fu);
        }
    }
    MMC_RecoverPublish();

    s_mmc_last_cnt    = g_mmc_isr_cnt;
    s_mmc_last_chg_us = GetTime64_Us();
    s_mmc_in_recover  = 0u;
}

/* 主循环开始前调用一次：把"计数上次变化时刻"对齐到当前时刻。
 * 不这样做的话 s_mmc_last_chg_us 还是 0，主循环第一轮就会 now-0 >= 2ms
 * 而误判为停滞（虽然第 1 级清锁存是幂等无害的，但会白记一次 recover_cnt）。 */
static void MMC_RecoverArm(void)
{
    s_mmc_last_cnt    = g_mmc_isr_cnt;
    s_mmc_last_chg_us = GetTime64_Us();
    s_mmc_in_recover  = 0u;
}

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
     * 内核电压 1V2 配置
     *
     * PA5 经 R11(2MΩ) 注入外部 DC-DC 的反馈节点 DC1FB：
     *   推挽输出高 -> 拉高 FB -> DC-DC 拉低输出 -> 1V2 最低
     *   推挽输出低 -> 拉低 FB -> DC-DC 抬高输出 -> 1V2 最高
     *----------------------------------------------------------------*/
    {
        GPIO_InitTypeDef vg = {0};

        RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
        vg.GPIO_Pin   = GPIO_Pin_5;
        vg.GPIO_Mode  = GPIO_Mode_Out_PP;
        vg.GPIO_Speed = GPIO_Speed_High;
        GPIO_Init(GPIOA, &vg);

        GPIO_SetBits(GPIOA, GPIO_Pin_5);        /* 1V2 最低 */
    }

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

    /* 5. 武装地磁恢复看门狗（对齐"计数上次变化时刻"，避免首轮误判） */
    MMC_RecoverArm();

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

        /* 转换期：有 GPS 数据就处理一块（限 150 字节 ~ 68us），否则原地重试。
         * 同时在这里跑地磁失效恢复：本窗口约 21ms 余量，恢复的第 1 级只需几十 us，
         * 第 2 级（软件复位）最多占 10~30ms —— 会吃掉本窗口的余量，
         * 但"地磁整段报废"比"气压计晚一拍"严重得多，所以允许。 */
        while ((int64_t)((t_trig + BARO_CONV_US - BARO_DRDY_GUARD_US) - GetTime64_Us()) > 0)
        {
            MMC_RecoverPoll();     /* 地磁失效恢复：先清锁存，2ms 不恢复则软复位 */
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
