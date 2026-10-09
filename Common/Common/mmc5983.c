/*******************************************************************************
 * File Name        : mmc5983.c
 * Description      : MMC5983MA 三轴磁传感器驱动（SPI1，片选 PE0）
 *
 * 器件行为与实测依据见 mmc5983.h 头部说明。
 * 寄存器位定义见 mmc5983ma_reg.h（其中写明 CTRL0/CTRL2 为只写、禁止读-改-写）。
 *
 * 本文件只保留正式量测路径。
 *******************************************************************************/

#include "mmc5983.h"
#include "mmc5983ma_reg.h"
#include "sensor_spi.h"
#include "board_io.h"
#include "sys_clk.h"
#include "justfloat.h"
#include <math.h>
#include <stdio.h>

/* SPI 器件句柄（SPI1 + 软件片选 PE0，网表 PE0 ; U1.63 U11.4） */
#define MMC_SPI_HANDLE()    MMC_SPI, MMC_CS_PORT, MMC_CS_PIN

/* 18bit 原始值到高斯 */
#define MMC_COUNTS_PER_G    MMC5983MA_COUNTS_PER_G_18BIT    /* 16384.0f */

/* SET/RESET 触发后的固定等待。实测操作本身约 13us，取 100us 留足余量。
 * 不读回 CTRL0 判断完成 —— 它是只写寄存器，读回值不可信。 */
#define MMC_PULSE_DELAY_US      100u

/*==============================================================================
 * 门控参数（全部集中在这里，启动时会打印出来便于核对）
 *============================================================================*/

/* 开机标定：滑动窗口收敛判定 */
#define MMC_CAL_WIN         64u     /* 判定窗口：最近多少对 offset 样本 */
#define MMC_CAL_SD_MAX      64.0    /* 窗口内三轴 offset 标准差上限（counts）。
                                     * 实测逐周期本底散布 18~28 counts，取 2~3 倍余量。 */
#define MMC_CAL_MAX_TRY     600u    /* 最大配对数（约 1.5 秒），超时判为标定失败 */

/* 零偏绝对值范围（counts，18bit 模式）。中心取自实测标定。
 * 半宽 8000 counts 约 0.49G，对应手册 Null Field Output ±0.5G；
 * "是否稳定"由收敛判据负责，这里的范围只抓接线/供电/器件级的粗错。 */
#define MMC_CAL_X_LO        123975.0
#define MMC_CAL_X_HI        139975.0
#define MMC_CAL_Y_LO        121248.0
#define MMC_CAL_Y_HI        137248.0
#define MMC_CAL_Z_LO        123505.0
#define MMC_CAL_Z_HI        139505.0

/* 一次转换的标称周期（us）。1000Hz 档 + BW=11 实测约 826us。 */
#define MMC_CONV_PERIOD_US      826u

/* 消费在途转换时的轮询窗口（us）。
 *
 * 必须【不小于一个完整转换周期】，否则无法区分"在途转换尚未完成"与
 * "根本没有在途转换"——超时返回后那个旧标志仍会在下一次触发读时被命中。
 * 取 1000us 对实测 826us 的周期留约 21% 余量；实测显示即使完全不关心相位，
 * 只要窗口够长，无论 RESET 落在周期的什么位置都能把在途转换消费干净。
 * 实测耗时统计见 s_consume_us_max。 */
#define MMC_CONSUME_TIMEOUT_US  1400u

/* 看门狗门限（us）：一个周期耗时超过它说明时序被破坏（例如 SPI 卡住、
 * 等待 Meas_M_Done 超时）。长测里统计超时次数与最大值。 */
#define MMC_WDOG_US             20000u

/* 测试分组：每跑 GROUP_CYC 个周期就静默 GROUP_IDLE_MS 毫秒，
 * 便于串口日志按组截取（连续高速输出难以截取）。 */

/* 标定阶段 RESET 后的默认延时（us）。
 * 只用于本文件的 mmc_calibrate：那一步要"发 RESET -> 等建立 -> 触发读"，
 * 与 mmc_cycle 不同 —— 后者用的是 MMC_ConsumePending 消费在途转换。 */
#define MMC_RST_LAG_DEFAULT_US  620u

#define MMC_GROUP_CYC       12u
#define MMC_GROUP_IDLE_MS   1500u

/* 零偏滑动更新权重 MMC_OFF_WEIGHT 现在定义在 mmc5983.h ——
 * 消费端（V3F main / V5F 解析）也要用同一权重，私有宏在那边看不到。 */

/*==============================================================================
 * 内部状态
 *============================================================================*/
static uint8_t  s_prd_set = MMC_PRD_SET_1000;   /* 当前自动 SET 间隔档位 */

/*==============================================================================
 * 底层寄存器访问
 *============================================================================*/
static int mmc_rd(uint8_t reg, uint8_t *v)
{
    return SPI_ReadReg(MMC_SPI_HANDLE(), reg, v);
}

static int mmc_wr(uint8_t reg, uint8_t v)
{
    return SPI_WriteReg(MMC_SPI_HANDLE(), reg, v);
}

/*------------------------------------------------------------------------------
 * 配置 CTRL0 / CTRL2
 *
 * 两者都是【只写】寄存器，不能读-改-写，因此每次都用完整影子值构造。
 *   cmm_en = 0 -> 关闭连续转换（单次触发模式用）
 *   auto_set != 0 -> 打开芯片自动周期 SET（需 Auto_SR_en + En_prd_set + Cmm_en）
 *----------------------------------------------------------------------------*/
static int mmc_cfg_ex(uint8_t auto_set, uint8_t prd_set, uint8_t cmm_en)
{
    uint8_t cr0 = MMC5983MA_CTRL0_INT_MEAS_EN;
    uint8_t cr2 = (uint8_t)MMC5983MA_CM_FREQ_1000HZ;

    if (cmm_en != 0u)
        cr2 |= MMC5983MA_CTRL2_CMM_EN;

    if (auto_set != 0u)
    {
        cr0 |= MMC5983MA_CTRL0_AUTO_SR_EN;
        cr2 |= MMC_PRD_SET_EN;
        cr2 |= (uint8_t)(prd_set & 0x70u);
    }

    if (mmc_wr(MMC5983MA_REG_CTRL0, cr0) != 0) return MMC_ERR_SPI;
    if (mmc_wr(MMC5983MA_REG_CTRL2, cr2) != 0) return MMC_ERR_SPI;
    return MMC_OK;
}

static int mmc_cfg(uint8_t auto_set, uint8_t prd_set)
{
    return mmc_cfg_ex(auto_set, prd_set, 1u);
}

/* 打开"每次测量都自动 SET"（Prd_set 编码 000 = 每 1 次测量） */
static int mmc_auto_set_every(void)
{
    uint8_t cr0 = (uint8_t)(MMC5983MA_CTRL0_INT_MEAS_EN
                            | MMC5983MA_CTRL0_AUTO_SR_EN);
    uint8_t cr2 = (uint8_t)(MMC5983MA_CTRL2_CMM_EN
                            | MMC5983MA_CM_FREQ_1000HZ
                            | MMC_PRD_SET_EN | MMC_PRD_SET_1);

    if (mmc_wr(MMC5983MA_REG_CTRL0, cr0) != 0) return MMC_ERR_SPI;
    if (mmc_wr(MMC5983MA_REG_CTRL2, cr2) != 0) return MMC_ERR_SPI;
    return MMC_OK;
}

/*------------------------------------------------------------------------------
 * 触发一次 Set 或 Reset
 * 传 MMC5983MA_CTRL0_SET 或 MMC5983MA_CTRL0_RESET。
 * 只写常量，不读回判断（CTRL0 只写）；用固定时延保证操作结束。
 * 返回固定时延（us）。
 *----------------------------------------------------------------------------*/
static uint32_t mmc_pulse(uint8_t bit)
{
    if (mmc_wr(MMC5983MA_REG_CTRL0, bit) != 0) return 0u;
    HWT_DelayUs(MMC_PULSE_DELAY_US);
    return MMC_PULSE_DELAY_US;
}

/*==============================================================================
 * 初始化
 *============================================================================*/
int MMC_Init(void)
{
    uint8_t  v = 0u;
    uint32_t i;

    /* 0) ★ 软件复位，确保从已知状态开始
     *
     * 手册 P15 对 CTRL1.SW_RST 的说明：
     *   "Writing 1 will cause the part to reset, similar to power-up.
     *    It will clear all registers and also re-read OTP as part of its
     *    startup routine. The power on time is 10 mS."
     *
     * 为什么必须做：实测器件会进入一种卡死状态 ——
     *   PROD_ID1 读回 0x30（SPI 完全正常）、STATUS 的 OTP_Rd_Done=1，
     *   但 CTRL2.CMM_EN=1 与 CTRL0.TM_M=1 两条路径都产不出测量
     *   （m_done 恒为 0，引脚零脉冲）。
     * 这种"寄存器读得到、测量引擎不动"的状态只能靠复位拉回。
     *
     * ★ 复位后必须等【10ms】—— 手册明确写了 power on time = 10mS。
     *   先前这里没有复位、只在 OTP 轮询里每次等 1ms，总等待时间不足，
     *   器件可能没完成上电启动就被写入配置。 */
    if (mmc_wr(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST) != 0)
        return MMC_ERR_SPI;
    HWT_DelayMs(10u);                   /* 手册要求 10ms 上电时间 */

    /* 1) 等 OTP 就绪（复位后芯片要重读内部 OTP，期间读回 0xFF） */
    for (i = 0u; i < 100u; i++)
    {
        if (mmc_rd(MMC5983MA_REG_STATUS, &v) == 0 &&
            (v & MMC5983MA_ST_OTP_RD_DONE))
            break;
        HWT_DelayUs(1000u);
    }
    if (i >= 100u)
        return MMC_ERR_OTP_TIMEOUT;
    printf("MMC,otp_ready,loops=%u,status=0x%02X\r\n",
           (unsigned)i, (unsigned)v);

    /* 2) Product ID 校验 */
    v = 0u;
    if (mmc_rd(MMC5983MA_REG_PROD_ID1, &v) != 0)
        return MMC_ERR_SPI;
    if (v != MMC5983MA_PROD_ID1_VAL)
        return MMC_ERR_NO_ID;

    /* 3) 配置：先关连续转换，只做静态配置与 INT 输出使能。
     *    cmm_en=0 -> CTRL2.CMM_EN 不置位 -> 器件不自动测量，也就不会拉 INT。
     *    CTRL0 里的 INT_meas_done_en 仍然置位（mmc_cfg_ex 内部写死），
     *    这样后面一开连续转换，测量完成就会拉高 INT。 */
    if (mmc_wr(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_BW_11) != 0)
        return MMC_ERR_SPI;                         /* BW=11: 0.5ms/800Hz/1.2mG */
    if (mmc_wr(MMC5983MA_REG_CTRL3, 0x00u) != 0)
        return MMC_ERR_SPI;                         /* 4 线 SPI */
    if (mmc_cfg_ex(0u, 0u, 0u) != MMC_OK)           /* cmm_en=0：不启动测量 */
        return MMC_ERR_SPI;

    /* 4) 打印实际生效的门控参数 */
    printf("GATE,cal_win=%u,cal_sd_max=%.1f,cal_max_try=%u,off_weight=%u,pulse_us=%u\r\n",
           (unsigned)MMC_CAL_WIN, MMC_CAL_SD_MAX, (unsigned)MMC_CAL_MAX_TRY,
           (unsigned)MMC_OFF_WEIGHT, (unsigned)MMC_PULSE_DELAY_US);
    printf("GATE,range_x=%.0f..%.0f,range_y=%.0f..%.0f,range_z=%.0f..%.0f\r\n",
           MMC_CAL_X_LO, MMC_CAL_X_HI, MMC_CAL_Y_LO, MMC_CAL_Y_HI,
           MMC_CAL_Z_LO, MMC_CAL_Z_HI);

    /*--------------------------------------------------------------------------
     * ★ 到这里为止：器件已配好，但【没有在测量】。
     *   启动连续转换必须用 MMC_Enable()，而且要在 DRDY 中断使能之后 ——
     *   否则第一个测量完成产生的 INT 上升沿会丢，而 INT 是【锁存】的
     *   （只会被"写 1 到 Status"清除），丢了第一个边沿就再也没有第二个。
     *   与 LSM6_Init / LSM6_Enable 的拆分理由完全相同。
     *------------------------------------------------------------------------*/
    printf("MMC,cfg_ok\r\n");
    return MMC_OK;
}

/*==============================================================================
 * 清除 INT 锁存，让 PE1 回到低电平
 *
 * 【为什么必须有这一步 —— 这是"进不了 DRDY 中断"的直接原因】
 * MMC5983MA 的 INT 是【高有效 + 电平锁存】的：
 *     "When the measurement is finished, this bit will remain '1' till next measurement."
 *     "Writing 1 into this bit will clear the corresponding interrupt."
 * 上电后只要有过一次测量完成，INT 就一直是高。
 *
 * 而 EXTI 配的是【上升沿触发】：引脚已经是高，就再也没有 0→1 边沿，
 * 中断永远不会进 —— 实测正是"PE1 恒高 + 中断计数为 0"。
 *
 * 所以必须【在使能 EXTI 之前】主动把锁存位清掉，让引脚回低，
 * 这样第一次测量完成才会产生一个真正的上升沿。
 *
 * 写 1 到 Status 的 Meas_M_Done(bit0) + Meas_T_Done(bit2) = 0x05。
 * Status(0x08) 可读可写，且这里是"写常量"不是读-改-写。
 *============================================================================*/
int MMC_ClearLatch(void)
{
    /* 【只写，不读回】
     * 先前我在这里"写完之后同一次片选内再读 Status"做校验，
     * 实测读回 0xFF（读失败），而且那个校验还把返回值变成错误、
     * 把整个流程打断。器件在"收到写"之后需要恢复时间，
     * 紧贴着发读命令是不成立的。
     *
     * 【本函数会被主循环每帧调用】所以这里【不能打印】——
     * 轮询清锁存时每次打一行会把串口刷屏，掩盖真正要看的数据。
     * 清锁存是否生效由调用方通过引脚电平自行观察。 */
    if (SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN, MMC5983MA_REG_STATUS,
                     (uint8_t)(MMC5983MA_ST_MEAS_M_DONE |
                               MMC5983MA_ST_MEAS_T_DONE)) != 0)
        return MMC_ERR_SPI;

    return MMC_OK;
}

/*==============================================================================
 * 启动连续转换：置 CTRL2.CMM_EN，器件开始按 CM_FREQ 自动测量
 *
 * 【必须在 DRDY 中断使能之后调用】原因见 MMC_Init 末尾说明。
 * 返回 MMC_OK 或负错误码。
 *============================================================================*/
int MMC_Enable(void)
{
    /*--------------------------------------------------------------------------
     * ★ 用【归档实测跑通的那套配置】：auto_set=1
     *
     * 归档 API（_newfunc.c 的 mmc_cycle / mmc_calibrate）实测跑通过，
     * 串口日志里有 CAP,...,SET,... 的真实测量数据为证。它启动测量用的是
     * mmc_auto_set_every()，即 mmc_cfg(1,0)：
     *     CTRL0 = INT_MEAS_EN | AUTO_SR_EN              = 0x24
     *     CTRL2 = CM_FREQ_1000HZ | CMM_EN | PRD_SET_EN  = 0x8F
     *
     * 我先前写的是 mmc_cfg_ex(0,0,1)（auto_set=0）：
     *     CTRL0 = INT_MEAS_EN                           = 0x04
     *     CTRL2 = CM_FREQ_1000HZ | CMM_EN               = 0x0F
     * 实测结果是器件完全不测量（20M 次采样引脚零翻转）。
     *
     * 差异就在 AUTO_SR_EN / PRD_SET_EN 这两位。既然归档版本有实测数据，
     * 就以它为准，不再自己另配一套。
     *------------------------------------------------------------------------*/
    if (mmc_cfg_ex(1u, 0u, 1u) != MMC_OK)
        return MMC_ERR_SPI;

    printf("MMC,enabled,cfg=auto_set_on\r\n");
    return MMC_OK;
}

/*==============================================================================
 * 单次触发模式
 *============================================================================*/
int MMC_OneShotConfig(void)
{
    return mmc_cfg_ex(0u, 0u, 0u);
}

int MMC_ContinuousConfig(void)
{
    return mmc_cfg_ex(0u, 0u, 1u);
}

/* 触发一次测量并读出结果（仅适用于已调用 MMC_OneShotConfig 之后） */
int MMC_TriggerAndRead(MMC_RawXYZ *out)
{
    uint64_t t0;
    uint8_t  st = 0u;
    uint8_t  d[7];

    if (out == 0) return MMC_ERR_PARAM;

    /*--------------------------------------------------------------------------
     * 写 1 启动一次测量（TM_M，结束自清）
     *
     * ★ 必须【同时带上 INT_meas_done_en】，否则每触发一次就把中断使能抹掉。
     *   CTRL0 是【只写】寄存器，每次写都是整字节覆盖：
     *     只写 TM_M(0x01) -> CTRL0 变成 0x01，bit2(INT_meas_done_en) 被清零
     *   后果：测量照做（RD 数据正常），但 INT 引脚永远不输出，
     *         实测表现为 pe1 恒低、irq=0、示波器无脉冲。
     *
     *   CTRL0 位定义（手册 P15）：
     *     bit0 TM_M | bit1 TM_T | bit2 INT_meas_done_en
     *     bit3 Set | bit4 Reset | bit5 Auto_SR_en | bit6 OTP_Read
     *   触发测量与中断使能是同一次写入的两个位，可并存：
     *     0x01 | 0x04 = 0x05
     *------------------------------------------------------------------------*/
    if (mmc_wr(MMC5983MA_REG_CTRL0,
               (uint8_t)(MMC5983MA_CTRL0_TM_M |
                         MMC5983MA_CTRL0_INT_MEAS_EN)) != 0)
        return MMC_ERR_SPI;

    t0 = GetTime64_Us();
    for (;;)
    {
        if (mmc_rd(MMC5983MA_REG_STATUS, &st) != 0) return MMC_ERR_SPI;
        if (st & MMC5983MA_ST_MEAS_M_DONE) break;
        if ((GetTime64_Us() - t0) > MMC_READ_TIMEOUT_US)
            return MMC_ERR_DRDY_TIMEOUT;
    }

    if (SPI_ReadRegs(MMC_SPI_HANDLE(), MMC5983MA_REG_XOUT0, d, 7u) != 0)
        return MMC_ERR_SPI;

    (void)mmc_wr(MMC5983MA_REG_STATUS, MMC5983MA_ST_MEAS_M_DONE);

    out->x = MMC5983MA_XYZ_X(d[0], d[1], d[6]);
    out->y = MMC5983MA_XYZ_Y(d[2], d[3], d[6]);
    out->z = MMC5983MA_XYZ_Z(d[4], d[5], d[6]);

    return MMC_OK;
}

/*==============================================================================
 * 消费在途转换
 *
 * 【问题】切到单次触发模式后，芯片可能还有一次在途转换。它完成时会置
 * Meas_M_Done；若不清掉，下一次 MMC_TriggerAndRead 的轮询会立刻命中这个
 * 旧标志，读到的是翻转前的数据（实测恒定偏低约 6300 counts）。
 *
 * 【做法】轮询一个【不小于一个完整转换周期】的窗口：
 *   - 期间标志置起 -> 确实有在途转换，读走并写 1 清掉
 *   - 整个窗口都无标志 -> 确实没有在途转换（已无连续转换在跑）
 * 两种情况结束后标志都是干净的，因此不需要关心 RESET 落在周期的哪个相位。
 *
 * 【窗口长度为什么是关键】实测表明无论 RESET 后的固定延时取 0~1600us、
 * 结果都一样；真正起作用的就是这个窗口足够长。
 *
 * 统计通过 s_consume_hit / s_consume_miss / s_consume_us_max 暴露。
 *----------------------------------------------------------------------------*/
uint32_t s_consume_hit    = 0u;     /* 消费到在途转换的次数 */
uint32_t s_consume_miss   = 0u;     /* 窗口内无标志、确认无在途转换的次数 */
uint32_t s_consume_us_max = 0u;     /* 命中时等待在途转换完成的最长时间(us) */

int MMC_ConsumePending(MMC_RawXYZ *out)
{
    uint64_t t0;
    uint8_t  st = 0u;
    uint8_t  d[7];

    t0 = GetTime64_Us();
    for (;;)
    {
        if (mmc_rd(MMC5983MA_REG_STATUS, &st) != 0) return MMC_ERR_SPI;
        if (st & MMC5983MA_ST_MEAS_M_DONE)
        {
            uint32_t el = (uint32_t)(GetTime64_Us() - t0);

            /* 在途转换已完成：读走并清标志 */
            if (SPI_ReadRegs(MMC_SPI_HANDLE(), MMC5983MA_REG_XOUT0, d, 7u) != 0)
                return MMC_ERR_SPI;
            (void)mmc_wr(MMC5983MA_REG_STATUS, MMC5983MA_ST_MEAS_M_DONE);
            s_consume_hit++;
            if (el > s_consume_us_max) s_consume_us_max = el;

            if (out != 0)
            {
                out->x = MMC5983MA_XYZ_X(d[0], d[1], d[6]);
                out->y = MMC5983MA_XYZ_Y(d[2], d[3], d[6]);
                out->z = MMC5983MA_XYZ_Z(d[4], d[5], d[6]);
            }
            return MMC_OK;
        }
        if ((GetTime64_Us() - t0) > MMC_CONSUME_TIMEOUT_US)
        {
            s_consume_miss++;
            return MMC_OK;      /* 没有在途转换，也是干净的 */
        }
    }
}

/*==============================================================================
 * 读一个新鲜样本
 *
 * 手册指定流程：轮询 Status 等 Meas_M_Done -> 连读 7 字节 -> 写 1 清标志。
 * 不能用"等 INT 引脚变高"：实测该引脚锁存不释放。
 * 不清标志会在同一个采样窗口内反复读到同一份数据。
 *============================================================================*/
int MMC_ReadXYZ(MMC_RawXYZ *out)
{
    uint64_t t0;
    uint8_t  st = 0u;
    uint8_t  d[7];

    if (out == 0) return MMC_ERR_PARAM;

    t0 = GetTime64_Us();
    for (;;)
    {
        if (mmc_rd(MMC5983MA_REG_STATUS, &st) != 0) return MMC_ERR_SPI;
        if (st & MMC5983MA_ST_MEAS_M_DONE) break;
        if ((GetTime64_Us() - t0) > MMC_READ_TIMEOUT_US)
            return MMC_ERR_DRDY_TIMEOUT;
    }

    if (SPI_ReadRegs(MMC_SPI_HANDLE(), MMC5983MA_REG_XOUT0, d, 7u) != 0)
        return MMC_ERR_SPI;

    (void)mmc_wr(MMC5983MA_REG_STATUS, MMC5983MA_ST_MEAS_M_DONE);

    out->x = MMC5983MA_XYZ_X(d[0], d[1], d[6]);
    out->y = MMC5983MA_XYZ_Y(d[2], d[3], d[6]);
    out->z = MMC5983MA_XYZ_Z(d[4], d[5], d[6]);

    return MMC_OK;
}

/*==============================================================================
 * 自动周期 SET 开关
 *============================================================================*/
int MMC_AutoSetOn(uint8_t prd_set)
{
    return mmc_cfg(1u, prd_set);
}

int MMC_AutoSetOff(void)
{
    return mmc_cfg(0u, 0u);
}

/*==============================================================================
 * 手动 SET / RESET
 * 内部先关自动 SET、做完再恢复，避免与芯片自动 SET 冲突。
 *============================================================================*/
uint32_t MMC_Set(void)
{
    uint32_t d;

    (void)mmc_cfg(0u, 0u);
    d = mmc_pulse(MMC5983MA_CTRL0_SET);
    (void)mmc_cfg(1u, s_prd_set);

    return d;
}

uint32_t MMC_Reset(void)
{
    uint32_t d;

    (void)mmc_cfg(0u, 0u);
    d = mmc_pulse(MMC5983MA_CTRL0_RESET);
    (void)mmc_cfg(1u, s_prd_set);

    return d;
}

/*==============================================================================
 * 一个完整测量周期（SET 段 + RESET 段），长短测共用
 *
 * 时序：
 *   1) 连续采 b 个 SET 极性样本（芯片自动每样本前 SET；首个样本与上一周期
 *      暂存的 RESET 配对，得到零偏增量 offset = (RST + SET)/2）
 *   2) 切单次触发 -> 发 RESET -> ConsumePending 消费在途转换 -> 触发读样本
 *   3) 恢复自动 SET，把本周期 RESET 暂存给下一周期的首个 SET 配对
 *
 * 零偏更新推迟到下一周期首个 SET 读完之后做：那个 SET 与本周期 RESET 时间上
 * 最接近（约 623us），而本周期末个 SET 距 RESET 约 1363us。外界磁场在配对
 * 求和时相消，配对窗口越短越好。
 *
 * 输出：*rst 为本周期 RESET 样本；*pair 为 offset=(RST+SET)/2 的原始 counts；
 *       *in_cycle_us 为本周期耗时（us）。三者都只用于打印与诊断。
 *============================================================================*/
static int mmc_cycle(uint32_t b, double *pox, double *poy, double *poz,
                     MMC_RawXYZ *rst, MMC_RawXYZ *pend, uint8_t *have_rst,
                     double *pair, uint32_t *in_cycle_us,
                     double *set_x, double *set_y, double *set_z)
{
    uint32_t k;
    MMC_RawXYZ raw;
    uint64_t t_cyc = GetTime64_Us();
    double s0x = 0.0, s0y = 0.0, s0z = 0.0;
    int rc;

    for (k = 0u; k < b; k++)
    {
        rc = MMC_ReadXYZ(&raw);
        if (rc != MMC_OK) return rc;

        if (k == 0u)
        {
            s0x = (double)raw.x; s0y = (double)raw.y; s0z = (double)raw.z;

            /* 与上一周期暂存的 RESET 配对 */
            if (*have_rst != 0u)
            {
                double one_over_w = 1.0 / (double)MMC_OFF_WEIGHT;
                double keep = 1.0 - one_over_w;
                double px = ((double)pend->x + s0x) / 2.0;
                double py = ((double)pend->y + s0y) / 2.0;
                double pz = ((double)pend->z + s0z) / 2.0;

                *pox = keep * (*pox) + one_over_w * px;
                *poy = keep * (*poy) + one_over_w * py;
                *poz = keep * (*poz) + one_over_w * pz;
                *have_rst = 0u;

                if (pair != 0) *pair = px;
            }
        }

        if (set_x != 0)
        {
            set_x[k] = (double)raw.x;
            set_y[k] = (double)raw.y;
            set_z[k] = (double)raw.z;
        }
    }

    /* --- 2) RESET 后取一个样本 --- */
    (void)mmc_cfg_ex(0u, 0u, 0u);       /* 切单次触发模式 */
    (void)mmc_pulse(MMC5983MA_CTRL0_RESET);

    /* 消费在途转换：窗口 MMC_CONSUME_TIMEOUT_US 不小于一个完整转换周期，
     * 因此"没消费到"本身就等于"确实没有在途转换"，不必关心 RESET 的相位 */
    rc = MMC_ConsumePending(0);
    if (rc != MMC_OK) return rc;

    rc = MMC_TriggerAndRead(&raw);
    if (rc != MMC_OK) return rc;

    rst->x = raw.x; rst->y = raw.y; rst->z = raw.z;

    /* --- 3) 恢复自动 SET，并把本次 RESET 暂存给下一周期首个 SET 配对 --- */
    if (mmc_auto_set_every() != MMC_OK) return MMC_ERR_SPI;
    *pend = raw;
    *have_rst = 1u;

    if (in_cycle_us != 0)
        *in_cycle_us = (uint32_t)(GetTime64_Us() - t_cyc);
    return MMC_OK;
}

/*==============================================================================
 * 开机标定：滑动窗口收敛判定
 *
 * 每轮采一对 (RESET, 下一个 SET)，得 offset = (RST + SET)/2。
 * 维持最近 MMC_CAL_WIN 个 offset 的环形缓冲，算三轴标准差；
 * 三轴都 <= MMC_CAL_SD_MAX 即认为收敛，取窗口均值作为零偏。
 * 最后检查零偏落在 MMC_CAL_*_LO/HI 范围内，抓接线/供电/器件级的粗错。
 *============================================================================*/
static int mmc_calibrate(uint32_t b, double *pox, double *poy, double *poz)
{
    uint32_t k;
    MMC_RawXYZ raw;
    MMC_RawXYZ pend;
    double win_x[MMC_CAL_WIN], win_y[MMC_CAL_WIN], win_z[MMC_CAL_WIN];
    uint32_t win_n = 0u, win_head = 0u, tries = 0u, ok = 0u;
    int rc;

    if (mmc_auto_set_every() != MMC_OK) return MMC_ERR_SPI;

    /* 先采一组 SET 样本，作为第一对的"后邻" */
    for (k = 0u; k < b; k++)
    {
        rc = MMC_ReadXYZ(&raw);
        if (rc != MMC_OK) return rc;
    }

    while (tries < MMC_CAL_MAX_TRY)
    {
        double nx, ny, nz;
        uint32_t q;

        (void)mmc_cfg_ex(0u, 0u, 0u);
        (void)mmc_pulse(MMC5983MA_CTRL0_RESET);
        HWT_DelayUs(MMC_RST_LAG_DEFAULT_US);
        rc = MMC_TriggerAndRead(&raw);
        if (rc != MMC_OK) return rc;
        nx = (double)raw.x; ny = (double)raw.y; nz = (double)raw.z;

        if (mmc_auto_set_every() != MMC_OK) return MMC_ERR_SPI;
        for (k = 0u; k < b; k++)
        {
            rc = MMC_ReadXYZ(&raw);
            if (rc != MMC_OK) return rc;
        }

        /* 配对：RESET 与紧接的下一个 SET 首样本 */
        nx = (nx + (double)raw.x) / 2.0;
        ny = (ny + (double)raw.y) / 2.0;
        nz = (nz + (double)raw.z) / 2.0;

        win_x[win_head] = nx;
        win_y[win_head] = ny;
        win_z[win_head] = nz;
        win_head = (win_head + 1u) % MMC_CAL_WIN;
        if (win_n < MMC_CAL_WIN) win_n++;
        tries++;

        if (win_n >= MMC_CAL_WIN)
        {
            double mx = 0.0, my = 0.0, mz = 0.0;
            double vx = 0.0, vy = 0.0, vz = 0.0;

            for (q = 0u; q < MMC_CAL_WIN; q++)
            {
                mx += win_x[q]; my += win_y[q]; mz += win_z[q];
            }
            mx /= (double)MMC_CAL_WIN;
            my /= (double)MMC_CAL_WIN;
            mz /= (double)MMC_CAL_WIN;
            for (q = 0u; q < MMC_CAL_WIN; q++)
            {
                double d;
                d = win_x[q] - mx; vx += d * d;
                d = win_y[q] - my; vy += d * d;
                d = win_z[q] - mz; vz += d * d;
            }
            vx /= (double)MMC_CAL_WIN;
            vy /= (double)MMC_CAL_WIN;
            vz /= (double)MMC_CAL_WIN;

            if (vx <= MMC_CAL_SD_MAX * MMC_CAL_SD_MAX &&
                vy <= MMC_CAL_SD_MAX * MMC_CAL_SD_MAX &&
                vz <= MMC_CAL_SD_MAX * MMC_CAL_SD_MAX)
            {
                *pox = mx; *poy = my; *poz = mz;
                ok = 1u;
                break;
            }
        }
    }

    printf("CAL,converged=%u,pairs=%u\r\n", (unsigned)ok, (unsigned)tries);
    if (ok == 0u)
    {
        printf("CAL_FAIL,no_converge\r\n");
        return MMC_ERR_PARAM;
    }
    printf("CAL,offset_counts=%.1f,%.1f,%.1f\r\n", *pox, *poy, *poz);

    if (*pox < MMC_CAL_X_LO || *pox > MMC_CAL_X_HI ||
        *poy < MMC_CAL_Y_LO || *poy > MMC_CAL_Y_HI ||
        *poz < MMC_CAL_Z_LO || *poz > MMC_CAL_Z_HI)
    {
        printf("CAL_FAIL,out_of_range\r\n");
        return MMC_ERR_PARAM;
    }
    printf("CAL,range_ok\r\n");

    /* 标定完成后把运行期零偏也置为标定值 */
    (void)pend;
    return MMC_OK;
}

/*==============================================================================
 * 门控余量：三轴零偏各自距范围边界的最小距离 / 该轴半宽，取三轴最小值。
 * 返回 0~0.999；接近 1 表示远离边界，接近 0 表示逼近边界。
 *============================================================================*/
static double mmc_gate_margin(double ox, double oy, double oz)
{
    double half_x = (MMC_CAL_X_HI - MMC_CAL_X_LO) / 2.0;
    double half_y = (MMC_CAL_Y_HI - MMC_CAL_Y_LO) / 2.0;
    double half_z = (MMC_CAL_Z_HI - MMC_CAL_Z_LO) / 2.0;
    double dx_lo = ox - MMC_CAL_X_LO, dx_hi = MMC_CAL_X_HI - ox;
    double dy_lo = oy - MMC_CAL_Y_LO, dy_hi = MMC_CAL_Y_HI - oy;
    double dz_lo = oz - MMC_CAL_Z_LO, dz_hi = MMC_CAL_Z_HI - oz;
    double m = ((dx_lo < dx_hi) ? dx_lo : dx_hi) / half_x;
    double my = ((dy_lo < dy_hi) ? dy_lo : dy_hi) / half_y;
    double mz = ((dz_lo < dz_hi) ? dz_lo : dz_hi) / half_z;

    if (my < m) m = my;
    if (mz < m) m = mz;
    if (m < 0.0) m = 0.0;
    if (m > 0.999) m = 0.999;
    return m;
}

/*==============================================================================
 * 主量测入口：分组对照实验（ASCII）
 *============================================================================*/
int MMC_StreamJustFloat(uint32_t b, uint32_t ngroups)
{
    uint32_t k;
    /* pend 必须清零：mmc_cycle 在 *have_rst != 0 时读它，而首周期 have_rst=0，
     * 编译器无法证明那条路径不可达，会报 -Wmaybe-uninitialized。 */
    MMC_RawXYZ raw, rst, pend = {0, 0, 0};
    double ox = 0.0, oy = 0.0, oz = 0.0;
    double cal_ox, cal_oy, cal_oz;
    uint8_t have_rst = 0u;
    uint32_t cyc = 0u;
    uint32_t g, cyc_in_grp;
    int rc;

    if (b == 0u || b > 16u) return MMC_ERR_PARAM;
    if (ngroups == 0u) return MMC_ERR_PARAM;

    rc = mmc_calibrate(b, &ox, &oy, &oz);
    if (rc != MMC_OK) return rc;
    cal_ox = ox; cal_oy = oy; cal_oz = oz;

    for (g = 0u; g < ngroups; g++)
    {
        printf("GROUP,%u,start\r\n", (unsigned)(g + 1u));
        ox = cal_ox; oy = cal_oy; oz = cal_oz;
        have_rst = 0u;

        for (cyc_in_grp = 0u; cyc_in_grp < MMC_GROUP_CYC; cyc_in_grp++)
        {
            double pair = 0.0;
            uint32_t in_us = 0u;

            /* 不缓存 SET 样本（本函数只打印，不逐样本上报） */
            rc = mmc_cycle(b, &ox, &oy, &oz, &rst, &pend, &have_rst, &pair, &in_us,
                           0, 0, 0);
            if (rc != MMC_OK) return rc;

            if (cyc_in_grp < 6u)
            {
                printf("CYC,g=%u,n=%u,rst=%.0f,pair=%.1f,off=%.1f,cyc_us=%u\r\n",
                       (unsigned)(g + 1u), (unsigned)cyc_in_grp,
                       (double)rst.x, pair, ox, (unsigned)in_us);
            }
            cyc++;
        }

        printf("GROUP,%u,end,off=%.1f,%.1f,%.1f\r\n",
               (unsigned)(g + 1u), ox, oy, oz);
        printf("GROUP,%u,consume,hit=%u,miss=%u,us_max=%u\r\n",
               (unsigned)(g + 1u), (unsigned)s_consume_hit,
               (unsigned)s_consume_miss, (unsigned)s_consume_us_max);

        if ((g + 1u) < ngroups)
        {
            printf("GROUP,%u,idle_%ums\r\n",
                   (unsigned)(g + 2u), (unsigned)MMC_GROUP_IDLE_MS);
            HWT_DelayUs((uint64_t)MMC_GROUP_IDLE_MS * 1000ULL);
        }
    }

    (void)k; (void)raw;
    printf("DONE,groups=%u,cycles=%u\r\n", (unsigned)ngroups, (unsigned)cyc);
    return MMC_OK;
}

/*==============================================================================
 * 长连续测量实验
 *
 * 与 MMC_StreamJustFloat 共用 mmc_cycle，但：
 *   - 不分组、不静默，连续跑 max_cycles 个周期后返回
 *   - 每个 SET 样本发一帧 4 通道 justfloat：X, Y, Z, offset_X（单位 G）
 *     帧长 24 字节（双同步头 8 + 4 float 16），比 ASCII 省 30% 带宽
 *   - 每 status_every 个周期插入一行 ASCII 状态（justfloat 上位机会跳过）：
 *     零偏范围、消费统计、周期耗时、看门狗超时次数
 *
 * 看门狗：某周期耗时超过 MMC_WDOG_US 说明时序被破坏，计数并记最大值。
 *============================================================================*/
int MMC_LongRun(uint32_t b, uint32_t max_cycles, uint32_t status_every)
{
    /* pend 清零的理由同 MMC_StreamJustFloat */
    MMC_RawXYZ rst, pend = {0, 0, 0};
    double set_x[16], set_y[16], set_z[16];   /* 本周期 SET 样本缓存 */
    double ox = 0.0, oy = 0.0, oz = 0.0;
    uint8_t have_rst = 0u;
    uint32_t n;
    uint64_t t_start;
    uint32_t us_max = 0u, wdog = 0u, wdog_max = 0u;
    double ox_min, ox_max, oy_min, oy_max, oz_min, oz_max;
    int rc;

    if (b == 0u || b > 16u) return MMC_ERR_PARAM;
    if (max_cycles == 0u) return MMC_ERR_PARAM;

    rc = mmc_calibrate(b, &ox, &oy, &oz);
    if (rc != MMC_OK) return rc;

    ox_min = ox_max = ox;
    oy_min = oy_max = oy;
    oz_min = oz_max = oz;

    printf("LONG,start,b=%u,max_cycles=%u,status_every=%u,wdog_us=%u\r\n",
           (unsigned)b, (unsigned)max_cycles, (unsigned)status_every,
           (unsigned)MMC_WDOG_US);
    printf("LONG,cal=%.1f,%.1f,%.1f\r\n", ox, oy, oz);

    t_start = GetTime64_Us();

    for (n = 0u; n < max_cycles; n++)
    {
        double pair = 0.0;
        uint32_t in_us = 0u;

        rc = mmc_cycle(b, &ox, &oy, &oz, &rst, &pend, &have_rst, &pair, &in_us,
                          set_x, set_y, set_z);
        if (rc != MMC_OK)
        {
            printf("LONG,abort,rc=%d,n=%u\r\n", rc, (unsigned)n);
            return rc;
        }

        if (in_us > us_max) us_max = in_us;
        if (in_us > MMC_WDOG_US)
        {
            wdog++;
            if (in_us > wdog_max) wdog_max = in_us;
        }

        if (ox < ox_min) ox_min = ox;
        if (ox > ox_max) ox_max = ox;
        if (oy < oy_min) oy_min = oy;
        if (oy > oy_max) oy_max = oy;
        if (oz < oz_min) oz_min = oz;
        if (oz > oz_max) oz_max = oz;

        /* 本周期采集到的 SET 样本已缓存，用更新后的零偏校正后逐样本发送。
         * 放在 RESET 与零偏更新之后发送，保证同一周期的样本用同一个零偏。 */
        {
            uint32_t q;
            for (q = 0u; q < b; q++)
            {
                justfloat_send4((float)((set_x[q] - ox) / 16384.0),
                                (float)((set_y[q] - oy) / 16384.0),
                                (float)((set_z[q] - oz) / 16384.0),
                                (float)(ox / 16384.0));
            }
        }
        if (status_every != 0u && ((n + 1u) % status_every) == 0u)
        {
            uint32_t el_s = (uint32_t)((GetTime64_Us() - t_start) / 1000000ULL);
            printf("LONG,n=%u,t=%us,off=%.1f,%.1f,%.1f "
                   "span=%.1f,%.1f,%.1f gate=%.3f us_max=%u wdog=%u\r\n",
                   (unsigned)(n + 1u), (unsigned)el_s, ox, oy, oz,
                   ox_max - ox_min, oy_max - oy_min, oz_max - oz_min,
                   mmc_gate_margin(ox, oy, oz),
                   (unsigned)us_max, (unsigned)wdog);
        }
    }

    printf("LONG,end,n=%u,t=%us\r\n", (unsigned)max_cycles,
           (unsigned)((GetTime64_Us() - t_start) / 1000000ULL));
    printf("LONG,off=%.1f,%.1f,%.1f\r\n", ox, oy, oz);
    printf("LONG,span=%.1f,%.1f,%.1f\r\n",
           ox_max - ox_min, oy_max - oy_min, oz_max - oz_min);
    printf("LONG,gate=%.3f\r\n", mmc_gate_margin(ox, oy, oz));
    printf("LONG,us_max=%u,wdog=%u,wdog_max=%u\r\n",
           (unsigned)us_max, (unsigned)wdog, (unsigned)wdog_max);
    printf("LONG,consume,hit=%u,miss=%u,us_max=%u\r\n",
           (unsigned)s_consume_hit, (unsigned)s_consume_miss,
           (unsigned)s_consume_us_max);
    return MMC_OK;
}
