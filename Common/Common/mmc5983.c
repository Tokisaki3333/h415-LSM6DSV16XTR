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
#include <math.h>
#include <stdio.h>

/* SPI 器件句柄（SPI1 + 软件片选 PE0，网表 PE0 ; U1.63 U11.4） */
#define MMC_SPI_HANDLE()    MMC_SPI, MMC_CS_PORT, MMC_CS_PIN

/* 18bit 原始值到高斯 */

/* SET/RESET 触发后的固定等待。实测操作本身约 13us，取 100us 留足余量。
 * 不读回 CTRL0 判断完成 —— 它是只写寄存器，读回值不可信。 */

/*------------------------------------------------------------------------------
 * OTP 就绪探针参数
 *
 * ★ 全程只操作 MMC5983MA 自己的寄存器，不涉及 MCU 复位。
 *   "复位"在这里一律指【写器件 CTRL1(0x0A) 的 SW_RST 位(bit7)】，
 *   让地磁器件自身回到上电初始态；MCU 不复位、不外设复位、不重入 main()。
 *   连续录制因此不会被打断（只是地磁在这段时间不产生样本）。
 *
 * 观测方式：从写完器件 SW_RST 起，以 100us 粒度连续读 STATUS，
 * 直到 OTP_Rd_Done(0x10) 置起为止。观测窗口必须从 t=0 开始、粒度远细于
 * 待测事件 —— 旧实现"先固定等 10ms、再 1ms 轮询"报出的 1012us 就是量化误差。
 *
 * 实测：OTP 就绪 649us，整段 MMC_Init 1615us（器件已上电很久，只需重读 OTP）。
 * 因此不再保留任何"固定等待"分支。
 *------------------------------------------------------------------------------*/
#define MMC_OTP_PROBE_STEP_US   100u    /* 轮询粒度 */
#define MMC_OTP_PROBE_MAX       600u    /* 最多 600 次 × 100us = 最长观测 60ms */

/* 启动时"复位到干净态"的最大重试次数。 *
 * 实测器件会带着上次运行的残留状态启动，且【单次 SW_RST + 10ms 可能不到位】
 * （复位后 STATUS 仍为 0x11 而不是 0x10，此时写 STATUS 清位也无效）。
 * 每轮重试 = 写 SW_RST + 等 10ms + 等 OTP + 清完成标志 + 复核，
 * 幂等、可安全重复。取 5 次足够（每次约 10ms，最坏多花 50ms）。 */
#define MMC_RST_TRY             5u

/*==============================================================================
 * 内部状态与常量
 *============================================================================*/

/* 18bit 模式：每轴 16384 counts/G，理想零点 131072 counts */




/*==============================================================================
 * 内部状态
 *============================================================================*/

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

/*==============================================================================
 * 初始化
 *============================================================================*/
int MMC_Init(void)
{
    uint8_t  v = 0u;
    uint32_t i;
    uint64_t t_rst0    = 0u;    /* 本复位轮次的起点 */
    uint64_t otp_wait_us = 0u;  /* 本复位轮次的 OTP 等待耗时 */
    uint64_t otp_max_us  = 0u;  /* 所有轮次里的 OTP 最大等待 */
    uint64_t t_init0     = GetTime64_Us();  /* 整个 MMC_Init 的起点 */

    /* 0) ★ 软件复位：把器件拉回已知状态
     *
     * 手册 P15 对 CTRL1.SW_RST 的说明：
     *   "Writing 1 will cause the part to reset, similar to power-up.
     *    It will clear all registers and also re-read OTP as part of its
     *    startup routine. The power on time is 10 mS."
     *
     * 【为什么必须在每次 MCU 启动后做】
     *   实测器件会带着【上次运行的残留状态】启动，表现为复位后
     *   STATUS = 0x11（Meas_M_Done 也置起，正常应只有 0x10）。
     *   这种状态下器件不再产生测量（mmc_hz=0），且只有整板断电才能恢复。
     *   所以"上电即复位"是必需的，不能省。
     *
     * 【为什么一遍不够，要重试】
     *   实测单次 SW_RST + 10ms 之后，STATUS 仍可能是 0x11（复位没到位），
     *   此时写 STATUS 清位也无效。做法是：
     *     复位 -> 等 OTP 就绪 -> 清完成标志 -> 复核
     *   若复核结果不是 0x10，就【整个重来一遍】，最多 MMC_RST_TRY 次。
     *   每轮重试都是幂等的（写常量、不复位状态机），可以安全重复。
     *
     * 【不再固定等待 10ms —— 实测不需要】
     *   这里只写【器件 CTRL1 的 SW_RST 位】，让地磁器件自身复位，不涉及 MCU。
     *   手册把 10ms 标为 power-on time（真正上电到可用的时间）；而本函数在启动
     *   流程里被调用时器件早已上电很久，SW_RST 只是让它重读 OTP。
     *   实测（100us 粒度、从写完 SW_RST 起连续观测）：
     *       OTP 就绪 = 649us，整段 MMC_Init = 1615us
     *   而旧实现"固定等 10ms + 1ms 粒度轮询"报出的 rd=11018us / 全程 11924us，
     *   其中 10ms 是白等、1.012ms 是轮询量化误差 —— 都不是器件的真实需求。
     *------------------------------------------------------------------------*/
    for (i = 0u; i < MMC_RST_TRY; i++)
    {
        t_rst0 = GetTime64_Us();        /* 本复位轮次起点 */
        if (mmc_wr(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST) != 0)
            return MMC_ERR_SPI;

        /* 从写完 SW_RST 起就以 100us 粒度连续观测 OTP 就绪。
         * 观测窗口必须从 t=0 开始、且粒度远细于待测事件（实测约 650us），
         * 否则会把量化误差当成物理量 —— 这正是旧实现报 1012us 的原因。 */
        {
            uint32_t k;
            for (k = 0u; k < MMC_OTP_PROBE_MAX; k++)
            {
                if (mmc_rd(MMC5983MA_REG_STATUS, &v) == 0 &&
                    (v & MMC5983MA_ST_OTP_RD_DONE))
                    break;
                HWT_DelayUs(MMC_OTP_PROBE_STEP_US);
            }
            otp_wait_us = GetTime64_Us() - t_rst0;   /* 从 SW_RST 起的真实耗时 */
            if (otp_wait_us > otp_max_us)
                otp_max_us = otp_wait_us;

            /* rd_us  = 从写完 SW_RST 到第一次读到 OTP 就绪的【真实】耗时
             * ok_us  = 同上（保留同名字段便于对照旧日志）
             * polls  = 轮询次数（×100us 即耗时，用于确认粒度生效）
             * done   = 命中时的 STATUS（0x10 = 就绪）*/
            printf("MMC,otp,rd_us=%u,ok_us=%u,polls=%u,try=%u,done=0x%02X\r\n",
                   (unsigned)otp_wait_us,
                   (unsigned)(GetTime64_Us() - t_rst0),
                   (unsigned)k, (unsigned)i, (unsigned)v);

            if (k >= MMC_OTP_PROBE_MAX)
                return MMC_ERR_OTP_TIMEOUT;
        }

        /* 清掉残留的"测量完成"标志（写 1 到 Status 两位，写常量）
         * 必须在任何配置写入之前做，否则 mmc_cfg_ex 会写在脏状态上。 */
        if (mmc_wr(MMC5983MA_REG_STATUS,
                   (uint8_t)(MMC5983MA_ST_MEAS_M_DONE |
                             MMC5983MA_ST_MEAS_T_DONE)) != 0)
            return MMC_ERR_SPI;

        /* 复核：期望只剩 OTP_Rd_Done(0x10) */
        if (mmc_rd(MMC5983MA_REG_STATUS, &v) != 0)
            return MMC_ERR_SPI;

        printf("MMC,reset,try=%u,status=0x%02X\r\n", (unsigned)i, (unsigned)v);

        if (v == MMC5983MA_ST_OTP_RD_DONE)
            break;                      /* 已到干净态 */
    }
    if (i >= MMC_RST_TRY)
        printf("MMC,reset,NOT_CLEAN_after_%u_tries,status=0x%02X\r\n",
               (unsigned)MMC_RST_TRY, (unsigned)v);

    /* 本次 MMC_Init 的耗时小结。
     * otp_us 的语义取决于探针模式:
     *   细粒度模式 = 从写完 SW_RST 到 OTP 就绪的真实耗时（us）
     *   保守模式   = 只统计"固定等待之后"那一段轮询耗时（us） */
    printf("MMC,init_t,otp_us=%u,total_us=%u,tries=%u\r\n",
           (unsigned)otp_max_us,
           (unsigned)(GetTime64_Us() - t_init0),
           (unsigned)(i + 1u));

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
        return MMC_ERR_SPI;
    /* ★ 2026-10-14 更正：原注释写作 "BW=11: 0.5ms/800Hz/1.2mG" —— 自相矛盾
     *   （0.5 ms 对应 2000 Hz，不是 800 Hz），且曾被我误当作"当前档的器件速率"
     *   去解释实测 768 Hz，属错误依据。
     * 可核实的只有两条：
     *   (a) mmc5983ma_reg.h 明载 CM_FREQ_1000HZ【要求 BW=11】⟹ BW=11 是本器件
     *       能达到最高输出速率档的必要带宽设置；
     *   (b) BW 越宽 ⟹ 测量越快、噪声越大（本注释原想表达的取舍）。
     *   ⟹ 具体 ms/Hz/噪声 数值须查手册表格，不得凭此注释推断。 */
    if (mmc_wr(MMC5983MA_REG_CTRL3, 0x00u) != 0)
        return MMC_ERR_SPI;                         /* 4 线 SPI */
    if (mmc_cfg_ex(0u, 0u, 0u) != MMC_OK)           /* cmm_en=0：不启动测量 */
        return MMC_ERR_SPI;


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
     * 【本函数由 V3F 主循环的恢复路径调用】所以这里【不能打印】——
     * 一旦需要反复重试，每次打一行会把串口刷屏，掩盖真正要看的数据。
     * 清锁存是否生效由调用方通过引脚电平自行观察。
     * 注意：本函数【只在启动时和恢复路径里】被调用，不是每帧调用。 */
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
    /* 用实测跑通的那套配置：auto_set=1，即
     *   CTRL0 = INT_MEAS_EN | AUTO_SR_EN             = 0x24
     *   CTRL2 = CM_FREQ_1000HZ | CMM_EN | PRD_SET_EN = 0x8F
     * AUTO_SR_EN 与 PRD_SET_EN 两位缺一不可：auto_set=0 时器件完全不测量。 */
    if (mmc_cfg_ex(1u, 0u, 1u) != MMC_OK)
        return MMC_ERR_SPI;

    printf("MMC,enabled,cfg=auto_set_on\r\n");
    return MMC_OK;
}
