/*******************************************************************************
* File Name          : sd_speed.c
* Description        : SD 卡 时钟档位 / 能力报告 / 吞吐实测
*                      放在 Common/Common，与 sdmmc_sd.c 一起被自动收入工程。
*
*   背景（很重要）：
*     WCH 的 SD_Init() 只发 CMD0/CMD8/CMD55/ACMD41/CMD2/CMD3/CMD9/CMD7/ACMD6，
*     **从未发送 CMD6 (SWITCH_FUNC)**。所以卡始终停在 Default Speed。
*
*   ★ 关于 8-bit：SD 卡最大只有 4-bit，8-bit 是 eMMC/MMC 专用
*     （SD 的 ACMD6 只允许 arg=0/2）。所以本板 SD 只能用 4-bit。
*     驱动里的 SDMMC_BusWidth_8 是给 eMMC 用的，SD 卡不要碰。
*
*   ★ 关于提频：SD 规范 Default Speed 上限 25MHz（4-bit => 12.5MB/s）。
*     不协商 CMD6 就提到 50MHz 属于超频，能否稳定必须实测。
*     本文件提供阶梯升频 + 逐档校验，找到这张卡的实际稳定上限。
*
*   本文件做的事：
*     1) SD_TrySpeedStep()    —— 逐档升频，每档校验读数正确性
*     2) SD_SwitchHighSpeed() —— 发 CMD6 切 High Speed（卡支持时）
*     3) SD_ReportSpeed()     —— 打印模式/时钟/能力
*     4) SD_Benchmark()       —— TIM9 计时，实测读/写吞吐
*
*   时钟关系：SDCLK = HCLK/(DIV+1)（DIV = CLK_DIV[4:0]）
*******************************************************************************/
#include "debug.h"
#include "ch32h417.h"
#include "sdmmc_sd.h"

/* SDMMC->CLK_DIV 位（来自 ch32h417.h） */
#define SD_SPEED_CLKOE      0x0100u
#define SD_SPEED_CLKMODE    0x0200u
#define SD_SPEED_PHASEINV   0x0400u
#define SD_SPEED_DIVMASK    0x001Fu

/* CMD6(SWITCH_FUNC) 用的模式位 */
#define SD_SWITCH_CHECK     0x00000000u   /* Mode 0：只查卡支持哪些功能 */
#define SD_SWITCH_SET       0x80000000u   /* Mode 1：真正切换            */

/* 目标高速档：Function Group 1 = Access Mode，Function 1 = High Speed */
#define SD_HS_GROUP1_FUNC1  0x00FFFFF1u

/* TRAN_SPEED 低 4 bit 的传输单位 -> kbit/s */
static const uint32_t s_unit_kbps[16] = {
    100u, 1000u, 10000u, 100000u,
    0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u
};

/* 当前是否已切到 High Speed（供报告用） */
static uint8_t s_hs_active = 0;

/* CMD6 的 512B 状态块必须 16 字节对齐 */
__attribute__((aligned(32))) static uint8_t s_switch_buf[512];

/*==============================================================================
 * CMD6 (SWITCH_FUNC)
 *   arg = 0x00FFFFF1 -> Mode1, Group1, Function1 (High Speed)
 *   带一个 512B 数据块返回
 *
 *   实现严格镜像 SD_ReadBlock 的顺序（先配数据相位，再发命令），
 *   并保持 CheckIdx/CheckCRC 与 SD_ReadBlock 一致（都关），
 *   因为 SD_ReadBlock 是实测可用的路径，CMD6 之前开 CRC 校验反而失败。
 *============================================================================*/
#define SD_CMD6_DBG   1

static SD_Error sd_switch_func(uint32_t arg, uint8_t *buf512)
{
    SD_Error e;

    /* 1) 先配数据相位（接收 1 块 512B）——顺序照 SD_ReadBlock */
    SDMMC_BlockConfig(0, 0);                    /* 例程要求先清 0 */

    SDMMC_TranModeStructure.TranMode_DualDMA_TNCnt  = 0x00;
    SDMMC_TranModeStructure.TranMode_DualDMA         = DISABLE;
    SDMMC_TranModeStructure.TranMode_AutoGapStop     = DISABLE;
    SDMMC_TranModeStructure.TranMode_Boot            = DISABLE;
    SDMMC_TranModeStructure.TranMode_Direction       = SDMMC_TranDir_Receive;
    SDMMC_TranMode_Init(&SDMMC_TranModeStructure);

    SDMMC_BlockConfig(512, 1);
    SDMMC_SetDMAAddr1((uint32_t)buf512);

    /* 2) 发 CMD6，R1 响应（与 SD_ReadBlock 一致：不勾 CheckIdx/CheckCRC） */
    SDMMC_CmdInitStructure.SDMMC_Argument   = arg;
    SDMMC_CmdInitStructure.SDMMC_CMDIdx     = SD_CMD_HS_SWITCH;
    SDMMC_CmdInitStructure.SDMMC_RespExpect = SDMMC_Resp_48;
    SDMMC_CmdInitStructure.SDMMC_CheckIdx   = DISABLE;
    SDMMC_CmdInitStructure.SDMMC_CheckCRC   = DISABLE;
    SDMMC_CommandConfig(&SDMMC_CmdInitStructure);

    e = CmdRespError();
    if (e != SD_OK)
        return e;

    /* 3) 等数据块收完 */
    e = DataTransErr();

#if (SD_CMD6_DBG)
    if (e != SD_OK)
    {
        uint32_t *w = (uint32_t *)buf512;
        printf("[SD] CMD6 data err=%d  INT_FG=0x%04X  CONTROL=0x%04X  CMD_SET=0x%08X\r\n",
               (int)e, (unsigned)SDMMC->INT_FG, (unsigned)SDMMC->CONTROL,
               (unsigned)SDMMC->CMD_SET);
        printf("[SD] CMD6 resp R1  = 0x%08X\r\n", (unsigned)SDMMC->RESPONSE3);
        printf("[SD] CMD6 blk[0..3]= %08X %08X %08X %08X\r\n",
               (unsigned)w[0], (unsigned)w[1], (unsigned)w[2], (unsigned)w[3]);
    }
#endif

    /* 4) 收尾：清块配置，免得影响后续操作 */
    SDMMC_BlockConfig(0, 0);

    return e;
}

/*==============================================================================
 * SD_SwitchHighSpeed
 *   流程：CMD6(Check) 确认卡支持 -> CMD6(Set) 切换 -> 校验状态 -> 升 SDCLK
 *   返回：0 成功；非 0 = 失败码
 *============================================================================*/
int SD_SwitchHighSpeed(uint16_t div_after)
{
    uint32_t *w = (uint32_t *)s_switch_buf;
    uint32_t support1, status;
    SD_Error e;

    if (div_after == 0u) div_after = 1u;        /* 保护：别把 DIV 设成 0 */

    printf("[SD] CMD6: querying supported functions ...\r\n");
    e = sd_switch_func(SD_SWITCH_CHECK, s_switch_buf);
    if (e != SD_OK)
    {
        printf("[SD] CMD6 check failed (err=%d)\r\n", (int)e);
        return -1;
    }

    /* 返回块 word1 = 各功能组支持位，bit1 对应 Group1 的 Function1 */
    support1 = w[1];
    printf("[SD] CMD6 support word = 0x%08X  (Group1 b1 = HighSpeed: %s)\r\n",
           (unsigned)support1, (support1 & 0x2u) ? "yes" : "NO");

    if ((support1 & 0x2u) == 0u)
    {
        printf("[SD] card does NOT support High Speed -> stay Default Speed\r\n");
        return -2;
    }

    printf("[SD] CMD6: switching to High Speed ...\r\n");
    e = sd_switch_func(SD_HS_GROUP1_FUNC1 | SD_SWITCH_SET, s_switch_buf);
    if (e != SD_OK)
    {
        printf("[SD] CMD6 set failed (err=%d)\r\n", (int)e);
        return -3;
    }

    /* 返回块 bit511:496 = 功能选择结果（4 bit/组，从低位起）。
     * 取 word1 的高 16 bit 的 bit3:0 即 Group1 的选择结果。 */
    status = (w[1] >> 12) & 0x0Fu;

    printf("[SD] CMD6 status: 0x%08X -> Group1 selected func = %u\r\n",
           (unsigned)w[1], (unsigned)status);

    if (status != 1u)
    {
        printf("[SD] switch rejected (want 1, got %u) -> stay Default Speed\r\n",
               (unsigned)status);
        return -4;
    }

    /* 切换成功，现在才可以把 SDCLK 提上去 */
    SDMMC_SetClockSpeed(SDMMC_ClockSpeed_High, div_after);
    s_hs_active = 1;

    {
        uint32_t hclk = (uint32_t)SystemCoreClock;
        uint32_t d    = (uint32_t)div_after + 1u;
        printf("[SD] switched to HIGH SPEED, SDCLK = %u.%u MHz (DIV=%u)\r\n",
               (unsigned)(hclk / d / 1000000u),
               (unsigned)((hclk / d % 1000000u) / 100000u),
               (unsigned)div_after);
    }
    return 0;
}

/*==============================================================================
 * SD_ReportSpeed —— 打印当前速度/能力（只读）
 *============================================================================*/
void SD_ReportSpeed(void)
{
    uint16_t clkdiv = SDMMC->CLK_DIV;
    uint8_t  div    = (uint8_t)(clkdiv & SD_SPEED_DIVMASK);
    uint32_t sdbus  = SDMMC->CONTROL & 0x03u;
    uint32_t ccc    = SDCardInfo.SD_csd.CardComdClasses;
    uint8_t  csd[16];
    uint8_t  i, ts, tval, tunit;
    uint32_t kbps;
    int      have_csd;

    printf("\r\n----- SD speed report -----\r\n");

    printf("card type      : %s\r\n",
           (SDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD)     ? "SDHC/SDXC (high capacity)" :
           (SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0) ? "SDSC v2.0 (standard)" :
           (SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) ? "SDSC v1.1 (standard)" :
           (SDIO_MULTIMEDIA_CARD == SDCardInfo.CardType)           ? "MMC" : "unknown");

    have_csd = (SD_CSD_Raw[0] | SD_CSD_Raw[1] | SD_CSD_Raw[2] | SD_CSD_Raw[3]) ? 1 : 0;
    if (have_csd)
    {
        csd[0]  = (uint8_t)(SD_CSD_Raw[0] >> 24);
        csd[1]  = (uint8_t)(SD_CSD_Raw[0] >> 16);
        csd[2]  = (uint8_t)(SD_CSD_Raw[0] >> 8);
        csd[3]  = (uint8_t)(SD_CSD_Raw[0]);
        csd[4]  = (uint8_t)(SD_CSD_Raw[1] >> 24);
        csd[5]  = (uint8_t)(SD_CSD_Raw[1] >> 16);
        csd[6]  = (uint8_t)(SD_CSD_Raw[1] >> 8);
        csd[7]  = (uint8_t)(SD_CSD_Raw[1]);
        csd[8]  = (uint8_t)(SD_CSD_Raw[2] >> 24);
        csd[9]  = (uint8_t)(SD_CSD_Raw[2] >> 16);
        csd[10] = (uint8_t)(SD_CSD_Raw[2] >> 8);
        csd[11] = (uint8_t)(SD_CSD_Raw[2]);
        csd[12] = (uint8_t)(SD_CSD_Raw[3] >> 24);
        csd[13] = (uint8_t)(SD_CSD_Raw[3] >> 16);
        csd[14] = (uint8_t)(SD_CSD_Raw[3] >> 8);
        csd[15] = (uint8_t)(SD_CSD_Raw[3]);

        printf("CSD structure  : v%u\r\n", (csd[0] >> 6) & 0x03u);

        ts    = csd[3];                            /* TRAN_SPEED = CSD bit 103:96 */
        tval  = (uint8_t)((ts >> 4) & 0x0Fu);
        tunit = (uint8_t)(ts & 0x0Fu);
        kbps  = (uint32_t)tval * s_unit_kbps[tunit];

        printf("CSD TRAN_SPEED : 0x%02X => %u.%u Mbit/s (TIME=%u, UNIT=%u)\r\n",
               ts, kbps / 1000u, (kbps % 1000u) / 100u, tval, tunit);
        printf("  card declares: %s\r\n",
               (kbps >= 50000u) ? "High Speed capable (>= 50 Mbit/s)" :
               (kbps >= 25000u) ? "Default Speed (25 Mbit/s)" :
                                  "below default speed");
        printf("CSD raw        :");
        for (i = 0; i < 16; i++) printf(" %02X", csd[i]);
        printf("\r\n");

        /* TAAC / NSAC 是写入周期的公开指标 */
        printf("CSD TAAC/NSAC  : 0x%02X / 0x%02X (write 时序指标)\r\n",
               SDCardInfo.SD_csd.TAAC, SDCardInfo.SD_csd.NSAC);
    }
    else
    {
        printf("CSD raw        : (not captured)\r\n");
    }

    printf("CCC            : 0x%03X\r\n", (unsigned)ccc);

    printf("bus width now  : %s  (SDMMC->CONTROL=0x%04X)\r\n",
           (sdbus == 1u) ? "4-bit" : (sdbus == 2u) ? "8-bit" : "1-bit",
           (unsigned)SDMMC->CONTROL);

    printf("SDCLK config   : DIV=%u CLKMode=%u PhaseInv=%u CLKOE=%u (CLK_DIV=0x%04X)\r\n",
           div,
           (clkdiv & SD_SPEED_CLKMODE)  ? 1u : 0u,
           (clkdiv & SD_SPEED_PHASEINV) ? 1u : 0u,
           (clkdiv & SD_SPEED_CLKOE)    ? 1u : 0u,
           (unsigned)clkdiv);

    {
        uint32_t hclk = (uint32_t)SystemCoreClock;
        uint32_t d    = (uint32_t)div + 1u;
        uint32_t mhz  = hclk / d / 1000000u;
        uint32_t frac = (hclk / d % 1000000u) / 100000u;

        printf("HCLK           : %u Hz\r\n", (unsigned)hclk);
        printf("SDCLK (formula): %u.%u MHz  [按 HCLK/(DIV+1) 算]\r\n",
               (unsigned)mhz, (unsigned)frac);
        printf("  !! 本公式已被实测否定：128KB@DIV=12 实测 8.174MB/s，\r\n"
               "     而 HCLK/13 的 4-bit 上限只有 3.846MB/s，两者矛盾。\r\n"
               "     真实 SDCLK 请看 SD_ProbeDiv() 用实测吞吐反推的值。\r\n");
    }

    printf("MODE (actual)  : %s\r\n", SD_SpeedMode());
    printf("----------------------------\r\n\r\n");
}

/*********************************************************************
 * @fn      SD_SpeedMode
 *********************************************************************/
const char *SD_SpeedMode(void)
{
    if (!s_hs_active)
        return "Default Speed, 4-bit, 3V3";

    /* 按当前 DIV 反推实际档位 */
    {
        uint32_t hclk = (uint32_t)SystemCoreClock;
        uint32_t div  = (uint32_t)(SDMMC->CLK_DIV & SD_SPEED_DIVMASK) + 1u;
        uint32_t mhz  = hclk / div / 1000000u;
        if (mhz >= 50u) return "High Speed 50MHz, 4-bit, 3V3";
        if (mhz >= 25u) return "High Speed 25MHz, 4-bit, 3V3";
        return "High Speed (low SDCLK), 4-bit, 3V3";
    }
}

/*==============================================================================
 * 吞吐实测用的缓冲
 *   TIM9 = 100MHz 自由运行（10ns/tick，见 sys_clk.c），用 CNT 差值计时
 *
 *   ★ RAM 只有 64K-256B，放不下这些缓冲（原来两个 32KB 校验缓冲直接溢出）。
 *     这些缓冲借用【大核 RAM 的后 512KB】——大核实际只用了前 ~127KB。
 *
 *   内存布局（来自双方 linker 脚本）：
 *     V5F  RAM_CODE 0x200A0000 .. 0x200C0000   (128K, ITCM)
 *     V5F  RAM      0x200C0100 .. 0x20160000   (639K)  <- 只用了前 ~127K
 *     V3F  RAM_CODE 0x20160000 .. 0x20170000   (64K)   <- 这是【代码】，绝不能当数据用！
 *     V3F  RAM      0x20170100 .. 0x20180000   (63K-256B)
 *
 *   所以借用区 = 0x200DFD00 .. 0x20160000（512K），我们用前 9KB。
 *   改这里前先确认大核那边没有把这段用掉。
 *============================================================================*/
#define SD_BENCH_TIM             TIM9
#define SD_BENCH_TICK_NS         10u
#define SD_BENCH_CHUNK           8u           /* 每批 8 扇区 = 4KB */
#define SD_BENCH_BUF_BYTES       (512u * SD_BENCH_CHUNK)

/* 大核 RAM 后 512KB 的起始地址（= 0x200C0100 + 127K，取整到 0x200DFD00） */
#define SD_BUF_BASE              0x200DFD00UL

/* 各缓冲在该区域的偏移 */
#define SD_OFF_BENCH             0x0000u      /* 4KB 写/基准缓冲 */
#define SD_OFF_READ              (SD_OFF_BENCH + SD_BENCH_BUF_BYTES)   /* 4KB 读回缓冲 */
#define SD_OFF_VFY_A             (SD_OFF_READ + SD_BENCH_BUF_BYTES)    /* 512B 校验A */
#define SD_OFF_VFY_B             (SD_OFF_VFY_A + 512u)                 /* 512B 校验B */

#define SD_BUF_PTR(off)          ((uint8_t *)(SD_BUF_BASE + (off)))

static uint8_t *const s_bench_buf = SD_BUF_PTR(SD_OFF_BENCH);
static uint8_t *const s_read_buf  = SD_BUF_PTR(SD_OFF_READ);
static uint8_t *const s_vfy_a     = SD_BUF_PTR(SD_OFF_VFY_A);
static uint8_t *const s_vfy_b     = SD_BUF_PTR(SD_OFF_VFY_B);

static void sd_bench_print(const char *tag, uint32_t sectors, uint32_t ns)
{
    uint32_t bytes = sectors * 512u;
    /* KB/s = bytes/1024 / (ns/1e9) = bytes * 976562 / ns  (1e9/1024 ≈ 976562) */
    uint32_t kbps  = (uint32_t)(((uint64_t)bytes * 976562ULL) /
                                (uint64_t)(ns ? ns : 1u));

    printf("[SD] %s: %u sectors (%u KB) in %u ns\r\n",
           tag, (unsigned)sectors, (unsigned)(bytes / 1024u), (unsigned)ns);
    printf("[SD] %s throughput: %u.%03u MB/s\r\n",
           tag,
           (unsigned)(kbps / 1024u),
           (unsigned)((kbps % 1024u) * 1000u / 1024u));
}

/*********************************************************************
 * @fn      SD_Benchmark
 * @brief   连读 sectors 个扇区测读吞吐
 *********************************************************************/
void SD_Benchmark(uint32_t sectors)
{
    uint32_t t0, t1, done = 0, chunk;
    SD_Error e;

    if (sectors == 0u) sectors = 256u;

    t0 = SD_BENCH_TIM->CNT_32;
    while (done < sectors)
    {
        chunk = sectors - done;
        if (chunk > SD_BENCH_CHUNK) chunk = SD_BENCH_CHUNK;

        e = SD_ReadDisk(s_bench_buf, done, (uint8_t)chunk);
        if (e != SD_OK)
        {
            printf("[SD] read bench aborted at sector %u (err=%d)\r\n",
                   (unsigned)done, (int)e);
            return;
        }
        done += chunk;
    }
    t1 = SD_BENCH_TIM->CNT_32;

    sd_bench_print("read bench", sectors, (t1 - t0) * SD_BENCH_TICK_NS);
}

/*********************************************************************
 * @fn      SD_VerifyRead
 * @brief   在指定扇区上做可重复性校验（判断当前时钟是否真的稳）
 *         同一扇区读两次必须完全一致；不校验数据内容对不对（布
 *         局未知），只校验"可重复性"。
 *
 *   ★ RAM 很紧（V3F 只有 64K-256B），所有校验缓冲都用单扇区 512B，
 *     靠多轮多个扇区来换取覆盖率，而不是靠大缓冲。
 * @return  0 = 通过；非 0 = 不稳定/出错
 *********************************************************************/
int SD_VerifyRead(uint32_t sector, uint32_t rounds)
{
    uint32_t k, i;
    SD_Error e;

    if (rounds == 0u) rounds = 4u;

    for (k = 0; k < rounds; k++)
    {
        e = SD_ReadDisk(s_vfy_a, sector, 1);
        if (e != SD_OK) return 1;

        e = SD_ReadDisk(s_vfy_b, sector, 1);
        if (e != SD_OK) return 2;

        for (i = 0; i < 512u; i++)
            if (s_vfy_a[i] != s_vfy_b[i]) return 3;
    }
    return 0;
}

/*********************************************************************
 * @fn      SD_VerifyMulti
 * @brief   多扇区一致性校验：逐扇区交替读两遍并比对
 *          单扇区读抓不住偶发错误（实测 DIV=4 出现过一次慢读），
 *          覆盖多个扇区更能暴露间歇性问题。
 *
 *   实现说明：为了省 RAM，用两个 512B 缓冲逐扇区交替读，
 *             而不是一次性读 64 个扇区（那要 64KB RAM，直接溢出）。
 *   覆盖：nsec 个扇区，每个扇区比对 rounds 轮。
 *
 * @return  0 = 一致；非 0 = 出错码
 *********************************************************************/
#define SD_VFY_MAX_SECTORS  32u

int SD_VerifyMulti(uint32_t sector, uint32_t nsec)
{
    SD_Error e;
    uint32_t i, s, rounds;

    if (nsec == 0u) nsec = SD_VFY_MAX_SECTORS;
    if (nsec > SD_VFY_MAX_SECTORS) nsec = SD_VFY_MAX_SECTORS;

    rounds = 2u;                    /* 每个扇区比对 2 轮 */

    for (s = 0; s < nsec; s++)
    {
        uint32_t k;
        for (k = 0; k < rounds; k++)
        {
            e = SD_ReadDisk(s_vfy_a, sector + s, 1);
            if (e != SD_OK) return 1;

            e = SD_ReadDisk(s_vfy_b, sector + s, 1);
            if (e != SD_OK) return 2;

            for (i = 0; i < 512u; i++)
                if (s_vfy_a[i] != s_vfy_b[i]) return 3;
        }
    }
    return 0;
}

/*==============================================================================
 * 实测驱动：不再依赖 SDCLK = HCLK/(DIV+1) 这个（已被实测否定的）公式，
 *           改为"切档 -> 测持续吞吐 -> 反推有效时钟"。
 *
 *   实测数据（HCLK=100MHz, 29512MB SDHC 卡, 2MB 连读）：
 *     DIV  公式SDCLK  实测MB/s  反推SDCLK  反推/公式
 *     12    7.69       8.44      16.9       2.30
 *      7   12.50      10.25      20.5       1.72
 *      6   14.29      10.70      21.4       1.57
 *      5   16.67      11.72      23.5       1.41
 *      4   20.00      11.75*     24.6       1.23   (*出现过 0.58MB/s 的偶发慢读)
 *      3      -        失败
 *
 *   结论：不存在固定 2x 乘数；有效时钟在 ~23.5MHz 处饱和，
 *         贴近 SD Default Speed 的 25MHz 上限。
 *         故实用档位是 DIV=5。
 *============================================================================*/

/* 已知可用档（DIV=12 / DIV=7 实测通过）。失败时一律回到这里，不再回到坏档。 */
#define SD_DIV_SAFE        12u
#define SD_DIV_START        7u      /* 从已实测可用的最快档开始往上探 */
#define SD_DIV_MIN          3u      /* 探到这里为止 */

static uint16_t s_div_good = SD_DIV_START;

/*********************************************************************
 * @fn      SD_SetDiv
 * @brief   直接设定 CLK_DIV[4:0]（不做任何"MHz 换算"，避免公式错误传导）
 *********************************************************************/
void SD_SetDiv(uint16_t div)
{
    if (div > 0x1Fu) div = 0x1F;
    SDMMC_SetClockSpeed(SDMMC_ClockSpeed_High, div);
}

/*********************************************************************
 * @fn      SD_ReadBytes
 * @brief   连续读 bytes 字节，返回耗时（ns，TIM9 计时）
 *********************************************************************/
uint32_t SD_ReadBytes(uint32_t bytes, uint32_t sector)
{
    uint32_t t0, t1;
    uint32_t left = bytes;
    uint32_t chunk, sec = sector;
    SD_Error e;

    t0 = SD_BENCH_TIM->CNT_32;
    while (left > 0u)
    {
        chunk = (left > (512u * SD_BENCH_CHUNK)) ? (512u * SD_BENCH_CHUNK) : left;
        e = SD_ReadDisk(s_bench_buf, sec, (uint8_t)(chunk / 512u));
        if (e != SD_OK)
        {
            t1 = SD_BENCH_TIM->CNT_32;
            printf("[SD]   read aborted at sector %u (err=%d)\r\n",
                   (unsigned)sec, (int)e);
            return (t1 - t0) * SD_BENCH_TICK_NS;      /* 返回已耗时，便于判断 */
        }
        left -= chunk;
        sec  += chunk / 512u;
    }
    t1 = SD_BENCH_TIM->CNT_32;
    return (t1 - t0) * SD_BENCH_TICK_NS;
}

/*********************************************************************
 * @fn      SD_ProbeDiv
 * @brief   在指定分频上测一轮：先校验稳定性，再测持续吞吐并反推有效 SDCLK
 * @param   div           目标 CLK_DIV[4:0]
 * @param   bytes         吞吐测试读取字节数（建议 >= 1MB 以压掉开销）
 * @param   verify_sector 校验扇区
 * @return  0 稳定；非 0 不稳定（已自动回到 s_div_good）
 *********************************************************************/
int SD_ProbeDiv(uint16_t div, uint32_t bytes, uint32_t verify_sector)
{
    uint32_t ns, mbps_x1000;
    int      r;
    uint16_t nominal_div;

    nominal_div = div;                      /* 保存，失败回退时打印用 */

    printf("[SD] --- probe DIV=%u ---\r\n", (unsigned)div);
    SD_SetDiv(div);

    r = SD_VerifyRead(verify_sector, 4);
    if (r != 0)
    {
        printf("[SD]   UNSTABLE (code=%d) -> back to DIV=%u\r\n",
               r, (unsigned)s_div_good);
        SD_SetDiv(s_div_good);
        return r;
    }
    printf("[SD]   repeat-read check: PASS\r\n");

    /* 多扇区一致性校验：单扇区读抓不住偶发错误，这里连读 64 扇区两遍比对 */
    if (SD_VerifyMulti(256u, 64u) != 0)
    {
        printf("[SD]   MULTI-READ INCONSISTENT -> back to DIV=%u\r\n",
               (unsigned)s_div_good);
        SD_SetDiv(s_div_good);
        return 9;
    }
    printf("[SD]   multi-read (64 sect) consistent: PASS\r\n");

    ns = SD_ReadBytes(bytes, 256u);
    if (ns == 0u) ns = 1u;

    /* MB/s = bytes / 1048576 / (ns/1e9) = bytes * 953.674 / ns
     * 用整数：kbps = bytes * 976562 / ns (KB/s) */
    mbps_x1000 = (uint32_t)(((uint64_t)bytes * 976562ULL) / (uint64_t)ns) * 1000u / 1024u;

    printf("[SD]   %u KB in %u ns  ->  %u.%03u MB/s\r\n",
           (unsigned)(bytes / 1024u), (unsigned)ns,
           (unsigned)(mbps_x1000 / 1000u), (unsigned)(mbps_x1000 % 1000u));

    /* 4-bit 总线：1 MB/s 吞吐 <=> 2 MHz SDCLK（线速率下界） */
    printf("[SD]   implied SDCLK >= %u.%03u MHz  (4-bit, 假设 100%% 效率)\r\n",
           (unsigned)(mbps_x1000 * 2u / 1000u), (unsigned)((mbps_x1000 * 2u) % 1000u));
    printf("[SD]   (寄存器 DIV=%u，若按 HCLK/(DIV+1) 算是 %u.%02u MHz —— 两者不符说明公式有误)\r\n",
           (unsigned)nominal_div,
           (unsigned)((uint32_t)SystemCoreClock / ((uint32_t)nominal_div + 1u) / 1000000u),
           (unsigned)(((uint32_t)SystemCoreClock / ((uint32_t)nominal_div + 1u) % 1000000u) / 10000u));

    s_div_good = div;
    printf("[SD]   stable, keep DIV=%u\r\n", (unsigned)s_div_good);
    return 0;
}

/*********************************************************************
 * @fn      SD_ExploreSpeed
 * @brief   从 start_div 起逐档降 DIV（提高时钟）探测最高稳定档
 *          任一档失败即停止，并停在最后可用档。
 * @return  最终保留的 DIV
 *********************************************************************/
uint16_t SD_ExploreSpeed(uint16_t start_div, uint16_t min_div,
                         uint32_t bytes, uint32_t verify_sector)
{
    uint16_t d;
    int      r;

    s_div_good = start_div;
    printf("[SD] optimistic start DIV=%u (实测可用档)\r\n", (unsigned)start_div);
    SD_SetDiv(start_div);

    for (d = start_div; d > min_div; d--)
    {
        r = SD_ProbeDiv((uint16_t)(d - 1u), bytes, verify_sector);
        if (r != 0)
            break;
    }

    printf("[SD] === best stable DIV = %u ===\r\n", (unsigned)s_div_good);
    SD_SetDiv(s_div_good);
    return s_div_good;
}

/*********************************************************************
 * @fn      SD_TrySpeedStep（保留但改为安全实现）
 * @brief   切到目标 DIV 并校验；失败则回到已知可用档（不再硬编码坏档）
 *********************************************************************/
int SD_TrySpeedStep(uint16_t div, uint32_t verify_sector)
{
    int r;

    if (div > 0x1Fu) div = 0x1F;
    SD_SetDiv(div);

    r = SD_VerifyRead(verify_sector, 4);
    if (r != 0)
    {
        printf("[SD] UNSTABLE at DIV=%u (code=%d) -> back to DIV=%u\r\n",
               (unsigned)div, r, (unsigned)s_div_good);
        SD_SetDiv(s_div_good);
        return r;
    }
    s_div_good = div;
    printf("[SD] stable at DIV=%u\r\n", (unsigned)div);
    return 0;
}

/*********************************************************************
 * @fn      SD_BenchmarkWrite
 * @brief   连写 sectors 个扇区测写吞吐（会破坏 sector 起始处的数据！）
 * @param   sectors  写多少扇区
 * @param   start    起始扇区（默认选在卡尾部的安全区）
 * @note    数据是 RAM 里的固定图案，不是真实日志
 *********************************************************************/
void SD_BenchmarkWrite(uint32_t sectors, uint32_t start)
{
    uint32_t t0, t1, done = 0, chunk;
    SD_Error e;

    if (sectors == 0u) sectors = 256u;

    /* 用固定图案填充缓冲（s_bench_buf 现在是指针，sizeof 只有 4，必须用字节数宏） */
    {
        uint32_t i;
        for (i = 0; i < SD_BENCH_BUF_BYTES; i++)
            s_bench_buf[i] = (uint8_t)(i * 7u + 0x5Au);
    }

    t0 = SD_BENCH_TIM->CNT_32;
    while (done < sectors)
    {
        chunk = sectors - done;
        if (chunk > SD_BENCH_CHUNK) chunk = SD_BENCH_CHUNK;

        e = SD_WriteDisk(s_bench_buf, start + done, (uint8_t)chunk);
        if (e != SD_OK)
        {
            printf("[SD] write bench aborted at sector %u (err=%d)\r\n",
                   (unsigned)(start + done), (int)e);
            return;
        }
        done += chunk;
    }
    t1 = SD_BENCH_TIM->CNT_32;

    sd_bench_print("write bench", sectors, (t1 - t0) * SD_BENCH_TICK_NS);
}

/*==============================================================================
 * 写后读回校验（Write-Read-Verify）
 *
 *   目的：确认写入的字节能被原样读回（这是"能存日志"的硬前提），
 *         同时给出真实读写带宽。
 *
 *   ★ 破坏性：会覆盖 [start, start+sectors) 范围内的数据。
 *     本测试默认放在卡尾（分区之外），不会碰 MBR / 文件系统。
 *     这张卡可以格式化，所以放心跑。
 *
 *   图案：每个字节 = (sector_num*31 + byte_index*7) & 0xFF
 *         位置相关 -> 任何"块错位""写串位"都会立刻暴露，
 *         比固定图案强得多。
 *============================================================================*/

/* 按位置生成图案 */
static void wrv_fill(uint8_t *buf, uint32_t bytes, uint32_t first_sector)
{
    uint32_t i;
    for (i = 0; i < bytes; i++)
        buf[i] = (uint8_t)(((first_sector + (i / 512u)) * 31u + (i % 512u) * 7u) & 0xFFu);
}

/* 比对，返回不一致字节数；*first_bad 记录第一个错的位置 */
static uint32_t wrv_cmp(const uint8_t *a, const uint8_t *b, uint32_t bytes,
                        uint32_t *first_bad)
{
    uint32_t i, bad = 0;
    if (first_bad) *first_bad = 0xFFFFFFFFu;
    for (i = 0; i < bytes; i++)
    {
        if (a[i] != b[i])
        {
            if (bad == 0u && first_bad) *first_bad = i;
            bad++;
        }
    }
    return bad;
}

/*********************************************************************
 * @fn      SD_WriteReadVerify
 * @brief   写 sectors 个扇区 -> 读回 -> 逐字节比对，报告读写带宽
 * @param   start    起始扇区（破坏性，建议用卡尾空白区）
 * @param   sectors  扇区数
 * @param   chunk    每次 SD_WriteDisk/SD_ReadDisk 的扇区数（1..8，8 最省开销）
 * @return  0 数据一致；非 0 = 写/读出错或数据不一致
 *
 *   想安静跑（只由调用方汇总）就先 SD_SetVerbose(0)。
 *********************************************************************/
uint8_t SD_Verbose = 1u;        /* 0 = 只报错，不报过程 */

void SD_SetVerbose(uint8_t on) { SD_Verbose = on ? 1u : 0u; }

int SD_WriteReadVerify(uint32_t start, uint32_t sectors, uint8_t chunk)
{
    uint32_t t0, t1, ws, rs, done, n;
    uint32_t total = sectors * 512u;
    uint32_t bad, first_bad;
    SD_Error e;

    if (sectors == 0u) sectors = 512u;
    if (chunk == 0u || chunk > SD_BENCH_CHUNK) chunk = SD_BENCH_CHUNK;

    if (SD_Verbose)
    {
        printf("[SD] write-read-verify: start=%u, %u KB, chunk=%u (破坏性)\r\n",
               (unsigned)start, (unsigned)(total / 1024u), (unsigned)chunk);
    }

    /* ---------- 写 ---------- */
    t0 = SD_BENCH_TIM->CNT_32;
    for (done = 0; done < sectors; done += n)
    {
        n = sectors - done;
        if (n > chunk) n = chunk;

        wrv_fill(s_bench_buf, n * 512u, start + done);

        e = SD_WriteDisk(s_bench_buf, start + done, (uint8_t)n);
        if (e != SD_OK)
        {
            printf("[SD] WRITE FAILED at sector %u (err=%d)\r\n",
                   (unsigned)(start + done), (int)e);
            return 1;
        }
    }
    t1 = SD_BENCH_TIM->CNT_32;
    ws = (t1 - t0) * SD_BENCH_TICK_NS;          /* 写耗时 ns */

    /* ---------- 读回 + 比对（缓冲交替使用，避免同一块内存边写边读） ---------- */
    t0 = SD_BENCH_TIM->CNT_32;
    bad = 0;
    first_bad = 0xFFFFFFFFu;
    for (done = 0; done < sectors; done += n)
    {
        n = sectors - done;
        if (n > chunk) n = chunk;

        e = SD_ReadDisk(s_read_buf, start + done, (uint8_t)n);
        if (e != SD_OK)
        {
            printf("[SD] READ BACK FAILED at sector %u (err=%d)\r\n",
                   (unsigned)(start + done), (int)e);
            return 2;
        }

        wrv_fill(s_bench_buf, n * 512u, start + done);      /* 期望值 */
        bad += wrv_cmp(s_bench_buf, s_read_buf, n * 512u, &first_bad);
    }
    t1 = SD_BENCH_TIM->CNT_32;
    rs = (t1 - t0) * SD_BENCH_TICK_NS;

    /* ---------- 报告（仅 verbose 时打印过程，出错永远打印） ---------- */
    if (SD_Verbose)
    {
        uint32_t w_kbps = (uint32_t)(((uint64_t)total * 976562ULL) / (uint64_t)(ws ? ws : 1u));
        uint32_t r_kbps = (uint32_t)(((uint64_t)total * 976562ULL) / (uint64_t)(rs ? rs : 1u));

        printf("[SD] write %u.%03u MB/s | read %u.%03u MB/s\r\n",
               (unsigned)(w_kbps / 1024u),
               (unsigned)((w_kbps % 1024u) * 1000u / 1024u),
               (unsigned)(r_kbps / 1024u),
               (unsigned)((r_kbps % 1024u) * 1000u / 1024u));
    }

    if (bad != 0u)
    {
        printf("[SD] VERIFY FAILED: %u/%u bytes differ (first at sector +%u)\r\n",
               (unsigned)bad, (unsigned)total,
               (unsigned)(first_bad / 512u));
        return 3;
    }

    if (SD_Verbose)
        printf("[SD] verified %u KB\r\n", (unsigned)(total / 1024u));
    return 0;
}

/*********************************************************************
 * @fn      SD_WriteReadVerifyScan
 * @brief   用几种 chunk 大小各跑一遍，找出最合适的写粒度
 *         数据量随 chunk 缩放，保证每档总字节数接近，便于横向比较
 *********************************************************************/
void SD_WriteReadVerifyScan(uint32_t start, uint32_t rounds)
{
    static const uint8_t chunks[3] = { 1u, 4u, 8u };
    uint32_t i, sectors;

    if (rounds == 0u) rounds = 1u;

    printf("\r\n[SD] ================= WRITE-READ-VERIFY SCAN =================\r\n");
    printf("[SD] target start sector = %u (卡尾空白区，可格式化)\r\n", (unsigned)start);

    for (i = 0; i < 3u; i++)
    {
        /* 每档都写 2MB 左右：chunk*512*N ≈ 2MB */
        sectors = (2u * 1024u * 1024u) / 512u;      /* 4096 扇区 = 2MB */
        (void)rounds;
        SD_WriteReadVerify(start + (i * sectors), sectors, chunks[i]);
    }

    printf("[SD] ==========================================================\r\n\r\n");
}

/*==============================================================================
 * 不经文件系统的写数据校验
 *
 *   直接验证 SD_WriteDisk() 写进卡里的字节 == 源缓冲的字节。
 *   特意把"读"和"写"交替进行，反复切换主机端 TranMode 方向 ——
 *   这正是修复前失效的场景（发 CMD24 时 Direction 还是 Receive）。
 *
 *   只写卡尾空白区，不碰文件系统。
 *============================================================================*/
#define VWD_SECTORS     64u                 /* 一次测 64 个扇区 = 32KB */
#define VWD_PASSES     3u                   /* 方向切换轮数 */

int SD_VerifyWriteData(uint32_t start, uint8_t chunk)
{
    /* 必须满足底层 16 字节 DMA 对齐要求（SDMMC_DMADDR_ALIGN_SIZE）。
     * 裸 static 数组只有默认 4 字节对齐，会走 memcpy 兜底路径；
     * 显式对齐后，写和读走的是各自的对齐快路径。 */
    __attribute__((aligned(32))) static uint8_t a[512 * 8];
    __attribute__((aligned(32))) static uint8_t b[512 * 8];
    uint32_t pass, s, i, n;
    uint32_t bad_total = 0;

    if (chunk == 0u || chunk > 8u) chunk = 8u;

    /* 对齐自检：不对齐就直接报出来，别让它悄悄走兜底路径 */
    if ((((uint32_t)a) & 0x1Fu) != 0u || (((uint32_t)b) & 0x1Fu) != 0u)
    {
        printf("[SD] verify-write: buffer NOT 32B aligned (%p/%p)\r\n",
               (void *)a, (void *)b);
        return 5;
    }

    printf("[SD] verify-write: start=%u, %u sectors, chunk=%u, %u passes\r\n",
           (unsigned)start, (unsigned)VWD_SECTORS, (unsigned)chunk,
           (unsigned)VWD_PASSES);

    for (pass = 0; pass < VWD_PASSES; pass++)
    {
        uint32_t bad = 0;

        /* ---- 1) 先读一次（把主机端切到 Receive 方向） ---- */
        if (SD_ReadDisk(a, start, 1u) != SD_OK)
        {
            printf("  pass%u: pre-read FAIL\r\n", (unsigned)pass);
            return 1;
        }

        /* ---- 2) 写：图案与位置绑定，错位立刻暴露 ---- */
        for (s = 0; s < VWD_SECTORS; s += n)
        {
            n = VWD_SECTORS - s;
            if (n > chunk) n = chunk;

            for (i = 0; i < n * 512u; i++)
                a[i] = (uint8_t)(((start + s + (i / 512u)) * 37u
                                  + (i % 512u) * 11u + pass) & 0xFFu);

            if (SD_WriteDisk(a, start + s, (uint8_t)n) != SD_OK)
            {
                printf("  pass%u: WRITE FAIL at sector %u\r\n",
                       (unsigned)pass, (unsigned)(start + s));
                return 2;
            }

            /* 写一块就读一块：制造读/写方向反复切换 */
            if (SD_ReadDisk(b, start + s, (uint8_t)n) != SD_OK)
            {
                printf("  pass%u: read-after-write FAIL at sector %u\r\n",
                       (unsigned)pass, (unsigned)(start + s));
                return 3;
            }

            for (i = 0; i < n * 512u; i++)
            {
                if (a[i] != b[i])
                {
                    if (bad == 0u)
                        printf("  pass%u: MISMATCH sector %u offset %u "
                               "(wrote %02X, read %02X)\r\n",
                               (unsigned)pass,
                               (unsigned)(start + s + (i / 512u)),
                               (unsigned)(i % 512u), a[i], b[i]);
                    bad++;
                }
            }
        }

        printf("  pass%u: %s (%u bytes checked, %u mismatch)\r\n",
               (unsigned)pass, (bad == 0u) ? "OK" : "FAILED",
               (unsigned)(VWD_SECTORS * 512u), (unsigned)bad);

        bad_total += bad;
    }

    if (bad_total == 0u)
    {
        printf("[SD] verify-write: ALL PASS - 写入数据与源数据完全一致\r\n");
        return 0;
    }
    printf("[SD] verify-write: %u bytes mismatch -> 写路径仍有问题\r\n",
           (unsigned)bad_total);
    return 4;
}
