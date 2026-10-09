/*******************************************************************************
 * File Name        : lsm6dsv16x.c
 * Description      : LSM6DSV16XTR 六轴 IMU 驱动（SPI2，片选 PD10）
 *
 * 器件行为与规格依据见 lsm6dsv16x.h 头部说明。
 * 寄存器位定义见 lsm6dsv16x_reg.h。
 *
 * 本文件只保留正式路径，不含临时调试脚手架。
 *******************************************************************************/

#include "lsm6dsv16x.h"
#include "lsm6dsv16x_reg.h"
#include "sensor_spi.h"
#include "board_io.h"
#include "sys_clk.h"
#include <stdio.h>

/* SPI 器件句柄（SPI2 + 软件片选 PD10，网表 PD10 ; U1.33 U4.12） */
#define LSM6_SPI_HANDLE()   IMU_SPI, IMU_CS_PORT, IMU_CS_PIN

/*==============================================================================
 * 底层寄存器访问
 *============================================================================*/
static int lsm6_rd(uint8_t reg, uint8_t *v)
{
    return SPI_ReadReg(LSM6_SPI_HANDLE(), reg, v);
}

static int lsm6_wr(uint8_t reg, uint8_t v)
{
    return SPI_WriteReg(LSM6_SPI_HANDLE(), reg, v);
}

/* LSM6_Init() / LSM6_Enable() 已移到 board_io.c 的 9.13 节（板级配置库）：
 * 那两个函数不依赖任何全局/静态量，两核都能直接调用，由消费方（大核）自己使能。 */

/*==============================================================================
 * 读一个新鲜陀螺样本
 *
 * 手册指定流程：轮询 STATUS_REG 的 GDA 位 -> 连读 6 字节 -> 无需清标志
 * （LSM6DSV16X 的 DRDY 是状态位，不是需要写 1 清除的中断标志；
 *  默认锁存模式下，读完对应输出寄存器的高字节后自动回 0）。
 *============================================================================*/
int LSM6_ReadGyroRaw(LSM6_GyroRaw *out)
{
    uint64_t t0;
    uint8_t  st = 0u;
    uint8_t  d[LSM6DSV16X_GYRO_BURST_LEN];

    if (out == 0) return LSM6_ERR_PARAM;

    t0 = GetTime64_Us();
    for (;;)
    {
        if (lsm6_rd(LSM6DSV16X_REG_STATUS, &st) != 0) return LSM6_ERR_SPI;
        if (st & LSM6DSV16X_ST_GDA) break;
        if ((GetTime64_Us() - t0) > LSM6_DRDY_TIMEOUT_US)
            return LSM6_ERR_DRDY_TIMEOUT;
    }

    if (SPI_ReadRegs(LSM6_SPI_HANDLE(), LSM6DSV16X_REG_OUTX_L_G,
                     d, LSM6DSV16X_GYRO_BURST_LEN) != 0)
        return LSM6_ERR_SPI;

    out->x = LSM6DSV16X_MAKE_I16(d[0], d[1]);
    out->y = LSM6DSV16X_MAKE_I16(d[2], d[3]);
    out->z = LSM6DSV16X_MAKE_I16(d[4], d[5]);

    return LSM6_OK;
}

int LSM6_ReadGyroDps(LSM6_GyroDps *out)
{
    LSM6_GyroRaw r;

    if (out == 0) return LSM6_ERR_PARAM;

    if (LSM6_ReadGyroRaw(&r) != LSM6_OK) return LSM6_ERR_SPI;

    out->x = (float)r.x * LSM6_GYRO_DPS_PER_LSB;
    out->y = (float)r.y * LSM6_GYRO_DPS_PER_LSB;
    out->z = (float)r.z * LSM6_GYRO_DPS_PER_LSB;

    return LSM6_OK;
}

/*==============================================================================
 * 等陀螺新样本就绪 —— 轮询 INT1 引脚，不读 STATUS_REG.GDA。
 * 读 GDA 要多走一次 SPI 事务，时间戳会偏晚；引脚是硬件信号，直接反映就绪时刻。
 * 高有效 + 锁存模式：读走输出寄存器高字节后引脚自动回低，无需清标志。
 *============================================================================*/
int LSM6_WaitGyroDrdy(void)
{
    uint64_t t0 = GetTime64_Us();

    while (IMU_INT1_READ() == 0u)
    {
        if ((GetTime64_Us() - t0) > LSM6_DRDY_TIMEOUT_US)
            return LSM6_ERR_DRDY_TIMEOUT;
    }
    return LSM6_OK;
}

/*==============================================================================
 * 温度
 *============================================================================*/
int LSM6_ReadTempRaw(int16_t *out)
{
    uint8_t d[2];

    if (out == 0) return LSM6_ERR_PARAM;

    if (SPI_ReadRegs(LSM6_SPI_HANDLE(), LSM6DSV16X_REG_OUT_TEMP_L, d, 2u) != 0)
        return LSM6_ERR_SPI;

    *out = LSM6DSV16X_MAKE_I16(d[0], d[1]);
    return LSM6_OK;
}

/*==============================================================================
 * 读整个输出块：OUT_TEMP_L(0x20) 起 14 字节 = 温度2 + 陀螺6 + 加速度计6
 *
 * 一次片选完成。地址 0x20~0x2D 连续无缺口（0x2E 起是 EIS/OIS 的寄存器），
 * IF_INC=1 已使能，因此这一条事务就把温度与六轴全部取回。
 *
 * buf 需至少 LSM6DSV16X_BLOCK_LEN(14) 字节。
 * 返回 LSM6_OK 或负错误码。
 *============================================================================*/
int LSM6_ReadBlock(uint8_t *buf)
{
    if (buf == 0) return LSM6_ERR_PARAM;

    if (SPI_ReadRegs(LSM6_SPI_HANDLE(), LSM6DSV16X_REG_OUT_TEMP_L_BLK,
                     buf, LSM6DSV16X_BLOCK_LEN) != 0)
        return LSM6_ERR_SPI;

    return LSM6_OK;
}

/*==============================================================================
 * 读整个输出块（结构化版本），便于直接使用而不必自己解析字节偏移
 *============================================================================*/
int LSM6_ReadAll(LSM6_AllData *out)
{
    uint8_t d[LSM6DSV16X_BLOCK_LEN];
    int rc;

    if (out == 0) return LSM6_ERR_PARAM;

    rc = LSM6_ReadBlock(d);
    if (rc != LSM6_OK) return rc;

    out->temp_raw = LSM6DSV16X_MAKE_I16(d[LSM6DSV16X_BLK_OFF_TEMP + 0u],
                                        d[LSM6DSV16X_BLK_OFF_TEMP + 1u]);
    out->gx = LSM6DSV16X_MAKE_I16(d[LSM6DSV16X_BLK_OFF_GX + 0u],
                                  d[LSM6DSV16X_BLK_OFF_GX + 1u]);
    out->gy = LSM6DSV16X_MAKE_I16(d[LSM6DSV16X_BLK_OFF_GY + 0u],
                                  d[LSM6DSV16X_BLK_OFF_GY + 1u]);
    out->gz = LSM6DSV16X_MAKE_I16(d[LSM6DSV16X_BLK_OFF_GZ + 0u],
                                  d[LSM6DSV16X_BLK_OFF_GZ + 1u]);
    out->ax = LSM6DSV16X_MAKE_I16(d[LSM6DSV16X_BLK_OFF_AX + 0u],
                                  d[LSM6DSV16X_BLK_OFF_AX + 1u]);
    out->ay = LSM6DSV16X_MAKE_I16(d[LSM6DSV16X_BLK_OFF_AY + 0u],
                                  d[LSM6DSV16X_BLK_OFF_AY + 1u]);
    out->az = LSM6DSV16X_MAKE_I16(d[LSM6DSV16X_BLK_OFF_AZ + 0u],
                                  d[LSM6DSV16X_BLK_OFF_AZ + 1u]);

    return LSM6_OK;
}

/* 温度换算：raw/256 + 25  (°C) */
float LSM6_TempToC(int16_t raw)
{
    return ((float)raw / LSM6DSV16X_TEMP_LSB_PER_C) + LSM6DSV16X_TEMP_ZERO_C;
}

/*==============================================================================
 * INT1 陀螺数据就绪中断（备用，当前测试走轮询）
 *============================================================================*/
int LSM6_Int1GyroDrdy(uint8_t enable)
{
    uint8_t v = enable ? LSM6DSV16X_INT1_DRDY_G : 0x00u;

    return (lsm6_wr(LSM6DSV16X_REG_INT1_CTRL, v) == 0) ? LSM6_OK : LSM6_ERR_SPI;
}

/*==============================================================================
 * 突发采集测试：采完再统一输出
 * 一次片选读全输出块 0x20..0x2D（温度2 + 陀螺6 + 加计6，地址连续无缺口），
 * 比分三次读省 2/3 事务开销。
 * 不边采边打印：7.68kHz 下 130us 一个样本，而一行 ASCII 约 650us。
 * 加计也在 7.68kHz —— 手册 §6.5 要求 HAODR 下两者同步，它本来就在跑。
 *============================================================================*/
#define LSM6_BURST_MAX      64u

int LSM6_TestBurst(uint32_t count)
{
    static uint8_t  s_d[LSM6_BURST_MAX][LSM6DSV16X_BLOCK_LEN];
    static uint64_t s_t[LSM6_BURST_MAX];
    uint32_t i;
    uint64_t t0, t1;
    uint32_t fail = 0u;
    uint32_t latch_fail = 0u;
    int rc;

    if (count == 0u || count > LSM6_BURST_MAX) return LSM6_ERR_PARAM;

    printf("BURST,start,n=%u,block=%u\r\n",
           (unsigned)count, (unsigned)LSM6DSV16X_BLOCK_LEN);

    t0 = GetTime64_Us();
    for (i = 0u; i < count; i++)
    {
        /* 先等 DRDY 引脚拉高（数据就绪），再一次性读回温度+六轴。
         * 必须先等：否则会在同一个样本窗口内重复读同一份数据，
         * 时间戳看着有间隔，实际不是新样本，测不出真实速率。 */
        rc = LSM6_WaitGyroDrdy();
        if (rc != LSM6_OK) fail++;

        if (LSM6_ReadBlock(s_d[i]) != LSM6_OK) fail++;

        s_t[i] = GetTime64_Us();

        /* 锁存校验：锁存模式下读走输出寄存器高字节后引脚应自动回低。
         * 若这里仍为高，说明锁存没按预期清除 —— 下一轮 WaitGyroDrdy 会
         * 立刻返回，读到的是同一份数据，测出的速率就是假的。
         * 注意要在读完之后查，读之前引脚必然是高的。 */
        if (IMU_INT1_READ() != 0u) latch_fail++;
    }
    t1 = GetTime64_Us();

    /* 采完统一上报：一行含四组，便于同一时刻对照六轴与温度 */
    for (i = 0u; i < count; i++)
    {
        const uint8_t *d = s_d[i];

        printf("REC,%u,%u,G,%d,%d,%d,A,%d,%d,%d,T,%d\r\n",
               (unsigned)i, (unsigned)(s_t[i] - t0),
               (int)LSM6DSV16X_MAKE_I16(d[2],  d[3]),      /* 陀螺 X */
               (int)LSM6DSV16X_MAKE_I16(d[4],  d[5]),      /* 陀螺 Y */
               (int)LSM6DSV16X_MAKE_I16(d[6],  d[7]),      /* 陀螺 Z */
               (int)LSM6DSV16X_MAKE_I16(d[8],  d[9]),      /* 加速度 X */
               (int)LSM6DSV16X_MAKE_I16(d[10], d[11]),     /* 加速度 Y */
               (int)LSM6DSV16X_MAKE_I16(d[12], d[13]),     /* 加速度 Z */
               (int)LSM6DSV16X_MAKE_I16(d[0],  d[1]));     /* 温度 */
    }

    printf("BURST,end,n=%u,total_us=%u,avg_gap_us=%u,fail=%u,latch_fail=%u\r\n",
           (unsigned)count,
           (unsigned)(t1 - t0),
           (unsigned)((count > 1u) ? ((t1 - t0) / (count - 1u)) : 0u),
           (unsigned)fail, (unsigned)latch_fail);

    return (fail == 0u) ? LSM6_OK : LSM6_ERR_DRDY_TIMEOUT;
}

/*==============================================================================
 * 大数据量压力测试：每秒上报一次统计量
 * V3F 只有 64KB RAM，无法缓存原始波形；压力测试要的是统计量，故边采边算。
 * 每秒一帧可以看出"随时间劣化"的趋势。上报内容：
 *   RPT,<秒>,n=,loss=,fail=,latch=   loss = 实际耗时 - 名义耗时(n/7680)，
 *                                    持续为正=丢样本，为负=重复读同一份
 *   D,<帧>,n=,min,max,mean,sd,oob=   相邻样本时间戳间隔，标称 130.2us
 *   G / A / T,<帧>,...               陀螺、加计、温度的极值与末值
 *============================================================================*/

/* 标称间隔：1/7680 Hz = 130.2us。方差以它为基准算偏差平方和，
 * 而不是先累加 Σg² 再相减 —— 后者会因两次大数整数除法截断而失真。 */
#define LSM6_GAP_NOM_US     130u

/* 间隔窗口：超出这个范围就计入 oob（严重离散的计数）*/
#define LSM6_GAP_LO_US      100u
#define LSM6_GAP_HI_US      200u

/* 饱和判据：±4000dps 下满量程约 ±28571 LSB；加速度计 ±2g 约 ±32768 LSB。
 * 用略小于满量程的值作为"接近饱和"的告警线。 */
#define LSM6_G_THRESH       28000
#define LSM6_A_THRESH       32000

int LSM6_StressTest(uint32_t seconds)
{
    uint32_t sec;
    uint32_t fail = 0u, latch_fail = 0u;
    uint64_t t_all0, t_all1;
    int rc;

    if (seconds == 0u) return LSM6_ERR_PARAM;

    printf("STRESS,start,seconds=%u,odr=7680,fs_g=4000\r\n", (unsigned)seconds);

    t_all0 = GetTime64_10Ns();

    for (sec = 0u; sec < seconds; sec++)
    {
        uint64_t t_sec0;            /* 10ns 单位；在排空遗留 DRDY 之后赋值 */
        uint64_t t_prev = 0u;
        uint32_t n = 0u, n_gap = 0u, oob = 0u, gsat = 0u, asat = 0u;
        uint64_t sum = 0u, sumd2 = 0u;
        /* 间隔极值用独立计数器，并单独统计"零间隔"次数。
         * 之前把 min 直接打印 dmin（带 0xFFFFFFFF 哨兵），语义绕了一层，
         * 出现了 min=0 与 sd=0 并存的矛盾结果。现在改成：
         *   g_count 记录间隔样本数，g_* 只在 g_count>0 时才有意义；
         *   g_zero 单独计数，与 sd 互为验证（有零间隔则 sd 必 >= 1）。 */
        uint64_t g_count = 0u;
        uint32_t g_min = 0xFFFFFFFFu, g_max = 0u, g_zero = 0u;

        int32_t gx_min = 32767, gx_max = -32768, gy_min = 32767, gy_max = -32768;
        int32_t gz_min = 32767, gz_max = -32768;
        int32_t ax_min = 32767, ax_max = -32768, ay_min = 32767, ay_max = -32768;
        int32_t az_min = 32767, az_max = -32768;
        int32_t tp_min = 32767, tp_max = -32768, tp_tail = 0;
        uint8_t d[LSM6DSV16X_BLOCK_LEN];

        /* 排空跨区间遗留的 DRDY，直到引脚为低。
         * 上一区间在 1 秒边界退出时可能已有未读样本，DRDY 保持高（锁存），
         * 不排空则第一次 WaitGyroDrdy 立刻返回，读到的是遗留数据。
         * 用循环而非"检查一次读一次"：后者在检查与读取之间若引脚被拉高，
         * 会吃掉一份刚到的新样本。上限 8 次为防御，正常 1~2 次即排空。 */
        {
            uint8_t  sink[LSM6DSV16X_BLOCK_LEN];
            uint32_t guard;

            for (guard = 0u; guard < 8u; guard++)
            {
                if (IMU_INT1_READ() == 0u) break;   /* 引脚已低，排空完成 */
                (void)LSM6_ReadBlock(sink);         /* 丢弃；读完引脚应回低 */
            }
        }
        t_sec0 = GetTime64_10Ns();

        /* 采满 1 秒 */
        while ((GetTime64_10Ns() - t_sec0) < 100000000ULL)   /* 1s = 1e8 x 10ns */
        {
            uint64_t t_now;
            int32_t v;

            rc = LSM6_WaitGyroDrdy();
            if (rc != LSM6_OK) fail++;

            if (LSM6_ReadBlock(d) != LSM6_OK) fail++;
            t_now = GetTime64_10Ns();

            if (IMU_INT1_READ() != 0u) latch_fail++;

            /* 陀螺三轴极值 + 饱和计数 */
            v = (int32_t)LSM6DSV16X_MAKE_I16(d[2], d[3]);
            if (v < gx_min) gx_min = v; if (v > gx_max) gx_max = v;
            if (v > LSM6_G_THRESH || v < -LSM6_G_THRESH) gsat++;
            v = (int32_t)LSM6DSV16X_MAKE_I16(d[4], d[5]);
            if (v < gy_min) gy_min = v; if (v > gy_max) gy_max = v;
            v = (int32_t)LSM6DSV16X_MAKE_I16(d[6], d[7]);
            if (v < gz_min) gz_min = v; if (v > gz_max) gz_max = v;

            /* 加速度计三轴极值 + 饱和计数 */
            v = (int32_t)LSM6DSV16X_MAKE_I16(d[8], d[9]);
            if (v < ax_min) ax_min = v; if (v > ax_max) ax_max = v;
            if (v > LSM6_A_THRESH || v < -LSM6_A_THRESH) asat++;
            v = (int32_t)LSM6DSV16X_MAKE_I16(d[10], d[11]);
            if (v < ay_min) ay_min = v; if (v > ay_max) ay_max = v;
            v = (int32_t)LSM6DSV16X_MAKE_I16(d[12], d[13]);
            if (v < az_min) az_min = v; if (v > az_max) az_max = v;

            /* 温度极值 */
            v = (int32_t)LSM6DSV16X_MAKE_I16(d[0], d[1]);
            if (v < tp_min) tp_min = v; if (v > tp_max) tp_max = v;
            tp_tail = v;

            /* 间隔统计：相邻样本时间戳之差。
             * 差值必须在 u64 上做 —— 强转 u32 会在跨 2^32 时回绕
             *（10ns 单位下约 42.9s）。用 GetTime64_10Ns() 而非 Us()，
             * 130us 间隔对应 13000 个计数，抖动分辨力高 100 倍。
             * 方差用 Σ(g-NOM)² 而非 Σg²-(Σg/n)²：后者会累加到约 1.3e8，
             * 逼近 uint32 上限，两次整数截断的误差被放大。 */
            if (t_prev != 0u)
            {
                /* u64 上相减后再换算，最后才落到 u32 */
                uint64_t dt_10ns = t_now - t_prev;
                uint32_t g       = (uint32_t)(dt_10ns / 100ULL);   /* us */
                int32_t  dv;

                if (g_count == 0u || g < g_min) g_min = g;
                if (g_count == 0u || g > g_max) g_max = g;
                if (g == 0u) g_zero++;
                g_count++;

                sum += (uint64_t)g;

                dv = (int32_t)g - (int32_t)LSM6_GAP_NOM_US;
                sumd2 += (uint64_t)(uint32_t)(dv * dv);

                n_gap++;
                if (g < LSM6_GAP_LO_US || g > LSM6_GAP_HI_US) oob++;
            }
            t_prev = t_now;
            n++;
        }

        /* 本秒时间戳缺口：实际耗时 - 名义耗时（都用 us 比较）
         * el 由 10ns 换算成 us；名义耗时 = n 个样本 / 7680Hz。 */
        {
            uint64_t el   = (GetTime64_10Ns() - t_sec0) / 100u;      /* 10ns -> us */
            int32_t  nom  = (int32_t)((n * 1000000ULL) / 7680ULL);   /* us */
            int32_t  loss = (int32_t)el - nom;
            uint32_t mean = (n_gap != 0u) ? (uint32_t)(sum / n_gap) : 0u;
            uint32_t var  = 0u;
            uint32_t sd   = 0u;

            if (n_gap > 1u)
            {
                /* 方差 = E[(g-NOM)²] - (mean-NOM)² */
                int32_t mean_dev = (int32_t)mean - (int32_t)LSM6_GAP_NOM_US;
                uint64_t e_d2 = sumd2 / n_gap;              /* E[(g-NOM)²] */
                uint64_t md2  = (uint64_t)(uint32_t)(mean_dev * mean_dev);
                var = (e_d2 > md2) ? (uint32_t)(e_d2 - md2) : 0u;
            }
            /* 整数开方（牛顿迭代） */
            if (var != 0u)
            {
                uint32_t x = var, y = (x + 1u) / 2u;
                while (y < x) { x = y; y = (x + var / x) / 2u; }
                sd = x;
            }

            printf("RPT,%u,n=%u,loss=%d,fail=%u,latch=%u\r\n",
                   (unsigned)(sec + 1u), (unsigned)n, (int)loss,
                   (unsigned)fail, (unsigned)latch_fail);
            printf("D,%u,n=%u,min=%u,max=%u,mean=%u,sd=%u,zero=%u,oob=%u\r\n",
                   (unsigned)(sec + 1u), (unsigned)n_gap,
                   (unsigned)((g_count != 0u) ? g_min : 0u),
                   (unsigned)g_max, (unsigned)mean, (unsigned)sd,
                   (unsigned)g_zero, (unsigned)oob);
            printf("G,%u,%d,%d,%d,%d,%d,%d,sat=%u\r\n", (unsigned)(sec + 1u),
                   (int)gx_min, (int)gx_max, (int)gy_min, (int)gy_max,
                   (int)gz_min, (int)gz_max, (unsigned)gsat);
            printf("A,%u,%d,%d,%d,%d,%d,%d,sat=%u\r\n", (unsigned)(sec + 1u),
                   (int)ax_min, (int)ax_max, (int)ay_min, (int)ay_max,
                   (int)az_min, (int)az_max, (unsigned)asat);
            printf("T,%u,%d,%d,%d\r\n", (unsigned)(sec + 1u),
                   (int)tp_min, (int)tp_max, (int)tp_tail);
        }
    }

    t_all1 = GetTime64_10Ns();
    printf("STRESS,end,seconds=%u,total_us=%u,fail=%u,latch=%u\r\n",
           (unsigned)seconds, (unsigned)((t_all1 - t_all0) / 100u),
           (unsigned)fail, (unsigned)latch_fail);

    return (fail == 0u) ? LSM6_OK : LSM6_ERR_DRDY_TIMEOUT;
}

