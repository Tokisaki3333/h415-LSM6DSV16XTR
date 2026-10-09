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

/*==============================================================================
 * 初始化
 *
 * 顺序依据：
 *   1) 软复位，确保从已知状态开始（CTRL3.SW_RESET 自动清除）
 *   2) 复位后所有控制寄存器为默认值，CTRL3 默认 0x44（BDU=1, IF_INC=1）
 *   3) 手册 §6.5：HAODR 必须在【掉电模式】下使能 —— 复位后两传感器
 *      的 ODR 默认就是 0000（掉电），此时写 OP_MODE 是安全的
 *   4) HAODR_CFG.SEL 选择 ODR 表（00 -> 最高 7680Hz）
 *   5) 最后写 ODR_[3:0] 非零值完成使能，此写发生在"已进入 HAODR 且
 *      仍处于掉电"的状态下，满足约束
 *============================================================================*/
int LSM6_Init(void)
{
    uint8_t v = 0u;

    /* 1) 软复位 */
    if (lsm6_wr(LSM6DSV16X_REG_CTRL3, LSM6DSV16X_CTRL3_SW_RESET) != 0)
        return LSM6_ERR_SPI;
    HWT_DelayUs(1000u);                 /* 等复位完成（手册未给具体时间，1ms 足够）*/

    /* 2) 校验 WHO_AM_I */
    v = 0u;
    if (lsm6_rd(LSM6DSV16X_REG_WHO_AM_I, &v) != 0)
        return LSM6_ERR_SPI;
    if (v != LSM6DSV16X_WHO_AM_I_VAL)
        return LSM6_ERR_NO_ID;

    /* 3) 接口配置：INT 引脚高有效 + 推挽 + 4线SPI（=0x00，即复位默认）。
     *    该寄存器【不随软复位复位】（手册 §9.3），所以必须显式写，
     *    不能依赖默认值 —— 若上一次运行把它改成低有效，这里就会读错沿。 */
    if (lsm6_wr(LSM6DSV16X_REG_IF_CFG, LSM6DSV16X_IFCFG_OUR_CFG) != 0)
        return LSM6_ERR_SPI;

    /* 4) 通用配置：BDU=1 + IF_INC=1（复位默认即 0x44，显式写出以免依赖复位状态）
     *    BDU 保证读低字节到高字节之间输出寄存器不更新，
     *    对 7.68kHz 这种高速率尤其必要。 */
    if (lsm6_wr(LSM6DSV16X_REG_CTRL3, LSM6DSV16X_CFG_CTRL3_DEFAULT) != 0)
        return LSM6_ERR_SPI;

    /* 5) 陀螺满量程：±4000 dps（bit3:0 = 1100），LPF1_G_BW 用默认档 000。
     *    选 4000dps 要求 OIS_G_EN = 0，该位复位默认即为 0，无需处理。 */
    if (lsm6_wr(LSM6DSV16X_REG_CTRL6, LSM6DSV16X_CFG_CTRL6_GYRO_4000) != 0)
        return LSM6_ERR_SPI;

    /* 6) 显式关闭陀螺数字 LPF1（CTRL7 = 0x00）。
     *
     *    手册未标注 CTRL7 的复位值，所以不能依赖上电默认 —— 显式写 0 消除
     *    这个不确定性。CTRL7 其余位全是 Qvar/模拟前端相关，本板不用。
     *
     *    关闭 LPF1 后陀螺 UI 链上仍有【模拟抗混叠】与【LPF2】，二者不可关：
     *      模拟抗混叠 —— 高性能模式自动生效
     *      LPF2      —— 截止由 ODR 决定，7.68kHz 时约 537Hz（Table 21）
     *
     *    选关闭的理由：群延迟最小（仅 LPF2，约 0.30ms，建立约 0.89ms）。
     *    陀螺要积分，群延迟会直接变成姿态误差；而 7.68kHz 已提供足够样本，
     *    MCU 侧可做线性相位 FIR，比传感器内的一阶 IIR 更好。
     *    代价是原始噪声升到约 81 mdps RMS —— 可事后滤掉，相位延迟换不回来。 */
    if (lsm6_wr(LSM6DSV16X_REG_CTRL7, LSM6DSV16X_CTRL7_ALL_OFF) != 0)
        return LSM6_ERR_SPI;

    /* 7) 打开 DRDY_MASK：滤波器建立期间不置 DRDY。
     *    实测依据：不打开时读到的是一条从 0 单调爬升的曲线，持续约 1.86ms，
     *    折合约 14 个采样周期（1/7680 = 130us），即滤波器从零向真值收敛的
     *    建立过程。DRDY_MASK=0 时这段过程也会置 DRDY，于是最初的样本全是
     *    无效的建立值。
     *    置 1 后 DRDY 直到建立结束才置起，此时读到的第一个样本即有效。 */
    if (lsm6_wr(LSM6DSV16X_REG_CTRL4, LSM6DSV16X_CTRL4_DRDY_MASK) != 0)
        return LSM6_ERR_SPI;

    /* 8) 进入 HAODR：写 OP_MODE=001，此时 ODR 仍为 0000（掉电），符合约束 */
    if (lsm6_wr(LSM6DSV16X_REG_CTRL1, LSM6DSV16X_XL_OPMODE_HAODR) != 0)
        return LSM6_ERR_SPI;
    if (lsm6_wr(LSM6DSV16X_REG_CTRL2, LSM6DSV16X_G_OPMODE_HAODR) != 0)
        return LSM6_ERR_SPI;

    /* 9) 把陀螺数据就绪路由到 INT1 引脚（PD11）。
     *    INT1_CTRL.INT1_DRDY_G = bit1。
     *    这样数据就绪会体现为 PD11 引脚拉高（IF_CFG.H_LACTIVE=0 即高有效），
     *    默认锁存模式下降沿读走输出寄存器高字节后自动回低。
     *    主循环直接轮询这个引脚，不必再读 STATUS_REG 占 SPI 带宽。 */
    if (lsm6_wr(LSM6DSV16X_REG_INT1_CTRL, LSM6DSV16X_INT1_DRDY_G) != 0)
        return LSM6_ERR_SPI;

    /* 10) HAODR 数据率表选择：00 -> 最高档 7680Hz */
    if (lsm6_wr(LSM6DSV16X_REG_HAODR_CFG, LSM6DSV16X_CFG_HAODR_SEL_00) != 0)
        return LSM6_ERR_SPI;

    /*--------------------------------------------------------------------------
     * 到这里为止：器件已配好，但【仍在掉电模式】（ODR=0000，不产数据）。
     *
     * ★ 为什么不在 Init 里顺手把 ODR 写下去
     *   若这里就使能，陀螺立刻开始产数据 -> PD11 的第一个上升沿在
     *   SENSOR_DMA_Init() 装 DMA / 使能 EXTI 之前就到达 -> 边沿丢失。
     *   而 DRDY 是【锁存】模式，INT 会一直保持高，之后没有新的上升沿
     *   -> 永远不进中断，DMA 也永远不启动 -> 死锁。
     *   所以"启动第一次转换"必须由 LSM6_Enable() 在中断 armed 之后单独做。
     *------------------------------------------------------------------------*/

    printf("LSM6,cfg_ok,odr=7680,fs_g=4000,dps_per_lsb=%.3f\r\n",
           (double)LSM6_GYRO_DPS_PER_LSB);
    return LSM6_OK;
}

/*==============================================================================
 * 配置回读核对 —— 判断写入是否真的生效
 *
 * 【为什么必须有这一步】
 * INT 引脚为低电平说明器件根本没在产数据。而"没在产数据"只有两种可能：
 *   ① 我写进去的 ODR 没生效（SPI 写失败 / 写错寄存器 / 写错值）
 *   ② 写生效了但器件因别的原因不产数据（模式约束不满足等）
 * 回读控制寄存器就能把两者分开 —— 这是唯一能定论的检查。
 *
 * 【为什么可以回读】
 * CTRL1/CTRL2/CTRL6/CTRL7/IF_CFG/HAODR_CFG 都是 R/W 寄存器，
 * 与 MMC5983MA 的"只写、禁止读改写"不同，LSM6DSV16X 这里读回是合法的。
 *
 * 期望值（Enable 之后）：
 *   CTRL1 = 0x1C  加速度计 OP_MODE=HAODR(001) + ODR=1100(7680Hz)
 *   CTRL2 = 0x1C  陀螺     OP_MODE=HAODR(001) + ODR=1100(7680Hz)
 *   HAODR_CFG = 0x00
 *   CTRL6 = 0x0C  ±4000dps
 *   CTRL7 = 0x00  LPF1 关闭
 *   IF_CFG = 0x00 高有效 + 推挽 + 4 线
 *============================================================================*/
int LSM6_VerifyConfig(void)
{
    uint8_t c1 = 0u, c2 = 0u, hc = 0u, c6 = 0u, c7 = 0u, ifc = 0u;

    (void)lsm6_rd(LSM6DSV16X_REG_CTRL1,     &c1);
    (void)lsm6_rd(LSM6DSV16X_REG_CTRL2,     &c2);
    (void)lsm6_rd(LSM6DSV16X_REG_HAODR_CFG, &hc);
    (void)lsm6_rd(LSM6DSV16X_REG_CTRL6,     &c6);
    (void)lsm6_rd(LSM6DSV16X_REG_CTRL7,     &c7);
    (void)lsm6_rd(LSM6DSV16X_REG_IF_CFG,    &ifc);

    printf("LSM6,verify,CTRL1=0x%02X(exp1C),CTRL2=0x%02X(exp1C),"
           "HAODR=0x%02X(exp00),CTRL6=0x%02X(exp0C),CTRL7=0x%02X(exp00),"
           "IFCFG=0x%02X(exp00)\r\n",
           (unsigned)c1, (unsigned)c2, (unsigned)hc,
           (unsigned)c6, (unsigned)c7, (unsigned)ifc);

    /* 判定：CTRL1/CTRL2 的 ODR 位（bit3:0）非零才算真正使能 */
    if (((c1 & 0x0Fu) == 0u) || ((c2 & 0x0Fu) == 0u))
        return LSM6_ERR_NO_ID;      /* ODR 仍是 0000 -> 器件还在掉电，没产数据 */
    return LSM6_OK;
}

/*==============================================================================
 * 启动第一次转换：把 ODR 写成非零，从掉电切到 HAODR 最高速
 *
 * 【必须在两个 DRDY 中断使能之后调用】
 *   原因见 LSM6_Init 末尾的说明：早使能会丢掉第一个上升沿，
 *   而锁存模式下丢了第一个边沿就再也没有第二个。
 *
 * 返回 LSM6_OK 或负错误码。
 *============================================================================*/
int LSM6_Enable(void)
{
    printf("LSM6,en0\r\n");

    /* 加速度计与陀螺同步进 HAODR（手册 §6.5：两者必须一致）*/
    if (lsm6_wr(LSM6DSV16X_REG_CTRL2, LSM6DSV16X_CFG_CTRL2_GYRO_HAODR_MAX) != 0)
        return LSM6_ERR_SPI;
    printf("LSM6,en1\r\n");

    if (lsm6_wr(LSM6DSV16X_REG_CTRL1, LSM6DSV16X_CFG_CTRL1_XL_HAODR_MAX) != 0)
        return LSM6_ERR_SPI;
    printf("LSM6,en2,spi2_ctlr1=0x%04X\r\n", (unsigned)SPI2->CTLR1);

    /* 建立等待 —— 必需，且 2ms 不够。
     *
     * 【实测结论】DRDY_MASK 并不屏蔽建立过程本身，只改变 DRDY 的置起时机：
     *   不加掩码：前两个样本全零，随后单调爬升到约 ±45dps，历时约 1.86ms
     *   加掩码后：不再出现全零样本，但仍有单调收敛过程，幅度降到约 4dps，
     *             时间常数仍约 550us
     * 所以必须显式等待，覆盖完整建立过程。取 4~5 个时间常数并留余量 = 5ms。 */
    /* 【改用不依赖计时器的忙等】
     * 原先用 HWT_DelayUs(5000)，它靠 GetTime64_Us() 前进才能退出。
     * 实测这个延时把整个初始化卡住（en2 之后再无任何输出）。
     * 改成固定次数的 CPU 忙等：不依赖任何计时器、必然返回，
     * 100MHz 下 200000 次约 5~10ms，足够覆盖滤波器建立。 */
    {
        volatile uint32_t spin = 200000u;
        while (spin--) { }
    }

    printf("LSM6,enabled,odr=7680\r\n");
    return LSM6_OK;
}

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
 * 等陀螺新样本就绪 —— 轮询 INT1 引脚（PD11），不读 STATUS_REG
 *
 * 为什么用引脚而不是 STATUS_REG.GDA：
 *   读 GDA 要走一次 SPI 事务（约 5.6us），而且"读到位"与"数据就绪"之间
 *   隔了一次事务的开销，时间戳会偏晚。DRDY 引脚是硬件信号，直接反映就绪时刻。
 *
 * 极性（IF_CFG.H_LACTIVE=0 -> 高有效）：数据就绪时引脚拉高并保持。
 * 锁存行为（CTRL4.DRDY_PULSED=0 -> 锁存模式）：读走输出寄存器的高字节后
 * 引脚自动回低，所以不需要任何"清标志"动作 —— 读数据本身就清了。
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
 * 测试：最高速度突发采集 + 采集结束后统一 ASCII 上报
 *
 * 【一次 SPI 事务读全部输出寄存器】
 *   起点 OUT_TEMP_L (0x20)，长度 14 字节，覆盖：
 *     [0..1]   OUT_TEMP_L/H    温度
 *     [2..7]   OUTX/Y/Z_L/H_G  陀螺三轴
 *     [8..13]  OUTX/Y/Z_L/H_A  加速度计三轴
 *   地址 0x20~0x2D 连续无缺口（0x2E 起是 EIS/OIS 的寄存器），且 IF_INC=1
 *   已使能，所以一次片选就把温度+六轴全部取回，比分三次读省 2/3 事务开销。
 *
 * 为什么不边采边打印：7.68kHz 对应 130us 一个样本，而一行 ASCII 约 60 字节
 * 在 921600 波特下需约 650us，边采边打印会把速率压到串口带宽的水平，
 * 测不出传感器真实速度。所以先只写 RAM，采完再统一输出。
 *
 * 加速度计也在 7.68kHz 运行 —— 这不是额外负担，而是 HAODR 的约束：
 * 手册 §6.5 规定陀螺用 HAODR 时加速度计必须同步进 HAODR，所以它本来就在跑，
 * 顺手读回来不占额外时间。
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
 *
 * 【内存约束】V3F 只有 64KB RAM（Link_v3f.ld: RAM = 64K - 256），
 * 无法缓存大数据量。但压力测试要的是【统计量】而非原始波形，所以边采边算，
 * 每帧只占几十字节，可以跑任意长度。
 *
 * 【每秒上报的理由】结束时只报一次看不出"随时间劣化"；每秒一帧能直接看到
 * 间隔离散度、极值、失步计数随时间的趋势。
 *
 * 【上报内容】
 *   RPT,<秒>,n=<本秒样本>,loss=<本秒时间戳缺口us>,fail=<累计>,latch=<累计>
 *      loss = 本秒实际耗时 - 名义耗时(样本数/7680)。持续为正说明丢样本；
 *             持续为负说明在重复读同一份数据。
 *   D,<帧>,n=<间隔数>,min,max,mean,sd,oob=<超窗数>
 *      间隔 = 相邻样本时间戳之差。标称 1/7680 = 130.2us。
 *      oob = 落在 [LO,HI] 之外的数量（严重离散的判据）。
 *   G,<帧>,minX,maxX,minY,maxY,minZ,maxZ   陀螺原始极值（0.140 dps/LSB）
 *   A,<帧>,minX,maxX,minY,maxY,minZ,maxZ   加速度计原始极值（±2g 下 0.061 mg/LSB）
 *   T,<帧>,min,max,tail                    温度原始值极值与末值
 *
 * 一帧约 210 字节，在 921600 波特下需约 2.3ms，占每秒的 0.23%，可忽略。
 *
 * 返回 LSM6_OK 或负错误码。
 *============================================================================*/

/* 标称间隔：1/7680 Hz = 130.2us。
 * 方差统计以它为基准算偏差平方和（见 LSM6_StressTest 内的说明），
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

        /* 区间头部排空跨区间遗留的 DRDY，直到引脚确认为低。
         *
         * 【为什么必需】上一区间在 1 秒边界处退出循环，此时传感器可能已经产出
         * 但没人读的样本，DRDY 引脚保持高（锁存模式）。若直接进采集循环，
         * 第一次 WaitGyroDrdy 会立刻返回，读到的是那份遗留数据，其时间戳距
         * 上一区间末样本只有几十微秒 —— 实测正是每区间恰好出现一个
         * min=30~82us 的异常间隔（oob=1），sd 也几乎全由它贡献。
         *
         * 【为什么用循环而非"检查一次读一次"】先前写成
         *     if (IMU_INT1_READ()) { ReadBlock(); }
         * 存在竞态：若引脚恰在"检查"与"读取"之间被拉高，那次读取会吃掉一份
         * "刚到"的新样本，导致下一轮等待落空。实测 10 秒内出现 1 次
         * DRDY_TIMEOUT（fail=1）、一个 min=50us 与 oob=2 的异常区间。
         * 改成"读到低为止"的有界循环后，读到几份遗留数据都能排干净，
         * 且退出条件（引脚为低）保证进入采集时相位是干净的。
         *
         * 上限 8 次是防御性的：正常情况下最多读 1~2 次即可排空。 */
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

            /* 间隔统计（相邻样本时间戳之差）
             *
             * 【时间戳必须全程用 u64，绝不可强转 u32】
             * t_now/t_prev/t_sec0 都是 GetTime64_Us()/GetTime64_10Ns() 的
             * uint64_t 返回值。差值也必须在 u64 上做：
             *   uint32_t g10 = (uint32_t)(t_now - t_prev);   <-- 错！
             * 强转 u32 会在时间戳跨过 2^32 时回绕（10ns 单位下约 42.9 秒）。
             * 正确写法是先做 u64 减法，只在换算成 us 之后再落到 u32
             * （us 值本身很小，u32 足够）。
             *
             * 用 GetTime64_10Ns() 而不是 GetTime64_Us()：
             * 亚秒分辨率从 1us 提到 10ns，130us 的间隔对应 13000 个计数，
             * 抖动分辨能力提升 100 倍。
             *
             * 方差用"相对标称值的偏差平方和"算，而不是 Σg² - (Σg/n)²：
             * 间隔都在标称值附近，偏离量小，Σ(g-NOM)² 数值极小，既无溢出风险，
             * 整数除法截断的相对误差也可忽略。而 Σg² 会累加到 ~1.3e8（逼近
             * uint32 上限），两次整数除法都在大数上截断，精度损失被放大。 */
            if (t_prev != 0u)
            {
                /* 全程 u64：先减，再换算成 us，最后才落到 u32 */
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
