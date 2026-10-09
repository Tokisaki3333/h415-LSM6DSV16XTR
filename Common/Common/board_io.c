/*******************************************************************************
* File Name          : board_io.c
* Description        : CH32H417WEU6 (QFN-68) 全 IO / 外设初始化实现
*                      详细引脚依据见 board_io.h 头部说明
*******************************************************************************/
#include "board_io.h"
#include "debug.h"
#include "sys_clk.h"        /* GetTime64_Us / GetTime64_10Ns / HWT_DelayUs */
#include "sensor_spi.h"     /* SPI_ReadRegs / SPI_WriteReg：地磁 DRDY ISR 内读写 */
#include "mmc5983ma_reg.h"  /* MMC5983MA_REG_* / CTRL0_* / XYZ_* 解码宏 */
#include "mmc5983.h"        /* MMC_RST_PULSE_WAIT_US：脉冲后的等待 */
#include "lsm6dsv16x_reg.h"  /* LSM6DSV16X_REG_OUT_TEMP_L_BLK：陀螺输出块 */

/*==============================================================================
 * 软件 I2C 时序：~200kHz
 *   I2C_HALF_LOOPS 为空循环次数，按 400MHz 主频标定；
 *   若系统主频不同，请按比例调整。
 *============================================================================*/
#define I2C_HALF_LOOPS          120u

void BMP_I2C_Delay(void)
{
    volatile uint32_t i = I2C_HALF_LOOPS;
    while(i--) { }
}

/* 开漏输出：写 0 拉低，写 1 释放 */
void BMP_I2C_SCL_HIGH(void) { GPIO_SetBits(BMP_I2C_PORT, BMP_I2C_SCL_PIN); }
void BMP_I2C_SCL_LOW(void)  { GPIO_ResetBits(BMP_I2C_PORT, BMP_I2C_SCL_PIN); }
void BMP_I2C_SDA_HIGH(void) { GPIO_SetBits(BMP_I2C_PORT, BMP_I2C_SDA_PIN); }
void BMP_I2C_SDA_LOW(void)  { GPIO_ResetBits(BMP_I2C_PORT, BMP_I2C_SDA_PIN); }

/* 板上无外部上拉：需要"高"的时候必须把引脚切到上拉输入，
 * 才能让内部上拉把线抬起来；否则开漏 + 无上拉 = 浮空。 */
static void bmp_i2c_pin_mode(uint16_t pin, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef g = {0};
    g.GPIO_Pin   = pin;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = mode;
    GPIO_Init(BMP_I2C_PORT, &g);
}

void BMP_I2C_SDA_TO_RX(void) { bmp_i2c_pin_mode(BMP_I2C_SDA_PIN, GPIO_Mode_IPU); }
void BMP_I2C_SDA_TO_TX(void) { bmp_i2c_pin_mode(BMP_I2C_SDA_PIN, GPIO_Mode_Out_OD); }

void BMP_I2C_BUS_IDLE(void)
{
    bmp_i2c_pin_mode(BMP_I2C_SCL_PIN, GPIO_Mode_IPU);
    bmp_i2c_pin_mode(BMP_I2C_SDA_PIN, GPIO_Mode_IPU);
}

uint8_t BMP_I2C_SDA_READ(void)
{
    return (GPIO_ReadInputDataBit(BMP_I2C_PORT, BMP_I2C_SDA_PIN) != 0) ? 1 : 0;
}
uint8_t BMP_I2C_SCL_READ(void)
{
    return (GPIO_ReadInputDataBit(BMP_I2C_PORT, BMP_I2C_SCL_PIN) != 0) ? 1 : 0;
}

/*==============================================================================
 * 9.1 GPIO：片选 / 中断输入 / LED / 按键
 *============================================================================*/
void BOARD_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    /* --- 所需端口时钟（HB2） --- */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO  |
                          RCC_HB2Periph_GPIOB |
                          RCC_HB2Periph_GPIOC |
                          RCC_HB2Periph_GPIOD |
                          RCC_HB2Periph_GPIOE |
                          RCC_HB2Periph_GPIOF, ENABLE);

    /* --- MMC5983MA 片选 PE0：推挽输出，空闲高（必须先建立空闲态） --- */
    g.GPIO_Pin   = MMC_CS_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(MMC_CS_PORT, &g);
    MMC_CS_HIGH();

#if (BOARD_IMU_HAS_CS)
    g.GPIO_Pin   = IMU_CS_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(IMU_CS_PORT, &g);
    IMU_CS_HIGH();
#endif

    /*--------------------------------------------------------------------------
     * --- 中断输入：磁强计 INT(PE1) + IMU INT1(PD11) ---
     *
     * ★ 配【浮空输入】—— 不加任何内部上下拉
     *
     * 【依据】这两个脚都是【器件输出】：
     *   手册 P9 ：pin15 = INT，类型栏是 `O`（Output）
     *   手册 P11："INT – This pin is Interrupt output. Active High."
     * 板上没有外部上下拉（网表：PE1 ; U1.64 U11.15，只有两个器件引脚）。
     * 器件自己会驱动这个脚，MCU 侧不该并接上下拉去和它抢。
     *
     * 【实测对照 —— 这一项试过三种，结论明确】
     *   ✗ 内部上拉：引脚被读成恒低。上拉把器件输出的高电平压下来，
     *               两边对着拉，电平不可信。（本次实测 edges=0, hi=0）
     *   ✗ 内部下拉：把器件输出直接拽到低，更错
     *   ✓ 浮空    ：让器件自己驱动。实测读到 pe1=1、且能采到跳变
     *
     * 规律：给一个【器件输出】的脚加内部上下拉，等于并行加载它的输出级，
     * 只会让电平失真。只有开漏输出才需要外部/内部上拉。
     *
     * 注：PD10 是 IMU 的 CS（输出），在 BOARD_SPI2_IMU_Init 里配置。
     *------------------------------------------------------------------------*/
    g.GPIO_Pin   = MMC_DRDY_PIN;
    g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(MMC_DRDY_PORT, &g);

    g.GPIO_Pin   = IMU_INT1_PIN;
    g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(IMU_INT1_PORT, &g);

    /* --- GNSS 模块辅助脚：普通输入，不驱动 --- */
    g.GPIO_Pin   = GNSS_AUX1_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GNSS_AUX1_PORT, &g);

    g.GPIO_Pin   = GNSS_AUX2_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GNSS_AUX2_PORT, &g);
}

/*==============================================================================
 * 9.2 磁强计 MMC5983MA —— SPI1
 *     引脚：SCK=PF5  MISO=PF3  MOSI=PD7（全部 AF5）  CS=PE0(GPIO)
 *     协议：SPI mode 0（CPOL=0, CPHA=1Edge），读命令 reg|0x80
 *============================================================================*/
void BOARD_SPI1_MMC_Init(void)
{
    GPIO_InitTypeDef g = {0};
    SPI_InitTypeDef  s = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOD |
                          RCC_HB2Periph_GPIOF, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_SPI1, ENABLE);   /* SPI1 在 HB2 */

    /* SCK PF5 (AF5) */
    GPIO_PinAFConfig(MMC_SPI_SCK_PORT, MMC_SPI_SCK_SRC, MMC_SPI_AF);
    g.GPIO_Pin   = MMC_SPI_SCK_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(MMC_SPI_SCK_PORT, &g);

    /* MOSI PD7 (AF5) */
    GPIO_PinAFConfig(MMC_SPI_MOSI_PORT, MMC_SPI_MOSI_SRC, MMC_SPI_AF);
    g.GPIO_Pin   = MMC_SPI_MOSI_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(MMC_SPI_MOSI_PORT, &g);

    /* MISO PF3 (AF5)，主模式全双工下 EVT 例程用浮空输入 */
    GPIO_PinAFConfig(MMC_SPI_MISO_PORT, MMC_SPI_MISO_SRC, MMC_SPI_AF);
    g.GPIO_Pin   = MMC_SPI_MISO_PIN;
    g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(MMC_SPI_MISO_PORT, &g);

    s.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    s.SPI_Mode              = SPI_Mode_Master;
    s.SPI_DataSize          = SPI_DataSize_8b;

    /*--------------------------------------------------------------------------
     * SPI 模式：Mode 0（CPOL=Low, CPHA=1Edge）
     *
     * 【为什么回退到 Mode 0】
     * 我一度按手册 P5 的 "SCK is stopped HIGH when CS is high" 判成 CPOL=1，
     * 改成 Mode 3。结果实测立即变差：
     *   Mode 0 时：RD 数据真实稳定（SET/RESET 极性正确翻转、零偏 80 周期收敛）
     *   Mode 3 时：数据冻结不变、数值非法（y≈1580）、测量周期从 1ms 变 7ms
     * 说明器件要的是 Mode 0。
     *
     * 手册那句描述的是"CS 为高时 SCK 停在高电平"这一【状态】，
     * 不代表 SPI 的 CPOL 定义 —— 是我理解错了。
     * 实测才是判据。
     *------------------------------------------------------------------------*/
    s.SPI_CPOL              = SPI_CPOL_Low;          /* mode 0 */
    s.SPI_CPHA              = SPI_CPHA_1Edge;

    s.SPI_NSS               = SPI_NSS_Soft;          /* 片选由 PE0 软件控制 */
    s.SPI_FirstBit          = SPI_FirstBit_MSB;      /* MMC5983MA: MSB first */
    s.SPI_CRCPolynomial     = 7;

    /*--------------------------------------------------------------------------
     * SCK 频率 = 6.25 MHz（FHCLK/16，BR=011，HSRXEN=0）
     *
     * 【为什么定在 6.25MHz —— 实测结论，不要再改】
     * 器件手册 P4：fc(SCK) max = 10 MHz、tc(SCK) min = 100ns。
     * 本板实测（用示波器量 SCK 周期校准过）：
     *   BR=011(/16) 6.25MHz + Mode 0  ->  正常：mmc_hz≈1027、极性交替、零偏收敛
     *   BR=010(/8)  12.5MHz + Mode 0  ->  器件完全不测量（m_done 置起但 INT 恒低、hz=0）
     *   BR=010(/8)  12.5MHz + Mode 3  ->  能跑但全错：hz 掉到 128、raw 数值非法、
     *                                     极性不再交替
     * 说明 MMC5983MA 只接受 Mode 0，且 12.5MHz 超出它的时序裕量。
     * HSRXEN=1 那一套的 /9=11.1MHz（超手册 11%）未再尝试 ——
     * 6.25MHz 对本路不构成瓶颈，没必要为它冒险。
     *
     * 【6.25MHz 为何够用】地磁每秒 1027 拍 × 19 字节 ≈ 19.5 KB/s，
     *   19 字节在 6.25MHz 下只需约 30us，占 973us 周期的 3%。
     *   瓶颈是器件的转换时间（BW=11 约 500us），不是 SPI 带宽。
     *   陀螺那一路（SPI2）单独跑 12.5MHz，与这里互不影响。
     *
     * 【注意】BR[2:0] 在通讯过程中不可修改（参考手册明确要求）；
     * spi_dma_rx_suspend/resume 只动 CTLR2 的 DMA 请求位，不碰 BR，符合该约束。*/
    s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode3;   /* BR=011 = /16 = 6.25MHz */
    SPI_Init(MMC_SPI, &s);

    SPI_Cmd(MMC_SPI, ENABLE);
}

/*==============================================================================
 * 9.3 六轴 IMU LSM6DSV16XTR —— SPI2
 *     引脚：SCK=PB13  MISO=PB14  MOSI=PC3（全部 AF5）
 *============================================================================*/
void BOARD_SPI2_IMU_Init(void)
{
    GPIO_InitTypeDef g = {0};
    SPI_InitTypeDef  s = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB |
                          RCC_HB2Periph_GPIOC | RCC_HB2Periph_GPIOD, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_SPI2, ENABLE);   /* SPI2 在 HB1 */

    /* SCK PB13 (AF5) */
    GPIO_PinAFConfig(IMU_SPI_SCK_PORT, IMU_SPI_SCK_SRC, IMU_SPI_AF);
    g.GPIO_Pin   = IMU_SPI_SCK_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(IMU_SPI_SCK_PORT, &g);

    /* MOSI PC3 (AF5) */
    GPIO_PinAFConfig(IMU_SPI_MOSI_PORT, IMU_SPI_MOSI_SRC, IMU_SPI_AF);
    g.GPIO_Pin   = IMU_SPI_MOSI_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(IMU_SPI_MOSI_PORT, &g);

    /* MISO PB14 (AF5) */
    GPIO_PinAFConfig(IMU_SPI_MISO_PORT, IMU_SPI_MISO_SRC, IMU_SPI_AF);
    g.GPIO_Pin   = IMU_SPI_MISO_PIN;
    g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(IMU_SPI_MISO_PORT, &g);

    s.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    s.SPI_Mode              = SPI_Mode_Master;
    s.SPI_DataSize          = SPI_DataSize_8b;
    s.SPI_CPOL              = SPI_CPOL_Low;          /* LSM6DSV16X: SPI mode 0/3 */
    s.SPI_CPHA              = SPI_CPHA_1Edge;
    s.SPI_NSS               = SPI_NSS_Soft;
    s.SPI_FirstBit          = SPI_FirstBit_MSB;
    s.SPI_CRCPolynomial     = 7;

    /* SCK 频率 = 12.5 MHz（FHCLK/8），与 SPI1 同档。
     * 手册 SPI 时序：fc(SPC) max = 10 MHz，tc(SPC) min = 100ns。
     * 分频档位与理由见 BOARD_SPI1_MMC_Init 里的详细说明。 */
    s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode2;   /* BR=010 = /8 */
    SPI_Init(IMU_SPI, &s);

#if (BOARD_IMU_HAS_CS)
    /* CS = PD10。手册 pin12：0 = SPI 通信模式。空闲置高（SPI idle），
     * 每次收发时拉低。 */
    g.GPIO_Pin   = IMU_CS_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(IMU_CS_PORT, &g);
    IMU_CS_HIGH();
#endif

    SPI_Cmd(IMU_SPI, ENABLE);
}

/*==============================================================================
 * 9.4 气压计 BMP388 —— 软件 I2C (PF0=SCL, PF1=SDA)
 *     板上无外部上拉，必须开漏 + 内部上拉。
 *     CSB 接 3V3 -> I2C 模式；SDO 接 GND -> 地址 0x76
 *============================================================================*/
void BOARD_BMP388_I2C_Init(void)
{
    GPIO_InitTypeDef g = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOF, ENABLE);

    g.GPIO_Pin   = BMP_I2C_SCL_PIN | BMP_I2C_SDA_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_Out_OD;     /* 开漏；输入时读引脚电平 */
    GPIO_Init(BMP_I2C_PORT, &g);

    /* 空闲态：两条线都释放（外部/内部上拉拉高） */
    BMP_I2C_SCL_HIGH();
    BMP_I2C_SDA_HIGH();

    /* PF2 预留脚：不配置，保持复位默认 */
}

/*==============================================================================
 * 9.5 GNSS ATGM336H —— USART2 (PD5=TX, PD6=RX, AF7)
 *============================================================================*/
void BOARD_USART2_GNSS_Init(void)
{
    GPIO_InitTypeDef  g = {0};
    USART_InitTypeDef u = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOD, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART2, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_PWR, ENABLE);

    /* TX PD5 (AF7) */
    GPIO_PinAFConfig(GNSS_TX_PORT, GNSS_TX_SRC, GNSS_USART_AF);
    g.GPIO_Pin   = GNSS_TX_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GNSS_TX_PORT, &g);

    /* RX PD6 (AF7) */
    GPIO_PinAFConfig(GNSS_RX_PORT, GNSS_RX_SRC, GNSS_USART_AF);
    g.GPIO_Pin   = GNSS_RX_PIN;
    g.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GNSS_RX_PORT, &g);

    u.USART_BaudRate            = GNSS_BAUDRATE;
    u.USART_WordLength          = USART_WordLength_8b;
    u.USART_StopBits            = USART_StopBits_1;
    u.USART_Parity              = USART_Parity_No;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    u.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(GNSS_USART, &u);

    USART_Cmd(GNSS_USART, ENABLE);
}

/*==============================================================================
 * 9.6 CAN1 —— PD0=RX, PD1=TX (AF9)
 *     默认静默模式；接好收发器与总线后改成 BOARD_CAN1_MODE = CAN_Mode_Normal
 *============================================================================*/
void BOARD_CAN1_Init(void)
{
    GPIO_InitTypeDef g = {0};
    CAN_InitTypeDef  c = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOD, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_CAN1, ENABLE);

    /* TX PD1 (AF9) */
    GPIO_PinAFConfig(CAN1_TX_PORT, CAN1_TX_SRC, CAN1_AF);
    g.GPIO_Pin   = CAN1_TX_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(CAN1_TX_PORT, &g);

    /* RX PD0 (AF9) */
    GPIO_PinAFConfig(CAN1_RX_PORT, CAN1_RX_SRC, CAN1_AF);
    g.GPIO_Pin   = CAN1_RX_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(CAN1_RX_PORT, &g);

    CAN_StructInit(&c);
    c.CAN_Mode      = BOARD_CAN1_MODE;
    c.CAN_SJW       = CAN_SJW_1tq;
    c.CAN_BS1       = CAN1_BS1;
    c.CAN_BS2       = CAN1_BS2;
    c.CAN_Prescaler = CAN1_BAUDRATE_PRESCALER;
    c.CAN_TTCM      = DISABLE;
    c.CAN_ABOM      = ENABLE;
    c.CAN_AWUM      = DISABLE;
    c.CAN_NART      = DISABLE;
    c.CAN_RFLM      = DISABLE;
    c.CAN_TXFP      = DISABLE;
    CAN_Init(CAN1, &c);
}

/*==============================================================================
 * 9.7 SD 卡 —— SDMMC 默认映射 RM=00
 *     D0=PC8  D1=PC9  D2=PC10  D3=PC11  CK=PC12  CMD=PD2  STR=PD3
 *     D4=PA14 D5=PA15 D6=PC6   D7=PC7（8bit 才用）
 *     要点（照 EVT SDMMC_SD 例程）：
 *       - SDMMC 引脚只做 GPIO 复用配置，不调用 GPIO_PinAFConfig
 *       - 需先设 VIO18 为软件可配模式，否则 SD 电平不受控
 *============================================================================*/
void BOARD_SDMMC_PinInit(void)
{
    GPIO_InitTypeDef g = {0};
    SDMMC_InitTypeDef sd = {0};

    /* VIO18 软件可配，按本板硬件取 MODE5 => 3V3 电平。
     * SDMMC 的 I/O 参考电平必须与 SD 卡一致（3V3），否则通信不可靠。 */
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_PWR, ENABLE);
    PWR_VIO18ModeCfg(PWR_VIO18CFGMODE_SW);
    PWR_VIO18LevelCfg(PWR_VIO18Level_MODE5);

    RCC_HBPeriphClockCmd(RCC_HBPeriph_SDMMC | RCC_HBPeriph_DMA1, ENABLE);

    /* 照 EVT SDMMC_SD / SDMMC_eMMC 例程：置 SWPMI 的 OR 位（bit0）后再配 SD 引脚。
     * 该位影响复用 I/O 的驱动能力/电压域，EVT 各 SD/eMMC 例程都保留了这一步。 */
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_SWPMI, ENABLE);
    SWPMI->OR |= (1 << 0);

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOA |
                          RCC_HB2Periph_GPIOC | RCC_HB2Periph_GPIOD, ENABLE);

    /* D0..D3 + CK = PC8..PC12 */
    g.GPIO_Pin   = SD_D0_PIN | SD_D1_PIN | SD_D2_PIN | SD_D3_PIN | SD_CK_PIN;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(GPIOC, &g);

    /* D6/D7 = PC6/PC7 —— 这两脚本就属于 SDMMC 默认映射（RM=00），始终配置 */
    g.GPIO_Pin   = SD_D6_PIN | SD_D7_PIN;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(GPIOC, &g);

#if (SD_BUS_WIDTH_8BIT)
    /* D4/D5 = PA14/PA15 —— 仅 8bit 模式需要 */
    g.GPIO_Pin   = SD_D4_PIN | SD_D5_PIN;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(GPIOA, &g);
#endif

    /* CMD + STR = PD2 / PD3 */
    g.GPIO_Pin   = SD_CMD_PIN;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(GPIOD, &g);

#if (SD_BUS_WIDTH_8BIT)
    g.GPIO_Pin   = SD_STR_PIN;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(GPIOD, &g);
#endif

    /* SDMMC 控制器：主机、低速档起步（时钟切换由协议层按需提速） */
    SDMMC_DeInit();
    sd.SDMMC_Mode         = SDMMC_Mode_Host;
    sd.SDMMC_ClockEdge    = SDMMC_SampleClock_Falling;
    sd.SDMMC_DMA_EN       = ENABLE;
    sd.SDMMC_BusWidth     = SDMMC_BusWidth_1;
    sd.SDMMC_PhaseInv     = SDMMC_Phase_No_Inverse;
    sd.SDMMC_ClockSpeed   = SDMMC_ClockSpeed_Low;
    sd.SDMMC_ClockDiv     = 0x1F;
    sd.SDMMC_TimeOut      = 0x0C;
    SDMMC_Init(&sd);
    SDMMC_ClockCmd(ENABLE);
}

/*==============================================================================
 * 9.8 EXTI：磁强计 DRDY(PE1->EXTI1) + IMU INT1(PD11->EXTI11)
 *                              IMU INT2(PD10->EXTI10)
 *     向量归属（startup_ch32h417_v*.S 中的弱符号）：
 *       EXTI7_0_IRQHandler   <- PE1
 *       EXTI15_8_IRQHandler  <- PD11 / PD10
 *     NVIC 分配给哪个核由应用层决定（NVIC_SetAllocateIRQ），此处只配置外设。
 *============================================================================*/
void BOARD_EXTI_Init(void)
{
    EXTI_InitTypeDef e = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO, ENABLE);

    /* --- 地磁 MMC5983MA INT = PE1 ---
     * 【上升沿】触发 —— 手册 P11 的 "Active High" 是对的。
     *
     * 实测过程（供后人别再走弯路）：
     *   器件中断功能【未使能】前，INT 引脚是 hi-Z（手册原文），
     *   所以那时读到的电平只反映 MCU 侧配置，不是器件在驱动：
     *     浮空/上拉 -> 读到高 ；下拉 -> 读到低
     *   我把这些"MCU 自己造成的电平"误当成器件极性，先后把触发沿
     *   改成下降沿、把引脚改成上拉，全错。
     *
     *   直到 INTERRUPT 真正使能（INT_meas_done_en 写进去）后，
     *   实测 INT 变为【恒高】—— 器件在自己的空闲态输出高、断言时驱动，
     *   这才是器件的真实极性：高有效。
     *
     * 所以：空闲低（hi-Z 时由 MCU 上拉决定）-> 断言时器件拉高 -> 上升沿。
     * 引脚配【上拉】，空闲有确定电平，上升沿有效。 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, MMC_DRDY_PIN_SRC);
    e.EXTI_Line    = MMC_DRDY_EXTI_LINE;
    e.EXTI_Mode    = EXTI_Mode_Interrupt;
    e.EXTI_Trigger = EXTI_Trigger_Rising;
    e.EXTI_LineCmd = ENABLE;
    EXTI_Init(&e);

    /* --- 陀螺 LSM6DSV16X INT1 = PD11 ---
     * IF_CFG.H_LACTIVE 复位默认 0 = 高有效（初始化里显式写 0 保证），
     * 所以同样是【上升沿】。 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, IMU_INT1_PIN_SRC);
    e.EXTI_Line    = IMU_INT1_EXTI_LINE;
    e.EXTI_Mode    = EXTI_Mode_Interrupt;
    e.EXTI_Trigger = EXTI_Trigger_Rising;
    e.EXTI_LineCmd = ENABLE;
    EXTI_Init(&e);

    /* NVIC 分配与优先级由应用层（main.c）做：
     * 两个 DRDY 都归小核 V3F，陀螺抢占级 0、地磁 1。 */
}

/*==============================================================================
 * EXTI 分组回调
 *
 *   ★ 分组归属（按 EXTI 线号，不是按引脚名）：
 *       EXTI_Line1  = PE1  -> EXTI7_0_IRQn   (IRQn=36)  <- 地磁 DRDY
 *       EXTI_Line11 = PD11 -> EXTI15_8_IRQn  (IRQn=33)  <- 陀螺 INT1
 *       EXTI_Line10 = PB10 -> EXTI15_8_IRQn  (IRQn=33)  <- 复位键
 *     EXTI 按【线号 0..7 / 8..15】分组，与引脚端口无关。
 *     PE1 是线 1，所以它走 EXTI7_0 —— 我先前误以为"PE1 在 EXTI15_8"，
 *     把处理体放进了错误的向量，而真正的 EXTI7_0 是个空函数、
 *     NVIC 也从没使能过。现象就是：器件 INT 已经恒高（断言中），
 *     中断计数却恒为 0。
 *============================================================================*/
/*==============================================================================
 * 地磁 DRDY 中断 —— 中断内手动触发下一拍 + SET/RESET 极性处理
 *
 * 【为什么把处理放进中断】
 *   完整一帧的耗时拆解（实测）：
 *     写 CTRL0 触发        ~10 us
 *     轮询等 Meas_M_Done   ~500 us   <- 长阻塞
 *     连读 7 字节          ~20 us
 *     写 STATUS 清锁存     ~10 us
 *   合计约 540 us，而地磁周期 1000 us。
 *   放在主循环里会受其它任务抖动影响，放中断里节拍最稳。
 *
 * 【允许长阻塞，但陀螺必须能抢断】
 *   本 ISR 最长约 640 us（含 RESET 脉冲等待 100 us）。
 *   陀螺周期只有 130 us（7680 Hz），所以陀螺 DRDY 的抢占级必须
 *   【高于】地磁 —— 否则地磁一执行就把陀螺堵住 4 个周期，直接丢帧。
 *   反向影响可忽略：地磁周期 1000 us，被抢断几十 us 无所谓。
 *   优先级在 main.c 里设：陀螺 EXTI15_8 = 0，地磁 EXTI7_0 = 1。
 *
 * 【中断内两条纪律】
 *   1) 不碰浮点：RISC-V ISR 一旦使用 FP 寄存器就要额外保存/恢复，
 *      会把 80ns 的裸进出变成数百 ns。所以这里只搬原始 counts。
 *   2) 不 printf：那是约 300 us 的阻塞，且串口是共享资源。
 *      上报交主循环每秒做一次。
 *
 * 【极性与零偏】
 *   外界磁场会让 SET/RESET 两拍读数相反，两者求和即得器件零偏：
 *       offset = (SET 样本 + RESET 样本) / 2
 *   配对窗口越短、外界磁场相消越干净，所以用
 *   【本拍 RESET 样本 + 下一拍 SET 样本】跨周期配对（间隔约 623 us）。
 *============================================================================*/
volatile int32_t  g_mmc_raw[3]      = {0, 0, 0};   /* X/Y/Z 原始 counts */
volatile uint8_t  g_mmc_polarity    = 0u;          /* 本拍极性：0=SET 1=RESET */
volatile uint32_t g_mmc_isr_cnt     = 0u;          /* 中断进入次数 */
volatile uint32_t g_mmc_spi_err     = 0u;          /* 中断内 SPI 失败次数 */
volatile uint64_t g_mmc_drdy_ts     = 0u;          /* 本拍 DRDY 时刻（10ns）*/

/*------------------------------------------------------------------------------
 * SET/RESET 配对累加（中断内完成）
 *
 * 【为什么在中断里配、而且用"上一拍"】
 *   极性每拍翻转，所以相邻两拍必然是一 SET 一 RESET ——
 *   用"上一拍样本 + 本拍样本"配，拿到的是【物理时间上最近的一对】。
 *   而按方向硬配（比如"RESET 配下一个 SET"）会人为拉长窗口：
 *   归档那种做法窗口约 623us，我先前那版固定 1280us。
 *   用最近的一对没有副作用：每拍都能构成一对，更新率不降。
 *
 * 【累加而不是覆盖】主循环每秒才取一次，如果各拍互相覆盖，
 *   只会剩下最后一对；累加 + 计数能给出这一秒内所有对的均值。
 *----------------------------------------------------------------------------*/
volatile int64_t  g_mmc_pair_sum[3] = {0, 0, 0};   /* 每秒配对均值之累加 */
volatile uint32_t g_mmc_pair_n      = 0u;          /* 本秒累计对数          */


/*------------------------------------------------------------------------------
 * 陀螺 DRDY 中断的产出（EXTI15_8_IRQHandler 内更新）
 *
 * 中断内一次读回 0x20 起 14 字节输出块，只搬原始 16 位值，不碰浮点、不 printf。
 * 顺序：[0..2] = 陀螺 XYZ，[3..5] = 加速度计 XYZ。
 * 换算（0.140 dps/LSB）与上报由主循环每秒做一次。
 *----------------------------------------------------------------------------*/
volatile int16_t  g_imu_raw[6]  = {0, 0, 0, 0, 0, 0};
volatile int16_t  g_imu_temp_raw = 0;
volatile uint32_t g_imu_isr_cnt  = 0u;
volatile uint32_t g_imu_spi_err  = 0u;
volatile uint64_t g_imu_drdy_ts  = 0u;   /* 本拍 DRDY 时刻（10ns）*/

/* 当前拍的极性：0 = SET（连续 + 自动 SET），1 = RESET（单次触发） */
static volatile uint8_t s_polarity = 0u;

/* 上一周期的 RESET 样本（暂存），与下一周期首个 SET 配对 —— 归档同一机制。
 * 配对窗口约 623us，是 SET/RESET 之间时间上最接近的一对。 */
static int32_t  s_pend_rst[3] = {0, 0, 0};
static uint8_t  s_have_rst    = 0u;

/*==============================================================================
 * 地磁 DRDY 中断 —— 严格照归档 mmc_cycle 的采样时序实现
 *
 * 【旧方法 mmc_cycle 的确切顺序（_newfunc.c 第 28-80 行）】
 *   1) for k in 0..b-1: MMC_ReadXYZ()        连续模式 + 自动 SET，读 b 个 SET 样本
 *   2) mmc_cfg_ex(0,0,0)                     切单次触发模式（关连续、关自动 SET）
 *   3) mmc_pulse(CTRL0_RESET)                发 RESET 脉冲（内部含 100us 等待）
 *   4) MMC_ConsumePending(0)                 ★ 消费在途转换（窗口 1400us，
 *                                            读到即丢弃；超时=本就没有在途）
 *   5) MMC_TriggerAndRead()                  触发并读【真正的 RESET 样本】
 *   6) mmc_auto_set_every()                  恢复自动 SET，进下一周期
 *   7) 配对：本周期 RESET + 下一周期首个 SET（间隔约 623us，最短）
 *
 * 【中断驱动的等价实现】
 *   SET 段   ：每拍 DRDY 中断读一个 SET 样本（连续模式 + 自动 SET）
 *   RESET 段 ：到达 SET 段末拍时，在同一个中断里
 *                切单次 -> 发 RESET -> ConsumePending 丢弃在途 -> 触发 RESET 拍
 *   RESET 拍 ：读出后暂存，恢复自动 SET，进下一周期
 *
 * 【中断内纪律】
 *   不碰浮点（FP 寄存器进出代价大）；不 printf（几百 us 阻塞）。
 *   上报交主循环每秒一次。
 *============================================================================*/
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(MMC_DRDY_EXTI_LINE) != RESET)
    {
        uint8_t d[7];

        EXTI_ClearITPendingBit(MMC_DRDY_EXTI_LINE);

        /* 记下本拍就绪时刻 —— 这是"数据就绪"的时间，不是"读完"的时间 */
        g_mmc_drdy_ts = GetTime64_10Ns();
        g_mmc_isr_cnt++;

        /* --- 1) 读 X/Y/Z：7 字节，三轴共用第 7 字节 d[6] ---
         *   X = d[0]<<10 | d[1]<<2 | d[6]>>6 & 0x03
         *   Y = d[2]<<10 | d[3]<<2 | d[6]>>4 & 0x03
         *   Z = d[4]<<10 | d[5]<<2 | d[6]>>2 & 0x03
         * 注意三轴低 2 位都在 d[6] 里，第三个参数必须都传 d[6]。 */
        if (SPI_ReadRegs(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                         MMC5983MA_REG_XOUT0, d, 7u) == 0)
        {
            g_mmc_raw[0] = (int32_t)MMC5983MA_XYZ_X(d[0], d[1], d[6]);
            g_mmc_raw[1] = (int32_t)MMC5983MA_XYZ_Y(d[2], d[3], d[6]);
            g_mmc_raw[2] = (int32_t)MMC5983MA_XYZ_Z(d[4], d[5], d[6]);
        }
        else
        {
            g_mmc_spi_err++;
        }
        g_mmc_polarity = s_polarity;

        /* --- 2) 清器件中断锁存（写 1 到 STATUS，两位一起清） ---
         * 不清的话 INT 一直保持高，之后再也没有上升沿。 */
        (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                           MMC5983MA_REG_STATUS,
                           (uint8_t)(MMC5983MA_ST_MEAS_M_DONE |
                                     MMC5983MA_ST_MEAS_T_DONE));

        /* --- 3) 配对：本周期 RESET + 下一周期首个 SET ---
         * 这正是归档的做法（_newfunc.c 第 42-44 行）：
         *     px = (pend->x + s0x) / 2.0
         * 其中 pend 是上一周期的 RESET 样本，s0 是本周期读到的第一个 SET。
         * 两者时间上最接近（约 623us），外界磁场相消最新鲜。
         *
         * 判据不用时间戳：读到"本周期第一个 SET"时，若手上还留着上一周期的
         * RESET 样本，就说明该配对了 —— 与固定周期数等价且不受抖动影响。 */
        if (s_polarity == 0u && s_have_rst != 0u)
        {
            g_mmc_pair_sum[0] += ((int64_t)s_pend_rst[0] + (int64_t)g_mmc_raw[0]) / 2;
            g_mmc_pair_sum[1] += ((int64_t)s_pend_rst[1] + (int64_t)g_mmc_raw[1]) / 2;
            g_mmc_pair_sum[2] += ((int64_t)s_pend_rst[2] + (int64_t)g_mmc_raw[2]) / 2;
            g_mmc_pair_n++;
            s_have_rst = 0u;
        }

        /* --- 4) 决定下一拍 --- */
        if (s_polarity == 0u)
        {
            /* 本拍是 SET。SET 段只取【一个】样本，
             * 接下来要切到 RESET 段（归档 b=1 的等价形式）。
             *
             * 归档顺序：先 cfg_ex(0,0,0) 关连续与自动 SET，
             * 再 pulse(RESET)，再 ConsumePending，
             * 最后 TriggerAndRead 读真正的 RESET 样本。 */
            /* 归档 mmc_cfg_ex(0,0,0)：CTRL0 = INT_MEAS_EN（关自动 SET），
             * CTRL2 = CM_FREQ_1000HZ（不带 CMM_EN，即关连续转换）。
             * 两个寄存器都必须写 —— CTRL2 的 CMM_EN 才是连续模式的开关。 */
            (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                               MMC5983MA_REG_CTRL0,
                               MMC5983MA_CTRL0_INT_MEAS_EN);
            (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                               MMC5983MA_REG_CTRL2,
                               (uint8_t)MMC5983MA_CM_FREQ_1000HZ);

            /* 发 RESET 脉冲。
             * 手册 P15：Reset 电流只走 500ns，且 Reset 位在操作结束时自清，
             * 之后没有额外等待要求。这里按手册值加 4 倍余量取
             * MMC_RST_PULSE_WAIT_US(2us)。
             *
             * 归档 mmc_pulse 用的是 100us —— 那是保守值，放在中断里会造成
             * 100us 忙等，把陀螺（周期 130us）拖后，这里不采用。 */
            (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                               MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_RESET);
            HWT_DelayUs(MMC_RST_PULSE_WAIT_US);

            /* 【不调用 MMC_ConsumePending】
             * 归档是在主循环里顺序执行、靠轮询 Status 拿数据，所以需要
             * "消费在途转换"把 RESET 之前那次转换读走丢弃。
             * 本工程改成中断驱动后，数据由 DRDY 中断取 ——
             * 每拍都对应一次完成的转换，不存在"读到旧转换"的问题，
             * 这一步就没有必要。
             * 实测它会白等一整个转换周期（MMC_CONV_PERIOD_US=826us）：
             *   带它  -> 该分支间隔 2.12ms
             *   不带  -> 应降到与另一分支相当（约 1ms 以内）
             */

            /* 触发并读真正的 RESET 样本：本中断返回后由 DRDY 拉起来 */
            (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                               MMC5983MA_REG_CTRL0,
                               (uint8_t)(MMC5983MA_CTRL0_TM_M |
                                         MMC5983MA_CTRL0_INT_MEAS_EN));

            s_polarity = 1u;
        }
        else
        {
            /* 本拍是 RESET。归档第 75-80 行：样本暂存，供下一周期首个 SET 配对，
             * 然后 mmc_auto_set_every() 恢复自动 SET，进下一周期。 */
            s_pend_rst[0] = g_mmc_raw[0];
            s_pend_rst[1] = g_mmc_raw[1];
            s_pend_rst[2] = g_mmc_raw[2];
            s_have_rst = 1u;

            /* 恢复自动 SET + 触发下一拍 SET（归档 mmc_auto_set_every） */
            (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                               MMC5983MA_REG_CTRL0,
                               (uint8_t)(MMC5983MA_CTRL0_TM_M |
                                         MMC5983MA_CTRL0_INT_MEAS_EN |
                                         MMC5983MA_CTRL0_AUTO_SR_EN));

            s_polarity = 0u;
        }
    }
}

/*==============================================================================
 * 9.8b PB10 复位键
 *
 *   按键特性：松开 = 外部弱上拉到 VCC，按下 = 强下拉到地。
 *   配成上拉输入 + EXTI10 下降沿，中断里整片软件复位。
 *
 *   启动保护：上电后 RST_GUARD_US 内忽略复位键。
 *   原因：按键可能在上电瞬间仍被按住（或松手抖动），若不屏蔽会形成
 *   "复位->按键仍按住->立即再复位"的高频连续复位循环，板子永远起不来。
 *   门限用 GetTime64_Us()（TIM9/TIM10 级联的 64 位安全计时，只读寄存器，
 *   ISR 里可用，不依赖任何中断）。
 *
 *   注意：EXTI15_8_IRQn = 33 (>31)，NVIC_SetAllocateIRQ 只对 >31 的有效，
 *         本中断正属于该区间，必须显式分配；否则默认归 V5F。
 *============================================================================*/
#define RST_GUARD_US   1000000ULL       /* 启动后 1 秒内禁止复位 */

static volatile uint64_t s_rst_boot_us = 0;     /* 复位键初始化时刻 */

/*------------------------------------------------------------------------------
 * 最小实验用：EXTI15_8 中断进入次数
 *
 * 中断里只做"清挂起位 + 计数"，主循环只打印这个计数。
 * 这是判断"EXTI 通路是否成立"的唯一依据 —— 不掺任何 DMA/时间戳/共享区逻辑。
 * 定义为全局非 static，供 main.c 用 extern 读取。
 *----------------------------------------------------------------------------*/
volatile uint32_t g_exti_irq_cnt = 0u;

void BOARD_RSTKEY_Init(void)
{
    GPIO_InitTypeDef g = {0};
    EXTI_InitTypeDef e = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOB, ENABLE);

    /* PB10 上拉输入 */
    g.GPIO_Pin   = RST_KEY_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    g.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(RST_KEY_PORT, &g);

    /* PB10 -> EXTI_Line10，下降沿 */
    GPIO_EXTILineConfig(RST_KEY_PORT_SRC, RST_KEY_PIN_SRC);
    e.EXTI_Line    = RST_KEY_EXTI_LINE;
    e.EXTI_Mode    = EXTI_Mode_Interrupt;
    e.EXTI_Trigger = EXTI_Trigger_Falling;
    e.EXTI_LineCmd = ENABLE;
    EXTI_Init(&e);

    /* 记下本时刻，作为启动保护窗口的基准 */
    s_rst_boot_us = GetTime64_Us();

    /*--------------------------------------------------------------------------
     * ★ 这里【不再】使能 EXTI15_8
     *
     * 本向量同时服务 PB10(复位键)、PE1(地磁 DRDY)、PD11(陀螺 DRDY)。
     * 使能它的时机有两个互相冲突的要求：
     *   复位键  —— 越早越好，否则 V3F 一旦卡住就只能断电重启
     *   DRDY    —— 必须在"清掉 INT 锁存、引脚回低"之后，否则自动测量
     *              已经在跑、引脚已经是高电平，EXTI 配的是上升沿，
     *              就永远等不到 0→1 边沿（实测现象：PE1 恒高 + 中断计数 0）
     *
     * 顺序必须是：先清锁存让引脚回低，再使能 NVIC（见 main.c）。
     * 复位键会晚几十毫秒可用，这是可接受的代价。
     *------------------------------------------------------------------------*/
}

/* EXTI15_8 分组中断服务 —— 本组合并了三条线：
 *   PB10  复位键（下降沿）
 *   PE1   地磁 DRDY（上升沿）
 *   PD11  陀螺 DRDY（上升沿）
 * 所以必须逐线判读并各自清挂起位。
 *
 * 【为什么把 DRDY 处理也放在这里而不是另开一个 ISR】
 * 一个分组只有一个向量，无法为同组的线分别注册处理函数；
 * 分成两个 ISR 会变成重复定义。处理体直接写在本函数内，
 * 本文件只做"判线 -> 转调"。
 *
 * ISR 用 WCH-Interrupt-fast 属性，与本文件的复位键 ISR 配套；
 * ch32h417_it.c 里只有系统异常向量。 */
void EXTI15_8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_8_IRQHandler(void)
{
    /*--------------------------------------------------------------------------
     * 本分组服务两条线（都属线号 8..15）：
     *   EXTI_Line11 = PD11 -> 陀螺 LSM6DSV16X INT1
     *   EXTI_Line10 = PB10 -> 复位键
     *
     * ★ 地磁不在这里！地磁是 PE1 = EXTI_Line1，属 EXTI7_0 分组，
     *   处理体在 EXTI7_0_IRQHandler。我先前把地磁也塞进本函数，
     *   那个分支永远不会命中（本向量根本收不到线 1 的事件）。
     *------------------------------------------------------------------------*/

    /* --- 陀螺 DRDY (PD11) —— 本中断抢占级 0，高于地磁 ---
     *
     * 【为什么用 0x20 起 14 字节整块读】
     *   LSM6DSV16X 输出块：0x20..0x2D = 温度2 + 陀螺6 + 加速度计6，
     *   低字节在前、三轴连续。一次片选读完整块，
     *   避免"读陀螺时被加速度计更新打断"导致的三轴不同步。
     *   单次 SPI 事务（1 命令 + 14 数据）比拆成多次片选少约 4 次
     *   CS 切换与 DMA suspend/resume，对 130us 的周期更友好。
     *
     * 【中断内纪律】
     *   不碰浮点（ISR 用 FP 寄存器要额外保存/恢复）；不 printf。
     *   只搬原始 16 位值，换算与上报由主循环做。 */
    if (EXTI_GetITStatus(IMU_INT1_EXTI_LINE) != RESET)
    {
        uint8_t d[LSM6DSV16X_BLOCK_LEN];

        EXTI_ClearITPendingBit(IMU_INT1_EXTI_LINE);

        g_imu_drdy_ts = GetTime64_10Ns();
        g_imu_isr_cnt++;

        if (SPI_ReadRegs(IMU_SPI, IMU_CS_PORT, IMU_CS_PIN,
                         LSM6DSV16X_REG_OUT_TEMP_L_BLK,
                         d, (uint16_t)LSM6DSV16X_BLOCK_LEN) == 0)
        {
            /* 输出块字节序：d[0..1]=温度, d[2..7]=陀螺 XYZ, d[8..13]=加计 XYZ
             * 每轴低字节在前，拼成 16 位补码。 */
            g_imu_raw[0] = (int16_t)((uint16_t)d[2]  | ((uint16_t)d[3]  << 8));
            g_imu_raw[1] = (int16_t)((uint16_t)d[4]  | ((uint16_t)d[5]  << 8));
            g_imu_raw[2] = (int16_t)((uint16_t)d[6]  | ((uint16_t)d[7]  << 8));
            g_imu_raw[3] = (int16_t)((uint16_t)d[8]  | ((uint16_t)d[9]  << 8));
            g_imu_raw[4] = (int16_t)((uint16_t)d[10] | ((uint16_t)d[11] << 8));
            g_imu_raw[5] = (int16_t)((uint16_t)d[12] | ((uint16_t)d[13] << 8));
            g_imu_temp_raw = (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
        }
        else
        {
            g_imu_spi_err++;
        }

        g_exti_irq_cnt++;
    }

    /* --- 复位键 (PB10) --- */
    if (EXTI_GetITStatus(RST_KEY_EXTI_LINE) != RESET)
    {
        EXTI_ClearITPendingBit(RST_KEY_EXTI_LINE);

        /* 启动保护：上电 1s 内按键一律忽略，避免高频连续复位起不来 */
        if ((GetTime64_Us() - s_rst_boot_us) >= RST_GUARD_US)
            NVIC_SystemReset();             /* 整片复位 */
    }
}

/*==============================================================================
 * 9.9 人机：LED / 按键
 *============================================================================*/
void BOARD_HMI_Init(void)
{
    GPIO_InitTypeDef g = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB | RCC_HB2Periph_GPIOF, ENABLE);

    /* LED 推挽输出，先灭 */
    g.GPIO_Pin   = LED1_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(LED1_PORT, &g);

    g.GPIO_Pin   = LED2_PIN;
    g.GPIO_Speed = GPIO_Speed_Low;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(LED2_PORT, &g);

    LED1_OFF();
    LED2_OFF();

    /* 按键上拉输入（PB10 已改作复位键，在 BOARD_RSTKEY_Init 里单独配置） */
    g.GPIO_Pin   = KEY2_PIN;
    g.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(KEY2_PORT, &g);
}

/*==============================================================================
 * 9.11 SPI DMA Mux 绑定
 *
 * 【两条 SPI 都必须用 DMA，所以不能共用通道】
 * 旧架构里 SPI1 与 SPI2 的 TX/RX 都落在 DMA1_Ch3/Ch2 上，只能二选一。
 * 新架构要求两条总线【同时】DMA 收发（地磁 DRDY 中断启动、陀螺 DRDY 中断启动），
 * 因此把 SPI2 挪到 DMA1_Ch4/Ch5，SPI1 保持原位。
 *
 * Mux 通道与 DMA 通道【一一对应】（DMA_MuxChannel1=0x00 -> DMA1_Ch1，依此类推），
 * 见 DMA_MuxChannelConfig() 的分组实现。请求号只取决于外设，与通道无关。
 *
 *   SPI1 (MMC5983MA)   TX = DMA1_Ch3 / Mux3 / REQ63
 *                      RX = DMA1_Ch2 / Mux2 / REQ64
 *   SPI2 (LSM6DSV16X)  TX = DMA1_Ch5 / Mux5 / REQ65
 *                      RX = DMA1_Ch4 / Mux4 / REQ66
 *
 * 请求号出处：EVT SPI_DMA 例程（SPI1_TX=63 SPI1_RX=64，SPI2_TX=65 SPI2_RX=66）。
 *============================================================================*/
#define SPI1_DMA_TX_CH      DMA1_Channel3
#define SPI1_DMA_TX_MUX     DMA_MuxChannel3
#define SPI1_DMA_TX_REQ     63u
#define SPI1_DMA_RX_CH      DMA1_Channel2
#define SPI1_DMA_RX_MUX     DMA_MuxChannel2
#define SPI1_DMA_RX_REQ     64u

#define SPI2_DMA_TX_CH      DMA1_Channel5
#define SPI2_DMA_TX_MUX     DMA_MuxChannel5
#define SPI2_DMA_TX_REQ     65u
#define SPI2_DMA_RX_CH      DMA1_Channel4
#define SPI2_DMA_RX_MUX     DMA_MuxChannel4
#define SPI2_DMA_RX_REQ     66u

void BOARD_SPI1_DMA_MuxInit(void)
{
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    DMA_MuxChannelConfig(SPI1_DMA_TX_MUX, SPI1_DMA_TX_REQ);
    DMA_MuxChannelConfig(SPI1_DMA_RX_MUX, SPI1_DMA_RX_REQ);
}

void BOARD_SPI2_DMA_MuxInit(void)
{
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    DMA_MuxChannelConfig(SPI2_DMA_TX_MUX, SPI2_DMA_TX_REQ);
    DMA_MuxChannelConfig(SPI2_DMA_RX_MUX, SPI2_DMA_RX_REQ);
}

/*==============================================================================
 * 9.10 总入口
 *============================================================================*/
void BOARD_Init(void)
{
    /* 1) GPIO 基线：片选先建立空闲高，再初始化任何总线 */
    BOARD_GPIO_Init();

    /* 2) 磁强计 SPI1 */
#if (BOARD_EN_SPI1_MMC)
    BOARD_SPI1_MMC_Init();
#endif

    /* 3) IMU SPI2 */
#if (BOARD_EN_SPI2_IMU)
    BOARD_SPI2_IMU_Init();
#endif

    /* 4) 气压计软件 I2C */
#if (BOARD_EN_I2C_BMP388)
    BOARD_BMP388_I2C_Init();
#endif

    /* 5) GNSS 串口 */
#if (BOARD_EN_USART2_GNSS)
    BOARD_USART2_GNSS_Init();
#endif

    /* 6) 中断引脚 */
#if (BOARD_EN_EXTI)
    BOARD_EXTI_Init();
#endif

    /* 7) CAN1 */
#if (BOARD_EN_CAN1)
    BOARD_CAN1_Init();
#endif

    /* 8) SD 卡 */
#if (BOARD_EN_SDMMC)
    BOARD_SDMMC_PinInit();
#endif

    /* 9) LED / 按键 */
#if (BOARD_EN_HMI)
    BOARD_HMI_Init();
#endif

    /* 10) PB10 复位键（EXTI10 下降沿 -> 软件复位） */
    BOARD_RSTKEY_Init();
}
