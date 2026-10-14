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
#include "lsm6dsv16x.h"      /* LSM6_ERR_*：陀螺初始化/使能的返回码 */
#include "sensor_dma.h"      /* SENSOR_IMU_FRAME_LEN / READ_CMD：陀螺 TX 帧长 */
#include "mipc_shm.h"        /* shm_publish_mmc：地磁结果发布到共享区 */

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

/* PF0/PF1 有【外部上拉】：网表 PF0 ; R34.2 U1.39 U3.2、PF1 ; R36.2 U1.40 U3.4。
 * 所以开漏输出 + GPIO_SetBits(释放) 就能得到正确的高电平，无需切上拉输入。 */
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

    /* 中断输入：地磁 INT(PE1) + 陀螺 INT1(PD11)，均为【浮空输入】。
     * 这两个是器件输出（手册 P9 pin15 类型 O、P11 "Active High"），
     * 板上有外部上拉，内部再拉会与器件输出级对拉、电平失真。 */
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

    /* Mode 0。手册 P5 的 "SCK is stopped HIGH when CS is high" 描述的是空闲
     * 【状态】而非 CPOL 定义；实测 Mode 3 下数据冻结、数值非法且周期变 7ms。 */
    s.SPI_CPOL              = SPI_CPOL_Low;          /* mode 0 */
    s.SPI_CPHA              = SPI_CPHA_1Edge;

    s.SPI_NSS               = SPI_NSS_Soft;          /* 片选由 PE0 软件控制 */
    s.SPI_FirstBit          = SPI_FirstBit_MSB;      /* MMC5983MA: MSB first */
    s.SPI_CRCPolynomial     = 7;

    /* SCK = FHCLK/16 = 6.25 MHz（BR=011，HSRXEN=0）。
     * 手册 P4：fc(SCK) max 10MHz。实测 12.5MHz 下器件完全不测量（Mode 0）
     * 或数据全错（Mode 3），故不超频。本路带宽非瓶颈（19 字节约 30us，
     * 占 973us 周期的 3%；瓶颈是器件转换时间）。
     * BR[2:0] 通讯中不可修改；CTLR2 的 DMA 请求位改动不涉及 BR。*/
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
 *     要点（照 EVT SDMMC_SD 例程）：SDMMC 引脚只做 GPIO 复用配置，
 *     不调用 GPIO_PinAFConfig；需先设 VIO18 为软件可配模式。
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
 * 9.8 EXTI 配置（只配外设；NVIC 归核由 main.c 的 NVIC_SetAllocateIRQ 决定）
 *     地磁 DRDY = PE1 -> EXTI_Line1  -> EXTI7_0_IRQHandler
 *     陀螺 INT1 = PD11 -> EXTI_Line11 -> EXTI15_8_IRQHandler
 *============================================================================*/
void BOARD_EXTI_Init(void)
{
    EXTI_InitTypeDef e = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO, ENABLE);

    /* 地磁 INT = PE1，上升沿（手册 P11 "Active High"）。
     * 注意：中断功能未使能前该脚是 hi-Z，此时读到的电平只反映 MCU 侧配置，
     * 不能用来判断器件极性。 */
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
 * EXTI 按【线号 0..7 / 8..15】分组，与端口无关：
 *     Line1  (PE1)  -> EXTI7_0_IRQn  (36)  地磁 DRDY
 *     Line11 (PD11) -> EXTI15_8_IRQn (33)  陀螺 INT1
 *     Line10 (PB10) -> EXTI15_8_IRQn (33)  复位键
 *============================================================================*/
/*==============================================================================
 * 地磁 DRDY 中断 —— 读本拍数据，并在同一中断内准备并触发下一拍
 *
 * 每拍耗时约 35us（读 7 字节 + 清锁存 + 极性处理 + 触发），
 * 地磁周期 977us，放中断里节拍最稳。
 * 抢占级必须低于陀螺：陀螺周期只有 130us，本 ISR 若优先会把陀螺推迟约
 * 1/4 周期；反向无影响。优先级在 main.c 设（陀螺 0 / 地磁 1）。
 *
 * 纪律：不碰浮点（RISC-V ISR 用 FP 寄存器需额外保存/恢复）；
 *       不 printf（几百 us 阻塞，串口也是共享资源）。上报交主循环。
 *============================================================================*/
volatile int32_t  g_mmc_raw[3]      = {0, 0, 0};   /* X/Y/Z 原始 counts */
volatile uint8_t  g_mmc_polarity    = 0u;          /* 本拍极性：0=SET 1=RESET */
volatile uint32_t g_mmc_isr_cnt     = 0u;          /* 中断进入次数 */
volatile uint32_t g_mmc_spi_err     = 0u;          /* 中断内 SPI 失败次数 */
volatile uint64_t g_mmc_drdy_ts     = 0u;          /* 本拍 DRDY 时刻（10ns）*/

/* SET/RESET 配对累加（中断内完成）。
 * 极性每拍翻转，故相邻两拍必为一 SET 一 RESET —— 用"上一拍 + 本拍"配对，
 * 即是物理时间上最近的一对，外界磁场相消最干净。
 * 累加而非覆盖：主循环每秒才取一次，覆盖会丢掉这一秒内的其它对。 */
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
volatile uint32_t g_imu_isr_cnt  = 0u;   /* 陀螺 DRDY 次数 */
volatile uint32_t g_imu_spi_err  = 0u;
volatile uint64_t g_imu_drdy_ts  = 0u;   /* 本拍 DRDY 时刻（10ns）*/

/* 当前拍的极性：0 = SET（连续 + 自动 SET），1 = RESET（单次触发） */
static volatile uint8_t s_polarity = 0u;

/* ★ 每 MMC_FLIP_PERIOD 个 SET 周期才做一次 RESET 周期（原实现为每拍翻转）。
 *   复位后由 RESET 分支清零，重新累计。
 *   ★ 2026-10-14：本周期计数降级为【兼容/兜底】路径 —— 当共享区的
 *     mmc_rst_hz_x10 == 0（V5F 未接管）时仍按它周期性翻转；
 *     V5F 一旦置 mmc_rst_hz_x10 != 0，翻转时机改由 mmc_rst_req 按需驱动。 */
#define MMC_FLIP_PERIOD         1024u
static volatile uint32_t s_mmc_flip_cnt = 0u;

/* ★★ 2026-10-14 新增：按需 RESET 的本地记账。
 *   语义："已受理到的 mmc_rst_req 序号"。在 MMC DRDY 中断里与共享区比对，
 *   不等则本拍切进 RESET 段。单写单读（只有本 ISR 写），无需原子操作。 */
static volatile uint32_t s_mmc_rst_seen = 0u;

/* ★★ 2026-10-14 新增：连续 SET 段是否已让器件进入【连续采样模式】。
 *
 * 机理（本轮修正的核心）：
 *   MMC5983MA 的 CTRL2.Cmm_en=1 时，器件【自主】按 CM_FREQ 连续测量并拉 DRDY；
 *   而 CTRL0.TM_M 是"触发【一次】测量"。原实现在连续 SET 分支里【每拍都写
 *   CTRL0 = TM_M|INT_MEAS_EN|AUTO_SR_EN】⟹ 等价于每拍单次触发 ⟹ 器件被迫等
 *   ISR 来喂下一拍 ⟹ 速率被 ISR 循环钳住（实测 768 Hz，而器件 BW=11 下可达
 *   1000+ Hz）。且当时 CTRL2.Cmm_en 在第一次 RESET 段被关掉后【从未恢复】。
 *
 * 正确做法（本轮）：
 *   连续 SET 段: 只保证 CTRL2 = CM_FREQ|Cmm_en|Prd_set_en 已被写一次，
 *                【不再每拍写 CTRL0】⟹ 器件自主全速采样。
 *   需要 RST 时: 切出连续模式 → 现有 RESET 特殊时序 → 收尾时【恢复连续模式】。
 * 本标志用于"只写一次"，避免每拍重复写 CTRL2。 */
static volatile uint8_t s_mmc_cmm_on = 0u;

/* 连续模式的 CTRL2 组合值：CM_FREQ_1000HZ | CMM_EN | PRD_SET_EN = 0x8F */
#define MMC_CTRL2_CMM_COMBO  ((uint8_t)(MMC5983MA_CM_FREQ_1000HZ | \
                                        MMC5983MA_CTRL2_CMM_EN |     \
                                        MMC5983MA_CTRL2_EN_PRD_SET))

/* 上一周期的 RESET 样本（暂存），与下一周期首个 SET 配对 —— 归档同一机制。
 * 配对窗口约 623us，是 SET/RESET 之间时间上最接近的一对。 */
static int32_t  s_pend_rst[3] = {0, 0, 0};
static uint8_t  s_have_rst    = 0u;

/*==============================================================================
 * 地磁 DRDY 中断
 *
 * SET 拍：读样本 -> 清锁存 -> 关连续/关自动SET -> 发 RESET 脉冲
 *         -> 等 MMC_RST_PULSE_WAIT_US -> 触发 RESET 拍
 * RESET 拍：读样本 -> 清锁存 -> 暂存供下周期配对 -> 恢复自动 SET 并触发 SET 拍
 *============================================================================*/
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(MMC_DRDY_EXTI_LINE) != RESET)
    {
        uint8_t d[7];

        EXTI_ClearITPendingBit(MMC_DRDY_EXTI_LINE);

        g_mmc_drdy_ts = GetTime64_10Ns();
        g_mmc_isr_cnt++;

        /* 写共享区计数，供 V5F 上报地磁实际采样率 */
        if (g_shm != 0)
            g_shm->mmc_start_cnt = g_mmc_isr_cnt;

        /* 读 7 字节：三轴低 2 位都在 d[6] 里，第三个参数必须都传 d[6] */
        if (SPI_ReadRegs(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                         MMC5983MA_REG_XOUT0, d, 7u) == 0)
        {
            g_mmc_raw[0] = (int32_t)MMC5983MA_XYZ_X(d[0], d[1], d[6]);
            g_mmc_raw[1] = (int32_t)MMC5983MA_XYZ_Y(d[2], d[3], d[6]);
            g_mmc_raw[2] = (int32_t)MMC5983MA_XYZ_Z(d[4], d[5], d[6]);

            /* 发布到共享区，供 V5F 在陀螺 DMA 中断里轮询取用。
             * 屏障由 shm_chan_ts_write 内的 __sync_synchronize 提供。
             * ★ 第 5 个参数是【本拍极性】：mipc_shm.h 的 shm_publish_mmc 需要它。 */
            if (g_shm != 0)
                shm_publish_mmc(g_mmc_drdy_ts,
                                g_mmc_raw[0], g_mmc_raw[1], g_mmc_raw[2],
                                s_polarity);
        }
        else
        {
            g_mmc_spi_err++;
        }
        g_mmc_polarity = s_polarity;

        /* 清锁存（写 1 到 STATUS 两位）。不清则 INT 恒高，之后没有上升沿。 */
        (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                           MMC5983MA_REG_STATUS,
                           (uint8_t)(MMC5983MA_ST_MEAS_M_DONE |
                                     MMC5983MA_ST_MEAS_T_DONE));

        /* ★★ 2026-10-14 修正：此处原累加 (RST + SET)/2 —— 那是【两相中点】，
         *   即 地磁解析根因_定位.md §五-1 明令禁止的 E4 口径。
         *   实测两相是两个不同心的球（球心距 3588 counts = 场强 43.15%），
         *   中点不是任何一相的球心，残余 |Δ|/2 = 1794 counts = 场强 21.6%，
         *   会产生 ±21.6% 的确定性方向调制。
         *   该累加器（g_mmc_pair_sum / g_mmc_pair_n）全工程【只写不读】，从未生效，
         *   故这里停止累加，避免后人误用。声明保留不改（避免 ABI/引用面变化）。
         *   ★ 若将来要用这个对，正确的量是【差分】(SET − RST)/2 —— 它等于场矢量，
         *     对硬磁零点免疫；不是零点。 */
        if (s_polarity == 0u && s_have_rst != 0u)
        {
            s_have_rst = 0u;            /* 仅消费暂存，不再累加中点 */
        }

        /* --- 4) 决定下一拍 ---
         * ★ 2026-10-14 改动：RESET 时机由【V5F 的按需请求】驱动，取代本地周期计数。
         *   动机：主算法端发起 ⟹ 时刻可知可记录 ⟹ SET/RESET 对在时间上良定义，
         *         可做【差分】(SET−RST)/2 交叉校验（对硬磁零点免疫）。
         *   兼容：mmc_rst_hz_x10 == 0 或共享区不可用时，退回本地 MMC_FLIP_PERIOD 周期。
         *   ★ RESET 段两个分支体内的寄存器操作与等待【一字未改】，
         *     只换了"何时进入"的判据。 */
        {
            /* ★★ 2026-10-14：状态优先 —— 正在 RESET 序列中（s_polarity==1）
             *   必须【走完】，绝不能被 want_rst 判据打断。
             *   否则：request 受理时已把 rst_seen 记到 req，下一拍 want_rst 变 0，
             *   状态机就再也进不到收尾分支 ⟹ s_polarity 永远停在 1、
             *   done 永不增长、pol 上报全错（把 SET 报成 RST）⟹ SET-only 管线全毁。
             *   该缺陷由 verify_rst_fsm.py 的状态机仿真抓出。 */
            if (s_polarity == 1u)
            {
                /* 本拍是 RESET：暂存供下一周期首个 SET 配对，然后恢复自动 SET */
                s_pend_rst[0] = g_mmc_raw[0];
                s_pend_rst[1] = g_mmc_raw[1];
                s_pend_rst[2] = g_mmc_raw[2];
                s_have_rst = 1u;

                /* ★★ 2026-10-14 修正（核心）：RESET 周期收尾必须【恢复连续模式】。
                 * 原实现只写 CTRL0，从不把 Cmm_en 恢复进 CTRL2 ⟹
                 * 第一次 RESET 之后器件永久退出连续采样，此后每拍都要 ISR 显式触发
                 * ⟹ 速率被钳住。这里把 CTRL2 写回连续模式组合，并清记账。 */
                (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                                   MMC5983MA_REG_CTRL0,
                                   (uint8_t)(MMC5983MA_CTRL0_INT_MEAS_EN |
                                             MMC5983MA_CTRL0_AUTO_SR_EN));
                (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                                   MMC5983MA_REG_CTRL2, MMC_CTRL2_CMM_COMBO);
                s_mmc_cmm_on = 1u;

                /* ★ 本次 RESET 周期结束：计数清零，回到连续 SET 段 */
                s_mmc_flip_cnt = 0u;
                s_polarity = 0u;

                /* ★★ 给 V5F 的回执。按需模式下 req/done 应同步增长；
                 *   若 req 持续领先 done，说明本 ISR 未在跑（地磁失效，见 MMC_RecoverPoll）。 */
                if (g_shm != 0)
                    g_shm->mmc_rst_done++;
            }
            else
            {
                uint8_t want_rst = 0u;

                if (g_shm != 0)
                {
                    if (g_shm->mmc_rst_hz_x10 != 0u)
                    {
                        /* 按需模式：存在未受理的请求 ⟹ 本拍切进 RESET 段 */
                        if (g_shm->mmc_rst_req != s_mmc_rst_seen)
                            want_rst = 1u;
                    }
                    else if (s_mmc_flip_cnt >= MMC_FLIP_PERIOD)
                    {
                        want_rst = 1u;      /* 兼容：本地周期到点 */
                    }
                }
                else if (s_mmc_flip_cnt >= MMC_FLIP_PERIOD)
                {
                    want_rst = 1u;
                }

                if (want_rst == 0u)
                {
                    /* ★★ 2026-10-14 修正（核心）：
                     * 连续 SET 段【不再每拍写 CTRL0 触发】。
                     * 器件在 Cmm_en=1 时自主按 CM_FREQ 连续测量并拉 DRDY；
                     * 每拍写 TM_M 等于强制单次触发 ⟹ 速率被 ISR 循环钳住。
                     * 这里只保证连续模式的 CTRL2 组合已被写过一次。 */
                    if (s_mmc_cmm_on == 0u)
                    {
                        (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                                           MMC5983MA_REG_CTRL0,
                                           (uint8_t)(MMC5983MA_CTRL0_INT_MEAS_EN |
                                                     MMC5983MA_CTRL0_AUTO_SR_EN));
                        (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                                           MMC5983MA_REG_CTRL2, MMC_CTRL2_CMM_COMBO);
                        s_mmc_cmm_on = 1u;
                    }
                    if (s_mmc_flip_cnt < 0xFFFFFFFFu) s_mmc_flip_cnt++;
                }
                else
                {
                    /* ★ 受理本请求（按需模式下才记账；兼容模式下计数已在 RESET 段末尾清零） */
                    if (g_shm != 0 && g_shm->mmc_rst_hz_x10 != 0u)
                        s_mmc_rst_seen = g_shm->mmc_rst_req;

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
                    s_mmc_cmm_on = 0u;      /* ★ 已切出连续模式 */

                    /* RESET 脉冲；等待时间见 mmc5983.h 的 MMC_RST_PULSE_WAIT_US */
                    (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                                       MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_RESET);
                    HWT_DelayUs(MMC_RST_PULSE_WAIT_US);

                    /* 不需要 MMC_ConsumePending：数据由 DRDY 中断取，
                     * 每拍都对应一次完成的转换，不存在读到旧转换的问题；
                     * 调用它反而会白等一整个转换周期（约 826us）。 */

                    /* 触发并读真正的 RESET 样本：本中断返回后由 DRDY 拉起来 */
                    (void)SPI_WriteReg(MMC_SPI, MMC_CS_PORT, MMC_CS_PIN,
                                       MMC5983MA_REG_CTRL0,
                                       (uint8_t)(MMC5983MA_CTRL0_TM_M |
                                                 MMC5983MA_CTRL0_INT_MEAS_EN));

                    s_polarity = 1u;
                }
            }
        }   /* ★ 结束 2026-10-14 新增的作用域块 */
    }
}

/*==============================================================================
 * 9.8b PB10 复位键
 * 上拉输入 + EXTI10 下降沿，中断里整片软件复位。
 * 上电后 RST_GUARD_US 内忽略按键：否则"复位->按键仍按住->立即再复位"
 * 会形成高频复位循环，板子起不来。
 * EXTI15_8_IRQn = 33 (>31)，必须显式 NVIC_SetAllocateIRQ，否则默认归 V5F。
 *============================================================================*/
#define RST_GUARD_US   1000000ULL       /* 启动后 1 秒内禁止复位 */

static volatile uint64_t s_rst_boot_us = 0;     /* 复位键初始化时刻 */

/* EXTI15_8（陀螺 + 复位键）进入次数 */
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

    /* 此处不使能 EXTI15_8：两个 DRDY 都要求"先清 INT 锁存让引脚回低、
     * 再开 NVIC"，否则锁存态下没有 0→1 边沿。NVIC 由 main.c 统一开。 */
}

/* EXTI15_8 分组中断服务：一组合并两条线，必须逐线判读并各自清挂起位。
 *   Line11 = PD11  陀螺 INT1（上升沿）
 *   Line10 = PB10  复位键（下降沿）
 * 一个分组只有一个向量，无法为同组的线分别注册处理函数。 */
void EXTI15_8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_8_IRQHandler(void)
{

    /* 陀螺 DRDY (PD11)：只启动 DMA，不搬数据。
     * 拉低 CS + 启动 SPI2 TX DMA（命令 + dummy）后立即返回；
     * 数据由 RX DMA 落进【大核自己的】缓冲，完成中断在大核侧解析并抬 CS。 */
    if (EXTI_GetITStatus(IMU_INT1_EXTI_LINE) != RESET)
    {
        EXTI_ClearITPendingBit(IMU_INT1_EXTI_LINE);
        g_imu_drdy_ts = GetTime64_10Ns();
        g_imu_isr_cnt++;
        if (g_shm != 0)
        {
            g_shm->imu_start_cnt = g_imu_isr_cnt;   /* 供 V5F 对比丢帧 */
            /* ★★ 把 DRDY 时刻经【共享内存】交给 V5F（2026-10-13）。
             * 跨核唯一通道就是共享内存：本 ISR 里写的 g_imu_drdy_ts 属于 V3F 镜像，
             * V5F 直接引用只会读到它自己的初值 0 ⟹ 发布时戳恒 0 ⟹
             * proc_chain 的 dt_s=0 ⟹ 提前 return ⟹ ESKF 从未被调用
             * ⟹ calib_n≡0/ready≡0/四元数停在 [1,0,0,0]。
             * 复用 chan_hdr_t 的 hi/lo + 屏障协议，解决 64 位跨核读取原子性。 */
            shm_chan_ts_write(&g_shm->imu_drdy, g_imu_drdy_ts);
            g_shm->imu_drdy.cnt++;
        }
        BOARD_IMU_DrdyStart();                      /* 拉低 CS + 启动 TX DMA */
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
 * 9.12 陀螺 SPI2 的 TX 启动（DMA 采集的小核半边）
 *
 * 分工照 v2：小核只把时钟打出去，不碰 RX。
 *   BOARD_IMU_DrdyStart()：填 TX 缓冲 -> 拉低 CS -> 启动 TX DMA -> 立即返回
 *   RX 通道由大核配到它自己的缓冲，完成中断也在大核：
 *     解析 -> 发布共享区 -> 抬 CS -> 重装 RX
 *
 * TX 缓冲是本核私有 static，不跨核共享。
 *============================================================================*/
#define IMU_DMA_TX_CH    DMA1_Channel5

static uint8_t s_imu_tx[SENSOR_IMU_FRAME_LEN];

void BOARD_IMU_DmaInit(void)
{
    DMA_InitTypeDef d = {0};
    uint8_t i;

    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);

    s_imu_tx[0] = (uint8_t)(SENSOR_IMU_READ_CMD | 0x80u);
    for (i = 1u; i < SENSOR_IMU_FRAME_LEN; i++) s_imu_tx[i] = 0xFFu;

    DMA_DeInit(IMU_DMA_TX_CH);
    d.DMA_PeripheralBaseAddr = (uint32_t)(&SPI2->DATAR);
    d.DMA_Memory0BaseAddr    = (uint32_t)s_imu_tx;
    d.DMA_DIR                = DMA_DIR_PeripheralDST;       /* 内存 -> 外设 */
    d.DMA_BufferSize         = SENSOR_IMU_FRAME_LEN;
    d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    d.DMA_Mode               = DMA_Mode_Normal;
    d.DMA_Priority           = DMA_Priority_VeryHigh;
    d.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(IMU_DMA_TX_CH, &d);

    /* Mux 必须 TX 与 RX 都绑 —— 用现成的组合函数，别只手绑 TX：
     * 少了 RX 的绑定，RX 通道就收不到外设请求，DMA 永不完成，
     * 大核的完成中断也就永不触发。 */
    BOARD_SPI2_DMA_MuxInit();

    /* 使能 SPI2 的 DMA 收发请求（RX 通道本体由大核配置，这里只管请求位开着） */
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
}

void BOARD_IMU_DrdyStart(void)
{
    IMU_CS_LOW();

    /* CFGR 是 16 位寄存器，必须按 16 位操作，否则会把高 16 位误清零 */
    IMU_DMA_TX_CH->CFGR &= (uint16_t)(~DMA_CFGR1_EN);
    IMU_DMA_TX_CH->MADDR = (uint32_t)s_imu_tx;
    IMU_DMA_TX_CH->CNTR  = SENSOR_IMU_FRAME_LEN;
    IMU_DMA_TX_CH->CFGR |= DMA_CFGR1_EN;

    /* 立即返回：不等传输完成。CS 抬起与数据解析都在大核的完成中断里做。 */
}

/*==============================================================================
 * 9.13 陀螺 LSM6DSV16X 的初始化与使能
 *
 * 【为什么放在板级配置库而不是驱动里】
 *   这两个函数不依赖任何全局量或静态量 —— 只用无状态的 SPI_ReadReg/WriteReg
 *   （句柄由参数给）与 GetTime64 计时。所以【两个核都能直接调用】，
 *   与总线对双核对称的性质一致，不需要任何跨核约定或条件编译。
 *
 *   消费方（大核）自己使能：它先把自己的 RX DMA 武装好，再调本函数启动陀螺，
 *   这样第一帧 DRDY 一产生就能被读取，不会出现"数据就绪却无人读 ->
 *   INT 锁存为高 -> 上升沿再也不来"的死局。
 *============================================================================*/

#define IMU_SPI_HANDLE()   IMU_SPI, IMU_CS_PORT, IMU_CS_PIN

/* 配置陀螺；使能前仍处于掉电模式（ODR=0000，不产数据） */
int BOARD_IMU_Init(void)
{
    uint8_t v = 0u;

    /* 1) 软复位 */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL3,
                     LSM6DSV16X_CTRL3_SW_RESET) != 0)
        return LSM6_ERR_SPI;
    HWT_DelayUs(1000u);                 /* 手册未给复位时间，1ms 足够 */

    /* 2) 校验 WHO_AM_I */
    if (SPI_ReadReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_WHO_AM_I, &v) != 0)
        return LSM6_ERR_SPI;
    if (v != LSM6DSV16X_WHO_AM_I_VAL)
        return LSM6_ERR_NO_ID;

    /* 3) 接口配置：INT 高有效 + 推挽 + 4线SPI。该寄存器【不随软复位复位】
     *    （手册 §9.3），必须显式写，不能依赖默认值。 */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_IF_CFG,
                     LSM6DSV16X_IFCFG_OUR_CFG) != 0)
        return LSM6_ERR_SPI;

    /* 4) BDU=1 + IF_INC=1。BDU 保证读低/高字节之间输出寄存器不更新，
     *    对 7.68kHz 这种高速率尤其必要。 */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL3,
                     LSM6DSV16X_CFG_CTRL3_DEFAULT) != 0)
        return LSM6_ERR_SPI;

    /* 5) 陀螺满量程 ±4000dps（要求 OIS_G_EN=0，复位默认为 0） */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL6,
                     LSM6DSV16X_CFG_CTRL6_GYRO_4000) != 0)
        return LSM6_ERR_SPI;

    /* 6) 关闭数字 LPF1（手册未给该寄存器复位值，故显式写 0）。
     *    关闭后仍有模拟抗混叠与 LPF2，群延迟最小（约 0.30ms），
     *    代价是噪声升到约 81 mdps RMS。陀螺要积分，群延迟直接变姿态误差。 */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL7,
                     LSM6DSV16X_CTRL7_ALL_OFF) != 0)
        return LSM6_ERR_SPI;

    /* 7b) ★ 加计满量程：CTRL8.FS_XL = 11 = ±16 g
     *   原来【漏了这一写】，器件停在复位默认 00 = ±2 g，
     *   而全链按 ±16 g 的标度 ACCEL_G_PER_LSB = 0.000488 g/LSB 换算，
     *   于是 |a| 被放大 8 倍（实测静止时读出 8.04 g），
     *   导致 ESKF 的模长门 | |a| − 1g | < 0.05g 永不满足、启动标定 valid=0、
     *   ready 始终为 0，整条重力链路失效。
     *   HP_LPF2_XL_BW 保持 000、XL_DualC_EN 保持 0（本板不做任何滤波）。 */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL8,
                     LSM6DSV16X_CTRL8_FS_XL_16G) != 0)
        return LSM6_ERR_SPI;

    /* 7) DRDY_MASK=1：滤波器建立期间（约 1.86ms）不置 DRDY，
     *    否则最初十几个样本全是无效的建立值。 */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL4,
                     LSM6DSV16X_CTRL4_DRDY_MASK) != 0)
        return LSM6_ERR_SPI;

    /* 8) 进入 HAODR（写 OP_MODE=001，此时 ODR 仍为 0000，满足手册 §6.5
     *    "HAODR 必须在掉电模式下使能"的约束） */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL1,
                     LSM6DSV16X_XL_OPMODE_HAODR) != 0)
        return LSM6_ERR_SPI;
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL2,
                     LSM6DSV16X_G_OPMODE_HAODR) != 0)
        return LSM6_ERR_SPI;

    /* 9) 陀螺数据就绪路由到 INT1(PD11)：INT1_CTRL.INT1_DRDY_G = bit1 */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_INT1_CTRL,
                     LSM6DSV16X_INT1_DRDY_G) != 0)
        return LSM6_ERR_SPI;

    /* 10) HAODR 数据率表：00 -> 最高档 7680Hz */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_HAODR_CFG,
                     LSM6DSV16X_CFG_HAODR_SEL_00) != 0)
        return LSM6_ERR_SPI;

    /* 到这里器件已配好但仍处于掉电模式。启动转换必须由 BOARD_IMU_Enable()
     * 在 EXTI 使能且消费方 RX 就绪之后单独做。 */
    printf("LSM6,cfg_ok,odr=7680,fs_g=4000,dps_per_lsb=%.3f\r\n",
           (double)LSM6_GYRO_DPS_PER_LSB);
    return LSM6_OK;
}

/* 启动转换：把 ODR 写成非零，从掉电切到 HAODR 最高速。
 * 必须在 DRDY 的 EXTI 使能【且】消费方的 RX 已就绪之后调用 ——
 * 早使能会让 INT 在无人读取时锁存为高，之后没有上升沿。 */
int BOARD_IMU_Enable(void)
{
    /* 加计与陀螺同步进 HAODR（手册 §6.5：两者必须一致） */
    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL2,
                     LSM6DSV16X_CFG_CTRL2_GYRO_HAODR_MAX) != 0)
        return LSM6_ERR_SPI;

    if (SPI_WriteReg(IMU_SPI_HANDLE(), LSM6DSV16X_REG_CTRL1,
                     LSM6DSV16X_CFG_CTRL1_XL_HAODR_MAX) != 0)
        return LSM6_ERR_SPI;

    /* 建立等待 5ms：DRDY_MASK 只改变置起时机、不屏蔽建立过程，
     * 加掩码后仍有单调收敛（幅度约 4dps，时间常数约 550us），取 4~5 个时间常数。 */
    HWT_DelayUs(5000u);

    printf("LSM6,enabled,odr=7680\r\n");
    return LSM6_OK;
}

/*==============================================================================
 * 9.11 SPI DMA Mux 绑定

 * 两条 SPI 各占一组通道（Mux N 对应 DMA1_ChN，请求号只取决于外设）：
 *   SPI1  TX=Ch3/Mux3/REQ63   RX=Ch2/Mux2/REQ64
 *   SPI2  TX=Ch5/Mux5/REQ65   RX=Ch4/Mux4/REQ66
 * 请求号出处：EVT SPI_DMA 例程。
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
