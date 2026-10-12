/*******************************************************************************
* File Name          : board_io.h
* Description        : CH32H417WEU6 (QFN-68) 全 IO / 外设初始化
*
*   板卡  : Board1_Schematic1 (BOM / 网表 版本 2026-10-08)
*   芯片  : CH32H417WEU6  QFN-68 8x8mm 0.40mm pitch EP5.8
*
*   依据  : H417WEU6.json   (管脚号 <-> 端口，权威)
*           h417.json      (外设 <-> 信号 <-> 端口(AF号))
*           Netlist_Schematic1_2026-10-08.tel (实际连接)
*           CH32H417EVT    (SDMMC_SD / SPI_DMA / CAN_TestMode / GPIO 例程)
*
*   与旧固件(前代板)的关键差异，务必注意：
*     1. MMC5983MA 片选 = PE0 (GPIO 输出)，旧固件是 PF4
*     2. MMC5983MA 数据就绪 = PE1 (EXTI)，旧固件是 PE0
*        (PE0 是片选，不能当输入用)
*     3. IMU (LSM6DSV16XTR) 在 SPI2: PB13/PB14/PC3，不在 SPI1
*        其中断 INT1/INT2 -> PD11 / PD10
*     4. BMP388 在 PF0(SCL)/PF1(SDA) 软件 I2C，不是 PD12/PD13
*     5. GNSS 在 USART2: PD5(TX)/PD6(RX)，PD12/PD13 是调试串口 USART1
*     6. SD 卡走 SDMMC 默认映射 RM=00: PC8/9/10/11(D0-D3) PC12(CK)
*        PD2(CMD) PD3(STR) PC6/PC7(D6/D7) PA14/PA15(D4/D5)
*     7. PA13 是 SWDIO，勿占用；PD12/PD13/PD14/PD15 已被占用
*     8. USBHS(PB8/PB9) 是专用功能，无 AF；SWJ 关闭后 USBHS 独占
*******************************************************************************/
#ifndef __BOARD_IO_H
#define __BOARD_IO_H

#include "ch32h417.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 0. 功能开关
 *    两个核共用本文件；各核把用不到的设成 0 即可（避免双核抢同一外设）。
 *    BOARD_Init() 会按下面的宏顺序初始化。
 *============================================================================*/
#ifndef BOARD_EN_SPI1_MMC
#define BOARD_EN_SPI1_MMC       1       /* 磁强计 SPI1 + 片选 PE0 */
#endif
#ifndef BOARD_EN_SPI2_IMU
#define BOARD_EN_SPI2_IMU       1       /* IMU SPI2 */
#endif
/* 下面这些模块的实现已归档到 _archive/legacy_disabled_features/，
 * 默认必须为 0 —— 否则 board_io.c 里被 #if 门控的初始化会引用不存在的符号。
 * 若要恢复某模块，把文件移回本目录并置 1。 */
#ifndef BOARD_EN_I2C_BMP388
#define BOARD_EN_I2C_BMP388     1       /* 气压计软件 I2C (PF0/PF1) */
#endif
#ifndef BOARD_EN_USART2_GNSS
#define BOARD_EN_USART2_GNSS    0       /* GNSS 串口（驱动已归档） */
#endif
#ifndef BOARD_EN_CAN1
#define BOARD_EN_CAN1           0       /* CAN1（未实现） */
#endif
#ifndef BOARD_EN_SDMMC
#define BOARD_EN_SDMMC          0       /* SD 卡（驱动已归档） */
#endif
#ifndef BOARD_EN_EXTI
#define BOARD_EN_EXTI           1       /* 磁强计/IMU 中断引脚 + EXTI */
#endif
#ifndef BOARD_EN_HMI
#define BOARD_EN_HMI            1       /* LED / 按键 */
#endif

/* CAN1 模式：默认静默，避免未接总线时反复报错 */
#ifndef BOARD_CAN1_MODE
#define BOARD_CAN1_MODE         CAN_Mode_Silent
#endif

/*==============================================================================
 * 1. 磁强计 MMC5983MA  —— SPI1 (AF5)
 *============================================================================*/
#define MMC_SPI                 SPI1
#define MMC_SPI_SCK_PORT        GPIOF
#define MMC_SPI_SCK_PIN         GPIO_Pin_5      /* 管脚 62  PF5 */
#define MMC_SPI_SCK_SRC         GPIO_PinSource5
#define MMC_SPI_MISO_PORT       GPIOF
#define MMC_SPI_MISO_PIN        GPIO_Pin_3      /* 管脚 60  PF3 */
#define MMC_SPI_MISO_SRC        GPIO_PinSource3
#define MMC_SPI_MOSI_PORT       GPIOD
#define MMC_SPI_MOSI_PIN        GPIO_Pin_7      /* 管脚 59  PD7 */
#define MMC_SPI_MOSI_SRC        GPIO_PinSource7
#define MMC_SPI_AF              GPIO_AF5

#define MMC_CS_PORT             GPIOE
#define MMC_CS_PIN              GPIO_Pin_0      /* 管脚 63  PE0  片选，空闲高 */
#define MMC_DRDY_PORT           GPIOE
#define MMC_DRDY_PIN            GPIO_Pin_1      /* 管脚 64  PE1  INT 数据就绪 */
#define MMC_DRDY_EXTI_LINE      EXTI_Line1
#define MMC_DRDY_PIN_SRC        GPIO_PinSource1

#define MMC_CS_LOW()            GPIO_ResetBits(MMC_CS_PORT, MMC_CS_PIN)
#define MMC_CS_HIGH()           GPIO_SetBits(MMC_CS_PORT, MMC_CS_PIN)
#define MMC_DRDY_READ()         GPIO_ReadInputDataBit(MMC_DRDY_PORT, MMC_DRDY_PIN)

/*==============================================================================
 * 2. 六轴 IMU  LSM6DSV16XTR  —— SPI2 (AF5)
 *============================================================================*/
#define IMU_SPI                 SPI2
#define IMU_SPI_SCK_PORT        GPIOB
#define IMU_SPI_SCK_PIN         GPIO_Pin_13     /* 管脚 31  PB13 */
#define IMU_SPI_SCK_SRC         GPIO_PinSource13
#define IMU_SPI_MISO_PORT       GPIOB
#define IMU_SPI_MISO_PIN        GPIO_Pin_14     /* 管脚 32  PB14 */
#define IMU_SPI_MISO_SRC        GPIO_PinSource14
#define IMU_SPI_MOSI_PORT       GPIOC
#define IMU_SPI_MOSI_PIN        GPIO_Pin_3      /* 管脚 18  PC3 */
#define IMU_SPI_MOSI_SRC        GPIO_PinSource3
#define IMU_SPI_AF              GPIO_AF5

/* U4 的 CS = PD10（网表：PD10 ; U1.33 U4.12）。
 * LSM6DSV16X 手册 pin12 CS：0 = SPI 通信模式，1 = SPI idle/I2C/I3C。
 * 故做 SPI 时必须拉低。 */
#ifndef BOARD_IMU_HAS_CS
#define BOARD_IMU_HAS_CS        1
#endif
#define IMU_CS_PORT             GPIOD
#define IMU_CS_PIN              GPIO_Pin_10     /* 管脚 33  PD10 -> U4.12 (CS) */
#define IMU_CS_LOW()            GPIO_ResetBits(IMU_CS_PORT, IMU_CS_PIN)
#define IMU_CS_HIGH()           GPIO_SetBits(IMU_CS_PORT, IMU_CS_PIN)

#define IMU_INT1_PORT           GPIOD
#define IMU_INT1_PIN            GPIO_Pin_11     /* 管脚 34  PD11 -> U4.4 (INT1) */
#define IMU_INT1_EXTI_LINE      EXTI_Line11
#define IMU_INT1_PIN_SRC        GPIO_PinSource11

#define IMU_INT1_READ()         GPIO_ReadInputDataBit(IMU_INT1_PORT, IMU_INT1_PIN)
/* LSM6DSV16X 只有 INT1 接到 MCU（PD11）；INT2 未连出，故无 IMU_INT2_* */

/*==============================================================================
 * 3. 气压计 BMP388  —— 软件 I2C
 *    U3: 2=SCL, 4=SDA, 1/10=VDDIO(3V3), 3/8/9=GND
 *    CSB 接高 -> I2C 模式；SDO 接 GND -> 地址 0x76
 *    板上该两脚有外部上拉，MCU 侧用开漏：写 0 拉低、写 1 释放
 *============================================================================*/
#define BMP_I2C_PORT            GPIOF
#define BMP_I2C_SCL_PIN         GPIO_Pin_0      /* 管脚 39  PF0 */
#define BMP_I2C_SDA_PIN         GPIO_Pin_1      /* 管脚 40  PF1 */
#define BMP_I2C_SPARE_PIN       GPIO_Pin_2      /* 管脚 41  PF2  (预留) */
#define BMP388_I2C_ADDR7        0x76

/*==============================================================================
 * 4. GNSS  ATGM336H  —— USART2 (AF7)
 *============================================================================*/
#define GNSS_USART              USART2
#define GNSS_TX_PORT            GPIOD
#define GNSS_TX_PIN             GPIO_Pin_5      /* 管脚 57  PD5 -> 模组 RXD */
#define GNSS_TX_SRC             GPIO_PinSource5
#define GNSS_RX_PORT            GPIOD
#define GNSS_RX_PIN             GPIO_Pin_6      /* 管脚 58  PD6 <- 模组 TXD */
#define GNSS_RX_SRC             GPIO_PinSource6
#define GNSS_USART_AF           GPIO_AF7
#ifndef GNSS_BAUDRATE
#define GNSS_BAUDRATE           921600
#endif
#define GNSS_AUX1_PORT          GPIOD
#define GNSS_AUX1_PIN           GPIO_Pin_4      /* 管脚 56  PD4 */
#define GNSS_AUX2_PORT          GPIOF
#define GNSS_AUX2_PIN           GPIO_Pin_4      /* 管脚 61  PF4 */

/*==============================================================================
 * 5. SD 卡  —— SDMMC 默认映射 RM=00
 *    注意：SDMMC 引脚按 EVT 例程只做 GPIO 复用配置，不做 GPIO_PinAFConfig
 *============================================================================*/
#define SD_D0_PORT              GPIOC
#define SD_D0_PIN               GPIO_Pin_8      /* 管脚 44 */
#define SD_D1_PORT              GPIOC
#define SD_D1_PIN               GPIO_Pin_9      /* 管脚 45 */
#define SD_D2_PORT              GPIOC
#define SD_D2_PIN               GPIO_Pin_10     /* 管脚 49 */
#define SD_D3_PORT              GPIOC
#define SD_D3_PIN               GPIO_Pin_11     /* 管脚 50 */
#define SD_D4_PORT              GPIOA
#define SD_D4_PIN               GPIO_Pin_14     /* 管脚 47 (8bit 才用) */
#define SD_D5_PORT              GPIOA
#define SD_D5_PIN               GPIO_Pin_15     /* 管脚 48 (8bit 才用) */
#define SD_D6_PORT              GPIOC
#define SD_D6_PIN               GPIO_Pin_6      /* 管脚 42 (8bit 才用) */
#define SD_D7_PORT              GPIOC
#define SD_D7_PIN               GPIO_Pin_7      /* 管脚 43 (8bit 才用) */
#define SD_CK_PORT              GPIOC
#define SD_CK_PIN               GPIO_Pin_12     /* 管脚 51 */
#define SD_CMD_PORT             GPIOD
#define SD_CMD_PIN              GPIO_Pin_2      /* 管脚 54 */
#define SD_STR_PORT             GPIOD
#define SD_STR_PIN              GPIO_Pin_3      /* 管脚 55 */
#define SD_BUS_WIDTH_8BIT       0               /* 1: 也配 D4..D7 并切 8bit */

/*==============================================================================
 * 6. CAN1 (AF9)
 *============================================================================*/
#define CAN1_RX_PORT            GPIOD
#define CAN1_RX_PIN             GPIO_Pin_0      /* 管脚 52 */
#define CAN1_RX_SRC             GPIO_PinSource0
#define CAN1_TX_PORT            GPIOD
#define CAN1_TX_PIN             GPIO_Pin_1      /* 管脚 53 */
#define CAN1_TX_SRC             GPIO_PinSource1
#define CAN1_AF                 GPIO_AF9
#define CAN1_BAUDRATE_PRESCALER 16
#define CAN1_BS1                CAN_BS1_6tq
#define CAN1_BS2                CAN_BS2_5tq

/*==============================================================================
 * 7. 人机：LED / 按键
 *============================================================================*/
#define LED1_PORT               GPIOB
#define LED1_PIN                GPIO_Pin_11     /* 管脚 27  PB11 */
#define LED2_PORT               GPIOB
#define LED2_PIN                GPIO_Pin_12     /* 管脚 30  PB12 */
#define KEY1_PORT               GPIOB
#define KEY1_PIN                GPIO_Pin_10     /* 管脚 26  PB10：已改作复位键 */
#define KEY2_PORT               GPIOF
#define KEY2_PIN                GPIO_Pin_14     /* 管脚 65  PF14 按键2 */

#define LED1_ON()               GPIO_ResetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()              GPIO_SetBits(LED1_PORT, LED1_PIN)
#define LED2_ON()               GPIO_ResetBits(LED2_PORT, LED2_PIN)
#define LED2_OFF()              GPIO_SetBits(LED2_PORT, LED2_PIN)
#define KEY1_DOWN()             (GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == RESET)
#define KEY2_DOWN()             (GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN) == RESET)

/* --- PB10 复位键 -----------------------------------------------------------
 * 按下 = 强下拉到地，松开 = 外部弱上拉到 VCC。
 * 用 EXTI10 下降沿触发，中断里调 NVIC_SystemReset() 复位整片。
 * 注意：EXTI15_8_IRQn = 33 (>31)，必须用 NVIC_SetAllocateIRQ 分配到 V3F，
 *       否则中断默认归 V5F，V3F 收不到。 */
#define RST_KEY_PIN             KEY1_PIN
#define RST_KEY_PORT            KEY1_PORT
#define RST_KEY_EXTI_LINE       EXTI_Line10
#define RST_KEY_PIN_SRC         GPIO_PinSource10
#define RST_KEY_PORT_SRC        GPIO_PortSourceGPIOB
#define RST_KEY_IRQn            EXTI15_8_IRQn    /* PB10 -> EXTI15_8 分组 */

/*==============================================================================
 * 8. 未使用但在板上已接的脚（保持默认，勿配置）
 *    15 PC0 / 20 PA5 / 21 PB0 / 22 PB1 / 25 PE15 : 仅到电阻
 *    23 PE13 / 24 PE14                           : AW32001 电源管理
 *    16 PC1 / 17 PC2 / 37 PD14 / 38 PD15         : J6 扩展口
 *    46 PA13                                     : SWDIO，保留调试
 *    67 PB8 / 68 PB9                             : USBHS（专用功能）
 *============================================================================*/

/*==============================================================================
 * 9. API
 *============================================================================*/
void BOARD_Init(void);                  /* 入口：按功能开关初始化全部 IO/外设 */

void BOARD_GPIO_Init(void);             /* 仅 GPIO（片选/中断/LED/按键） */
void BOARD_SPI1_MMC_Init(void);         /* 磁强计 SPI1 + 片选 PE0 */
void BOARD_SPI2_IMU_Init(void);         /* IMU SPI2 */
void BOARD_BMP388_I2C_Init(void);       /* 气压计软件 I2C 引脚 */
void BOARD_USART2_GNSS_Init(void);      /* GNSS 串口 */
void BOARD_CAN1_Init(void);             /* CAN1 */
void BOARD_SDMMC_PinInit(void);         /* SD 卡引脚 + SDMMC 控制器 */
void BOARD_EXTI_Init(void);             /* 磁强计 DRDY + IMU INT1/INT2 */
void BOARD_RSTKEY_Init(void);           /* PB10 复位键：EXTI10 下降沿 + 复位 ISR */


/*------------------------------------------------------------------------------
 * 中断计数（调试用）
 *   g_exti_irq_cnt : EXTI15_8（陀螺 INT1 + 复位键）进入次数
 *----------------------------------------------------------------------------*/
extern volatile uint32_t g_exti_irq_cnt;

/*------------------------------------------------------------------------------
 * 地磁 DRDY 中断的产出（EXTI7_0_IRQHandler 内更新）
 *
 * 中断内【手动触发下一拍】并处理 SET/RESET 极性：
 *   1) 读 X/Y/Z 7 字节
 *   2) 写 STATUS 清锁存（否则 INT 一直高，再无上升沿）
 *   3) 按下一拍需要的极性准备器件并触发测量
 *
 * 中断内只搬原始 counts：不碰浮点、不 printf（理由见 board_io.c）。
 * 只搬原始 counts，不做任何零偏/配对修正。
 *----------------------------------------------------------------------------*/
extern volatile int32_t  g_mmc_raw[3];      /* X/Y/Z 原始 counts              */
extern volatile uint8_t  g_mmc_polarity;    /* 本拍极性：0=SET  1=RESET        */
extern volatile uint32_t g_mmc_isr_cnt;     /* 中断进入次数                    */
extern volatile uint32_t g_mmc_spi_err;     /* 中断内 SPI 失败次数             */
extern volatile uint64_t g_mmc_drdy_ts;

/* SET/RESET 配对累加（中断内用"上一拍样本"配对 —— 相邻两拍必然异极性，
 * 是物理时间上最近的一对；按方向硬配会人为拉长窗口）。
 * 主循环每秒取一次：均值 = sum / n，然后清零。 */

/*------------------------------------------------------------------------------
 * 陀螺 DRDY 中断的产出（EXTI15_8_IRQHandler 内更新）
 *
 * 中断内一次读回 LSM6DSV16X 输出块 0x20..0x2D（14 字节）：
 *   温度 2 + 陀螺 XYZ 6 + 加速度计 XYZ 6，低字节在前。
 * 只搬原始 16 位值，不碰浮点、不 printf；换算与上报由主循环做。
 *   g_imu_raw[0..2] = 陀螺 XYZ 原始值（±4000dps 下 0.140 dps/LSB）
 *   g_imu_raw[3..5] = 加速度计 XYZ 原始值
 *----------------------------------------------------------------------------*/
extern volatile int16_t  g_imu_raw[6];
extern volatile int16_t  g_imu_temp_raw;
extern volatile uint32_t g_imu_isr_cnt;
extern volatile uint32_t g_imu_spi_err;
extern volatile uint64_t g_imu_drdy_ts;

/* SPI1/SPI2 的 DMA Mux 通道配置（对应 DMA1 通道 2/3，与旧架构一致）
 *   SPI1_TX=请求63, SPI1_RX=64 ; SPI2_TX=65, SPI2_RX=66
 *   本函数只做 Mux 绑定，不使能通道；DMA_Init/DMA_Cmd/中断由应用层做。 */
void BOARD_SPI1_DMA_MuxInit(void);
void BOARD_SPI2_DMA_MuxInit(void);

/* 陀螺 SPI2 的 TX 侧（DMA 采集的小核半边，见 board_io.c 9.12） */
void BOARD_IMU_DmaInit(void);
void BOARD_IMU_DrdyStart(void);
int  BOARD_IMU_Init(void);        /* 配置陀螺（不启动转换）*/
int  BOARD_IMU_Enable(void);      /* 启动陀螺转换（须在 EXTI 与消费方 RX 就绪后）*/

/* 软件 I2C 位操作（供 BMP388 驱动使用）
 * 板上 PF0/PF1 有外部上拉，因此：
 *   - 输出时用开漏（写 0 拉低 / 写 1 释放），高电平由【外部上拉】抬起
 *     （PF0/PF1 的 R34/R36，见网表） */
void    BMP_I2C_SCL_HIGH(void);         /* 释放 SCL（开漏输出） */
void    BMP_I2C_SCL_LOW(void);          /* 拉低 SCL */
void    BMP_I2C_SDA_HIGH(void);         /* 释放 SDA（开漏输出） */
void    BMP_I2C_SDA_LOW(void);          /* 拉低 SDA */
uint8_t BMP_I2C_SDA_READ(void);
uint8_t BMP_I2C_SCL_READ(void);
void    BMP_I2C_Delay(void);

/* 中断服务：EXTI 分组回调（用户在自己的 ch32h417_it.c 里实现） */
void BOARD_MMC_DRDY_IRQHandler(void);       /* EXTI0 分组：PE1 */
void BOARD_IMU_INT_IRQHandler(void);        /* EXTI15_8 分组：PD11/PD10 */

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_IO_H */
