/*******************************************************************************
 * File Name        : bmp388_reg.h
 * Description      : BMP388 气压传感器 寄存器定义
 *
 * 器件     : Bosch BMP388
 * 依据     : BST-BMP388-DS001-07 Rev 1.7（I2C 模式寄存器手册）
 * 接口     : I2C 7bit 地址 0x76（本板 SDO 接 PF2 且外部下拉 -> SDO=0）；SPI
 *
 * 【交叉验证记录】
 *   下列位域已用「独立解码」方式与本工程实跑数据对上，可信度高：
 *     - STATUS(0x03) 就绪掩码 0x60 = bit6 drdy_temp | bit5 drdy_press
 *       本工程以此为轮询依据，实测有效
 *     - OSR(0x1C) 写入 0x0B -> 解码 osr_t=001(x2), osr_p=011(x8)
 *       与转换时间公式 2.2 + 2x2.3 + 8x2.3 = 25.2ms 完全吻合（实测 25200us）
 *     - PWR_CTRL(0x1B) 写入 0x13 -> 解码 mode=01(强制), temp_en=1, press_en=1
 *     - ODR(0x1D) 默认值应为 0x00，实测读回 0x00
 *     - CONFIG(0x1F) 默认值应为 0x00，实测读回 0x00；写 0x02 后 IIR 生效
 *       （此前误写入 ODR(0x1D) 导致读数卡死，故 0x1D/0x1F 的区别是确认过的）
 *
 * 【硬性规则：本器件禁止读-改-写】
 *   配置寄存器一律"整字节按期望值写死"，不要"读出来 -> 改一位 -> 写回去"。
 *   本工程的所有 BMP388 配置写入都是直接写常量：
 *     0x19 = 0x46 / 0x1C = 0x0B / 0x1F = 0x02 / 0x1B = 0x13
 *   只有数据、状态、EVENT、INT_STATUS、ERR_REG 这些才允许读。
 *   注意 0x02 ERR_REG、0x10 EVENT、0x11 INT_STATUS 的部分位"读后清零"，
 *   连读两次结果不同，更不能参与读-改-写。
 *
 * 【待人工核查项】（按手册记录，但未经本工程实测）
 *     - 0x02 ERR_REG 仅 bit2:0 有定义
 *     - 0x11 INT_STATUS 仅 bit3/bit1/bit0 有定义
 *     - 0x18 FIFO_CONFIG_2 的 data_select 在 bit4:3、subsampling 在 bit2:0
 *     - 0x1B PWR_CTRL 仅 bit5:4(mode)、bit1(temp_en)、bit0(press_en) 有定义
 *     - 0x1F CONFIG 的 iir_filter 在 bit3:1（bit0 保留）
 *******************************************************************************/

#ifndef __BMP388_REG_H
#define __BMP388_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * I2C 从机地址（7bit：111011x）
 *============================================================================*/
#define BMP388_I2C_ADDR_SDO_GND         0x76u   /* 写 0xEC / 读 0xED */
#define BMP388_I2C_ADDR_SDO_VDDIO       0x77u   /* 写 0xEE / 读 0xEF */

/*==============================================================================
 * 寄存器地址表
 *============================================================================*/
#define BMP388_REG_CHIP_ID              0x00u
#define BMP388_REG_ERR_REG              0x02u
#define BMP388_REG_STATUS               0x03u
#define BMP388_REG_DATA_0               0x04u   /* press[7:0]     */
#define BMP388_REG_DATA_1               0x05u   /* press[15:8]    */
#define BMP388_REG_DATA_2               0x06u   /* press[23:16]   */
#define BMP388_REG_DATA_3               0x07u   /* temp[7:0]      */
#define BMP388_REG_DATA_4               0x08u   /* temp[15:8]     */
#define BMP388_REG_DATA_5               0x09u   /* temp[23:16]    */
#define BMP388_REG_SENSORTIME_0         0x0Cu
#define BMP388_REG_SENSORTIME_1         0x0Du
#define BMP388_REG_SENSORTIME_2         0x0Eu
#define BMP388_REG_EVENT                0x10u
#define BMP388_REG_INT_STATUS           0x11u
#define BMP388_REG_FIFO_LENGTH_0        0x12u
#define BMP388_REG_FIFO_LENGTH_1        0x13u
#define BMP388_REG_FIFO_DATA            0x14u
#define BMP388_REG_FIFO_WTM_0           0x15u
#define BMP388_REG_FIFO_WTM_1           0x16u
#define BMP388_REG_FIFO_CONFIG_1        0x17u
#define BMP388_REG_FIFO_CONFIG_2        0x18u
#define BMP388_REG_INT_CTRL             0x19u
#define BMP388_REG_IF_CONF              0x1Au
#define BMP388_REG_PWR_CTRL             0x1Bu
#define BMP388_REG_OSR                  0x1Cu
#define BMP388_REG_ODR                  0x1Du
#define BMP388_REG_CONFIG               0x1Fu
#define BMP388_REG_CALIB00              0x31u
#define BMP388_REG_CALIB_LEN            21u
#define BMP388_REG_CMD                  0x7Eu

/*==============================================================================
 * 复位默认值（可用于读回自检）
 *   注意 EVENT / INT_STATUS / ERR_REG 的部分位「读后清零」，
 *   因此每一项只能比对一次，且要在尚未读取过之前比对。
 *============================================================================*/
#define BMP388_POR_CHIP_ID              0x50u
#define BMP388_POR_ERR_REG              0x00u
#define BMP388_POR_STATUS               0x00u
#define BMP388_POR_DATA_2               0x80u
#define BMP388_POR_DATA_5               0x80u
#define BMP388_POR_EVENT                0x01u
#define BMP388_POR_INT_STATUS           0x00u
#define BMP388_POR_FIFO_WTM_0           0x01u
#define BMP388_POR_FIFO_WTM_1           0x00u
#define BMP388_POR_FIFO_CONFIG_1        0x02u
#define BMP388_POR_FIFO_CONFIG_2        0x02u
#define BMP388_POR_INT_CTRL             0x02u
#define BMP388_POR_IF_CONF              0x00u
#define BMP388_POR_PWR_CTRL             0x00u
#define BMP388_POR_OSR                  0x02u
#define BMP388_POR_ODR                  0x00u
#define BMP388_POR_CONFIG               0x00u

/*------------------------------------------------------------------------------
 * 0x00 CHIP_ID        R    默认 0x50
 *----------------------------------------------------------------------------*/
#define BMP388_CHIP_ID_VAL              0x50u

/*------------------------------------------------------------------------------
 * 0x02 ERR_REG        R    默认 0x00
 *   bit0 fatal_err 致命错误
 *   bit1 cmd_err   命令执行失败，读后清零
 *   bit2 conf_err  传感器配置错误，读后清零
 *----------------------------------------------------------------------------*/
#define BMP388_ERR_FATAL_ERR            0x01u
#define BMP388_ERR_CMD_ERR              0x02u
#define BMP388_ERR_CONF_ERR             0x04u

/*------------------------------------------------------------------------------
 * 0x03 STATUS         R    默认 0x00
 *   bit4 cmd_rdy     0=命令执行中，1=命令解码器就绪
 *   bit5 drdy_press  压力数据就绪，读压力数据寄存器后清零
 *   bit6 drdy_temp   温度数据就绪，读温度数据寄存器后清零
 *   本工程就绪判据 = bit6 与 bit5 同时为 1
 *----------------------------------------------------------------------------*/
#define BMP388_STATUS_CMD_RDY           0x10u
#define BMP388_STATUS_DRDY_PRESS        0x20u
#define BMP388_STATUS_DRDY_TEMP         0x40u
#define BMP388_STATUS_READY_MASK        0x60u   /* 本工程轮询用，已实测有效 */

/*------------------------------------------------------------------------------
 * 0x04~0x09 数据（R）  24bit 小端，低字节在前，压力在前、温度在后
 *------------------------------------------------------------------------------*/
#define BMP388_DATA_PRESS(r0, r1, r2)   ((((uint32_t)(r2)) << 16) | \
                                         (((uint32_t)(r1)) <<  8) | \
                                          ((uint32_t)(r0)))
#define BMP388_DATA_TEMP(r3, r4, r5)    ((((uint32_t)(r5)) << 16) | \
                                         (((uint32_t)(r4)) <<  8) | \
                                          ((uint32_t)(r3)))

/*------------------------------------------------------------------------------
 * 0x0C~0x0E 传感器时间（R）  24bit 小端，低字节在前
 *----------------------------------------------------------------------------*/
#define BMP388_SENSORTIME(r0, r1, r2)   ((((uint32_t)(r2)) << 16) | \
                                         (((uint32_t)(r1)) <<  8) | \
                                          ((uint32_t)(r0)))

/*------------------------------------------------------------------------------
 * 0x10 EVENT          R    默认 0x01
 *   bit0 por_detected：上电或软复位后为 1，读后清零
 *----------------------------------------------------------------------------*/
#define BMP388_EVENT_POR_DETECTED       0x01u

/*------------------------------------------------------------------------------
 * 0x11 INT_STATUS     R    读后清零
 *   bit0 fwm_int   FIFO 水印中断
 *   bit1 ffull_int FIFO 满中断
 *   bit3 drdy      数据就绪中断
 *----------------------------------------------------------------------------*/
#define BMP388_INT_STATUS_FWM_INT       0x01u
#define BMP388_INT_STATUS_FFULL_INT     0x02u
#define BMP388_INT_STATUS_DRDY          0x08u

/*------------------------------------------------------------------------------
 * 0x12~0x13 FIFO_LENGTH  R   9bit FIFO 字节计数
 *----------------------------------------------------------------------------*/
#define BMP388_FIFO_LEN_LOW(r0)         ((uint16_t)(r0))
#define BMP388_FIFO_LEN_HIGH(r1)        ((uint16_t)((r1) & 0x01u) << 8)

/*------------------------------------------------------------------------------
 * 0x15~0x16 FIFO_WTM   R/W  9bit 水印阈值（默认 0x0001）
 *----------------------------------------------------------------------------*/
#define BMP388_FIFO_WTM_LOW(r0)         ((uint16_t)(r0))
#define BMP388_FIFO_WTM_HIGH(r1)        ((uint16_t)((r1) & 0x01u) << 8)

/*------------------------------------------------------------------------------
 * 0x17 FIFO_CONFIG_1  R/W  默认 0x02
 *----------------------------------------------------------------------------*/
#define BMP388_FIFO_CFG1_MODE           0x01u   /* bit0 0=禁用FIFO 1=使能   */
#define BMP388_FIFO_CFG1_STOP_ON_FULL   0x02u   /* bit1 1=满后停止写入      */
#define BMP388_FIFO_CFG1_TIME_EN        0x04u   /* bit2 1=返回传感器时间帧  */
#define BMP388_FIFO_CFG1_PRESS_EN       0x08u   /* bit3 1=存压力数据        */
#define BMP388_FIFO_CFG1_TEMP_EN        0x10u   /* bit4 1=存温度数据        */

/*------------------------------------------------------------------------------
 * 0x18 FIFO_CONFIG_2  R/W  默认 0x02
 *   bit2:0 fifo_subsampling  降采样因子 = 2^value
 *   bit4:3 data_select       0=未滤波 1=滤波数据（10/11 保留，同未滤波）
 *----------------------------------------------------------------------------*/
#define BMP388_FIFO_CFG2_SUBSAMPLING_Msk (0x07u << 0)
#define BMP388_FIFO_CFG2_DATA_SELECT_Msk (0x03u << 3)
#define BMP388_FIFO_CFG2_DATA_UNFILTERED 0x00u
#define BMP388_FIFO_CFG2_DATA_FILTERED   (0x01u << 3)

/*------------------------------------------------------------------------------
 * 0x19 INT_CTRL       R/W  默认 0x02
 *   注意 bit2 = int_latch 默认 1，即复位后中断为锁存模式。
 *----------------------------------------------------------------------------*/
#define BMP388_INT_CTRL_INT_OD          0x01u   /* bit0 0=推挽 1=开漏          */
#define BMP388_INT_CTRL_INT_LEVEL       0x02u   /* bit1 0=低有效 1=高有效      */
#define BMP388_INT_CTRL_INT_LATCH       0x04u   /* bit2 0=非锁存 1=锁存        */
#define BMP388_INT_CTRL_FWTM_EN         0x08u   /* bit3 FIFO 水印中断使能      */
#define BMP388_INT_CTRL_FFULL_EN        0x10u   /* bit4 FIFO 满中断使能        */
#define BMP388_INT_CTRL_DRDY_EN         0x40u   /* bit6 数据就绪中断使能       */

/*------------------------------------------------------------------------------
 * 0x1A IF_CONF        R/W  默认 0x00
 *   本工程用 I2C，spi3 保持 0。
 *----------------------------------------------------------------------------*/
#define BMP388_IF_CONF_SPI3             0x01u   /* bit0 1=SPI 3 线             */
#define BMP388_IF_CONF_I2C_WDT_EN       0x02u   /* bit1 I2C 看门狗使能         */
#define BMP388_IF_CONF_I2C_WDT_SEL      0x04u   /* bit2 0=1.25ms 1=40ms 超时   */

/*------------------------------------------------------------------------------
 * 0x1B PWR_CTRL       R/W  默认 0x00
 *   bit0 press_en | bit1 temp_en | bit5:4 mode
 *     mode: 00=睡眠  01/10=强制模式  11=正常模式
 *   本工程取值 0x13 = mode=01(强制) + temp_en=1 + press_en=1，已实测有效。
 *----------------------------------------------------------------------------*/
#define BMP388_PWR_CTRL_PRESS_EN        0x01u   /* bit0 */
#define BMP388_PWR_CTRL_TEMP_EN         0x02u   /* bit1 */
#define BMP388_PWR_CTRL_MODE_Msk        (0x03u << 4)
#define BMP388_PWR_CTRL_MODE_SLEEP      (0x00u << 4)
#define BMP388_PWR_CTRL_MODE_FORCED     (0x01u << 4)
#define BMP388_PWR_CTRL_MODE_NORMAL     (0x03u << 4)
#define BMP388_PWR_CTRL_VERIFIED_VALUE  0x13u   /* 本工程实测可用 */

/*------------------------------------------------------------------------------
 * 0x1C OSR            R/W  默认 0x02
 *   bit2:0 osr_p | bit5:3 osr_t，编码 0..5 = x1/x2/x4/x8/x16/x32
 *   本工程取值 0x0B -> osr_t=001(x2), osr_p=011(x8)，已实测有效。
 *   转换时间：t_conv = 2.2ms + osr_t x 2.3ms + osr_p x 2.3ms
 *             上述配置 -> 2.2 + 2x2.3 + 8x2.3 = 25.2ms
 *----------------------------------------------------------------------------*/
#define BMP388_OSR_P_Msk                (0x07u << 0)
#define BMP388_OSR_T_Msk                (0x07u << 3)
#define BMP388_OSR_X1                    0u
#define BMP388_OSR_X2                    1u
#define BMP388_OSR_X4                    2u
#define BMP388_OSR_X8                    3u
#define BMP388_OSR_X16                   4u
#define BMP388_OSR_X32                   5u
#define BMP388_OSR_VERIFIED_VALUE        0x0Bu  /* osr_t=x2, osr_p=x8 */

/*------------------------------------------------------------------------------
 * 0x1D ODR            R/W  默认 0x00
 *   bit4:0 odr_sel，输出数据率分频因子 = 2^odr_sel，允许 0~17（超出饱和于 17）
 *   注意：本寄存器是【输出数据率】，不是 IIR 滤波。曾把 IIR 值误写入此寄存器
 *         导致读数不再更新；误写入值 0x02 对应 ODR_50（20ms），与 20Hz 采样
 *         周期冲突。
 *----------------------------------------------------------------------------*/
#define BMP388_ODR_SEL_Msk              (0x1Fu << 0)
#define BMP388_ODR_SEL_200HZ             0x00u   /* 预分频 1 */
#define BMP388_ODR_SEL_100HZ             0x01u
#define BMP388_ODR_SEL_50HZ              0x02u
#define BMP388_ODR_SEL_25HZ              0x03u
#define BMP388_ODR_SEL_12P5HZ            0x04u
#define BMP388_ODR_SEL_6P25HZ            0x05u
#define BMP388_ODR_SEL_3P1HZ             0x06u
#define BMP388_ODR_SEL_1P5HZ             0x07u
#define BMP388_ODR_SEL_MAX               0x11u

/*------------------------------------------------------------------------------
 * 0x1F CONFIG         R/W  默认 0x00
 *   bit3:1 iir_filter（bit0 保留）
 *   000=旁路(系数0) 001=1 010=3 011=7 100=15 101=31 110=63 111=127
 *   本工程取值 0x02 -> iir_filter=001 -> 系数 1
 *----------------------------------------------------------------------------*/
#define BMP388_CONFIG_IIR_Msk           (0x07u << 1)
#define BMP388_CONFIG_IIR_BYPASS        (0x00u << 1)   /* 系数 0   */
#define BMP388_CONFIG_IIR_1             (0x01u << 1)   /* 系数 1   */
#define BMP388_CONFIG_IIR_3             (0x02u << 1)   /* 系数 3   */
#define BMP388_CONFIG_IIR_7             (0x03u << 1)   /* 系数 7   */
#define BMP388_CONFIG_IIR_15            (0x04u << 1)   /* 系数 15  */
#define BMP388_CONFIG_IIR_31            (0x05u << 1)   /* 系数 31  */
#define BMP388_CONFIG_IIR_63            (0x06u << 1)   /* 系数 63  */
#define BMP388_CONFIG_IIR_127           (0x07u << 1)   /* 系数 127 */
#define BMP388_CONFIG_VERIFIED_VALUE    0x02u          /* = IIR_1 */

/*------------------------------------------------------------------------------
 * 0x7E CMD            W（读回 0x00）
 *----------------------------------------------------------------------------*/
#define BMP388_CMD_NOP                  0x00u
#define BMP388_CMD_EXTMODE_EN_MIDDLE    0x34u
#define BMP388_CMD_FIFO_FLUSH           0xB0u
#define BMP388_CMD_SOFTRESET            0xB6u

/*==============================================================================
 * 接口模式说明
 *   I2C：直接使用完整 8 位寄存器地址，R/W 由从机地址最低位决定。
 *   SPI：地址 bit7 用作 R/W（发 (reg & 0x7F)，读时 reg | 0x80）。
 *   两种接口下寄存器定义完全一致。
 *
 *   本板 CSB（SPI_CS）由 MCU 驱动，故工作于 SPI；SDO 接 PF2 且外部下拉，
 *   若切到 I2C 则地址为 0x76。
 *============================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* __BMP388_REG_H */
