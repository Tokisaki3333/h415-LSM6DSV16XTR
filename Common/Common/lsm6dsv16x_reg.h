/*******************************************************************************
 * File Name        : lsm6dsv16x_reg.h
 * Description      : LSM6DSV16XTR 六轴 IMU 寄存器定义（SPI2，片选 PD10）
 *
 * 来源：DS13510 Rev 4（198 页）—— 每项都标注了核对依据（页码 + 表号）。
 * 本文件只收录【已核对】的项，未核对的寄存器不在此定义。
 *
 *================================================================================
 * 【本板用法：仅陀螺仪，最高 ODR，±4000 dps】
 *
 *   为什么开最高 ODR，以及为什么轮询在这里是可行的：
 *     陀螺对外输出的是"角速度"而非"角增量"，所以对采样率的要求来自
 *     角速度信号的带宽，而不是积分精度。采样率越高，抗混叠与后续滤波越灵活。
 *
 *   已确认的两个前提：
 *     1) 选 ±4000 dps 时手册要求陀螺 OIS 链必须关闭（OIS_G_EN = 0）。
 *        该位在 UI_CTRL1_OIS (70h) 中，复位默认值就是 0，无需额外写。
 *     2) 数据就绪有两条读取路径：
 *          STATUS_REG(1Eh).GDA (bit1)            —— 轮询
 *          INT1_CTRL(0Dh).INT1_DRDY_G (bit1)     —— 中断（本板 INT1 = PD11）
 *        两者位置都在 bit1，便于先轮询验证后再切中断。
 *
 *================================================================================
 * 【禁止读-改-写】
 *   LSM6DSV16X 的控制寄存器（CTRL1~CTRL10、FIFO_CTRL*、INT*_CTRL 等）读回的是
 *   控制器当前值，其中含大量保留位与自动清除位（BOOT/SW_RESET）。
 *   对保留位写 1 会破坏器件工作（手册多处注明"This bit must be set to 0 for
 *   the correct operation of the device"）。
 *   -> 一律用完整影子值/常量写入，绝不读-改-写。
 *   -> 需要保留原值的地方，自己维护本地影子变量。
 *******************************************************************************/

#ifndef __LSM6DSV16X_REG_H
#define __LSM6DSV16X_REG_H

/*==============================================================================
 * 1. 器件标识
 *   DS13510 Rev4 §9.13 (page 64)：WHO_AM_I (0Fh)，固定值 70h
 *============================================================================*/
#define LSM6DSV16X_REG_WHO_AM_I         0x0Fu
#define LSM6DSV16X_WHO_AM_I_VAL         0x70u

/*==============================================================================
 * 2. 控制寄存器地址
 *   地址取自手册 §9 各小节标题（page 64~74）
 *============================================================================*/
#define LSM6DSV16X_REG_CTRL1            0x10u   /* page 65   加速度计控制 */
#define LSM6DSV16X_REG_CTRL2            0x11u   /* page 66   陀螺控制    */
#define LSM6DSV16X_REG_CTRL3            0x12u   /* page 67   通用控制    */
#define LSM6DSV16X_REG_CTRL4            0x13u   /* page 68   数据就绪    */
#define LSM6DSV16X_REG_CTRL5            0x14u   /* page 69              */
#define LSM6DSV16X_REG_CTRL6            0x15u   /* page 69   陀螺满量程/滤波 */
#define LSM6DSV16X_REG_CTRL7            0x16u   /* page 70   高精度/滤波 */
#define LSM6DSV16X_REG_CTRL8            0x17u   /* page 71              */
#define LSM6DSV16X_REG_CTRL9            0x18u   /* page 72              */
#define LSM6DSV16X_REG_CTRL10           0x19u   /* page 74              */

#define LSM6DSV16X_REG_FIFO_CTRL1       0x07u   /* page 59   水印低字节  */
#define LSM6DSV16X_REG_FIFO_CTRL2       0x08u   /* page 59              */
#define LSM6DSV16X_REG_FIFO_CTRL3       0x09u   /* page 60              */
#define LSM6DSV16X_REG_FIFO_CTRL4       0x0Au   /* page 61              */

#define LSM6DSV16X_REG_IF_CFG           0x03u   /* page 58   接口配置（INT 极性）*/

#define LSM6DSV16X_REG_INT1_CTRL        0x0Du   /* page 63   INT1 路由  */
#define LSM6DSV16X_REG_INT2_CTRL        0x0Eu   /* page 64   INT2 路由（本板未连出）*/

/*==============================================================================
 * IF_CFG (03h) —— 接口配置（含 INT 引脚极性）
 *   Table 29，bit7..bit0:
 *     SDA_PU_EN | SHUB_PU_EN | ASF_CTRL | H_LACTIVE | PP_OD | SIM | 0 | I2C_I3C_disable
 *      (bit7)      (bit6)      (bit5)     (bit4)    (bit3) (bit2)(bit1)    (bit0)
 *
 *   H_LACTIVE  —— INT1/INT2 激活电平，默认 0
 *       0: 中断输出引脚高有效
 *       1: 中断输出引脚低有效
 *   PP_OD —— 推挽/开漏，默认 0（推挽）
 *
 *   本板用默认值（高有效 + 推挽），与 PD11 上拉输入匹配。
 *   注意：该寄存器【不随软复位复位】（手册 §9.3 明示）。
 *============================================================================*/
#define LSM6DSV16X_IFCFG_SDA_PU_EN      0x80u
#define LSM6DSV16X_IFCFG_SHUB_PU_EN     0x40u
#define LSM6DSV16X_IFCFG_ASF_CTRL       0x20u
#define LSM6DSV16X_IFCFG_H_LACTIVE      0x10u   /* 1=低有效；本板用 0=高有效 */
#define LSM6DSV16X_IFCFG_PP_OD          0x08u   /* 1=开漏；本板用 0=推挽 */
#define LSM6DSV16X_IFCFG_SIM            0x04u   /* 1=3线SPI；本板用 0=4线 */
#define LSM6DSV16X_IFCFG_I2C_I3C_DIS    0x01u   /* 1=关闭I2C/I3C */
/* 本板取值：高有效 + 推挽 + 4线SPI = 0x00（复位默认，但该寄存器不随软复位，故显式写）*/
#define LSM6DSV16X_IFCFG_OUR_CFG        0x00u

#define LSM6DSV16X_REG_CTRL_STATUS      0x1Au   /* page 74              */
#define LSM6DSV16X_REG_FIFO_STATUS1     0x1Bu   /* page 75              */
#define LSM6DSV16X_REG_FIFO_STATUS2     0x1Cu   /* page 75              */
#define LSM6DSV16X_REG_ALL_INT_SRC      0x1Du   /* page 76              */
#define LSM6DSV16X_REG_STATUS           0x1Eu   /* page 77   数据就绪状态 */
#define LSM6DSV16X_REG_MD1_CFG          0x5Eu   /* 嵌入式功能路由到 INT1 */
#define LSM6DSV16X_REG_UI_CTRL1_OIS     0x70u   /* page 106  OIS 链使能  */

/*==============================================================================
 * 3. 数据输出寄存器地址
 *   DS13510 Rev4 §9.29~§9.34 (page 78~80)
 *   全部为 16 位二进制补码，L 在前 H 在后（小端字节序）
 *============================================================================*/
#define LSM6DSV16X_REG_OUT_TEMP_L       0x20u
#define LSM6DSV16X_REG_OUT_TEMP_H       0x21u
#define LSM6DSV16X_REG_OUTX_L_G         0x22u   /* 陀螺 X 低字节 —— 连续读起点 */
#define LSM6DSV16X_REG_OUTX_H_G         0x23u
#define LSM6DSV16X_REG_OUTY_L_G         0x24u
#define LSM6DSV16X_REG_OUTY_H_G         0x25u
#define LSM6DSV16X_REG_OUTZ_L_G         0x26u
#define LSM6DSV16X_REG_OUTZ_H_G         0x27u
#define LSM6DSV16X_REG_OUTX_L_A         0x28u   /* 加速度计 X 低字节 */
#define LSM6DSV16X_REG_OUTX_H_A         0x29u
#define LSM6DSV16X_REG_OUTY_L_A         0x2Au
#define LSM6DSV16X_REG_OUTY_H_A         0x2Bu
#define LSM6DSV16X_REG_OUTZ_L_A         0x2Cu
#define LSM6DSV16X_REG_OUTZ_H_A         0x2Du

/* 陀螺三轴可从 0x22 连读 6 字节（IF_INC=1 时地址自增，默认即为 1）*/
#define LSM6DSV16X_GYRO_BURST_LEN       6u
#define LSM6DSV16X_XL_BURST_LEN         6u

/* 全部输出寄存器块：0x20~0x2D 共 14 字节，连续无缺口
 *   0x20/0x21  OUT_TEMP_L/H    温度（16 位补码；256 LSB/°C，25°C 时约 0 LSB）
 *   0x22~0x27  陀螺 X/Y/Z      各 16 位补码，L 在前
 *   0x28~0x2D  加速度计 X/Y/Z  各 16 位补码，L 在前
 * 0x2E 起是 UI_OUTX_L_G_OIS_EIS（EIS/OIS 通道），不属于本块。
 * 一次 SPI 事务读 14 字节即可拿全"温度 + 六轴"。 */
#define LSM6DSV16X_REG_OUT_TEMP_L_BLK   0x20u
#define LSM6DSV16X_BLOCK_LEN            14u
/* 块内偏移 */
#define LSM6DSV16X_BLK_OFF_TEMP         0u
#define LSM6DSV16X_BLK_OFF_GX           2u
#define LSM6DSV16X_BLK_OFF_GY           4u
#define LSM6DSV16X_BLK_OFF_GZ           6u
#define LSM6DSV16X_BLK_OFF_AX           8u
#define LSM6DSV16X_BLK_OFF_AY           10u
#define LSM6DSV16X_BLK_OFF_AZ           12u

/* 温度换算：TSen = 256 LSB/°C，脚注3"The output of the temperature sensor
 * is 0 LSB (typ.) at 25 °C" —— 即 raw=0 对应 25°C。
 * 注意 Toff = ±15°C 是【绝对容差】，不是可补偿的常数偏移。 */
#define LSM6DSV16X_TEMP_LSB_PER_C       256.0f
#define LSM6DSV16X_TEMP_ZERO_C          25.0f

/*==============================================================================
 * 4. CTRL1 (10h) —— 加速度计
 *   Table 51，bit7..bit0:
 *     0(保留,必须为0) | OP_MODE_XL_2 | OP_MODE_XL_1 | OP_MODE_XL_0
 *     | ODR_XL_3 | ODR_XL_2 | ODR_XL_1 | ODR_XL_0
 *============================================================================*/
/* OP_MODE_XL_[2:0]，bit6:4 */
#define LSM6DSV16X_XL_OPMODE_HP         0x00u   /* 高性能（默认，7.5Hz~7.68kHz）*/
#define LSM6DSV16X_XL_OPMODE_HAODR      0x10u   /* 高精度 ODR（15Hz~7.68kHz）*/
#define LSM6DSV16X_XL_OPMODE_LPM1       0x40u
#define LSM6DSV16X_XL_OPMODE_LPM2       0x50u
#define LSM6DSV16X_XL_OPMODE_LPM3       0x60u
#define LSM6DSV16X_XL_OPMODE_NORMAL     0x70u   /* 7.5Hz~1.92kHz */

/* ODR_XL_[3:0]，bit3:0
 * Table 51：与陀螺同样的编码方式，最高档同为非顺序的 1100：
 *   0000 掉电(默认)
 *   0001 7.5Hz    0010 15Hz     0011 30Hz     0100 60Hz     0101 120Hz
 *   0110 240Hz    0111 480Hz    1000 960Hz    1001 1.92kHz  1010 3.84kHz
 *   1011 3840Hz
 *   1100 【7.68 kHz (high-performance mode)】   <- 最高档
 *   其余保留 */
#define LSM6DSV16X_XL_ODR_POWERDOWN     0x00u
#define LSM6DSV16X_XL_ODR_7H5           0x01u
#define LSM6DSV16X_XL_ODR_15H           0x02u
#define LSM6DSV16X_XL_ODR_30H           0x03u
#define LSM6DSV16X_XL_ODR_60H           0x04u
#define LSM6DSV16X_XL_ODR_120H          0x05u
#define LSM6DSV16X_XL_ODR_240H          0x06u
#define LSM6DSV16X_XL_ODR_480H          0x07u
#define LSM6DSV16X_XL_ODR_960H          0x08u
#define LSM6DSV16X_XL_ODR_1920H         0x09u
#define LSM6DSV16X_XL_ODR_3840H         0x0Au
#define LSM6DSV16X_XL_ODR_7680H         0x0Cu   /* 最高档（高性能模式）*/

/*==============================================================================
 * 5. CTRL2 (11h) —— 陀螺仪
 *   Table 53，bit7..bit0:
 *     0(保留,必须为0) | OP_MODE_G_2 | OP_MODE_G_1 | OP_MODE_G_0
 *     | ODR_G_3 | ODR_G_2 | ODR_G_1 | ODR_G_0
 *   Table 54：OP_MODE_G  000=高性能(默认) 001=高精度ODR 011=ODR触发
 *                        100=sleep 101=低功耗 010/110/111=保留
 *   Table 55：ODR_G  0000=掉电 0001=7.5Hz ... 1011=7.68kHz，其余保留
 *============================================================================*/
#define LSM6DSV16X_G_OPMODE_HP          0x00u   /* 高性能（默认）*/
#define LSM6DSV16X_G_OPMODE_HAODR       0x10u   /* 高精度 ODR */
#define LSM6DSV16X_G_OPMODE_ODRTRIG     0x30u   /* ODR 触发 */
#define LSM6DSV16X_G_OPMODE_SLEEP       0x40u
#define LSM6DSV16X_G_OPMODE_LP          0x50u

/* ODR_G_[3:0]，bit3:0
 * Table 55（最高档已由人工核对确认）：
 *   0000 掉电(默认)
 *   0001 7.5Hz    0010 15Hz     0011 30Hz     0100 60Hz     0101 120Hz
 *   0110 240Hz    0111 480Hz    1000 960Hz    1001 1.92kHz  1010 3.84kHz
 *   1011 3840Hz
 *   1100 【7.68 kHz (high-performance mode)】   <- 最高档
 *   其余保留
 *
 * 注意：最高档不是顺序递增的 1011，而是非顺序的 1100。
 * 日志：曾按"二进制递增 = 频率倍增"推断 1011 为 7680Hz，实际 1011 是 3840Hz
 * 那一档。这类非顺序编码必须查表，不能靠规律推理。 */
#define LSM6DSV16X_G_ODR_POWERDOWN      0x00u
#define LSM6DSV16X_G_ODR_7H5            0x01u
#define LSM6DSV16X_G_ODR_15H            0x02u
#define LSM6DSV16X_G_ODR_30H            0x03u
#define LSM6DSV16X_G_ODR_60H            0x04u
#define LSM6DSV16X_G_ODR_120H           0x05u
#define LSM6DSV16X_G_ODR_240H           0x06u
#define LSM6DSV16X_G_ODR_480H           0x07u
#define LSM6DSV16X_G_ODR_960H           0x08u
#define LSM6DSV16X_G_ODR_1920H          0x09u
#define LSM6DSV16X_G_ODR_3840H          0x0Au
#define LSM6DSV16X_G_ODR_7680H          0x0Cu   /* 最高档（高性能模式）*/

/*==============================================================================
 * 6. CTRL3 (12h) —— 通用
 *   Table 56，bit7..bit0:
 *     BOOT | BDU | 0 | 0 | 0 | IF_INC | 0 | SW_RESET
 *   脚注1：保留位（bit5,4,3,2）必须为 0
 *   默认值：BOOT=0, BDU=1, IF_INC=1, SW_RESET=0  ->  0x44
 *============================================================================*/
#define LSM6DSV16X_CTRL3_BOOT           0x80u   /* 重载内存内容，自动清除 */
#define LSM6DSV16X_CTRL3_BDU            0x40u   /* 1=读数期间输出寄存器不更新 */
#define LSM6DSV16X_CTRL3_IF_INC         0x04u   /* 1=多字节访问时地址自增（默认1）*/
#define LSM6DSV16X_CTRL3_SW_RESET       0x01u   /* 软件复位，自动清除 */
#define LSM6DSV16X_CTRL3_DEFAULT        0x44u   /* BDU=1 | IF_INC=1 */
#define LSM6DSV16X_CTRL3_IF_INC_BDU     0x44u   /* 常用值：开 BDU 与地址自增 */

/*==============================================================================
 * 7. CTRL4 (13h) —— 数据就绪行为
 *   Table 58，bit7..bit0:
 *     0 | 0 | 0 | INT2_on_INT1 | DRDY_MASK | INT2_DRDY_TEMP | DRDY_PULSED | INT2_IN_LH
 *   DRDY_PULSED：0=锁存模式（读完高位后自动回 0）；1=脉冲模式（65us）
 *============================================================================*/
#define LSM6DSV16X_CTRL4_INT2_ON_INT1   0x10u
#define LSM6DSV16X_CTRL4_DRDY_MASK      0x08u   /* 1=滤波器建立完成前屏蔽 DRDY */
#define LSM6DSV16X_CTRL4_INT2_DRDY_TEMP 0x04u
#define LSM6DSV16X_CTRL4_DRDY_PULSED    0x02u
#define LSM6DSV16X_CTRL4_INT2_IN_LH     0x01u

/*==============================================================================
 * 8. CTRL6 (15h) —— 陀螺满量程与 LPF1 带宽
 *   Table 62，bit7..bit0:
 *     bit7      = 0（保留，必须为 0）
 *     bit6:4    = LPF1_G_BW_[2:0]
 *     bit3:0    = FS_G_[3:0]
 *
 *   位宽依据：字段全称为 LPF1_G_BW_[2:0] 即 3 位；Table 64 给出 8 档编码
 *   （000~111），与 3 位一致。而 FS_G_[3:0] 明确 4 位。
 *   手册位图在 PDF 文本流里把 LPF1_G_BW_0 与 FS_G_3 并成了一列，
 *   这里按 bit6:4 / bit3:0 解读。
 *
 *   Table 63 FS_G_[3:0]：
 *     0000=±125(默认) 0001=±250 0010=±500 0011=±1000 0100=±2000 1100=±4000
 *     其余保留（注意最高档 1100 同样是非顺序编码）
 *   脚注1【重要】：选 ±4000 dps 时，陀螺 OIS 链必须关闭
 *     （OIS_G_EN 位在 UI_CTRL1_OIS(70h)，其复位默认值即为 0）
 *============================================================================*/
#define LSM6DSV16X_FS_G_125             0x00u
#define LSM6DSV16X_FS_G_250             0x01u
#define LSM6DSV16X_FS_G_500             0x02u
#define LSM6DSV16X_FS_G_1000            0x03u
#define LSM6DSV16X_FS_G_2000            0x04u
#define LSM6DSV16X_FS_G_4000            0x0Cu   /* 本板选用 */

/* LPF1_G_BW_[2:0]，bit6:4，低通滤波带宽选择
 * （手册 Table 64 给出各档带宽，且注明仅在 OIS/EIS 关闭时可用）*/
#define LSM6DSV16X_LPF1_G_BW_0          0x00u
#define LSM6DSV16X_LPF1_G_BW_1          0x10u
#define LSM6DSV16X_LPF1_G_BW_2          0x20u
#define LSM6DSV16X_LPF1_G_BW_3          0x30u
#define LSM6DSV16X_LPF1_G_BW_4          0x40u
#define LSM6DSV16X_LPF1_G_BW_5          0x50u
#define LSM6DSV16X_LPF1_G_BW_6          0x60u
#define LSM6DSV16X_LPF1_G_BW_7          0x70u

/*==============================================================================
 * 8b. CTRL7 (16h) —— 陀螺数字 LPF1 使能
 *   Table 65，bit7..bit0:
 *     AH_QVAR_EN | INT2_DRDY_AH_QVAR | AH_QVAR_C_ZIN_1 | AH_QVAR_C_ZIN_0
 *     | 0 | 0 | 0 | LPF1_G_EN
 *   Table 66：LPF1_G_EN 使能陀螺数字 LPF1；OIS 链关闭时带宽由 CTRL6 的
 *             LPF1_G_BW_[2:0] 选择。
 *
 *   本板取值：0（关闭 LPF1）
 *     关闭后陀螺 UI 链上仍有【模拟抗混叠】与【LPF2】，二者都不可关：
 *       模拟抗混叠 —— 高性能模式自动生效
 *       LPF2      —— 截止频率由 ODR 决定（Table 21），7.68kHz 时约 537Hz
 *     所以"关掉滤波器"实际是"只保留固定的 LPF2 抗混叠"。
 *
 *   为什么关 LPF1：
 *     1) 关掉后群延迟最小（LPF2 约 0.30ms，建立约 0.89ms），陀螺要积分，
 *        群延迟会直接变成姿态误差
 *     2) 7.68kHz 已给出足够样本，MCU 侧可做【线性相位 FIR】，比传感器内的
 *        一阶 IIR 更好（群延迟恒定、截止与阶数完全可控）
 *     3) 代价是原始噪声从约 27 mdps(60Hz档) 升到约 81 mdps RMS，
 *        但噪声可以事后滤掉，相位延迟换不回来
 *
 *   注意 LPF1 仅在【高性能模式】可用；低功耗模式下自动旁路。
 *============================================================================*/
#define LSM6DSV16X_CTRL7_AH_QVAR_EN     0x80u
#define LSM6DSV16X_CTRL7_INT2_DRDY_AH_QVAR  0x40u
#define LSM6DSV16X_CTRL7_AH_QVAR_C_ZIN_1    0x20u
#define LSM6DSV16X_CTRL7_AH_QVAR_C_ZIN_0    0x10u
#define LSM6DSV16X_CTRL7_LPF1_G_EN      0x02u   /* 1=使能陀螺数字 LPF1 */
#define LSM6DSV16X_CTRL7_ALL_OFF        0x00u   /* 本板取值：全部关闭 */

/*==============================================================================
 * 9. UI_CTRL1_OIS (70h) —— OIS 链使能
 *   Table 202，bit7..bit0:
 *     0 | 0 | SIM_OIS | 0 | 0 | OIS_XL_EN | OIS_G_EN | SPI2_READ_EN
 *   OIS_G_EN 默认值 = 0（陀螺 OIS 链关闭）—— 正好满足 ±4000 dps 的要求
 *============================================================================*/
#define LSM6DSV16X_OIS_SIM              0x20u
#define LSM6DSV16X_OIS_XL_EN            0x04u
#define LSM6DSV16X_OIS_G_EN             0x02u   /* 必须保持 0（用了 ±4000 dps）*/
#define LSM6DSV16X_OIS_SPI2_READ_EN     0x01u

/*==============================================================================
 * 10. STATUS_REG (1Eh) —— 数据就绪状态（只读）
 *    Table 82/83，bit7..bit0:
 *      TIMESTAMP_ENDCOUNT | 0 | OIS_DRDY | GDA_EIS
 *      | AH_QVARDA | TDA | GDA | XLDA
 *    GDA 与 INT1_CTRL 的 INT1_DRDY_G 都在 bit1，位置一致
 *============================================================================*/
#define LSM6DSV16X_ST_TIMESTAMP_ENDCOUNT 0x80u
#define LSM6DSV16X_ST_OIS_DRDY          0x20u
#define LSM6DSV16X_ST_GDA_EIS           0x10u
#define LSM6DSV16X_ST_AH_QVARDA         0x08u
#define LSM6DSV16X_ST_TDA               0x04u
#define LSM6DSV16X_ST_GDA               0x02u   /* 陀螺新数据可用 —— 轮询判据 */
#define LSM6DSV16X_ST_XLDA              0x01u   /* 加速度计新数据可用 */

/*==============================================================================
 * 11. INT1_CTRL (0Dh) —— INT1 引脚路由
 *    Table 45，bit7..bit0:
 *      0 | INT1_CNT_BDR | INT1_FIFO_FULL | INT1_FIFO_OVR | INT1_FIFO_TH
 *      | 0 | INT1_DRDY_G | INT1_DRDY_XL
 *    本板 INT1 接在 PD11（IMU_INT1_PIN）
 *============================================================================*/
#define LSM6DSV16X_INT1_CNT_BDR         0x40u
#define LSM6DSV16X_INT1_FIFO_FULL       0x20u
#define LSM6DSV16X_INT1_FIFO_OVR        0x10u
#define LSM6DSV16X_INT1_FIFO_TH         0x08u
#define LSM6DSV16X_INT1_DRDY_G          0x02u   /* 陀螺数据就绪 -> INT1 */
#define LSM6DSV16X_INT1_DRDY_XL         0x01u

/*==============================================================================
 * 12. 灵敏度与换算
 *    规格表（page 9 附近）：
 *      角速度灵敏度 G_So：±125->4.375   ±250->8.75   ±500->17.50
 *                         ±1000->35      ±2000->70    ±4000->140      mdps/LSB
 *      加速度灵敏度 LA_So：±2g->0.061  ±4g->0.122  ±8g->0.244  ±16g->0.488  mg/LSB
 *    灵敏度容差 ±0.3 %；温漂 陀螺 ±0.007 %/°C，加速度 ±0.01 %/°C
 *============================================================================*/
#define LSM6DSV16X_G_SENS_125_MDPS      4375      /* 单位 0.001 mdps/LSB */
#define LSM6DSV16X_G_SENS_250_MDPS      8750
#define LSM6DSV16X_G_SENS_500_MDPS      17500
#define LSM6DSV16X_G_SENS_1000_MDPS     35000
#define LSM6DSV16X_G_SENS_2000_MDPS     70000
#define LSM6DSV16X_G_SENS_4000_MDPS     140000

/* ±4000 dps 下的换算常量 */
#define LSM6DSV16X_G_MDPS_PER_LSB       140.0f  /* mdps / LSB */
#define LSM6DSV16X_G_DPS_PER_LSB        0.140f  /* dps  / LSB */

#define LSM6DSV16X_XL_MG_PER_LSB_2G     0.061f
#define LSM6DSV16X_XL_MG_PER_LSB_4G     0.122f
#define LSM6DSV16X_XL_MG_PER_LSB_8G     0.244f
#define LSM6DSV16X_XL_MG_PER_LSB_16G    0.488f

/*==============================================================================
 * 13. 已核对的器件规格（用于零偏/噪声评估）
 *    角速度零偏 G_TyOff          : ±1 dps（室温）
 *    角速度零偏温漂 G_OffDr      : ±0.006 dps/°C
 *    角速度噪声密度 Rn           : 2.8 mdps/√Hz（高性能模式）
 *    低功耗 RMS 噪声 RnRMS       : 60 mdps RMS
 *    加速度零偏 LA_TyOff         : ±12 mg
 *    加速度零偏温漂 LA_OffDr     : ±0.07 mg/°C
 *    加速度噪声密度 An           : 60 µg/√Hz（高性能模式）
 *============================================================================*/
#define LSM6DSV16X_G_ZRL_DPS            1.0f    /* 零速率电平 ±1 dps */
#define LSM6DSV16X_G_ZRL_DRIFT_DPS_C    0.006f
#define LSM6DSV16X_G_NOISE_MDPS_RTHZ    2.8f    /* mdps/√Hz */
#define LSM6DSV16X_XL_ZRL_MG            12.0f
#define LSM6DSV16X_XL_NOISE_UG_RTHZ     60.0f

/*==============================================================================
 * 14. 数据合成辅助
 *    16 位二进制补码，L 为低字节
 *============================================================================*/
#define LSM6DSV16X_MAKE_I16(lo, hi)     \
    ((int16_t)(((uint16_t)(hi) << 8) | (uint16_t)(lo)))

/*==============================================================================
 * 14b. HAODR（高精度 ODR）模式
 *
 *   使能方式：不是独立使能位，而是把 OP_MODE 设为 001
 *     CTRL2 (11h) OP_MODE_G_[2:0]  = 001  -> 陀螺进 HAODR
 *     CTRL1 (10h) OP_MODE_XL_[2:0] = 001  -> 加速度计进 HAODR
 *   （手册 §6.5 Table 54：000=高性能 001=高精度ODR 010=保留 ...）
 *
 *   为什么用 HAODR：器件间 ODR 一致性优于高性能模式
 *     Table 20 给出 HAODR 下三套 ODR（由 HAODR_SEL 选）：
 *       HAODR_SEL=00 -> 15 / 30 / 60 / 120 / 240 / 480 / 960 / 1920 / 3840 / 【7680】
 *       HAODR_SEL=01 -> 15.625 / 31.25 / ... / 8000
 *       HAODR_SEL=10 -> 12.5 / 25 / ... / 6400
 *     最高档编码同为 1100。默认 HAODR_SEL=00 即得 7680 Hz，无需额外写。
 *
 *   必须遵守的约束（手册 §6.5 与 Table 20 的说明）：
 *     1) 使能/关闭 HAODR 必须在【掉电模式】下进行
 *     2) 陀螺用 HAODR 时，加速度计【也必须】用 HAODR（反之亦然）
 *     3) HAODR 会同时作用于 UI 加速度计、UI 陀螺、EIS 陀螺和温度；
 *        不作用于 OIS 通道
 *     4) 与模拟前端/Qvar、活动-静止检测不兼容
 *        （本板不使用这两项功能，故无冲突）
 *     5) HAODR 在 ODR 编码 0001/0010 上为保留（最低两档不可用）
 *============================================================================*/
#define LSM6DSV16X_REG_HAODR_CFG        0x62u   /* HAODR 数据率配置 */

/* HAODR_CFG (62h) 位域：bit7:2 为保留（必须为 0），bit1:0 = HAODR_SEL_[1:0]
 * Table 179 的位图在 PDF 里同样是"6 个保留列 + 2 个字段列"，与 [1:0] 吻合 */
#define LSM6DSV16X_HAODR_SEL_00         0x00u   /* 15Hz~7680Hz（默认）*/
#define LSM6DSV16X_HAODR_SEL_01         0x01u   /* 15.625Hz~8000Hz */
#define LSM6DSV16X_HAODR_SEL_10         0x02u   /* 12.5Hz~6400Hz */
#define LSM6DSV16X_HAODR_SEL_MASK       0x03u

/*==============================================================================
 * 15. 选择的工作点（本板当前配置，写代码时引用这些而不是散落的字面量）
 *
 *   陀螺    ：HAODR 模式 + 最高 ODR 7680Hz + ±4000 dps
 *   加速度计：HAODR 模式（约束 2 要求与陀螺同步进 HAODR）
 *   通用    ：BDU=1 + IF_INC=1（复位默认即 0x44，显式写出以免依赖复位状态）
 *
 *   配置顺序（约束 1）：先确保两传感器都在掉电模式，再进 HAODR。
 *============================================================================*/
#define LSM6DSV16X_CFG_CTRL2_GYRO_HAODR_MAX \
    (LSM6DSV16X_G_OPMODE_HAODR | LSM6DSV16X_G_ODR_7680H)     /* = 0x1C */
#define LSM6DSV16X_CFG_CTRL1_XL_HAODR_MAX   \
    (LSM6DSV16X_XL_OPMODE_HAODR | LSM6DSV16X_XL_ODR_7680H)   /* = 0x1C */
#define LSM6DSV16X_CFG_CTRL6_GYRO_4000  \
    (LSM6DSV16X_FS_G_4000)                                   /* = 0x0C */
#define LSM6DSV16X_CFG_CTRL3_DEFAULT    \
    (LSM6DSV16X_CTRL3_BDU | LSM6DSV16X_CTRL3_IF_INC)         /* = 0x44 */
#define LSM6DSV16X_CFG_HAODR_SEL_00     \
    (LSM6DSV16X_HAODR_SEL_00)                                /* = 0x00 */

#endif /* __LSM6DSV16X_REG_H */
