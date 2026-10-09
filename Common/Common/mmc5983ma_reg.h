/*******************************************************************************
 * File Name        : mmc5983ma_reg.h
 * Description      : MMC5983MA 三轴磁传感器 寄存器定义（仅记录已逐位校核的字段）
 *
 * 器件     : MEMSIC MMC5983MA Rev. A (2/25/2019)
 * 接口     : SPI（4 线，命令字节 bit7=1 读 / bit7=0 写，bit6:0=地址）、I2C
 * 数据格式 : 三轴各 18bit 无符号
 *
 * 【校核说明】
 *   本文件只收录经位图逐位确认过的字段。凡未确认位号者一律不写，避免误用。
 *   未收录项（不要凭猜测使用）：
 *     - Control Register 3 的 St_enm / St_enp 位号（手册段落有描述、位图未逐位确认）
 *
 * 【硬性规则：本器件禁止读-改-写】
 *   本器件的配置寄存器不是常规的读写寄存器设计：
 *     - Internal Control 0/1/2/3 (0x09~0x0C) 手册 Mode 全部标为 W（只写）
 *     - 读它们时保留位可能返回无意义的值
 *   因此【绝对不要】"读出来 -> 改某一位 -> 写回去"。那样会把读回的垃圾位
 *   一起写进寄存器、破坏配置。本项目实测踩过：Set/Reset 的读-改-写会间歇性
 *   让 RESET 失效。
 *
 *   正确做法（二选一）：
 *     1) 本地缓存：在 RAM 里维护一份影子值，改动后整字节写出
 *     2) 直接按期望值写死：触发类操作（Set/Reset）直接写常量即可
 *
 *   允许读的寄存器只有：0x00~0x07 数据、0x08 Status、0x2F Product ID。
 *
 * 【重要教训：位图与文字流的差别】
 *   手册位图里存在「多列合并标注」的字段，例如 Control Register 1 的
 *   YZ-inhibit 同时标在 bit4 与 bit3 两列下。从 PDF 抽取的文字流会把它读成
 *   两个独立字段，从而导致其后所有字段的位号错位。本文件的位号以位图为准。
 *******************************************************************************/

#ifndef __MMC5983MA_REG_H
#define __MMC5983MA_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 寄存器地址表
 *============================================================================*/
#define MMC5983MA_REG_XOUT0             0x00u   /* Xout[17:10]                */
#define MMC5983MA_REG_XOUT1             0x01u   /* Xout[9:2]                  */
#define MMC5983MA_REG_YOUT0             0x02u   /* Yout[17:10]                */
#define MMC5983MA_REG_YOUT1             0x03u   /* Yout[9:2]                  */
#define MMC5983MA_REG_ZOUT0             0x04u   /* Zout[17:10]                */
#define MMC5983MA_REG_ZOUT1             0x05u   /* Zout[9:2]                  */
#define MMC5983MA_REG_XYZOUT2           0x06u   /* Xout[1:0]|Yout[1:0]|Zout[1:0] */
#define MMC5983MA_REG_TOUT              0x07u   /* 温度输出                   */
#define MMC5983MA_REG_STATUS            0x08u   /* 器件状态                   */
#define MMC5983MA_REG_CTRL0             0x09u   /* Internal Control 0         */
#define MMC5983MA_REG_CTRL1             0x0Au   /* Internal Control 1         */
#define MMC5983MA_REG_CTRL2             0x0Bu   /* Internal Control 2         */
#define MMC5983MA_REG_CTRL3             0x0Cu   /* Internal Control 3         */
#define MMC5983MA_REG_PROD_ID1          0x2Fu   /* Product ID 1               */

/*------------------------------------------------------------------------------
 * Product ID 1 (0x2F)      Mode: R      复位值 0x30 (0011 0000)
 *----------------------------------------------------------------------------*/
#define MMC5983MA_PROD_ID1_VAL          0x30u

/*------------------------------------------------------------------------------
 * Tout (0x07)              Mode: R
 *   无符号。范围 -75~125 摄氏度，约 0.8 摄氏度/LSB，0x00 代表 -75 摄氏度。
 *----------------------------------------------------------------------------*/
#define MMC5983MA_TOUT_OFFSET_C         (-75.0f)
#define MMC5983MA_TOUT_LSB_PER_C        (0.8f)

/*------------------------------------------------------------------------------
 * Status (0x08)            Mode: R/W
 *
 *   位分配（手册 P14，Status 段 Addr: 08H）—— 逐位，bit7 在最左：
 *     位  7   6   5   4               3   2   1              0
 *     名  re  re  re  otp_read_done   re  re  meas_t_done    meas_m_done
 *     值  0   0   0   0x10            0   0   0x02           0x01
 *
 *   ★ 这组值经用户核对确认，是【正确】的，不要再改：
 *       0x01 = bit0 = meas_m_done
 *       0x02 = bit1 = meas_t_done
 *       0x10 = bit4 = otp_read_done
 *
 *   ★ 我先后两次擅自按 PDF 抽取文本"重新解读"这两个值（改成 0x04/0x20、
 *     再改成 0x04/0x20），两次都错。抽取文本会丢列，按它数位必然错位。
 *     教训：本文件的寄存器定义以【实测跑通 + 用户核对】为准，
 *     不因 PDF 抽取文本的排版而改动。
 *
 *   实测行为（2000 次连续采样、40ms 窗口）：
 *     - 常态值为 0x10，即 otp_read_done(bit4) = 1，表示芯片已就绪
 *     - meas_m_done(bit0) 为【窄脉冲】，每约 826us 出现一次
 *     - 写 1 到 meas_m_done / meas_t_done 可清除对应中断
 *----------------------------------------------------------------------------*/
#define MMC5983MA_ST_MEAS_M_DONE        0x01u   /* bit0 磁场测量完成          */
#define MMC5983MA_ST_MEAS_T_DONE        0x02u   /* bit1 温度测量完成          */
#define MMC5983MA_ST_OTP_RD_DONE        0x10u   /* bit4 OTP 读取完成 = 芯片就绪 */

/*------------------------------------------------------------------------------
 * Internal Control 0 (0x09)   Mode: W（只写）   复位值 0x00
 *
 *   位分配：bit0 TM_M | bit1 TM_T | bit2 INT_meas_done_en | bit3 Set
 *           bit4 Reset | bit5 Auto_SR_en | bit6 OTP_Read | bit7 保留
 *
 *   注：本寄存器 Mode 标为 W（只写），但 Set/Reset 位可读回以判断操作是否
 *       结束（自清除）。实测读回值可用。
 *----------------------------------------------------------------------------*/
#define MMC5983MA_CTRL0_TM_M            0x01u   /* 写1触发一次磁场测量，结束自清 */
#define MMC5983MA_CTRL0_TM_T            0x02u   /* 写1触发一次温度测量，结束自清 */
#define MMC5983MA_CTRL0_INT_MEAS_EN     0x04u   /* 写1使能测量完成中断输出      */
#define MMC5983MA_CTRL0_SET             0x08u   /* 写1执行 SET，结束自清        */
#define MMC5983MA_CTRL0_RESET           0x10u   /* 写1执行 RESET，结束自清      */
#define MMC5983MA_CTRL0_AUTO_SR_EN      0x20u   /* 写1开启自动 SET/RESET        */
#define MMC5983MA_CTRL0_OTP_READ        0x40u   /* 写1重读 OTP，完成自清        */

/* TM_M 与 TM_T 不可同时置 1 */

/*------------------------------------------------------------------------------
 * Internal Control 1 (0x0A)   Mode: W（只写）   复位值 0x00
 *
 *   位分配：bit0 BW0 | bit1 BW1 | bit2 X-inhibit
 *           bit3 YZ-inhibit | bit4 YZ-inhibit（bit4/bit3 两列合并标注）
 *           bit5 保留 | bit6 保留 | bit7 SW_RST
 *
 *   BW 档位（BW1 BW0）：
 *     00 -> 8ms   / 100Hz 带宽 / 0.4 mG RMS / 该档最大 50 Hz
 *     01 -> 4ms   / 200Hz 带宽 / 0.6 mG RMS / 该档最大 100 Hz
 *     10 -> 2ms   / 400Hz 带宽 / 0.8 mG RMS / 该档最大 225 Hz
 *     11 -> 0.5ms / 800Hz 带宽 / 1.2 mG RMS / 该档最大 580 Hz
 *            （CM_Freq=111 时可达 1000 Hz，且要求 BW=11）
 *----------------------------------------------------------------------------*/
#define MMC5983MA_CTRL1_BW0             0x01u   /* 位0 BW0                     */
#define MMC5983MA_CTRL1_BW1             0x02u   /* 位1 BW1                     */
#define MMC5983MA_CTRL1_X_INHIBIT       0x04u   /* 位2 写1关闭 X 通道          */
#define MMC5983MA_CTRL1_YZ_INHIBIT      0x18u   /* 位4:3 写1关闭 Y、Z 通道     */
#define MMC5983MA_CTRL1_SW_RST          0x80u   /* 位7 写1软复位（约10ms）     */

#define MMC5983MA_CTRL1_BW_00           0x00u
#define MMC5983MA_CTRL1_BW_01           0x01u
#define MMC5983MA_CTRL1_BW_10           0x02u
#define MMC5983MA_CTRL1_BW_11           0x03u

/*------------------------------------------------------------------------------
 * Internal Control 2 (0x0B)   Mode: W（只写）   复位值 0x00
 *
 *   位分配：bit2:0 CM_Freq[2:0] | bit3 Cmm_en
 *           bit6:4 Prd_set[2:0] | bit7 En_prd_set
 *
 *   CM_Freq[2:0]（连续模式测量频率，标称值以 BW=00 为基准）：
 *     000 连续模式关闭
 *     001 1 Hz      010 10 Hz     011 20 Hz     100 50 Hz
 *     101 100 Hz    110 200 Hz（要求 BW=01）   111 1000 Hz（要求 BW=11）
 *
 *   Cmm_en：写1开启连续模式；开启时 CM_Freq 不能为 000。
 *
 *   Prd_set[2:0]（每多少次测量做一次 SET）：
 *     000 每 1 次    001 每 25 次   010 每 75 次   011 每 100 次
 *     100 每 250 次  101 每 500 次  110 每 1000 次 111 每 2000 次
 *     （1000 Hz 下 110 = 约每秒一次，111 = 约每两秒一次）
 *
 *   En_prd_set：写1开启周期 SET；需 Auto_SR_en 与 Cmm_en 同时为 1。
 *----------------------------------------------------------------------------*/
#define MMC5983MA_CTRL2_CM_FREQ_Pos     0u
#define MMC5983MA_CTRL2_CM_FREQ_Msk     (0x07u << 0)
#define MMC5983MA_CTRL2_CMM_EN          0x08u   /* 位3 写1开启连续模式         */
#define MMC5983MA_CTRL2_PRD_SET_Pos     4u
#define MMC5983MA_CTRL2_PRD_SET_Msk     (0x07u << 4)
#define MMC5983MA_CTRL2_EN_PRD_SET      0x80u   /* 位7 写1开启周期 SET         */

#define MMC5983MA_CM_FREQ_OFF           0x00u
#define MMC5983MA_CM_FREQ_1HZ           0x01u
#define MMC5983MA_CM_FREQ_10HZ          0x02u
#define MMC5983MA_CM_FREQ_20HZ          0x03u
#define MMC5983MA_CM_FREQ_50HZ          0x04u
#define MMC5983MA_CM_FREQ_100HZ         0x05u
#define MMC5983MA_CM_FREQ_200HZ         0x06u   /* 要求 BW=01 */
#define MMC5983MA_CM_FREQ_1000HZ        0x07u   /* 要求 BW=11 */

/*------------------------------------------------------------------------------
 * Internal Control 3 (0x0C)   Mode: W（只写）   复位值 0x00
 *
 *   位分配（已确认部分）：bit1 St_enp | bit2 St_enm | bit6 Spi_3w
 *
 *   Spi_3w   ：写1进入 3 线 SPI 模式（默认 4 线）。
 *   St_enp   ：写1向内部线圈通正向电流、产生附加磁场，用于检测传感器饱和。
 *   St_enm   ：写1通反向电流，用途同上。
 *
 *   注：St_enp / St_enm 的位号来自手册段落描述，位图未逐位确认，
 *       使用前请对照原版手册位图。其余位为保留且本文件不定义。
 *----------------------------------------------------------------------------*/
#define MMC5983MA_CTRL3_SPI_3W          0x40u   /* 位6 写1进入 3 线 SPI 模式   */

/*==============================================================================
 * 数据合并（18bit 无符号）
 *
 *   raw = (XOUT0 << 10) | (XOUT1 << 2) | (XYZOUT2 >> 6 & 0x03)
 *   raw = (YOUT0 << 10) | (YOUT1 << 2) | (XYZOUT2 >> 4 & 0x03)
 *   raw = (ZOUT0 << 10) | (ZOUT1 << 2) | (XYZOUT2 >> 2 & 0x03)
 *============================================================================*/
#define MMC5983MA_XYZ_X(r0, r1, r2)  ((((uint32_t)(r0)) << 10) | \
                                      (((uint32_t)(r1)) <<  2) | \
                                      (((uint32_t)(r2) >> 6) & 0x03u))
#define MMC5983MA_XYZ_Y(r0, r1, r2)  ((((uint32_t)(r0)) << 10) | \
                                      (((uint32_t)(r1)) <<  2) | \
                                      (((uint32_t)(r2) >> 4) & 0x03u))
#define MMC5983MA_XYZ_Z(r0, r1, r2)  ((((uint32_t)(r0)) << 10) | \
                                      (((uint32_t)(r1)) <<  2) | \
                                      (((uint32_t)(r2) >> 2) & 0x03u))

/*==============================================================================
 * 电气与标度
 *============================================================================*/
#define MMC5983MA_COUNTS_PER_G_16BIT       4096.0f     /* 16bit 模式  */
#define MMC5983MA_COUNTS_PER_G_18BIT       16384.0f    /* 18bit 模式  */
#define MMC5983MA_FS_GAUSS                 8.0f        /* 满量程 ±8 G */
#define MMC5983MA_RMS_NOISE_MG_BW00        0.4f
#define MMC5983MA_RMS_NOISE_MG_BW01        0.6f
#define MMC5983MA_RMS_NOISE_MG_BW10        0.8f
#define MMC5983MA_RMS_NOISE_MG_BW11        1.2f

/*==============================================================================
 * SET / RESET 操作说明（本器件最容易被误用的部分）
 *
 *   SET 与 RESET 是【同一个操作的两个方向】：向内部磁耦合线圈通入大电流脉冲
 *   （手册标称 500ns），产生方向相反（180 度）的强磁场，把 AMR 薄膜的磁化
 *   矢量对齐到该方向。二者都不改变被测的外界磁场，只改变传感器自身的
 *   【感知极性】。位自清除。
 *
 *   由此得到两个用途：
 *
 *   1) 恢复/维护（任意强磁 >10 G 干扰后必须做）
 *      强磁会翻转或打乱薄膜磁化方向，使传感器特性改变，需 SET/RESET 复位。
 *
 *   2) 消除电桥零偏（Offset）
 *      SET   后测量： Output1 = +H + Offset
 *      RESET 后测量： Output2 = -H + Offset
 *      外界磁场 H 的符号随 SET/RESET 翻转，而电桥固有 Offset 不翻转，于是
 *          H      = (Output1 - Output2) / 2      <- 抵消 Offset
 *          Offset = (Output1 + Output2) / 2      <- 测出 Offset
 *      这是消除 Offset 随温度漂移的标准手段。
 *
 *   注意：芯片的 En_prd_set「周期 SET」只发 SET、不配 RESET，
 *         因此它只具备用途 1，不能做上述 (Output1-Output2)/2 消零偏。
 *
 *   【实测观察】SET 之后磁场需要约 2 个采样周期（1000 Hz 下约 2ms）才稳定；
 *   RESET 之后没有可观测的过渡。故成对使用时建议丢弃每段前 2 个样本。
 *============================================================================*/

#ifdef __cplusplus
}
#endif

#endif /* __MMC5983MA_REG_H */
