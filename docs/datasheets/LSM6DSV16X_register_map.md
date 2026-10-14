# LSM6DSV16X 寄存器手册（文字表格版）

> 来源：DS13510 Rev 4 (May 2023)，第 8-17 节。R/W=读写，R=只读；默认值为上电值；output 表示输出寄存器无固定默认值。

---

## 第 8 节 寄存器映射（主接口 SPI/I2C/I3C，Table 24）

| 寄存器名 | 类型 | 地址Hex | 地址Binary | 默认值 | 备注 |
|---|---|---|---|---|---|
| FUNC_CFG_ACCESS | R/W | 01 | 00000001 |  |  |
| PIN_CTRL | R/W | 02 | 00000010 | 00100011 |  |
| IF_CFG | R/W | 03 | 00000011 | 00000000 |  |
| RESERVED | - | 04-05 |  |  |  |
| ODR_TRIG_CFG | R/W | 06 | 00000110 | 00000000 |  |
| FIFO_CTRL1 | R/W | 07 | 00000111 | 00000000 |  |
| FIFO_CTRL2 | R/W | 08 | 00001000 | 00000000 |  |
| FIFO_CTRL3 | R/W | 09 | 00001001 | 00000000 |  |
| FIFO_CTRL4 | R/W | 0A | 00001010 | 00000000 |  |
| COUNTER_BDR_REG1 | R/W | 0B | 00001011 | 00000000 |  |
| COUNTER_BDR_REG2 | R/W | 0C | 00001100 | 00000000 |  |
| INT1_CTRL | R/W | 0D | 00001101 | 00000000 |  |
| INT2_CTRL | R/W | 0E | 00001110 | 00000000 |  |
| WHO_AM_I | R | 0F | 00001111 | 01110000 | R(SPI2) |
| CTRL1 | R/W | 10 | 00010000 | 00000000 | R(SPI2) |
| CTRL2 | R/W | 11 | 00010001 | 00000000 | R(SPI2) |
| CTRL3 | R/W | 12 | 00010010 | 01000100 | R(SPI2) |
| CTRL4 | R/W | 13 | 00010011 | 00000000 | R(SPI2) |
| CTRL5 | R/W | 14 | 00010100 | 00000000 | R(SPI2) |
| CTRL6 | R/W | 15 | 00010101 | 00000000 | R(SPI2) |
| CTRL7 | R/W | 16 | 00010110 | 00000000 | R(SPI2) |
| CTRL8 | R/W | 17 | 00010111 | 00000000 | R(SPI2) |
| CTRL9 | R/W | 18 | 00011000 | 00000000 | R(SPI2) |
| CTRL10 | R/W | 19 | 00011001 | 00000000 | R(SPI2) |
| CTRL_STATUS | R | 1A | 00011010 | output |  |
| FIFO_STATUS1 | R | 1B | 00011011 | output |  |
| FIFO_STATUS2 | R | 1C | 00011100 | output |  |
| ALL_INT_SRC | R | 1D | 00011101 | output |  |
| STATUS_REG | R | 1E | 00011110 | output |  |
| RESERVED | - | 1F |  |  |  |
| OUT_TEMP_L | R | 20 | 00100000 | output |  |
| OUT_TEMP_H | R | 21 | 00100001 | output |  |
| OUTX_L_G | R | 22 | 00100010 | output | 陀螺X低 |
| OUTX_H_G | R | 23 | 00100011 | output | 陀螺X高 |
| OUTY_L_G | R | 24 | 00100100 | output | 陀螺Y低 |
| OUTY_H_G | R | 25 | 00100101 | output | 陀螺Y高 |
| OUTZ_L_G | R | 26 | 00100110 | output | 陀螺Z低 |
| OUTZ_H_G | R | 27 | 00100111 | output | 陀螺Z高 |
| OUTX_L_A | R | 28 | 00101000 | output | 加速度X低 |
| OUTX_H_A | R | 29 | 00101001 | output | 加速度X高 |
| OUTY_L_A | R | 2A | 00101010 | output | 加速度Y低 |
| OUTY_H_A | R | 2B | 00101011 | output | 加速度Y高 |
| OUTZ_L_A | R | 2C | 00101100 | output | 加速度Z低 |
| OUTZ_H_A | R | 2D | 00101101 | output | 加速度Z高 |
| UI_OUTX_L_G_OIS_EIS | R | 2E | 00101110 | output |  |
| UI_OUTX_H_G_OIS_EIS | R | 2F | 00101111 | output |  |
| UI_OUTY_L_G_OIS_EIS | R | 30 | 00110000 | output |  |
| UI_OUTY_H_G_OIS_EIS | R | 31 | 00110001 | output |  |
| UI_OUTZ_L_G_OIS_EIS | R | 32 | 00110010 | output |  |
| UI_OUTZ_H_G_OIS_EIS | R | 33 | 00110011 | output |  |
| UI_OUTX_L_A_OIS_DualC | R | 34 | 00110100 | output |  |
| UI_OUTX_H_A_OIS_DualC | R | 35 | 00110101 | output |  |
| UI_OUTY_L_A_OIS_DualC | R | 36 | 00110110 | output |  |
| UI_OUTY_H_A_OIS_DualC | R | 37 | 00110111 | output |  |
| UI_OUTZ_L_A_OIS_DualC | R | 38 | 00111000 | output |  |
| UI_OUTZ_H_A_OIS_DualC | R | 39 | 00111001 | output |  |
| AH_QVAR_OUT_L | R | 3A | 00111010 | output |  |
| AH_QVAR_OUT_H | R | 3B | 00111011 | output |  |
| RESERVED | - | 3C-3F |  |  |  |
| TIMESTAMP0 | R | 40 | 01000000 | output | R(SPI2) |
| TIMESTAMP1 | R | 41 | 01000001 | output | R(SPI2) |
| TIMESTAMP2 | R | 42 | 01000010 | output | R(SPI2) |
| TIMESTAMP3 | R | 43 | 01000011 | output | R(SPI2) |
| UI_STATUS_REG_OIS | R | 44 | 01000100 | output |  |
| WAKE_UP_SRC | R | 45 | 01000101 | output |  |
| TAP_SRC | R | 46 | 01000110 | output |  |
| D6D_SRC | R | 47 | 01000111 | output |  |
| STATUS_MASTER_MAINPAGE | R | 48 | 01001000 | output |  |
| EMB_FUNC_STATUS_MAINPAGE | R | 49 | 01001001 | output |  |
| FSM_STATUS_MAINPAGE | R | 4A | 01001010 | output |  |
| MLC_STATUS_MAINPAGE | R | 4B | 01001011 | output |  |
| RESERVED | - | 4C-4E |  |  |  |
| INTERNAL_FREQ_FINE | R | 4F | 01001111 | output |  |
| FUNCTIONS_ENABLE | R/W | 50 | 01010000 | 00000000 |  |
| DEN | R/W | 51 | 01010001 | 00001110 |  |
| INACTIVITY_DUR | R/W | 54 | 01010100 | 00000100 |  |
| INACTIVITY_THS | R/W | 55 | 01010101 | 00000000 |  |
| TAP_CFG0 | R/W | 56 | 01010110 | 00000000 |  |
| TAP_CFG1 | R/W | 57 | 01010111 | 00000000 |  |
| TAP_CFG2 | R/W | 58 | 01011000 | 00000000 |  |
| TAP_THS_6D | R/W | 59 | 01011001 | 00000000 |  |
| TAP_DUR | R/W | 5A | 01011010 | 00000000 |  |
| WAKE_UP_THS | R/W | 5B | 01011011 | 00000000 |  |
| WAKE_UP_DUR | R/W | 5C | 01011100 | 00000000 |  |
| FREE_FALL | R/W | 5D | 01011101 | 00000000 |  |
| MD1_CFG | R/W | 5E | 01011110 | 00000000 |  |
| MD2_CFG | R/W | 5F | 01011111 | 00000000 |  |
| RESERVED | - | 60-61 |  |  |  |
| HAODR_CFG | R/W | 62 | 01100010 | 00000000 |  |
| EMB_FUNC_CFG | R/W | 63 | 01100011 | 00000000 |  |
| UI_HANDSHAKE_CTRL | R/W | 64 | 01100100 | 00000000 |  |
| UI_SPI2_SHARED_0 | R/W | 65 | 01100101 | 00000000 |  |
| UI_SPI2_SHARED_1 | R/W | 66 | 01100110 | 00000000 |  |
| UI_SPI2_SHARED_2 | R/W | 67 | 01100111 | 00000000 |  |
| UI_SPI2_SHARED_3 | R/W | 68 | 01101000 | 00000000 |  |
| UI_SPI2_SHARED_4 | R/W | 69 | 01101001 | 00000000 |  |
| UI_SPI2_SHARED_5 | R/W | 6A | 01101010 | 00000000 |  |
| CTRL_EIS | R/W | 6B | 01101011 | 00000000 |  |
| RESERVED | - | 6C-6E |  |  |  |
| UI_INT_OIS | R/W | 6F | 01101111 | 00000000 |  |
| UI_CTRL1_OIS | R/W | 70 | 01110000 | 00000000 |  |
| UI_CTRL2_OIS | R/W | 71 | 01110001 | 00000000 |  |
| UI_CTRL3_OIS | R/W | 72 | 01110010 | 00000000 |  |
| X_OFS_USR | R/W | 73 | 01110011 | 00000000 |  |
| Y_OFS_USR | R/W | 74 | 01110100 | 00000000 |  |
| Z_OFS_USR | R/W | 75 | 01110101 | 00000000 |  |
| RESERVED | - | 76-77 |  |  |  |
| FIFO_DATA_OUT_TAG | R | 78 | 01111000 | output |  |
| FIFO_DATA_OUT_X_L | R | 79 | 01111001 | output |  |
| FIFO_DATA_OUT_X_H | R | 7A | 01111010 | output |  |
| FIFO_DATA_OUT_Y_L | R | 7B | 01111011 | output |  |
| FIFO_DATA_OUT_Y_H | R | 7C | 01111100 | output |  |
| FIFO_DATA_OUT_Z_L | R | 7D | 01111101 | output |  |
| FIFO_DATA_OUT_Z_H | R | 7E | 01111110 | output |  |

> 保留寄存器禁止写入；启动加载寄存器含出厂校准值，不可修改。

---

## 第 10 节 SPI2 寄存器映射（仅辅助 SPI 接口，Table 225）

| 寄存器名 | 类型 | 地址Hex | 默认值 |
|---|---|---|---|
| SPI2_WHO_AM_I | R | 0F | 01110000 |
| SPI2_STATUS_REG_OIS | R | 1E | output |
| SPI2_OUT_TEMP_L | R | 20 | output |
| SPI2_OUT_TEMP_H | R | 21 | output |
| SPI2_OUTX_L_G_OIS | R | 22 | output |
| SPI2_OUTX_H_G_OIS | R | 23 | output |
| SPI2_OUTY_L_G_OIS | R | 24 | output |
| SPI2_OUTY_H_G_OIS | R | 25 | output |
| SPI2_OUTZ_L_G_OIS | R | 26 | output |
| SPI2_OUTZ_H_G_OIS | R | 27 | output |
| SPI2_OUTX_L_A_OIS | R | 28 | output |
| SPI2_OUTX_H_A_OIS | R | 29 | output |
| SPI2_OUTY_L_A_OIS | R | 2A | output |
| SPI2_OUTY_H_A_OIS | R | 2B | output |
| SPI2_OUTZ_L_A_OIS | R | 2C | output |
| SPI2_OUTZ_H_A_OIS | R | 2D | output |
| SPI2_HANDSHAKE_CTRL | R/W | 6E | 00000000 |
| SPI2_INT_OIS | R/W | 6F | 00000000 |
| SPI2_CTRL1_OIS | R/W | 70 | 00000000 |
| SPI2_CTRL2_OIS | R/W | 71 | 00000000 |
| SPI2_CTRL3_OIS | R/W | 72 | 00000000 |

---

## 第 12 节 嵌入式功能寄存器映射（FUNC_CFG_ACCESS.EMB_FUNC_REG_ACCESS=1，Table 262）

| 寄存器名 | 类型 | 地址Hex | 默认值 |
|---|---|---|---|
| PAGE_SEL | R/W | 02 | 00000001 |
| EMB_FUNC_EN_A | R/W | 04 | 00000000 |
| EMB_FUNC_EN_B | R/W | 05 | 00000000 |
| EMB_FUNC_EXEC_STATUS | R | 07 | output |
| PAGE_ADDRESS | R/W | 08 | 00000000 |
| PAGE_VALUE | R/W | 09 | 00000000 |
| EMB_FUNC_INT1 | R/W | 0A | 00000000 |
| FSM_INT1 | R/W | 0B | 00000000 |
| RESERVED | - | 0C |  |
| MLC_INT1 | R/W | 0D | 00000000 |
| EMB_FUNC_INT2 | R/W | 0E | 00000000 |
| FSM_INT2 | R/W | 0F | 00000000 |
| RESERVED | - | 10 |  |
| MLC_INT2 | R/W | 11 | 00010001 |
| EMB_FUNC_STATUS | R | 12 | output |
| FSM_STATUS | R | 13 | output |
| RESERVED | - | 14 |  |
| MLC_STATUS | R | 15 | output |
| PAGE_RW | R/W | 17 | 00000000 |
| RESERVED | - | 18-43 |  |
| EMB_FUNC_FIFO_EN_A | R/W | 44 | 00000000 |
| EMB_FUNC_FIFO_EN_B | R/W | 45 | - |
| FSM_ENABLE | R/W | 46 | 00000000 |
| RESERVED | - | 47 |  |
| FSM_LONG_COUNTER_L | R/W | 48 | 00000000 |
| FSM_LONG_COUNTER_H | R/W | 49 | 00000000 |
| RESERVED | - | 4A |  |
| INT_ACK_MASK | R/W | 4B | 00000000 |
| FSM_OUTS1 | R | 4C | output |
| FSM_OUTS2 | R | 4D | output |
| FSM_OUTS3 | R | 4E | output |
| FSM_OUTS4 | R | 4F | output |
| FSM_OUTS5 | R | 50 | output |
| FSM_OUTS6 | R | 51 | output |
| FSM_OUTS7 | R | 52 | output |
| FSM_OUTS8 | R | 53 | output |
| RESERVED | - | 54-5D |  |
| SFLP_ODR | R/W | 5E | 01011011 |
| FSM_ODR | R/W | 5F | 01001011 |
| MLC_ODR | R/W | 60 | 00010101 |
| STEP_COUNTER_L | R | 62 | output |
| STEP_COUNTER_H | R | 63 | output |
| EMB_FUNC_SRC | R/W | 64 | output |
| EMB_FUNC_INIT_A | R/W | 66 | 00000000 |
| EMB_FUNC_INIT_B | R/W | 67 | 00000000 |
| MLC1_SRC | R | 70 | output |
| MLC2_SRC | R | 71 | output |
| MLC3_SRC | R | 72 | output |
| MLC4_SRC | R | 73 | output |

---

## 第 14 节 嵌入式高级功能页（PAGE_SEL[3:0] 选页）

**Table 347. Page 0（PAGE_SEL=0000）**

| 寄存器名 | 类型 | 地址Hex | 默认值 |
|---|---|---|---|
| SFLP_GAME_GBIASX_L | R/W | 6E | 00000000 |
| SFLP_GAME_GBIASX_H | R/W | 6F | 00000000 |
| SFLP_GAME_GBIASY_L | R/W | 70 | 00000000 |
| SFLP_GAME_GBIASY_H | R/W | 71 | 00000000 |
| SFLP_GAME_GBIASZ_L | R/W | 72 | 00000000 |
| SFLP_GAME_GBIASZ_H | R/W | 73 | 00000000 |
| FSM_EXT_SENSITIVITY_L | R/W | BA | 00100100 |
| FSM_EXT_SENSITIVITY_H | R/W | BB | 00010110 |
| FSM_EXT_OFFX_L | R/W | C0 | 00000000 |
| FSM_EXT_OFFX_H | R/W | C1 | 00000000 |
| FSM_EXT_OFFY_L | R/W | C2 | 00000000 |
| FSM_EXT_OFFY_H | R/W | C3 | 00000000 |
| FSM_EXT_OFFZ_L | R/W | C4 | 00000000 |
| FSM_EXT_OFFZ_H | R/W | C5 | 00000000 |
| FSM_EXT_MATRIX_XX_L | R/W | C6 | 00000000 |
| FSM_EXT_MATRIX_XX_H | R/W | C7 | 00111100 |
| FSM_EXT_MATRIX_XY_L | R/W | C8 | 00000000 |
| FSM_EXT_MATRIX_XY_H | R/W | C9 | 00000000 |
| FSM_EXT_MATRIX_XZ_L | R/W | CA | 00000000 |
| FSM_EXT_MATRIX_XZ_H | R/W | CB | 00000000 |
| FSM_EXT_MATRIX_YY_L | R/W | CC | 00000000 |
| FSM_EXT_MATRIX_YY_H | R/W | CD | 00111100 |
| FSM_EXT_MATRIX_YZ_L | R/W | CE | 00000000 |
| FSM_EXT_MATRIX_YZ_H | R/W | CF | 00000000 |
| FSM_EXT_MATRIX_ZZ_L | R/W | D0 | 00000000 |
| FSM_EXT_MATRIX_ZZ_H | R/W | D1 | 00111100 |
| EXT_CFG_A | R/W | D4 | 00000101 |
| EXT_CFG_B | R/W | D5 | 00000010 |

**Table 348. Page 1（PAGE_SEL=0001）**

| 寄存器名 | 类型 | 地址Hex | 默认值 |
|---|---|---|---|
| FSM_LC_TIMEOUT_L | R/W | 7A | 00000000 |
| FSM_LC_TIMEOUT_H | R/W | 7B | 00000000 |
| FSM_PROGRAMS | R/W | 7C | 00000000 |
| FSM_START_ADD_L | R/W | 7E | 00000000 |
| FSM_START_ADD_H | R/W | 7F | 00000000 |
| PEDO_CMD_REG | R/W | 83 | 00000000 |
| PEDO_DEB_STEPS_CONF | R/W | 84 | 00001010 |
| PEDO_SC_DELTAT_L | R/W | D0 | 00000000 |
| PEDO_SC_DELTAT_H | R/W | D1 | 00000000 |
| MLC_EXT_SENSITIVITY_L | R/W | E8 | 00000000 |
| MLC_EXT_SENSITIVITY_H | R/W | E9 | 00111100 |

**Table 349. Page 2（PAGE_SEL=0010）**

| 寄存器名 | 类型 | 地址Hex | 默认值 |
|---|---|---|---|
| EXT_FORMAT | R/W | 00 | 00000000 |
| EXT_3BYTE_SENSITIVITY_L | R/W | 02 | 00000000 |
| EXT_3BYTE_SENSITIVITY_H | R/W | 03 | 00001100 |
| EXT_3BYTE_OFFSET_XL | R/W | 06 | 00000000 |
| EXT_3BYTE_OFFSET_L | R/W | 07 | 01010100 |
| EXT_3BYTE_OFFSET_H | R/W | 08 | 00111111 |

> **页寄存器写流程示例**（写 Page1 的 84h = 06h）：
> 1. FUNC_CFG_ACCESS(01h).EMB_FUNC_REG_ACCESS=1
> 2. PAGE_RW(17h).PAGE_WRITE=1
> 3. PAGE_SEL(02h)[3:0]=0001
> 4. PAGE_ADDRESS(08h)=84h
> 5. PAGE_VALUE(09h)=06h
> 6. PAGE_RW(17h).PAGE_WRITE=0
> 7. FUNC_CFG_ACCESS(01h).EMB_FUNC_REG_ACCESS=0
>
> 读流程：第 2 步改为 PAGE_READ=1，第 5 步改为读 PAGE_VALUE(09h)。

---

## 第 16 节 传感器集线器寄存器映射（FUNC_CFG_ACCESS.SHUB_REG_ACCESS=1，Table 439）

| 寄存器名 | 类型 | 地址Hex | 默认值 |
|---|---|---|---|
| SENSOR_HUB_1 | R | 02 | output |
| SENSOR_HUB_2 | R | 03 | output |
| SENSOR_HUB_3 | R | 04 | output |
| SENSOR_HUB_4 | R | 05 | output |
| SENSOR_HUB_5 | R | 06 | output |
| SENSOR_HUB_6 | R | 07 | output |
| SENSOR_HUB_7 | R | 08 | output |
| SENSOR_HUB_8 | R | 09 | output |
| SENSOR_HUB_9 | R | 0A | output |
| SENSOR_HUB_10 | R | 0B | output |
| SENSOR_HUB_11 | R | 0C | output |
| SENSOR_HUB_12 | R | 0D | output |
| SENSOR_HUB_13 | R | 0E | output |
| SENSOR_HUB_14 | R | 0F | output |
| SENSOR_HUB_15 | R | 10 | output |
| SENSOR_HUB_16 | R | 11 | output |
| SENSOR_HUB_17 | R | 12 | output |
| SENSOR_HUB_18 | R | 13 | output |
| MASTER_CONFIG | R/W | 14 | 00000000 |
| SLV0_ADD | R/W | 15 | 00000000 |
| SLV0_SUBADD | R/W | 16 | 00000000 |
| SLV0_CONFIG | R/W | 17 | 10000000 |
| SLV1_ADD | R/W | 18 | 00000000 |
| SLV1_SUBADD | R/W | 19 | 00000000 |
| SLV1_CONFIG | R/W | 1A | 00010000 |
| SLV2_ADD | R/W | 1B | 00000000 |
| SLV2_SUBADD | R/W | 1C | 00000000 |
| SLV2_CONFIG | R/W | 1D | 00000000 |
| SLV3_ADD | R/W | 1E | 00000000 |
| SLV3_SUBADD | R/W | 1F | 00000000 |
| SLV3_CONFIG | R/W | 20 | 00000000 |
| DATAWRITE_SLV0 | R/W | 21 | 00000000 |
| STATUS_MASTER | R | 22 | output |

---

## 寄存器位域详细描述
各寄存器每一位含义、取值表见同目录 `LSM6DSV16X_寄存器位域描述.txt`（原文第 9/11/13/15/17 节）。
