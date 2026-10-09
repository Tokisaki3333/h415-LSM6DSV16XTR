# 归档：DMA 采集路径（死代码，做参考）

**归档时间**：2026-10-09
**来源**：`h415-imu42605-` 主工程
**原因**：这套代码描述的是"DMA 采集架构"，与当前实现（两个 DRDY 中断内**阻塞 SPI 读**）不一致；
文件本身也不再被任何代码调用，但其中的架构设计有参考价值，故移出主工程而非删除。

---

## 移出的文件

| 归档名 | 原路径 | 大小 |
|---|---|---|
| `Common_sensor_dma.c` | `Common/Common/sensor_dma.c` | 16,476 B |
| `Common_sensor_dma.h` | `Common/Common/sensor_dma.h` | 9,102 B |
| `V5F_User_src_sensor_dma_rx.c` | `V5F/User/src/sensor_dma_rx.c` | 9,434 B |

---

## 这套代码原本的设计（供后续接入 DMA 时参考）

```
小核 V3F（100MHz）—— 两个 DRDY 中断，短中断
  地磁 DRDY (PE1)  : 记 DRDY 时刻 -> 拉低 CS -> 启动 TX DMA -> 立即返回
  陀螺 DRDY (PD11) : 同上
  ★ ISR 内不碰浮点（RISC-V 下 FP 寄存器保存/恢复会把 80ns 进出变成几百 ns）

大核 V5F（400MHz）—— 两个 DMA 完成中断，负责收尾
  SPI1 RX 完成 (DMA1_Ch2, 地磁) : 解析 -> 发共享区 -> 抬 CS -> 重装 RX
  SPI2 RX 完成 (DMA1_Ch4, 陀螺) : 同上
  ★ CS 抬起放在 ISR 即将退出的位置，用解析耗时当自然延时

时间戳捆绑：DRDY 中断里读 GetTime64_10Ns() 存共享变量，
            DMA 完成中断里取出，与该帧数据一起发布。
```

**CS 必须由大核抬**：小核启动 TX 后立即返回，无法知道 SPI 何时传完；
只有 RX DMA 完成中断（最后一个字节收到）才知道可以释放片选。

---

## 通道参数（将来接 DMA 会用到）

| 器件 | SPI | 通道 | Mux | 请求号 |
|---|---|---|---|---|
| 地磁 MMC5983MA | SPI1 | TX=DMA1_Ch3 / RX=DMA1_Ch2 | Mux3 / Mux2 | 63 / 64 |
| 陀螺 LSM6DSV16X | SPI2 | TX=DMA1_Ch5 / RX=DMA1_Ch4 | Mux5 / Mux4 | 65 / 66 |

**帧长**：地磁 `SENSOR_MMC_FRAME_LEN = 7+1 = 8` 字节（命令 + 7 数据）；
陀螺 `SENSOR_IMU_FRAME_LEN`（命令 + 14 数据）。

**地磁是 7 字节不是 6**：三轴共用中间与末尾字节，读法见 `mmc5983ma_reg.h` 的
`MMC5983MA_XYZ_X/Y/Z` 宏。

---

## 它定义的全局变量（已在新位置补上定义）

`sensor_dma.c` 原本定义了两个时间戳，被 `board_io.h` 声明、两个 DRDY ISR 写入。
**归档后这两个定义已移入 `board_io.c`**，否则会链接失败：

```c
/* board_io.c */
volatile uint64_t g_mmc_drdy_ts = 0u;   /* 地磁本拍 DRDY 时刻（10ns）*/
volatile uint64_t g_imu_drdy_ts = 0u;   /* 陀螺本拍 DRDY 时刻（10ns）*/
```

其余符号（`g_mmc_tx/rx`、`g_imu_tx/rx`、`g_mmc_enable_ts`、
`g_mmc_first_isr_ts` 等）只在该文件内部使用，无外部声明，随文件一起归档。

---

## 从构建中移除的方式

两个工程都通过 `.cproject` 的 `<sourceEntries><entry excluding="...">` 排除：

```
V3F/.cproject:
  ...|Common/sensor_dma.c|Common/sensor_dma.h|User/sensor_dma_rx.c

V5F/.cproject:
  ...|Common/sensor_dma.c|Common/sensor_dma.h|User/src/sensor_dma_rx.c
```

`V5F/CH32串口陀螺仪_V5F.wvproj` 里对应的 `<file>` 条目也已删除。

**注意**：`V3F/obj/` 与 `V5F/obj/` 下的 `subdir.mk` 是 Eclipse CDT 自动生成的
（从 `.cproject` 派生）。若构建时仍去编译这些文件，清理对应工程的 `obj/` 后重建即可。

---

## 同时清理的过期注释

以下位置的注释原本指向这套已移除的代码，已改为描述当前实现：

| 文件 | 原文引用 |
|---|---|
| `Common/Common/board_io.c` | `SENSOR_DMA_Init`、`SENSOR_DMA_ArmDrdy`、"处理体本身在 sensor_dma.c" |
| `Common/Common/hardware.c` | 双核分工表里的三个文件路径 |
| `Common/Common/lsm6dsv16x.c` | `SENSOR_DMA_Init() 装 DMA / 使能 EXTI` |
| `Common/Common/sensor_spi.c` | `RX DMA 请求一旦打开（SENSOR_DMA_Init 里做的）` |
| `V5F/User/main.c` | 头部架构说明、主循环挂载点说明 |

---

## ⚠ 归档后暴露的已知缺口（尚未处理）

**两个核之间目前没有数据通道。**

- V5F 的 `mipc_v5_get_shared()` 会**死等** `g_shm` 非空
- 而 `g_shm` 依赖 V3F 把共享区地址写进 `IPC_MSG0`
- **V3F 的 `mipc_v3.c` 目前并不写 IPC**（它只定义了自己的 `g_shm`）

原来 V3F 侧的共享区发布是由 `sensor_dma.c` 承担的，该文件归档后这个缺口就显性化了。
接入共享区时需要补上 V3F 侧的发布（或改用其他握手方式）。

V5F 的 `main.c` 头部注释已把这个缺口标注出来。
