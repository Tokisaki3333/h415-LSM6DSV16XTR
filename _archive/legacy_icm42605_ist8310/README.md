# 归档：ICM-42605 + IST8310 旧子系统

归档日期：本次架构重构
归档原因：**前代板子（设计类似但硬件层不同）复制过来的固件**，本次按新硬件重建。

## 为什么整体归档而不是逐个删

这 13 个文件构成一条**闭包子系统**，内部互相依赖，外部只被自己引用：

```
spi_hw.c/.h          旧 SPI1 驱动 + ICM-42605 初始化
   ^
   |  include
   |
SPI_rx.c/.h          旧 8K 中断 + SPI DMA 帧 + v5f_hold_t
   ^
   |  include                          v5f_proc.h ──include──> SPI_rx.h
   |                                   v5f_proc.h ──include──> v5f_tune.h
   |
proc_*.c (7 个)      八步处理链，只被 SPI_rx.c 的中断调用
```

**依赖方向全部向内**，没有任何外部文件需要它们。所以整体移走是安全的。

## 与新架构的对应关系

| 旧（本归档） | 新 | 说明 |
|---|---|---|
| `spi_hw.c` SPI1 驱动 | `Common/Common/sensor_dma.c` | 保留"小核拉 CS + 启动 TX DMA、大核抬 CS"的模式 |
| `SPI_rx.c` 的 `DMA1_Channel2_IRQHandler` | `V5F/User/src/sensor_dma_rx.c` | 陀螺改为走 DMA1_Ch4（旧架构两条 SPI 抢 Ch2/Ch3） |
| `SPI_rx.c` 的 `rxfifo` 帧解析 | `sensor_dma_rx.c` 的 `DMA1_Channel4_IRQHandler` | 帧长 15 -> 15，但布局按 LSM6DSV16X |
| `v5f_hold_t`（`SPI_rx.h`） | `mipc_shm.h` 的 `imu_chan_t` / `mmc_chan_t` | 改为直接发共享区，不再经中转结构 |
| IST8310 磁力计 | MMC5983MA | 三轴 16 位 -> 18 位原始值 |
| ICM-42605 陀螺 | LSM6DSV16X | 量程/ODR/寄存器全不同 |
| `proc_*` 八步处理链 | **尚未实现** | 见下 |

## 八步处理链的价值（以后参考用）

`proc_*` 那 7 个文件 + `v5f_proc.h` 是**完整的姿态解算实现**，虽然绑定旧器件，
但算法结构可复用。运行时序（旧 `SPI_rx.c` 的中断里）：

```
1. proc_gyro_bias      陀螺零偏估计
2. proc_static_detect  静止检测（判断能否更新零偏）
3. proc_attitude       姿态解算（四元数）
4. proc_accel_cal      加速度计标定（牵引/挂钩，外部放行）
5. proc_acc_gate       加速度门控
6. proc_velocity       速度（地理系）
7. proc_ekf            EKF 融合
8. proc_ekf_gate       EKF 观测门控（外部放行）
```

**设计要点（值得继承）**：
- 每步是独立函数，输入 `v5f_hold_t` + 门控结构，返回状态码
- 有"门控"概念：某些步骤的更新需要外部放行，避免在错误的时机改状态
- 用"双缓冲 + 帧号"保证一个中断内读到的是同一帧的原始值

**新架构里对应的挂载点**：`V5F/User/src/sensor_dma_rx.c` 的
`DMA1_Channel4_IRQHandler` 里标注了 `--- 主运动解析的挂载点 ---` 的位置。

## 恢复方式

若要重新启用（例如查旧算法实现），把文件移回原位：

```
_archive/legacy_icm42605_ist8310/Common/Common/*   ->  Common/Common/
_archive/legacy_icm42605_ist8310/V5F/User/inc/*    ->  V5F/User/inc/
_archive/legacy_icm42605_ist8310/V5F/User/src/*    ->  V5F/User/src/
```

**注意**：恢复后必须同时把 `V5F` 工程里的 `excludeResources` 清理掉
（见 `V5F/CH32串口陀螺仪_V5F.wvproj`），否则 MRS 仍会跳过它们。

## 归档时同时摘除的残留 include

归档前，这两个文件 include 了 `spi_hw.h` 但**零符号使用**，
归档后会导致编译失败，所以已一并摘除：

- `Common/Common/hardware.c` —— 文件本身已是空壳，仅保留以维持工程编译列表
- `V3F/User/mipc_v3.c` —— 只用 IPC，与 SPI 无关

两处均已备份为 `.bak`（在原位置）。
