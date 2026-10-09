# 参考实现调研：通用 IMU(+地磁+气压) 模块的实现思路对比

定位：本模块是**通用型 IMU+地磁+气压 传感器/姿态模块**，不是飞控。所以对比的判据不是"能不能飞"，
而是**同样的精度下谁的路径更短、CPU/总线开销更低、状态更可解释**。

调研对象：PX4(ICM-42605 驱动 + 热补偿)、ArduPilot(Invensense 驱动 / IST8310 驱动)、
Betaflight(gyro.c)、Paparazzi(invensense3.c 驱动)、xioTechnologies Fusion(AHRS 互补滤波)。

---

## 0. 先确认的两个"我们没写错"的地方

| 项 | 我们 | 参考 | 结论 |
|---|---|---|---|
| ICM-42605 `GYRO_CONFIG0(0x4F)=0x03` | ±2000 dps + ODR 8 kHz | [PX4 `GYRO_FS_SEL_2000_DPS`/`GYRO_ODR_8kHz = Bit1\|Bit0`](https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/drivers/imu/invensense/icm42605/InvenSense_ICM42605_registers.hpp) 完全同表 | ✅ 一致 |
| `ACCEL_CONFIG0(0x50)=0x03` | ±16 g + 8 kHz | 同上 `ACCEL_FS_SEL_16G`/`ACCEL_ODR_8kHz` | ✅ 一致 |
| IST8310 初始化 | `CNTL2=0x0C`(DREN+DRP)、`PDCNTL=0xC0`、`AVGCNTL=0x24`(=X/Z 16× + Y 16×)、`CNTL1=0x01` 单次测量 + DRDY | [ArduPilot IST8310](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Compass/AP_Compass_IST8310.cpp)：单次测量、`AVGCNTL_VAL_Y_16|XZ_16`、100 Hz、0.3 µT/LSB | ✅ 硬件平均已经吃到，不用改 |
| 气压 | BMP388 片内 IIR 关、软件 32 点 O(1) 滑动平均（193.8 Hz → 0.165 s） | 各家多为片内 IIR + OSR；我们的做法**已经是最简** | ✅ 无需改 |

---

## 1. 我们现在的路径（实测）

双核分工：
- **V3F**：ICM-42605 初始化；`EXTI7_0`（PE0=INT1 DRDY）里 `SPI_ReadMulti(0x1D,14)` + `shm_publish_gyro(ts)`（`cnt++`）。
- **V5F**：`DMA1_Channel2_IRQHandler`（SPI1 RX DMA，15 B）里跑**整条链**：
  `gyro_bias → static_detect → attitude → accel_cal → acc_gate → velocity → EKF(16 状态 ESKF) → ekf_gate`，
  然后 `justfloat_report()` 组帧（VER=100：4×float32 q + `00 00 80 7F` = 20 B）入 CDC EP2。

**实测（两个 VER=100 录像，脚本 `_rate23b.py` 同源方法）**：

| 量 | 061843 | 061641 |
|---|---|---|
| CDC 帧率 | 8096 Hz | 8070 Hz |
| EKF 四元数真实更新率 | 352.0 Hz | 350.9 Hz |
| **每个更新重复的帧数** | **恰好 23**（p10=p90=max=23） | **恰好 23** |
| 相同帧占比 | 95.7%（88354/92370） | 95.7%（160556/167854） |

即：**输出 8.08 kHz × 20 B = 161 kB/s，其中 95.7% 是同一份姿态的复制**；姿态实际 351 Hz。
复算：`python tools/acceptance/_rate.py <export.txt> <时长s>`。
代码侧的闸门在 `v5f_proc_ekf()`：

```c
tk = h->imu.fresh.drdy_tick;
if (tk == 0) return;  if (tk <= s_last_tick) { s_last_tick = tk; return; }
dt = (float)(tk - s_last_tick) * 1e-8f;   s_last_tick = tk;
if (dt <= 0.0f || dt > V5F_EKF_DT_MAX_S) return;   /* dt==0 → 整链空转 */
```

`proc_attitude.c` / `proc_gyro_bias.c` / `proc_accel_cal.c` / `proc_velocity.c` 同样是 `drdy_tick` 门控；
`proc_acc_gate.c` / `proc_static_detect.c` **不看 tick**，所以它们真的按 8.08 kHz 在跑（256 样本/格 = 32 ms，
注释里的粒度是对的）——但它们看到的是同一份数据的 23 次复制。

**后果（两类，取决于 23:1 出在哪一侧，需现场确认）**：
- 若 8 kHz 采样被**丢弃**（只积分 1/23）→ 相当于 351 Hz 无抗混叠抽取，>175 Hz 的分量折进姿态；
- 若数据是**陈旧复制** → `acc_gate` 的 256 点窗口只有 ~11 个独立样本，**方差被系统性低估 ~23 倍**，
  阈值/牵引判定会偏，门控抖动与这个有关。

**这个 23 必须先钉死**（成本极低）：在日志里加两个计数器即可——
V3F 的 EXTI 进入次数/s、`g_shm->gyro.hdr.cnt` 增量、V5F 环路次数。三者一比就知道是 DRDY 只有 351 Hz，
还是 V5F 环路自计时 8 kHz 而数据 351 Hz 到。

---

## 2. 各家怎么做的（可直接借鉴的部分）

### 2.1 ArduPilot `AP_InertialSensor_Invensense.cpp` —— 抽取与下采样
```c
_accum.gyro += g;                                  // 8k/4k 原始样本
if (_accum.gyro_count % _gyro_fifo_downsample_rate == 0) {
    _accum.gyro *= _fifo_gyro_scale;                // 整数盒式平均 → 1 kHz
    _rotate_and_correct_gyro(...); _notify_new_gyro_raw_sample(...);
}
```
- **盒式平均 + 定比下采样到估计器速率（1 kHz）**，而不是抽点。既抗混叠又省 8× CPU。
- 加速度削顶：`fabsf(a.?) > unscaled_clip_limit` → `increment_clip_count(accel_instance)`，
  **削顶次数上抛给 EKF** 去放大噪声（`_clip_limit = 29.5 g`）。
- **用温度检测 FIFO 错位**：`int16_t t2 = int16_val(data,3); if (!_check_raw_temp(t2)) { _fifo_reset(true); }`
  ——相邻样本温度不可能跳变，一跳变就是流错位。比校验和更早、更便宜。
- IST8310：单次测量 + 16× 硬件平均 + 100 Hz（与我们一致）。

### 2.2 Betaflight `gyro.c` —— 溢出/削顶策略
```c
#define GYRO_OVERFLOW_TRIGGER_THRESHOLD 31980  // 97.5% FS
#define GYRO_OVERFLOW_RESET_THRESHOLD   30340  // 92.5% FS
// 触发后：三轴都 <92.5% FS 连续 50 ms 才复位；期间输出上一帧值
```
- **迟滞 + 保持**，且 `gyroOverflowDetected()` 作为对外状态位；`gyroGetTemperature()` 供热补偿。
- 相比之下我们只把削顶当**显示/记录**（OLED + `clip%d/1000`），没有反馈进估计器，也没有迟滞状态。

### 2.3 PX4 ICM-42605 驱动 —— 取数与上游补偿
- **FIFO**（2048 B）+ 水位中断，包结构 `header|accel6|gyro6|temp|timestamp16`，
  即**样本自带硬件时间戳**和温度；我们靠 MCU `GetTime64_10Ns()` + EXTI，dt 里含中断延迟抖动。
- 热补偿：`offset = X0 + X1·ΔT + X2·ΔT² + X3·ΔT³`，`corrected = (raw − offset)·SCL`，
  按轴拟合、`TREF/TMIN/TMAX` 夹取，系数**固化在参数里**，在 sensors 模块（估计器上游）统一施加。
  官方明确写了限制：**"Scale factors are assumed to be temperature invariant"**。
- 温度换算常数与我们一致：`TEMPERATURE_SENSITIVITY = 132.48 LSB/°C`、`TEMPERATURE_OFFSET = 25 °C`
  （我们 `SPI_rx.c` 里 `raw/132.48 + 25.0f`）。

### 2.4 Paparazzi `invensense3.c` —— 驱动层表格
- 每个 FS 一张比例表、**每台设备各自的 ODR/FS 编码**（42605/42688/40609/IIM42652 不同）；
- **抗混叠滤波器(AAF)系数表，且 4x605/4x609 单独一张**（`invensense3_aaf4x605`）：
  `{3dB 带宽, AAF_DELT, AAF_DELTSQR, AAF_BITSHIFT}`；
- FIFO 包头校验 `(data[0] & 0xFC) != 0x68` 才认样。

### 2.5 xioTechnologies Fusion(AHRS 互补滤波) —— "通用模块"的轻量选项
- 全算法 ~700 行，无矩阵、无状态协方差：`gain = 0.5`，启动增益 `10 → 0.5` 在 3 s 内斜坡下降；
- 门控阈值用**角度**表达：`accelerationRejection ≈ 0.5·sin(θ)`，推荐 accel 10~15°、mag 20~25°；
- **反锁死**：`recoveryTrigger`（连续被拒 +1、通过 −9，超过 `rejectionTimeout` 就强制放行一次）；
- **超量程恢复**：任一轴 `|ω| > overrangeThreshold` → `SoftRestart()`（保留输出、重新斜坡增益），
  而不是把轨到 FS 的数据积进去。

---

## 3. 建议的动作（按性价比排序）

1. **定一个估计器速率 + 盒式平均抽取**（照抄 ArduPilot 的 `_accum` 思路）。
   现在 8.08 kHz 全链跑、只产出 351 Hz，改完 CPU 直接降一个数量级，且顺带做掉抗混叠。
   同时 CDC 只在**有新解**时发（或按固定速率发），USB 从 161 kB/s 降到 ~7 kB/s。
2. **把 23:1 钉死并写进日志**（三个计数器）。在查明前，不要动任何门控阈值——因为窗口统计的
   等效样本数可能只有标称的 1/23。
3. **削顶进估计器**：Betaflight 式迟滞（97.5% 触发 / 92.5% + 50 ms 复位）+ ArduPilot 式
   削顶计数 → 放大该帧的 R。只影响极限区，主路径不变（符合"极限测试不影响主路径"）。
4. **温度→零偏的固定多项式**：我们已经在读片内温度，先把它**记进日志**跑一次冷启动/热机，
   看零偏-温度斜率是否值得固化；系数固化符合"基线恒定、不要自愈"。注意它对
   **路径成比例的 130 ppm 项无能为力**（PX4 自己也假设 scale 与温度无关）。
5. **地磁门控加反锁死计数**（Fusion 的 `recoveryTrigger`）：我们已有 `mag_r_deg`(度) 与 `mag_rej`，
   只差"连续被拒 N 次强制放行一次"。
6. 若坚持"通用模块"定位，可把 **Fusion 级 AHRS 作为 lite 输出**（无 mag/无 baro 也能给 roll/pitch/yaw），
   与现有 16 状态 ESKF 并存；对外契约回归"补偿后的传感器样本 + 姿态 + 速率"。

**不建议照抄的**：Betaflight 的完整滤波链（PT1+双二阶+动态陷波）——那是为有桨的飞控服务的；
我们没有电机转速源，动态陷波无从做起。PX4/ArduPilot 的 IMU 温度室标定流程同理，代价太大，
我们只需要"记录温度 + 事后拟合一次"。

---

## 4. 参考链接

- PX4 ICM-42605 寄存器/ODR/FS 枚举 —— https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/src/drivers/imu/invensense/icm42605/InvenSense_ICM42605_registers.hpp
- PX4 热标定与补偿（多项式形式、参数命名、限制） —— https://docs.px4.io/v1.12/en/advanced_config/sensor_thermal_calibration
  （开发版镜像：https://bkueng.gitbooks.io/px4-devguide/content/en/tutorials/sensor_thermal_calibration.html）
- Betaflight `gyro.c`（削顶迟滞、温度、滤波链） —— https://raw.githubusercontent.com/betaflight/betaflight/master/src/main/sensors/gyro.c
- ArduPilot Invensense 驱动（盒式抽取、削顶计数、温度查错） —— https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp
- ArduPilot IST8310 驱动 —— https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Compass/AP_Compass_IST8310.cpp
- Paparazzi `invensense3.c`（AAF 表、按器件编码） —— https://docs.paparazziuav.org/latest/invensense3_8c_source.html
- xio Fusion AHRS C 源码（增益斜坡、拒绝/恢复、超量程恢复） —— https://raw.githubusercontent.com/xioTechnologies/Fusion/main/Fusion/FusionAhrs.c
- Fusion 参数说明（推荐阈值 10~15°/20~25°） —— https://docs.rs/fusion-ahrs/latest/fusion_ahrs/struct.Ahrs.html
