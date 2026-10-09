# GPS 融合架构选型：单滤波器 vs 姿态/导航解耦

问题：GPS 直接进同一个 EKF，会不会把"自稳定姿态"带坏？有没有两种可选的架构与参考实现？

结论先说：**有，而且两条路都有成熟参考。**
- 方案 A = 单滤波器 + 强门控（PX4 EKF2 / ArduPilot EKF3 的路线）；
- 方案 B = 姿态滤波与导航滤波解耦（ArduPilot DCM + AP_InertialNav 的路线，也正是"隔离"的直觉）；
- 方案 C = 联邦滤波 / 多车道冗余（EKF3 三车道、Carlson 联邦滤波；工程量最大）。

---

## 1. 为什么"直接进"确实有风险（机理，不是玄学）

同一滤波器里 GPS 只是 p/v 的观测，但它通过**互协方差**而不是观测方程影响姿态：

| 通道 | 机理 | 后果 |
|---|---|---|
| q ↔ v | 过程模型 `v̇ = R(q)·f + g`，故 `∂v/∂q = −[f]×`（比力反对称阵） | 有比力/机动时 P_qv ≠ 0，坏的速度观测会经增益摸到姿态 |
| q ↔ ba | 加速度计零偏与重力观测共用同一组量测 | 长期 GPS 位置/速度偏差可慢性污染 ba，再以"假倾斜"形式进 q |
| b_baro ↔ 高度 | 气压零偏与 GPS 高度对同一状态竞争 | 两个慢源互相拉扯，收敛变慢、易被坏 GPS 拖走 |
| 延时 | GPS 5~10 Hz、链路延时 100 ms 量级（PX4 默认 `EKF2_GPS_DELAY=110` ms） | 不补偿就按"运动×延时"产生系统性伪残差，高速/转弯最明显 |
| 杆臂 | 天线不在 IMU 原点 | 转弯时 ω×r 直接变成伪位置/速度误差（1 m 臂、100 dps ≈ 1.75 m/s） |

反过来，隔离的代价也很明确：**丢掉 q 与 p/v 的相关性 → 次优**；严格做法要用联邦滤波/协方差交叉，
否则两级之间的信息会被重复使用。

---

## 2. 现状（我们已经是"方案 A 的一半"）

`V5F/User/src/proc_ekf.c`：
- 16 状态：`p(3) v(3) q(4) ba(3) bg(3) b_baro(+pzz/pbb)`；
- `ekf_m1m2_gps()`（≈行 880）：位置（RMC lat/lon → 本地 ENU，带 origin latch）、高度（GGA）、
  速度（RMC speed+course → v_north/v_east），各自独立 R 与门位；
- R 已按 C/N0 自适应放大（`V5F_EKF_C_N0_REF_DBHZ` / `snr_avg_dbhz`）；
- `gate.ekf_gps_pos/alt/vel`（≈行 1606-1616）综合 fix_quality、`SV_MIN`、`HDOP_MAX`、`VDOP_MAX`、
  `SNR_MIN_DBHZ`、RMC status、freshness flags；
- 通过位记在 `gate_bits`（`V5F_EKF_GB_GPS_POS/ALT/VEL`），异常进 `nis[]`；
- 静止时用 GPS 速度做 `s_b_dop`（ZUPT 辅助）。

**缺的护栏**：延时补偿、杆臂、GPS 航向的显式开关与判据、glitch 半径、拒绝计数（现在只有"通过位"）。

---

## 3. 方案 A：单滤波器 + 强门控（参考实现与默认值）

| 项 | PX4 EKF2 | ArduPilot EKF3 |
|---|---|---|
| 用哪些量 | `EKF2_GPS_CTRL=7` = lat/lon + 高度 + 3D 速度（**双天线航向是 bit3，默认关**） | 位置/速度分开，各自门限 |
| 门限 | `EKF2_GPS_P_GATE=5.0` SD、`EKF2_GPS_V_GATE=5.0` SD（新息/σ 门） | `EK3_POS_I_GATE=500`、`EK3_VEL_I_GATE=500`（百分数 = 5σ） |
| 延时 | `EKF2_GPS_DELAY=110` ms（显式建模） | 内部按测量时间戳回放 |
| 突变 | 健康位（`EKF2_GPS_CHECK=245` 位掩码） | `EK3_GLITCH_RAD=25` m：**glitch 半径触发则拒绝/挂起**；设为 0 时改为**剪切新息**而不是拒绝 |
| 冗余 | 单滤波器（+传感器冗余） | **多车道**（最多 3 条独立 EKF 并行，`EK3_PRIMARY` 选主，按健康分切换） |

适用：模块要直接对外提供 p/v（自成导航源），或必须靠 GPS 反过来约束惯导。
参考：PX4 `ekf2_params.c`、ArduPilot `AP_NavEKF3.cpp`（参数默认值均可在源码里核到）。

## 4. 方案 B：姿态与导航解耦（推荐给"通用 IMU 模块"）

参考实现 **ArduPilot DCM**（`AP_AHRS_DCM.cpp`）——姿态互补滤波**只吃 IMU**，GPS 的介入被显式限制：

```c
#define GPS_SPEED_MIN 3              // 地速 <3 m/s 不用 GPS 航向
// 地磁与 GPS 航向差 >45° 且 地速 > 3 m/s，连续 2 s 才用 GPS 航向替代地磁
const float error = fabsf(wrap_180(degrees(yaw) - AP::gps().ground_course()));
if (error > 45 && _wind.xy().length() < AP::gps().ground_speed()*0.8f) { ... }
// _gps_use == GPSUse::Disable 时，该估计器完全不用 GPS
results.configured_to_use_gps = _gps_use != GPSUse::Disable;
```
- 姿态：互补滤波（重力 + 磁），**GPS 永远不参与 roll/pitch**；GPS 只作为**航向的备用源**，
  且要满足速度阈值 + 与地磁连续不一致 2 s 两个条件；
- 平移/风：另一套速度（GPS 速度 → 风三角）+ `AP_InertialNav`，与姿态分离；
- 对外契约天然是两个话题：attitude / nav。

同类思想在 ROS 生态是标准做法：`robot_localization` 建议**两个实例**——
连续源（odom+IMU）一个滤波器，跳变源（GPS）另一个，必要时再融合
（见 https://deepwiki.com/cra-ros-pkg/robot_localization/6.1-basic-state-estimation ）。

**映射到我们的固件**（改动最小化）：
1. 把 `p(3) v(3) b_baro` 从姿态 EKF 里拆出去 → 新建 `V5F/User/src/proc_nav.c`，
   以 **50~100 Hz** 跑（GPS 本来就 5~10 Hz，气压 ~194 Hz）；
2. 新滤波器把 `h->ekf.q` 当**已知输入**（cascade），GPS/气压只进 p/v/alt，**不进 q/ba/bg**；
3. 姿态 EKF 保留 q/bg/ba + 现有地磁/重力门控机制，**编译期开关 `V5F_NAV_EN`（默认 0）**：
   关掉时行为与现在完全一致（可回退，符合"不要瞎改扩大问题"）；
4. `gate.ekf_gps_*`、`gate_bits`、`nis[]`、C/N0→R 这套现成机制**整体搬到 nav 滤波器**，字段名不变，
   这样录像分析脚本不用改；
5. 对外：模块只输出姿态（+ 可选原始 GPS/气压），**位置/速度由飞控自己的导航滤波器负责**，
   避免两个滤波器吃同一份 GPS 造成的重复计数。

代价：q 与 p/v 不再相关（次优）。若将来真要做"模块自带导航"，再上方案 C：
联邦滤波（信息分配系数 / 协方差交叉）或 EKF3 式多车道，姿态/导航各一条车道，按健康分切换。

---

## 5. 无论选哪个方案，都必须做的护栏

1. **延时**：给 RMC/GGA 打接收时间戳（shm `hdr.ts` 已有），按 100~150 ms 量级补偿，并记录残差。
2. **杆臂**：常量 `GPS_LEVER_B[3]`，转弯时补偿 `ω×r`（必要时加 `ω×(ω×r)` 的向心项）。
3. **R**：继续用 C/N0，再叠加 HDOP/VDOP 与速度精度；坏天气时速度观测比位置更可信（PX4 位置噪声
   0.5 m vs 速度 0.3 m/s，速度先降权再丢位置）。
4. **门**：σ 门（5σ 量级）+ **glitch 半径**（位置突变 >25 m 挂起）+ **拒绝计数**（不只是通过位）；
   并明确"超门是剪切新息还是拒绝"。
5. **航向**：默认**不用 GPS 航向**（PX4 默认关）；只在"地磁判坏 + 速度 > 3 m/s + 连续 2 s 一致"时
   作为替代源，且必须记 `mag_cmp_thm/thp` 那类一致性量。
6. **观测模型纪律**：GPS 只准进 p/v(/alt)；**禁止**单独驱动 bg、重力/倾斜观测、地磁相关状态。
7. **全记录**：每个 GPS 更新的 NIS、R、门位、延时、杆臂补偿量、拒绝原因 —— 延续"外门控 + 全记录"。

---

## 6. 参考链接

- PX4 EKF2 GPS 参数（`GPS_CTRL`/`P_GATE`/`V_GATE`/`DELAY`/`GPS_CHECK` 默认值） —— https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.14.0/src/modules/ekf2/ekf2_params.c
- ArduPilot EKF3 参数（`POS_I_GATE`/`VEL_I_GATE`=5σ、`GLITCH_RAD`=25 m、车道选择） —— https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_NavEKF3/AP_NavEKF3.cpp
- ArduPilot EKF3 位置/速度融合实现 —— https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp
- ArduPilot DCM（姿态只吃 IMU；GPS 航向的速度阈值/一致性判据；`GPSUse::Disable`） —— https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_AHRS/AP_AHRS_DCM.cpp
- ArduPilot EKF 文档索引（含 "EKF3 Affinity and Lane Switching"） —— https://ardupilot.org/dev/docs/ekf.html
- PX4 热补偿/传感器上游补偿（架构上"驱动→估计器"的责任划分） —— https://docs.px4.io/v1.12/en/advanced_config/sensor_thermal_calibration
- ROS `robot_localization` 双实例（连续源 / 跳变源分离） —— https://deepwiki.com/cra-ros-pkg/robot_localization/6.1-basic-state-estimation
