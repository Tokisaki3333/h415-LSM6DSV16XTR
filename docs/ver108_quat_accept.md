# VER=108：CDC 上报切回**验收模式**（EKF 四元数 JustFloat 20 B）

## 改了什么（只动 `V5F/User/inc/v5f_tune.h` 两行）

| 宏 | VER=107 | VER=108 |
|---|---|---|
| `V5F_FW_VER` | `107u` | **`108u`** |
| `V5F_CDC_QUAT_ONLY` | `0u`（调试 162 通道 654 B 帧） | **`1u`（验收）** |
| `V5F_CDC_DEBUG_DIV` | `24u` | `24u`（不变；验收模式下不生效，切回调试时才用） |

`V5F/User/src/SPI_rx.c` **一字未动**：验收分支本来就在 `justfloat_report()` 开头
（`#if (V5F_CDC_QUAT_ONLY != 0u)`），且发的就是 **`g_v5f_hold.ekf.q[0..3]`**（EKF 结果，
不是旧链 `att.q`），组帧后 `return`，因此不受调试抽帧影响。本版只是把它启用。

## 流格式（CDC / EP2 bulk IN）

```
4 x float32 小端 : w, x, y, z   (= g_v5f_hold.ekf.q[0..3])
+ 帧尾 00 00 80 7F
= 20 B / 帧
```
- **无帧头、无 `fw_tag`、无校验和**（JustFloat 无标签位）⇒ `rec.py` / `cols_162`（找 `A5 5A`）
  在验收模式下抓不到帧是正常的；`fw_tag` 只在调试帧第 76 列里，想核对版本要切回 `0u` 或看 OLED。
- 速率 = IMU 中断率 ≈ **8.08 kHz ⇒ 20 B × 8080 ≈ 161 kB/s**（200 Hz 的整倍数关系不用管，
  接收方按帧尾同步即可）。
- 输出是**阶梯状**：q 只在 EKF 更新时变化，相邻帧可能完全相同。

## 自检（不编译也能查）

```
python tools/ekf_session/patch_v108_quat_accept.py      # 幂等；重跑只做校验
```
脚本会打印并断言：VER/QUAT_ONLY/DIV 三个宏**各只定义一次**（唯一性 lint）、
验收分支存在且发 EKF 四元数、帧尾 `00 00 80 7F`、按 20 B 入 CDC、分支内 `return`、
调试抽帧在验收分支之后、调试 162 通道组帧代码保留完好，最后给出 `fw_tag` 变化
（调试帧第 76 列 `7053831 → 7119367`）。

## 回退

```powershell
Copy-Item bak_src\V5F\User\inc\v5f_tune.h.bak_v108 V5F\User\inc\v5f_tune.h -Force   # 回到 VER=107 调试模式
```
或只把 `V5F_CDC_QUAT_ONLY` 改回 `0u`（同一版本内来回切换，不影响其它任何东西）。

## 记录

- 打补丁脚本：`tools/ekf_session/patch_v108_quat_accept.py`（备份 `bak_src/V5F/User/inc/v5f_tune.h.bak_v108` = 打补丁前原态；**不写在源码目录**）
- 历史沿革：VER=100 验收模式(1u) → VER=101 切调试(0u) + `DEBUG_DIV=24u` → **VER=108 切回验收(1u)**
