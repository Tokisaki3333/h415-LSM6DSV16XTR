# GPS（ATGM336H）用法

> 记录时间：本次会话。当前状态：**链路已验证可用，但暂停上报**（轮询与解析保持运行）。

## 1. 硬件连接

| 信号 | MCU 引脚 | 网表 | 说明 |
|---|---|---|---|
| `USART2_TX` | **PD5** (AF7) | `R28.2 → U5.3` | MCU → 模组 RXD |
| `USART2_RX` | **PD6** (AF7) | `R27.2 → U5.2` | 模组 TXD → MCU |
| AUX1 | PD4 | | 辅助脚，`BOARD_GPIO_Init` 配为 IPU |
| AUX2 | PF4 | | 同上 |

宏定义在 `Common/Common/board_io.h`：`GNSS_USART`(=USART2) / `GNSS_TX_PORT` / `GNSS_TX_PIN` / `GNSS_TX_SRC` / `GNSS_RX_PORT` / `GNSS_RX_PIN` / `GNSS_RX_SRC` / `GNSS_USART_AF`(=AF7) / `GNSS_BAUDRATE`。

## 2. DMA 配置

```
控制器/通道 : DMA2_Channel2          （环形模式，缓冲 4096 字节）
DMAMUX      : DMA_MuxChannel10 → 请求号 88 (USART2_RX)
外设地址    : &USART2->DATAR
缓冲        : RxBuffer2[4096]        （GPS.c 内定义）
```

**请求号出处**：参考手册 DMA 请求表 —— `USART2_TX=87`、`USART2_RX=88`、`USART3_TX=89`、`USART3_RX=90`。

**DMAMUX 通道与 DMA 通道的对应**（由 `DMA_MuxChannelConfig()` 实现推出）：
`Mux1..7 → DMA1_Ch1..7`（SPI 那组已实证：Mux2↔Ch2、Mux3↔Ch3、Mux4↔Ch4、Mux5↔Ch5），
`Mux9..16 → DMA2_Ch1..8`（手册 P152："DMAMUX 通道 9 到 16 对应 DMA2 通道 1 到 8"）。

**无冲突**：地磁/陀螺用 DMA1 的 Ch2/Ch3/Ch4/Ch5，GPS 用 DMA2_Ch2，控制器不同。

## 3. 初始化序列（`GPS_USART_Init()`）

顺序有依赖，不要调换：

```
① RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA2, ENABLE)
② DMA_Init(DMA2_Channel2, ...)                环形、字节、外设→内存
③ DMA_MuxChannelConfig(DMA_MuxChannel10, 88)  绑 USART2_RX 请求
④ RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART2) + RCC_HB2PeriphClockCmd(GPIOD)
⑤ GPIO_PinAFConfig(PD5/PD6, AF7) + GPIO_Init
⑥ USART_Init(USART2, 115200 8N1) + USART_Cmd(ENABLE)
⑦ DMA_Cmd(DMA2_Channel2, ENABLE)
⑧ USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE)   ★ 必需
⑨ 发 PCAS 配置指令（定位频率、启用哪些语句）
```

### ★ 最容易漏的一步：`USART_DMACmd`

**只使能 DMA 通道不够** —— 外设不发 DMA 请求时，通道永远等不到数据，表现为
`GPS_Check()` 恒返回 0、`RxBuffer2` 内容不变。

`board_io.c` 的 `BOARD_USART2_GNSS_Init()` **只做 `USART_Init`+`USART_Cmd`，不使能 DMA**，
所以这一步必须在 `GPS_USART_Init()` 里自己做。

### PCAS 配置指令

```c
GPS_Send_Cmd("$PCAS02,100*1E\r\n");                              /* 定位频率 10Hz */
GPS_Send_Cmd("$PCAS03,1,0,1,1,1,0,0,0,0,0,,,1,1,,,,0*32\r\n");   /* RMC/GGA/GSA 开、GSV 关 */
```

模组上电后需 100~500ms 才接收指令，**若指令丢失会退回默认输出**。
`gps_cfg_retry()` 按 `s_cfg_tick[] = {0, 0.5, 1, 2, 5}s` 重发，最多 5 次，由 `GPS_Check()` 内部调用。

## 4. 运行期用法

```c
GPS_USART_Init();                       /* 初始化一次 */

for (;;)
{
    if (50ms 时间到)                     /* 20Hz 轮询 */
    {
        (void)GPS_Check();              /* 搬运 + 解析，返回本次处理字节数 */
    }
}
```

`GPS_Check()` 做两件事：
- 调 `gps_cfg_retry()` 处理指令重发
- 比对 DMA 写指针（`GPS_RX_BUFFER_SIZE - DMA2_Channel2->CNTR`）与读指针，
  把新增数据交给 `parse_gps_data()`，**处理环形缓冲回绕**（分两段调用）

**轮询频率建议 ≥ 语句到达率的 2 倍**。当前 10Hz 语句、20Hz 轮询，余量 5 倍。
一条标准 NMEA 语句 77 字节，115200 下约 6.7ms 传完；实测解析约 **0.45µs/字节**
（143 字节 ≈ 65µs、243 ≈ 118µs、473 ≈ 219µs）。

## 5. 解析结果（共享区）

解析结果发布到 `mipc_shm.h` 的 GPS 通道，**V3F 写、V5F 读**：

| 通道 | 字段 | 单位/说明 |
|---|---|---|
| `gps_rmc` | `status` | NMEA 原字符：`'A'`=有效 / `'V'`=无效 |
| | `lat_e7` / `lon_e7` | **×1e7 度**（南纬/西经为负）→ 除以 1e7 得度 |
| | `speed_mps` | 对地速度 m/s |
| | `course_deg` / `magvar_deg` | 航向 / 磁偏角 |
| | `date_ddmmyy` | UTC 日期 |
| `gps_gga` | `quality` | 0=无效 1=单点 2=差分 4=RTK固定 5=RTK浮点 |
| | `sv` | 卫星数 |
| | `hdop_x100` | HDOP ×100 |
| | `alt_cm` | 海拔 cm |
| `gps_gsa` | `pdop_x100` / `vdop_x100` | DOP ×100 |
| `gps_gsv` | `sats_view` / `snr_n` / `snr_min` / `snr_avg_x10` | 视野内卫星数、有效 C/N0 数量与统计 |

每帧发布都走 `shm_chan_ts_write()` + `hdr.cnt++`（时间戳用 `GetTime64_10Ns()`）。

## 6. 上报格式（暂停中）

曾有的一行输出（`GPS_DBG` 已删除，需要时按下表自行加回）：

```c
printf("GPS,st=%c,lat=%d,lon=%d,spd=%.2f,sv=%u,q=%u,hdop=%u,alt=%d,cnt=%u\r\n",
       (char)g_shm->gps_rmc.status,
       (int)g_shm->gps_rmc.lat_e7, (int)g_shm->gps_rmc.lon_e7,
       (double)g_shm->gps_rmc.speed_mps,
       (unsigned)g_shm->gps_gga.sv, (unsigned)g_shm->gps_gga.quality,
       (unsigned)g_shm->gps_gga.hdop_x100,
       (int)g_shm->gps_gga.alt_cm,
       (unsigned)g_shm->gps_rmc.hdr.cnt);
```

实测样例（已验证解析正确）：

```
GPS,st=A,lat=362296390,lon=1204327698,spd=0.21,sv=09,q=01,hdop=300,alt=4759,cnt=878
→ 36.2296390°N, 120.4327698°E, 0.21m/s, 9 星, 单点定位, HDOP 3.00, 海拔 47.59m
```

## 7. 文件与接口

| 文件 | 说明 |
|---|---|
| `Common/Common/GPS.c` | 全部实现：DMA/串口初始化、NMEA 增量解析状态机、共享区发布 |
| `Common/Common/GPS.h` | `GPS_USART_Init()` / `GPS_Check()` / `GPS_Send_Cmd()` / `parse_gps_data()` |

**`GPS.c` 依赖 `board_io.h`**（取 `GNSS_*` 宏）与 `mipc_shm.h`（发布共享区）。

解析状态机支持 **DMA 回绕跨段续读**（段边界由长度精确控制，不依赖 `'\0'` 哨兵
—— 哨兵会被 DMA 覆盖导致误读回绕区旧数据）。
