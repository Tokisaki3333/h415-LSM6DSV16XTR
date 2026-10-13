/* ============================================================================
 * justfloat 上报（FireWater / VOFA+ 二进制格式）
 *
 *   通用帧头（小端，全部 float32，无整数、无包头长度、无校验）:
 *     float32 = 0x7F800000   (+Inf 位型)   同步头 1
 *     float32 = 0x7F800000                 同步头 2（双写，抗误同步）
 *     float32  chan0 = 类型   1.0 = ATT 帧   2.0 = RAW 帧
 *     float32  chan1 = seq    递增序号
 *
 *   ---- 类型 1：ATT 帧（16 float = 64 B）★ 本版＝唯一上报流，IMU 满速率 ----
 *     chan2..5   qw, qx, qy, qz     四元数（体->导航 ENU）
 *     chan6      bias_ok            1=陀螺零偏已收敛
 *     chan7      grav_err_deg       重力环本帧新息（度）
 *     chan8      grav_applied       1=本帧重力环施加了修正
 *     chan9      mag_corr_en        1=地磁修正开
 *     chan10     imu_cnt_lo         陀螺帧计数（低 24 位）
 *     chan11     mmc_cnt_lo         磁拍计数（低 24 位）
 *     chan12     mmc_pol            0=SET 1=RESET
 *     chan13     imu_temp_raw       温度原始值
 *     chan14     mag_err_deg        地磁环本帧新息（度）
 *     chan15     eskf_ready         1=启动标定完成
 *     ★ 帧长恒为 64 B，绝不随内容变化；绝不混入任何非 justfloat 字节。
 *
 *   ---- 类型 2：RAW 帧（19 float = 76 B）—— 本版不再发送，保留代码备用 ----
 *
 *   ---- ★ 聚合上报（字节流内容不变）----
 *     Win10 侧驱动对【发起 USB 传输的频率】敏感。本层把"取数"与"发起上报"解耦：
 *       快路径 jf_att_push() / jf_raw_push()  每轮循环调用，只往暂存区拷字节
 *       慢路径 jf_agg_poll()                  按水位整块交给 hid_up_enqueue
 *     ★ 暂存区按【定长帧】记账：帧长由 s_frame_bytes 决定，
 *       同一时刻只使用一种帧（ATT 或 RAW），绝不混发。
 * ========================================================================== */
#ifndef JUSTFLOAT_H
#define JUSTFLOAT_H

#include <stdint.h>

#define JF_SYNC            0x7F800000u     /* +Inf 的 IEEE754 位型 */

#define JF_TYPE_ATT        1.0f
#define JF_TYPE_DBG        2.0f
#define JF_TYPE_RAW        2.0f     /* 旧别名（RAW 路径已不用，仅为编译兼容保留） */

/* ---- ★ 本版唯一上报流：QUAT 帧 ----
 *   float[0]  qw
 *   float[1]  qx
 *   float[2]  qy
 *   float[3]  qz
 *   float[4]  0x7F800000   ← 帧尾标识 1
 *   float[5]  0x7F800000   ← 帧尾标识 2
 *   ⟹ 共 6 float = 24 B
 *   ★ 【没有 type / seq 帧头】—— 帧尾的两次 00 00 80 7F 才是唯一标识
 *   ★ 四元数负载恒为 4 个 float，帧长恒定 24 B，只含四元数，
 *     绝不混入任何其它数据 */
#define JF_QUAT_NFLOATS     4u                      /* 四元数负载 float 数 */
#define JF_QUAT_BYTES       (JF_QUAT_NFLOATS * 4u)  /* 16 B 负载 */
#define JF_QUAT_WIRE_BYTES  24u                     /* 16 B + 8 B 帧尾标识 */

/* ---- ★ 本版上报流：FULL 帧（全数据调试）----
 *   float[0,1]   sync ×2              帧首
 *   float[2]     type = 2.0
 *   float[3]     seq
 *   float[4..7]  qw,qx,qy,qz          四元数（体->导航）
 *   float[8..10] gyro_dps[3]          ESKF 实际用的角速度（已扣零偏）
 *   float[11..13] accel_g[3]          ESKF 实际用的加计
 *   float[14..16] mag_u[3]            ESKF 实际用的净场单位矢量
 *   float[17]    flag                 bit0 mag_valid, bit1 new_mag,
 *                                     bit2 grav_open, bit3 ready
 *   float[18]    t_us                 本帧时刻 (10ns/10)
 *   float[19]    imu_cnt              连续性校验位
 *   float[20]    mmc_cnt
 *   float[21]    mag_resid_deg
 *   float[22]    acc_resid_deg
 *   float[23..25] mag_raw_cur[3]     本拍原始 counts（18 位）
 *   float[26..28] mag_raw_prev[3]    上一拍原始 counts
 *   float[29]    flag2  bit0 pol_cur, bit1 pol_prev,
 *                       bit2 mmc_prev_valid, bit3 ready,
 *                       ★ bit4 layout_v2 (=1 表示含 [30..32] 零偏字段)
 *   float[30..32] gyro_bias_dps[3]    ★ 追踪器当前零偏估计（dps）
 *   float[33]    acc_thr              ★ 自适应重力残差门限（纯历史量）
 *   float[34..36] Pxx,Pyy,Pzz         协方差对角
 *   float[37..39] Pxy,Pxz,Pyz         协方差非对角（重力→偏航泄漏通道）
 *   float[40]    calib_n              启动标定累计样本数
 *   float[41]    ts_lo24              ★ v4：GetTime64_10Ns() 的低 24 位
 *   共 42 float = 168 B，帧长恒定。
 *
 *   ★ v4 时戳精度（2026-10-13）：
 *     原 [18] 写 (float)((double)ts*1e-1)。float32 在 3.6e8 量级的步长为 32，
 *     即【上报粒度只有 3.2 us】；而帧间隔 ≈130 us ⟹ 报出来的 dt 有 2.5% 量化误差。
 *     实测逐帧 Δt 散布 ±30%（中位 1304、1%~99% 为 1040~1568、相邻唯一值差 768），
 *     完全无法用来算 dt —— 这是纯上报缺陷，固件内部用原始 u64 不受影响。
 *
 *   ★★ 编码方式（必须有，勿改）：【数值拆分，且必须拆在 2^24，不能用按位拷贝】
 *       [18] = ts >> 24            (高位)
 *       [41] = ts & 0xFFFFFF       (低位, 24 位)
 *       还原: ts = (u64)F[18] * 16777216ULL + (u64)F[41]
 *
 *     为什么【不能】按位拷贝到伪 float：
 *       justfloat 同步字 00 00 80 7F (小端) = float 0x7F800000 = +Infinity。
 *       按位拷贝会让该字段遍历每一个 u32 值 ⟹ 0x7F800000 必然被命中
 *       （每 2^32 × 10ns = 42.9 s 一次），上位机把数据误判为帧头 ⟹ 错位/卡死。
 *       这是结构性必然，不是概率问题。
 *     为什么拆在 2^24 就安全：
 *       float32 尾数 24 位 ⟹ [0, 2^24) 与 2^24 的倍数都【精确可表示】⟹ 无损；
 *       两者都是有限整数 ⟹ 位型永远到不了 0x7F800000(+Inf)，也到不了 NaN。
 *       精确上限 ts < 2^24 × 2^24 = 2^48 个 10ns = 32 天，远超任何录制。
 *     ★ 仍必须【按数值读】(不能 view 成 u32)：F[18]*16777216 + F[41]。
 *   ★ 水位不变：3 × 168 = 504 B ≤ HS 单包 512 B。
 *   ★ 布局演进：v1=30f/120B；v2=33f/132B（加零偏）；v3=41f/164B（加 ESKF 持久状态）；
 *     v4=42f/168B（时戳保精度）。仍遵循【尾部追加】，float[0..17] 与 [19..40] 的偏移不变。
 *   ★ 全部原始输入都在帧内，PC 可逐帧重放。
 *   ★ 布局演进：v1 = 30f/120B；v2 = 33f/132B（加零偏）；v3 = 41f/164B（加持久状态）。
 *     一律【尾部追加】，float[0..29] 偏移与含义自始至终不变，
 *     故解析端可用帧长（120/132/164）直接区分版本，老文件仍可读。
 *   ★ 为什么必须报持久状态：acc_thr 与 P 编码了自开机以来的全部历史，
 *     PC 侧拿一段截取数据做复刻时【无法反推】，实测导致 grav_open
 *     8.2% vs 板载 53.6%、姿态发散 147°。有了本段即可把复刻初始化到同一起点。
 *   ★ 为何不用独立"快照帧"：justfloat 按【定长帧】记账（s_frame_bytes），
 *     同一时刻只使用一种帧，绝不混发——可变帧长会让上位机失效。
 *     164B × 7673.5Hz = 1.26 MB/s，带宽足够，故逐帧上报。
 *   ★ 水位不变：3 × 164 = 492 B ≤ HS 单包 512 B。 */
#define JF_FULL_NFLOATS     42u
#define JF_FULL_BYTES       (JF_FULL_NFLOATS * 4u)  /* 168 B */
#define JF_FULL_WIRE_BYTES  168u

/* 旧 DBG 帧定义（jf_dbg_push 保留不用，仅为编译兼容） */
#define JF_DBG_NFLOATS     19u
#define JF_DBG_BYTES       (JF_DBG_NFLOATS * 4u)
#define JF_DBG_WIRE_BYTES  84u

#define JF_ATT_NFLOATS     16u
#define JF_RAW_NFLOATS     19u
#define JF_ATT_BYTES       (JF_ATT_NFLOATS * 4u)   /* 64 B */
#define JF_RAW_BYTES       (JF_RAW_NFLOATS * 4u)   /* 76 B */

/* 实测 USB 帧长（聚合层用）。RAW 源码头是 76 B，板上实际 84 B。 */
#define JF_RAW_WIRE_BYTES  84u
#define JF_ATT_WIRE_BYTES  JF_ATT_BYTES             /* ATT 恒为 64 B */

/* ---------------- 聚合参数 ---------------- */
/* 慢路径节拍：>=8 ms（即 <=125 Hz）。PC 侧驱动对发起频率敏感，不得高于此。
 * ★ 注意：本值只决定"检查并入队的节拍下限"，真正决定块长的是水位。 */
#define JF_AGG_PERIOD_US   8000u

/* 暂存区：必须【远小于】上行环容量（DEF_UART_BUF_SIZE = 8192 B）。
 * 容量取 24 帧，水位取 6 帧 —— 与 RAW 模式同一套经验值。 */
#define JF_AGG_STAGE_FRAMES  24u
/* ★ 暂存区必须按【最大帧长】计算:
 *   当前最大帧长 = 92 B (FULL 帧) 。
 *   若按旧 84 B 计算 (2016 B), 而帧长是 92 B,
 *   s_stage_n = 23 时会写到 st_stage[2208] —— 越界 192 B。
 *   24 × 92 = 2208 B 。 */
#define JF_AGG_STAGE_BYTES   (JF_AGG_STAGE_FRAMES * JF_FULL_WIRE_BYTES)  /* 2880 B */
/* ★ 水位 = 3 帧：块长 3×132 = 396 B < HS 单包 512 B，
 *   每块恰好一个 bulk 包、无拆包开销。
 *   （帧长 92→120 时从 5 降到 4；120→132 时必须再从 4 降到 3，
 *     否则 4×132 = 528 B > 512 B，会被拆成两个包） */
#define JF_AGG_FORCE_FRAMES  3u

/* 兼容旧名 */
#define JF_AGG_FRAMES      JF_AGG_FORCE_FRAMES
#define JF_AGG_BYTES       (JF_AGG_FRAMES * JF_FULL_WIRE_BYTES)

/* ★ FULL 帧（全数据调试）：一帧含 ESKF 全部输入与输出 */
uint8_t jf_full_push(float qw, float qx, float qy, float qz,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz,
                     float flag, float t_us, float imu_cnt, float mmc_cnt,
                     float mag_resid, float acc_resid,
                     float mcur_x, float mcur_y, float mcur_z,
                     float mprev_x, float mprev_y, float mprev_z,
                     float flag2,
                     float bias_x, float bias_y, float bias_z,
                     float acc_thr,
                     float pxx, float pyy, float pzz,
                     float pxy, float pxz, float pyz,
                     float calib_n,
                     float ts_lo24);

/* ★ QUAT 帧（仅四元数 24 B），保留 */

uint8_t jf_quat_push(float qw, float qx, float qy, float qz);

/* DBG 帧快路径（保留，默认不用） */
uint8_t jf_dbg_push(float qw, float qx, float qy, float qz,
                    int16_t gx, int16_t gy, int16_t gz,
                    int16_t ax, int16_t ay, int16_t az,
                    int32_t mx, int32_t my, int32_t mz,
                    float ts_lo, float ts_hi, float imu_cnt, float mmc_cnt);

/* ATT 帧快路径（保留） */
uint8_t jf_att_push(float qw, float qx, float qy, float qz,
                    float bias_ok, float grav_err_deg,
                    float grav_applied, float mag_corr_en,
                    float imu_cnt, float mmc_cnt, float mmc_pol,
                    float temp_raw, float mag_err_deg, float eskf_ready);

/* RAW 帧快路径（保留，默认不用） */
uint8_t jf_raw_push(int16_t gx, int16_t gy, int16_t gz,
                    int16_t ax, int16_t ay, int16_t az,
                    int32_t mx, int32_t my, int32_t mz,
                    float ts_lo, float ts_hi, float imu_cnt,
                    float mmc_cnt, float mmc_pol, float temp_raw);

/* 立即入队版（少量帧时用；内部走同一定长账本） */
uint8_t jf_send_att(float qw, float qx, float qy, float qz,
                    float bias_ok, float grav_err_deg,
                    float grav_applied, float mag_corr_en,
                    float imu_cnt, float mmc_cnt, float mmc_pol,
                    float temp_raw);

/* 慢路径：主循环每轮调用。到点即把暂存区整块入队。 */
void jf_agg_poll(void);

uint32_t jf_agg_pending(void);

extern volatile uint32_t g_jf_drop;      /* 上行环满/强制送失败 丢帧数 */
extern volatile uint32_t g_jf_agg_tx;    /* 实际发起的 USB 传输次数 */
extern volatile uint32_t g_jf_enq_fail;  /* 入队失败次数（环满）*/

#endif /* JUSTFLOAT_H */
