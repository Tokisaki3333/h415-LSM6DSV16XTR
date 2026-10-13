/* justfloat 上报实现。帧结构见 justfloat.h。
 * USB CDC 上是唯一内容：不经过任何 TYPE/SEQ/CRC 打包。
 * 原始整数一律先转 float32 再写，绝不混入 u8/u16 原始字节。
 *
 * ★ 本版：ATT 帧（16 float = 64 B，常量帧长）以 IMU 满速率上报。
 * ★ 快慢路径分离：
 *     jf_att_push()  每 IMU 帧调用，只往暂存区拷 64 B，不碰 USB
 *     jf_agg_poll()  每轮循环调用，按水位整块入队（>=8 ms 才允许发起）
 *   暂存区里是连续的定长 ATT 帧，字节流与逐帧上报完全一致。
 * ★ 绝不混发：s_frame_bytes 记录当前帧长，同一时刻只用一种帧。 */
#include "justfloat.h"
#include "usbd_compatibility_hid.h"
#include "sys_clk.h"

volatile uint32_t g_jf_drop;        /* 上行环满 / 暂存区强制送失败 的丢帧数 */
volatile uint32_t g_jf_agg_tx;      /* 实际发起的 USB 传输次数 */
volatile uint32_t g_jf_enq_fail;    /* hid_up_enqueue 失败次数（环满）*/

static float s_seq;

/* ---- 暂存区（按定长帧记账）---- */
static uint8_t  s_stage[JF_AGG_STAGE_BYTES];
static uint32_t s_stage_n;          /* 已暂存【帧】数 */
static uint32_t s_frame_bytes = JF_ATT_WIRE_BYTES;   /* 当前帧长 */
static uint64_t s_agg_next_us;      /* 下一次允许发起 USB 的时刻 */

/* 写一个 float32（小端） */
static uint32_t put_f(uint8_t *buf, uint32_t o, float v)
{
    union { float f; uint32_t u; } cv;
    cv.f = v;
    buf[o++] = (uint8_t)(cv.u);       buf[o++] = (uint8_t)(cv.u >> 8);
    buf[o++] = (uint8_t)(cv.u >> 16); buf[o++] = (uint8_t)(cv.u >> 24);
    return o;
}

/* 帧头：两个同步头 + 类型 + 序号 */
static uint32_t put_head(uint8_t *buf, float type)
{
    union { float f; uint32_t u; } cv;
    uint32_t o = 0u, i;
    cv.u = JF_SYNC;
    for (i = 0u; i < 2u; i++)
    {
        buf[o++] = (uint8_t)(cv.u);       buf[o++] = (uint8_t)(cv.u >> 8);
        buf[o++] = (uint8_t)(cv.u >> 16); buf[o++] = (uint8_t)(cv.u >> 24);
    }
    o = put_f(buf, o, type);
    s_seq += 1.0f;
    o = put_f(buf, o, s_seq);
    return o;
}

/* 把暂存区整块交给 USB 环。返回 1 = 已接纳（暂存区清空）。 */
static uint8_t agg_flush(void)
{
    uint32_t bytes;

    if (s_stage_n == 0u) return 1u;

    bytes = s_stage_n * s_frame_bytes;
    if (bytes > 0xFFFFu) bytes = 0xFFFFu;      /* hid_up_enqueue 的 len 是 u16 */

    if (!hid_up_enqueue(s_stage, (uint16_t)bytes))
    {
        g_jf_enq_fail++;
        g_jf_drop += s_stage_n;                /* 整块丢弃，不写半块 */
        s_stage_n = 0u;
        return 0u;
    }
    s_stage_n = 0u;
    return 1u;
}

/* 暂存一帧（定长 s_frame_bytes）。背压：到水位先送一次。 */
static uint8_t stage_frame(uint8_t *frame)
{
    if (s_stage_n >= JF_AGG_FORCE_FRAMES)
    {
        agg_flush();
        if (s_stage_n >= JF_AGG_STAGE_FRAMES)
        {
            g_jf_drop++;
            return 0u;
        }
    }
    {
        uint8_t *dst = s_stage + (s_stage_n * s_frame_bytes);
        uint32_t i;
        for (i = 0u; i < s_frame_bytes; i++) dst[i] = frame[i];
    }
    s_stage_n++;
    return 1u;
}

/* ---------------- FULL 帧（全数据调试，132 B）----------------
 *  帧内包含 ESKF 实际用到的全部输入与输出，PC 可逐帧重放对比。
 *  v2：尾部追加 [30..32] = gyro_bias_dps[3]，前 30 个 float 偏移不变。 */
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
                     float ts_lo24)
{
    uint8_t buf[JF_FULL_WIRE_BYTES];
    uint32_t o;

    s_frame_bytes = JF_FULL_WIRE_BYTES;

    o = put_head(buf, JF_TYPE_DBG);
    o = put_f(buf, o, qw);  o = put_f(buf, o, qx);
    o = put_f(buf, o, qy);  o = put_f(buf, o, qz);
    o = put_f(buf, o, gx);  o = put_f(buf, o, gy);  o = put_f(buf, o, gz);
    o = put_f(buf, o, ax);  o = put_f(buf, o, ay);  o = put_f(buf, o, az);
    o = put_f(buf, o, mx);  o = put_f(buf, o, my);  o = put_f(buf, o, mz);
    o = put_f(buf, o, flag);
    o = put_f(buf, o, t_us);
    o = put_f(buf, o, imu_cnt); o = put_f(buf, o, mmc_cnt);
    o = put_f(buf, o, mag_resid); o = put_f(buf, o, acc_resid);
    o = put_f(buf, o, mcur_x);  o = put_f(buf, o, mcur_y);  o = put_f(buf, o, mcur_z);
    o = put_f(buf, o, mprev_x); o = put_f(buf, o, mprev_y); o = put_f(buf, o, mprev_z);
    o = put_f(buf, o, flag2);
    /* v2：追踪器零偏（尾部追加，前 30 个 float 的偏移完全不变） */
    o = put_f(buf, o, bias_x); o = put_f(buf, o, bias_y); o = put_f(buf, o, bias_z);
    /* v3：ESKF 持久状态（PC 侧复刻初始化必需；纯历史量，无法从截取数据反推） */
    o = put_f(buf, o, acc_thr);
    o = put_f(buf, o, pxx); o = put_f(buf, o, pyy); o = put_f(buf, o, pzz);
    o = put_f(buf, o, pxy); o = put_f(buf, o, pxz); o = put_f(buf, o, pyz);
    o = put_f(buf, o, calib_n);
    /* v4：时戳低 24 位。[18] 为高 24 位。
     * 还原: ts = (u64)F[18]*16777216 + F[41]
     * ★ 必须数值拆分、不可按位拷贝 —— 理由见 justfloat.h（同步字 = +Inf 陷阱）。 */
    o = put_f(buf, o, ts_lo24);
    if (o != JF_FULL_WIRE_BYTES)
    {
        while (o < JF_FULL_WIRE_BYTES) buf[o++] = 0u;
    }
    return stage_frame(buf);
}

/* ---------------- QUAT 帧：4 float 四元数 + 帧尾两次 sync = 24 B ----------------
 *   float[0..3] = qw, qx, qy, qz
 *   float[4..5] = 0x7F800000, 0x7F800000   ← 帧尾标识（唯一标识，无帧头）
 * ★ 帧长恒定 24 B，只含四元数，绝不混入任何其它数据 */
uint8_t jf_quat_push(float qw, float qx, float qy, float qz)
{
    uint8_t buf[JF_QUAT_WIRE_BYTES];
    uint32_t o = 0u;
    union { float f; uint32_t u; } cv;

    s_frame_bytes = JF_QUAT_WIRE_BYTES;

    o = put_f(buf, o, qw);  o = put_f(buf, o, qx);
    o = put_f(buf, o, qy);  o = put_f(buf, o, qz);

    cv.u = JF_SYNC;
    /* 帧尾标识：连续两次 0x7F800000 */
    buf[o++] = (uint8_t)(cv.u);       buf[o++] = (uint8_t)(cv.u >> 8);
    buf[o++] = (uint8_t)(cv.u >> 16); buf[o++] = (uint8_t)(cv.u >> 24);
    buf[o++] = (uint8_t)(cv.u);       buf[o++] = (uint8_t)(cv.u >> 8);
    buf[o++] = (uint8_t)(cv.u >> 16); buf[o++] = (uint8_t)(cv.u >> 24);

    if (o != JF_QUAT_WIRE_BYTES)
    {
        while (o < JF_QUAT_WIRE_BYTES) buf[o++] = 0u;
    }
    return stage_frame(buf);
}

/* ---------------- DBG 帧：2 sync + 19 float = 84 B（保留，默认不用） ----------------
 * 逐 float 布局（含 sync 占位）:
 *   [0,1] sync   [2] type=2.0   [3] seq
 *   [4..7]   qw,qx,qy,qz     四元数（体->导航 ENU）
 *   [8..10]  gx,gy,gz        陀螺 int16 原始 counts
 *   [11..13] ax,ay,az        加计 int16 原始 counts
 *   [14..16] mx,my,mz        磁 int32 18 位原始 counts
 *   [17,18]  ts_lo, ts_hi    DRDY 时间戳（10 ns 计数）低/高 32 位
 *   [19]     imu_cnt         陀螺帧序号
 *   [20]     mmc_cnt         磁拍序号
 */
uint8_t jf_dbg_push(float qw, float qx, float qy, float qz,
                    int16_t gx, int16_t gy, int16_t gz,
                    int16_t ax, int16_t ay, int16_t az,
                    int32_t mx, int32_t my, int32_t mz,
                    float ts_lo, float ts_hi, float imu_cnt, float mmc_cnt)
{
    uint8_t buf[JF_DBG_WIRE_BYTES];
    uint32_t o;

    s_frame_bytes = JF_DBG_WIRE_BYTES;

    o = put_head(buf, JF_TYPE_DBG);
    o = put_f(buf, o, qw);   o = put_f(buf, o, qx);
    o = put_f(buf, o, qy);   o = put_f(buf, o, qz);
    o = put_f(buf, o, (float)gx); o = put_f(buf, o, (float)gy);
    o = put_f(buf, o, (float)gz);
    o = put_f(buf, o, (float)ax); o = put_f(buf, o, (float)ay);
    o = put_f(buf, o, (float)az);
    o = put_f(buf, o, (float)mx); o = put_f(buf, o, (float)my);
    o = put_f(buf, o, (float)mz);
    o = put_f(buf, o, ts_lo);   o = put_f(buf, o, ts_hi);
    o = put_f(buf, o, imu_cnt); o = put_f(buf, o, mmc_cnt);
    if (o != JF_DBG_WIRE_BYTES)
    {
        while (o < JF_DBG_WIRE_BYTES) buf[o++] = 0u;
    }
    return stage_frame(buf);
}

/* ---------------- ATT 帧：16 float = 64 B，恒定 ---------------- */
uint8_t jf_att_push(float qw, float qx, float qy, float qz,
                    float bias_ok, float grav_err_deg,
                    float grav_applied, float mag_corr_en,
                    float imu_cnt, float mmc_cnt, float mmc_pol,
                    float temp_raw, float mag_err_deg, float eskf_ready)
{
    uint8_t buf[JF_ATT_BYTES];
    uint32_t o;

    s_frame_bytes = JF_ATT_WIRE_BYTES;

    o = put_head(buf, JF_TYPE_ATT);
    o = put_f(buf, o, qw);           o = put_f(buf, o, qx);
    o = put_f(buf, o, qy);           o = put_f(buf, o, qz);
    o = put_f(buf, o, bias_ok);      o = put_f(buf, o, grav_err_deg);
    o = put_f(buf, o, grav_applied); o = put_f(buf, o, mag_corr_en);
    o = put_f(buf, o, imu_cnt);      o = put_f(buf, o, mmc_cnt);
    o = put_f(buf, o, mmc_pol);      o = put_f(buf, o, temp_raw);
    o = put_f(buf, o, mag_err_deg);  o = put_f(buf, o, eskf_ready);
    /* 帧长必须恰为 64 B（16 float × 4） */
    if (o != JF_ATT_BYTES)
    {
        while (o < JF_ATT_BYTES) buf[o++] = 0u;
    }
    return stage_frame(buf);
}

/* 兼容旧接口：立即入队版（少量帧时用；本版走同一暂存账本） */
uint8_t jf_send_att(float qw, float qx, float qy, float qz,
                    float bias_ok, float grav_err_deg,
                    float grav_applied, float mag_corr_en,
                    float imu_cnt, float mmc_cnt, float mmc_pol,
                    float temp_raw)
{
    return jf_att_push(qw, qx, qy, qz, bias_ok, grav_err_deg,
                       grav_applied, mag_corr_en, imu_cnt, mmc_cnt,
                       mmc_pol, temp_raw, 0.0f, 0.0f);
}

/* ---------------- RAW 帧：19 float -> 84 B（保留，默认不用） ---------------- */
uint8_t jf_raw_push(int16_t gx, int16_t gy, int16_t gz,
                    int16_t ax, int16_t ay, int16_t az,
                    int32_t mx, int32_t my, int32_t mz,
                    float ts_lo, float ts_hi, float imu_cnt,
                    float mmc_cnt, float mmc_pol, float temp_raw)
{
    uint8_t buf[JF_RAW_WIRE_BYTES];
    uint32_t o;

    s_frame_bytes = JF_RAW_WIRE_BYTES;

    o = put_head(buf, JF_TYPE_RAW);
    o = put_f(buf, o, (float)gx);  o = put_f(buf, o, (float)gy);
    o = put_f(buf, o, (float)gz);
    o = put_f(buf, o, (float)ax);  o = put_f(buf, o, (float)ay);
    o = put_f(buf, o, (float)az);
    o = put_f(buf, o, (float)mx);  o = put_f(buf, o, (float)my);
    o = put_f(buf, o, (float)mz);
    o = put_f(buf, o, ts_lo);      o = put_f(buf, o, ts_hi);
    o = put_f(buf, o, imu_cnt);    o = put_f(buf, o, mmc_cnt);
    o = put_f(buf, o, mmc_pol);    o = put_f(buf, o, temp_raw);
    o = put_f(buf, o, 0.0f);       o = put_f(buf, o, 0.0f);
    if (o != JF_RAW_WIRE_BYTES)
    {
        while (o < JF_RAW_WIRE_BYTES) buf[o++] = 0u;
    }
    return stage_frame(buf);
}

/* ---- 慢路径：限频到 <=125 Hz ---- */
void jf_agg_poll(void)
{
    uint64_t now = GetTime64_Us();

    if ((int64_t)(s_agg_next_us - now) > 0) return;

    s_agg_next_us = now + (uint64_t)JF_AGG_PERIOD_US;

    if (agg_flush()) g_jf_agg_tx++;
}

uint32_t jf_agg_pending(void)
{
    return s_stage_n;
}
