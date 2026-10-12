/* 处理函数 2：动静判定（后验，吃前一项的结果）
 *
 * 输入    g_proc.gyro_dps[]  —— 处理函数 1 输出的零偏校正后角速度
 * 统计量  s = max_axis | W 帧滑窗均值 |，W = DET_W = 122 帧
 * 判运动  s > 阈值            连续 DET_DEB_ON  帧
 * 判静止  s < 阈值*0.6        连续 DET_DEB_OFF 帧      <- 迟滞 + 去抖
 *
 * 门控：本函数写 g_gate，供【下一帧】的零偏校正读取 —— 本帧的判定用本帧的
 * 校正输出、作用于下一帧的牵引（后验、依赖前项结果）。
 *
 * 阈值表：证据（累积静止粒度数）越少阈值越高，最高 2x 基础阈值；证据够了回到
 * DET_THR_BASE。证据由处理函数 1 维护（回退时扣减），所以回退后阈值自动回升。
 * 运行期只做移位/查表/比较，不做 exp/sqrt 现算。
 *
 * AC 旁路：主路阈值表索引来自"累积静止证据"，而证据只在主路判静止时累加，
 * 于是上电零偏残差超过表上限时会形成闭环并永久锁死。交流能量对直流零偏不可见，
 * 所以用它作旁路绕开自环。旁路只做"或"：把"运动"改判成"静止"。
 * ★ 门限按噪声带宽规范化：原值按 124.66us 帧率标定，本板 130.3us 帧率下
 *   交流能量实测高 8 倍，直接用原值会让旁路每帧开合振荡。
 *
 * 全部状态为文件静态，仅被 V5F 处理主循环单线程调用，无需加锁。 */
#include "proc.h"
#include "proc_tune.h"

static float    s_win[3][DET_W];
static float    s_sum[3];
static float    s_sq[3];
static uint16_t s_head;
static uint16_t s_fill;
static uint16_t s_on_cnt;
static uint16_t s_off_cnt;
static uint8_t  s_state;            /* 1 = 静止（上电初值） */
static uint16_t s_ac_cnt;

static const float s_thr_tab[DET_THR_NBUCK] = {
    0.3246f, 0.3246f, 0.3246f, 0.3246f, 0.3246f, 0.3054f, 0.2549f, 0.2156f,
    0.1854f, 0.1628f, 0.1462f, 0.1342f, 0.1258f, 0.1200f, 0.1160f, 0.1134f,
    0.1116f, 0.1104f, 0.1096f, 0.1091f
};

static float jf_abs(float x) { return (x < 0.0f) ? -x : x; }

void proc_static_detect_init(void)
{
    uint32_t i, j;

    for (i = 0u; i < 3u; i++)
    {
        for (j = 0u; j < DET_W; j++) s_win[i][j] = 0.0f;
        s_sum[i] = 0.0f;
        s_sq[i]  = 0.0f;
    }
    s_head = 0u; s_fill = 0u;
    s_on_cnt = 0u; s_off_cnt = 0u;
    s_state = 1u;
    s_ac_cnt = 0u;
    g_gate.is_static = 1u;
    g_gate.rollback_granules = DET_ROLLBACK_GRAN;
}

uint8_t proc_static_detect_step(void)
{
    float    inv, level, e_ac, e_tot, thr_on, thr_off, ac_scale;
    uint16_t idx;
    uint8_t  i;

    /* ---- 滑窗更新（三轴，O(1)） ---- */
    for (i = 0u; i < 3u; i++)
    {
        float x   = g_proc.gyro_dps[i];
        float old = s_win[i][s_head];
        s_sum[i] += x - old;
        s_sq[i]  += x * x - old * old;
        s_win[i][s_head] = x;
    }
    s_head = (uint16_t)((s_head + 1u) % DET_W);
    if (s_fill < DET_W) s_fill++;

    inv   = (s_fill > 0u) ? (1.0f / (float)s_fill) : 0.0f;
    level = 0.0f;
    for (i = 0u; i < 3u; i++)
    {
        float a = jf_abs(s_sum[i] * inv);
        if (a > level) level = a;
    }
    g_proc.level_dps = level;

    /* ---- 交流能量 / 总能量 ---- */
    e_ac = 0.0f; e_tot = 0.0f;
    for (i = 0u; i < 3u; i++)
    {
        float m_ax  = s_sum[i] * inv;
        float ms_ax = s_sq[i] * inv;
        float v_ax  = ms_ax - m_ax * m_ax;
        if (v_ax < 0.0f) v_ax = 0.0f;
        e_ac  += v_ax;
        e_tot += ms_ax;
    }

    /* 门限按噪声带宽规范化（本板 130.3us vs 标定用 124.66us） */
    ac_scale = 0.12466f / 0.1303f;
    if (ac_scale < 1.0f) ac_scale = 1.0f;

    if (e_ac < (DET_AC_E_ON * ac_scale) && e_tot < (DET_AC_E_TOT * ac_scale))
    {
        if (s_ac_cnt < DET_AC_DEB_ON) s_ac_cnt++;
    }
    else
    {
        s_ac_cnt = 0u;
    }
    {
        uint8_t ac_bypass = (s_ac_cnt >= DET_AC_DEB_ON) ? 1u : 0u;

        if (s_fill < DET_W)
        {
            /* 滑窗未满：不判，锁定静止 */
            s_state   = 1u;
            s_on_cnt  = 0u;
            s_off_cnt = 0u;
        }
        else
        {
            idx = (uint16_t)(g_proc.bias_evidence_gran >> DET_THR_SHIFT);
            if (idx >= DET_THR_NBUCK) idx = (uint16_t)(DET_THR_NBUCK - 1u);
            thr_on = s_thr_tab[idx];
            if (thr_on < DET_THR_BASE) thr_on = DET_THR_BASE;
            thr_off = thr_on * DET_THR_OFF_RATIO;

            if (g_proc.bias_ok == 0u && ac_bypass != 0u)
            {
                s_state   = 1u;
                s_on_cnt  = 0u;
                s_off_cnt = 0u;
            }
            else if (s_state != 0u)
            {
                if (level > thr_on)
                {
                    s_on_cnt++;
                    if (s_on_cnt >= DET_DEB_ON) { s_state = 0u; s_on_cnt = 0u; }
                }
                else s_on_cnt = 0u;
            }
            else
            {
                if (level < thr_off)
                {
                    s_off_cnt++;
                    if (s_off_cnt >= DET_DEB_OFF) { s_state = 1u; s_off_cnt = 0u; }
                }
                else s_off_cnt = 0u;
            }
        }
    }

    g_proc.is_static          = s_state;
    g_gate.is_static          = s_state;
    g_gate.rollback_granules  = DET_ROLLBACK_GRAN;
    return 0u;
}
