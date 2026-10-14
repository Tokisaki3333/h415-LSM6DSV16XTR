/* 处理函数 0：地磁净场提取
 *
 * 输入    shm->mmc：18 位原始值 + SET/RESET 极性，逐拍到达（约 2.08 kHz）
 * 输出    g_proc.mag_g[]：净场矢量（G），已扣器件零场偏置
 *
 * 物理模型（MMC5983MA 手册第 4986-5059 行）：
 *     m_SET = +B + b
 *     m_RST = -B + b
 *   => B = (m_SET - m_RST) / 2      <- 净场
 *      b = (m_SET + m_RST) / 2      <- 器件零场偏置（桥偏置）
 *
 * ★ 符号必须按 POL 统一（本次实验最重要的一条）：
 *   相邻两对的 SET/RST 次序相反（POL 0->1 与 1->0），直接相减会让净场
 *   逐对翻转 180 度。实测未修正时相邻帧方向差恒为 179.912 度（std 0.054 度，
 *   是确定性翻转而非噪声）；修正后降到 0.088 度。
 *   规则：POL[前] == 0 取 +1，否则取 -1。
 *
 * 器件零场偏置用实测常量逐轴扣除（见 proc_tune.h）。实测 |b| = 0.04233 G，
 * 六面标定面间散布 0.1955%，跨上电 0.07%，与姿态无关，可直接当常量。
 *
 * 卡死检测：本固件历史上 MMC 多次中途停止更新（IMU/BMP 仍正常），
 * 表现为时戳与极性都不再变化。连续 MAG_STALL_FRAMES 帧无新数据即判卡死，
 * 之后磁修正全部停用，姿态仅靠陀螺与重力环维持。
 *
 * 全部状态为文件静态，仅被 V5F 处理主循环单线程调用，无需加锁。 */
#include "proc.h"
#include "proc_tune.h"
#include "mipc_shm.h"
#include <math.h>       /* sqrtf：产出净场模长用（与 proc_eskf.c 等一致） */

#ifdef PC_PROBE
/* PC 侧探针（上报列，不参与算法）：由 pc_chain_main.c 读取 */
float g_pc_magbranch = 0.0f;   /* 进入"有新磁拍"分支的累计次数 */
float g_pc_magn      = 0.0f;   /* 最近一次的 |mag_g| */
float g_pc_magstall  = 0.0f;   /* 最近一次的 s_stall */
float g_pc_magvalid  = 0.0f;   /* 最近一次算出的 mag_valid */
float g_pc_mv        = 0.0f;   /* 传给 ESKF 的 mag_valid */
float g_pc_mn        = 0.0f;   /* 传给 ESKF 的 new_mag */
float g_pc_mr        = 0.0f;   /* 传给 ESKF 的 mag_ref_ok */
float g_pc_magcnt    = 0.0f;   /* 磁观测累计次数 */
float g_pc_caln      = 0.0f;   /* 启动标定累计 IMU 样本数 */
float g_pc_calmag    = 0.0f;   /* 启动标定累计磁样本数 */
#endif

static int32_t  s_prev[3];              /* 上一拍的原始值 */
static uint8_t  s_prev_pol;             /* 上一拍的极性：0=SET 1=RESET */
static float    s_last_u[3];            /* 上一次【已接受】的净值单位矢量 */
static uint8_t  s_have_u;               /* s_last_u 是否已就绪 */
static uint8_t  s_new_mag;              /* ★ 本帧是否有【新的】磁拍（供 EKF 门控观测） */

/* ★ 配对失效判据：与上一拍已接受净值的夹角上限。
 *   物理依据（实测）：相邻净值夹角中位 0.000°、90% 0.120°；
 *   即便 3000 dps 转动，0.5 ms 内也只变约 1.5°。
 *   取 10° ⟹ 比真实物理变化宽 6 倍，比失效幅度(150°+)小 15 倍。
 *   不需要额外标定，不引入任何仿真参数。 */
#define MAG_COS_MAX_STEP  0.98481f      /* cos(10°) */
static uint8_t  s_have_prev;            /* 是否已存有上一拍 */
static uint32_t s_last_cnt;             /* 上次看到的 mmc 帧序号 */
static uint32_t s_stall;                /* 连续无新数据的帧数 */

/* ★ 2026-10-14 删除 s_bias_u[3]（器件零场偏置单位方向）。
 *   它仅由 MAG_DEV_BIAS_UX/UY/UZ 初始化，全仓【0 处引用】⟹ gcc -Wunused-variable。
 *   对应的三个实测常量在 proc_tune.h 中仍以注释保留（实测值，可回退），
 *   但暂不参与计算：现有实现按【逐相球心】扣零点，未做逐轴偏置方向扣除。
 *   若将来启用逐轴扣除，恢复本行即可：
 *     static const float s_bias_u[3] = { MAG_DEV_BIAS_UX, MAG_DEV_BIAS_UY, MAG_DEV_BIAS_UZ };
 */

void proc_mag_init(void)
{
    uint32_t i;

    for (i = 0u; i < 3u; i++) s_prev[i] = 0;
    s_prev_pol  = 0u;
    s_have_u    = 0u;
    s_last_u[0] = 0.0f; s_last_u[1] = 0.0f; s_last_u[2] = 0.0f;
    s_have_prev = 0u;
    s_last_cnt  = 0u;
    s_stall     = 0u;

    g_proc.mag_valid   = 0u;
    g_proc.mag_stall   = 0u;
    g_proc.mag_bad_acc = 0u;
    {
        uint32_t k;
        for (k = 0u; k < 3u; k++) { g_proc.mag_g[k] = 0.0f; g_proc.mag_u[k] = 0.0f; }
    }
    g_proc.mag_norm_g = 0.0f;
}

/* 每 IMU 帧调用一次。有新磁拍就更新净场，否则只推进卡死计数。 */
uint8_t proc_mag_step(const volatile shared_mem_t *shm)
{
    int32_t  cur[3];
    uint32_t cnt;
    uint8_t  pol;
    uint32_t i;

    s_new_mag = 0u;
    cnt = shm->mmc.hdr.cnt;
    if (cnt != s_last_cnt)
    {
        s_new_mag = 1u;
#ifdef PC_PROBE
        g_pc_magbranch += 1.0f;
        g_pc_magstall = (float)s_stall;
#endif
        /* 有新拍：先做发布-订阅协议的屏障与读取 */
        __sync_synchronize();
        s_last_cnt = cnt;
        s_stall    = 0u;
        /* ★★ 有新磁拍 ⟹ 卡死状态解除。
         * 原实现 mag_stall 置 1 后【永不清除】⟹ 一次 200 ms 掉线就让地磁永久失效
         * （直到重启）。实测 rec_20261014_000422：magcnt 只涨到 4 就停住、
         * mag_valid 仅 0.39% 的帧为 1、地磁对整个 51.6 s 录制零作用。
         * 新磁拍到达即说明通道已恢复，必须复位。 */
        g_proc.mag_stall = 0u;

        /* ★★ 板上不做配对：本核只把【原始 counts + 极性】原样交给上报，
         *   配对/差分/归一化全部移到 PC 端按时间对齐做。
         *   原因：V3F 侧已改为"每 MMC_FLIP_PERIOD 个 SET 周期才做一次 RESET 周期"，
         *   相邻两拍绝大多数【同极性】，原来"仅当极性相反才配对"的半差公式
         *   （(m_SET − m_RESET)/2）几乎不再触发；即使触发，配对的两拍也相隔
         *   整个 1024 拍块（约 1 秒），结果没有物理意义。
         *   ⟹ 这里保留原始值读取与卡死计数，不再计算 g_proc.mag_u。
         *   PC 端由 FULL 帧里的 mag_raw_cur / mag_raw_prev / flag2(pol) 自行配对。 */
        pol       = shm->mmc.pol;
        cur[0]    = shm->mmc.mx;
        cur[1]    = shm->mmc.my;
        cur[2]    = shm->mmc.mz;
        /* ★★ 产出净场矢量（2026-10-14 启用）
         * SET-only 口径：本固件每 1024 个 SET 才做一次 RESET ⟹ 相邻拍几乎同极性,
         * 半差公式 (m_SET − m_RST)/2 已失效 ⟹ 器件零场偏置只能【按常量扣除】。
         * 常量零点由四份好录制的分相球心均值定出（跨录制 0.1~0.3%，见 proc_tune.h）。
         * 在此之前这里硬写 mag_valid=0，导致 proc_eskf.c 的地磁观测永远被门控掉。 */
        {
            float gx = ((float)cur[0] - MAG_ZERO_X_CNT) / MAG_CNT_PER_G;
            float gy = ((float)cur[1] - MAG_ZERO_Y_CNT) / MAG_CNT_PER_G;
            float gz = ((float)cur[2] - MAG_ZERO_Z_CNT) / MAG_CNT_PER_G;
            float n2 = gx*gx + gy*gy + gz*gz;
            float n  = sqrtf(n2);
            g_proc.mag_g[0] = gx; g_proc.mag_g[1] = gy; g_proc.mag_g[2] = gz;
            g_proc.mag_norm_g = n;
#ifdef PC_PROBE
            g_pc_magn = n;
#endif
            /* ★★ 相位零点必须分相（2026-10-14 修正）
             * 曾误判为"RST 通道有系统缺陷"。实测（14 份 1024:1 录制、514 个 RST 样本）：
             *   · RST 是【单拍】，与相邻 SET 仅差 5 帧（0.65 ms）⟹ 1500 dps 下只转 0.098°，
             *     所以 SET-RST 在转动中【可配对】
             *   · 两相是两个不同心的球：球心距实测 3820~4452 counts
             *   · 用 SET 相球心解释 RST 样本 ⟹ 方向残差 28.1°；换成 RST 相自己的球心
             *     ⟹ 11.3°，与 SET 的 10.1° 同档 ⟹ 那 28° 纯属【相位零点失配】
             * 本行以下用 SET 相常量零点，故只接受 SET 拍。
             * TODO(更优)：用 (SET+RST)/2 在线标定器件偏置、净场取 (SET−RST)/2，
             *   则两相样本都可用，且不必硬编码零点。 */
            if (pol != 0u) {
                g_proc.mag_valid = 0u;
            } else if ((n > MAG_MIN_UNIT) && (g_proc.mag_stall == 0u)) {
                float inv = 1.0f / n;
                g_proc.mag_u[0] = gx * inv;
                g_proc.mag_u[1] = gy * inv;
                g_proc.mag_u[2] = gz * inv;
                g_proc.mag_valid = 1u;
#ifdef PC_PROBE
                g_pc_magvalid = 1.0f;
#endif
                s_last_u[0] = g_proc.mag_u[0];
                s_last_u[1] = g_proc.mag_u[1];
                s_last_u[2] = g_proc.mag_u[2];
                s_have_u = 1u;
            } else {
                g_proc.mag_valid = 0u;
            }
        }

        for (i = 0u; i < 3u; i++) s_prev[i] = cur[i];
        s_prev_pol  = pol;
        s_have_prev = 1u;
    }
    else
    {
        if (s_stall < 0xFFFFFFFFu) s_stall++;
        if (s_stall >= MAG_STALL_FRAMES)
        {
            g_proc.mag_stall = 1u;
            g_proc.mag_valid = 0u;
        }
    }

    return 0u;
}

/* ★ 本帧是否有【新的】磁拍。
 *   EKF 只应在有真正新磁样本的那一拍做地磁观测 ——
 *   否则同一份样本会被重复注入多次（实测 1039.5 Hz 磁 / 7670 Hz EKF
 *   ⟹ 每份样本被用了 5 次），地磁权重被放大约 5 倍，
 *   等效于把仿真定下的 σ_mag = 0.44° 压到 0.44°/√5。 */
uint8_t proc_mag_is_new(void)
{
    return s_new_mag;
}
