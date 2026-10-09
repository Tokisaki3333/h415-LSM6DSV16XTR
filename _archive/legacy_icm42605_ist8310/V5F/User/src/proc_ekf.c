/* ============================================================================
 * proc_ekf.c   第 7 步：16 维导航 EKF（误差状态，影子模式）
 *              第 8 步：EKF 各观测的**外部门控**（下游写、上游下一帧读）
 *
 * ---- 状态 ----
 *   标称 x  = [ p(3,m,ENU) v(3,m/s) q(4,机体->导航) ba(3,m/s^2) bg(3,rad/s) b_baro(1,m) ]
 *   误差 dx = [ dp(3) dv(3) dth(3,导航系) dba(3) dbg(3) db_baro(1) ]   共 16 维
 *   姿态误差用**全局(导航系)**约定：q_true = Exp(dth) (x) q_hat
 *   索引： dp 0..2  dv 3..5  dth 6..8  dba 9..11  dbg 12..14  db_baro 15
 *
 * ---- 导航系 ----
 *   真 ENU（x = 东, y = 北, z = 上），对齐那一帧用**磁力计一次性**把 x 轴定到真东，
 *   于是地磁水平分量的方位恒等于磁偏角 D（已知常数）。此后偏航闭环在**滤波器内部**由
 *   M7 承担 —— 旧 proc_attitude 链仍然不闭环，两者分工不混（见 ekf_design.md 0.1/4.1）。
 *
 * ---- 单位 ----
 *   内部一律 SI + 弧度。只有上报那一步才换成度 / dps（见 ekf_publish）。
 *
 * ---- 自环纪律（本仓库铁律，逐条落实）----
 *   1) 第 8 步（下游）算门，第 7 步（上游）**下一帧**读 —— 与"第 2 步写第 1 步读"同构。
 *   2) 门判据只用**外生量**：原始 LSB、离线标定常量、stat / acc_valid / mag.trust /
 *      GPS 报文质量字段。绝不取滤波器自己的输出（新息、P、sigma）。
 *   3) 新息 chi2（内层）在第 7 步算，与门**分开**记录：gate_bits bit9 = 本周期有观测被剔除。
 *      门回答"物理上能不能用"，chi2 回答"这一帧数值上像不像野值"，两件事不许混。
 *   4) 不用 h->imu.gyro_dps / accel_g（那两个被在线牵引改过，会与 EKF 的零偏状态互相污染），
 *      一律从原始 lsb 出发，只套离线标定常量。
 *
 * ---- 周期阶段机（实测逼出来的结构，别再合并回一帧）----
 *   第一版把"协方差最后一行 + 全部观测更新 + 标称推进"全塞在周期末那一帧，实测
 *   ISR 110 us / 预算 124.56 us = 88%，只要叠加别的中断就会漏 DRDY（实测 45 s 里
 *   47 个坏帧，且与尖峰帧一一对应）。现在把一个周期摊成 23 帧，每帧只做一件事：
 *
 *     stage 0..15  预积分 + 协方差传播**一行**（每帧约 512 flops，实测 31 us）
 *     stage 16     周期末：清 gate_bits，只做记账
 *     stage 17     M6 重力/倾斜（3 维，最贵的一道）
 *     stage 18     M5 软 ZUPT（3 维）
 *     stage 19     M3 气压（1 维）+ M7 磁偏航（1 维）
 *     stage 20     M1 水平位置（2 维）+ M2 高度（1 维）
 *     stage 21     M4 多普勒速度（2 维）；清 gps/baro 新样本号志
 *     stage 22     对称化 + 标称推进 + 建新 F -> 回 stage 0
 *
 *   代价：周期 = 23 帧 -> 349 Hz（原想 501 Hz）。对飞控足够，换来的是尖峰消失。
 * ========================================================================== */
#include "SPI_rx.h"
#include "v5f_proc.h"

#include <math.h>     /* sqrtf / atan2f / sinf / cosf：-Ofast 下是单条指令，不引入 libm 调用 */

/* ---------------- 状态索引 ---------------- */
#define EKF_N        17u    /* VER=132: +地磁偏航偏置 */
#define IX_P         0u
#define IX_V         3u
#define IX_Q         6u     /* 标称四元数占 4 个 */
#define IX_BA        10u
#define IX_BG        13u
#define IX_BB        16u    /* 标称量第 17 个 */
#define IX_MB       17u    /* VER=132 地磁偏航偏置（存储槽） */
#define EKF_NX       18u    /* VER=132 */
/* ---- 误差状态索引 EI_*（**与存储索引 IX_* 不是同一套！**）-----------------
 * 存储 s_x[EKF_NX=17]：p(0-2) v(3-5) q(6-9,**四元数 4 维**) ba(10-12) bg(13-15) bb(16)
 * 误差 dx[EKF_N =16] ：p(0-2) v(3-5) dth(6-8,**旋转矢量 3 维**) ba(9-11) bg(12-14) bb(15)
 * 四元数多占一个存储槽，故 ba 起的**误差索引 = 存储索引 - 1**。
 * 铁律：s_P / s_Pn / s_F / s_G / s_H / s_K / s_dx **一律用 EI_**；只有 s_x 用 IX_*。
 * （VER=131 之前 s_F 与 ekf_inject 误用 IX_*，导致 ba/bg 估计失效）*/
#define EI_P         0u
#define EI_V         3u
#define EI_Q         6u     /* 旋转矢量 3 维（不是四元数） */
#define EI_BA        9u
#define EI_BG       12u
#define EI_BB       15u
#define EI_MB       16u    /* VER=132 地磁偏航偏置（误差态） */

#define DEG2RAD      0.017453292f
#define RAD2DEG      57.2957795f
#define EKF_PI       3.14159265f
#define EKF_TWOPI    6.28318531f

/* ---------------- 内部状态 ----------------
 * ★ 这里所有大数组都是 **static** 而不是局部变量：本文件的最深调用链跑在 DMA1 中断里，
 *   而链接脚本给的栈只有 2 KB（Link_v5f.ld 的 __stack_size）。第一版把 16x16 的中间量
 *   放在栈上，直接把栈踩穿 —— 症状是每秒一两次、单帧的 h->ekf 局部被清零
 *   （|q| -> 0.003、sigma_yaw -> 185 度、press_avg -> 0）。只有这一个中断会调本文件，
 *   所以 static 是安全的。 */
static float    s_x[EKF_NX];
static float    s_P[EKF_N][EKF_N];
static float    s_Pn[EKF_N][EKF_N];
static float    s_F[EKF_N][EKF_N];
static float    s_G[EKF_N][EKF_N];
static float    s_H[3][EKF_N];        /* 观测矩阵（各道观测共用，逐道清零） */
static float    s_PHt[EKF_N][3];
static float    s_K[EKF_N][3];
static float    s_KS[EKF_N][3];
static float    s_KHP[EKF_N][EKF_N];   /* ★VER=86 K*(H P)，Joseph 形式用 */
static float    s_dx[EKF_N];

static uint8_t  s_aligned;
static uint8_t  s_origin_ok;
static uint8_t  s_F_ok;
static uint8_t  s_prop_row;
static uint8_t  s_stage;              /* 阶段机 0..22 */
static uint8_t  s_prop_ok;
static uint8_t  s_pub_row;      /* ★VER=48 清零前的 s_prop_row 快照（供上报）*/
static uint8_t  s_pub_stage;    /* ★VER=48 清零前的 s_stage 快照（供上报）*/            /* 本周期的协方差传播是否已完成（第一周期没有） */
static uint8_t  s_rej[5];             /* 各道观测连续被 chi2 剔除的次数（逃脱阀用）：
                                       * 0=重力/倾斜 1=速度 2=气压 3=磁偏航 4=位置 */
static float    s_dt_e;
static float    s_dt_prev;
static float    s_t_run;
static float    s_dth[3], s_dvb[3];
static float    s_dq[4];              /* 本周期姿态增量的**复合**结果（逐帧右乘）*/
                                       /* 不能拿 s_dth 相加代替：三轴同时转动时
                                        * 相加与复合差一个交换子 = 圆锥误差 */
static float    s_Rn[3][3];
static float    s_Rf[3][3];
static float    s_a_nav[3];
static float    s_alin_g;             /* 本周期 |a_lin|（机体系比力去掉零偏后的模长，g）：
                                       * 整流不确定度按它缩放，进 Q_tt */
static uint8_t  s_sat;                /* 本周期见过加计削顶帧 */
static uint8_t  s_chi2_soft;
static float    s_mag_fhb;
static float    s_mag_rx, s_mag_ry;
static float    s_mag_vx, s_mag_vy;      /* ★VER=67 投影向量 v */
static float    s_mag_v0x, s_mag_v0y;    /* ★VER=67 模型 v0 */
static float    s_mag_yawpre;            /* ★VER=67 修正前 EKF 偏航(度) */      /* ★VER=46 二维新息分量 */
static float    s_mag_dqx, s_mag_dqy, s_mag_dqz;
static float    s_mag_cmp_thm, s_mag_cmp_thp;   /* ★VER=78 罗盘：实测/预测航向(度) */
static float    s_mag_cmp_amn, s_mag_cmp_mhn;   /* ★VER=78 罗盘：比力模长(g) / 法平面内磁场模长 */
static float    s_tilt_dqx, s_tilt_dqy, s_tilt_dqz;  /* ★VER=50 M6 实际注入的姿态修正(度) */
static float    s_tilt_prx, s_tilt_pry, s_tilt_prz;
static uint8_t  s_mag_anchor;   /* ★VER=51 启动一次快速偏航对齐是否已做 */  /* ★VER=50 M6 预测重力方向(机体系) */  /* ★VER=46 本次实际注入的姿态修正(度) */      /* ★VER=45 机体系水平占比 = 向量有效性判据 */
static float    s_boot_t;       /* ★VER=62 开机计时(s)，用于整角对齐的时间窗 */
static uint8_t  s_mag_gate;           /* 第 8 步的外部门结论（供上报）*/
static uint8_t  s_mag_used;           /* 本周期 M7 是否真的更新了 */
static float    s_mag_bh;             /* Bh/|B| 死点判据量 */
static float    s_mag_r;              /* M7 新息（度）*/          /* 本周期有观测被软加权（记 bit9）*/
static uint8_t  s_mn_ok;              /* 稳健模长门结论（step8 写，step7 下一帧读）*/
/* 气压差值观测的环形缓冲：存 (h_baro, p_z) 历史，取最老的一笔做差分 */
static float    s_bh_ring[V5F_EKF_BARO_HP_N];
static float    s_bp_ring[V5F_EKF_BARO_HP_N];
static uint8_t  s_bh_idx;
static uint8_t  s_bh_fill;
static float    s_mag_dth[3];
/* VER=103 删除 s_mag_tilt_bad_s：它只服务于“运动中禁磁”那条 hold，已随它一起删 */
static uint32_t s_ist_last;
static float    s_wmag;                /* VER=118 本周期 |w| (rad/s)，供 Q 标度误差项 */
static float    s_w_int[3];           /* ★VER=83 永不归零的陀螺角增量积分 int(w dt) (rad) */
static float    s_w_snap[3];          /* ★VER=83 磁 DRDY 时刻的积分快照 */
static uint64_t s_mag_ts_last;        /* ★VER=83 上次见到的磁 DRDY 时间戳 (10ns 单位) */
static float    s_mag_age_ms;
static float    s_tilt_bad_s;         /* ★VER=84 倾角参考连续失效时长(s) */
static float    s_tilt_inv_ms;        /* ★VER=84 上报：倾角参考失效时长(ms) */
static uint8_t  s_grav_ok;            /* ★VER=84 上报：重力观测门状态 */
static float    s_mag_rs;
/* VER=130 静止态偏航牵引：s_yaw_p_min 为**运行时** P88 地板（默认纯数值下限），
 * s_yaw_fast 为"本帧静止且磁可信"标志（在 ekf_m7_mag 开头判定）。 */
static float    s_yaw_p_min = V5F_EKF_YAW_P_MIN;
static uint8_t  s_yaw_fast;
/* VER=126 地磁失效门（陀螺一致性）状态 */
static float    s_md_psi, s_md_psim, s_md_psig;
static uint16_t s_md_n;
static uint32_t s_md_hold;
static uint8_t  s_mag_dist;
/* VER=132 供验收帧读取的地磁偏航偏置（deg）；用全局量避免改 v5f_ekf_t 布局 */
volatile float   g_v5f_ekf_mb_deg;
static float    s_mag_mode;            /* VER=124 上报：本观测帧生效的地磁入口 0/1/2 */             /* ★VER=84 上报：磁 R 放大倍数 */
static uint8_t  s_mag_hold;           /* ★VER=84 上报：磁因倾角失效被暂停 */         /* ★VER=83 上报：磁数据到使用时刻的年龄 (ms) */
/* VER=123 地磁两入口的切换判据：重力矢量（倾角）误差估计，u32 定点，单位 1e-6 deg */
static uint32_t s_grav_err;
static float    s_ff_t;                /* 上次累加器更新时刻（s_t_run） */
static uint32_t s_ff_n, s_ff_ok;       /* 两观测帧之间的 DRDY 帧数 / 其中有重力牵引的帧数 */
static uint32_t s_mag_cnt_upd;        /* ★VER=73 上一次真正施加磁观测时的 ist 样本号 */

static uint64_t s_last_tick;

static int32_t  s_lat0_e7, s_lon0_e7;
static float    s_m_per_deg_lat, s_m_per_deg_lon;

static float    s_b_dop;

static float    s_baro_h;
static float    s_baro_h_last;
static uint8_t  s_baro_new;
static uint8_t  s_baro_seen;

static float    s_alt_wait;           /* 距上次 GPS 高度观测的秒数（同位置一起降频） */
static float    s_baro_t_last;
static uint8_t  s_baro_t_seen;
static float    s_pos_wait;
static float    s_baro_wait;          /* 距上次气压观测的秒数（按设计降到 6.1 Hz） */
static uint8_t  s_gps_new;

static uint16_t s_gate_bits;
static float    s_nis[5];             /* 位置 / 速度 / 气压 / 重力 / 磁偏航 */
static float    s_sig[4];             /* 偏航(度) / 水平位置(m) / 水平速度(m/s) / 倾角(度) */

/* ---------------------------------------------------------------------------
 * 小工具
 * ------------------------------------------------------------------------- */
static float wrap_pi(float a)
{
    while (a >  EKF_PI) a -= EKF_TWOPI;
    while (a < -EKF_PI) a += EKF_TWOPI;
    return a;
}

static void q_mul(const float *a, const float *b, float *o)
{
    float w = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    float x = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    float y = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    float z = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    o[0] = w; o[1] = x; o[2] = y; o[3] = z;
}

static void q_norm(float *q)
{
    float s, n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (n2 < 1e-12f) { q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f; return; }
    s = 1.0f / sqrtf(n2);
    q[0] *= s; q[1] *= s; q[2] *= s; q[3] *= s;
}

static void q_to_R(const float *q, float R[3][3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1.0f - 2.0f*(y*y + z*z);
    R[0][1] = 2.0f*(x*y - w*z);
    R[0][2] = 2.0f*(x*z + w*y);
    R[1][0] = 2.0f*(x*y + w*z);
    R[1][1] = 1.0f - 2.0f*(x*x + z*z);
    R[1][2] = 2.0f*(y*z - w*x);
    R[2][0] = 2.0f*(x*z - w*y);
    R[2][1] = 2.0f*(y*z + w*x);
    R[2][2] = 1.0f - 2.0f*(x*x + y*y);
}

static void rot_bn(const float R[3][3], const float *v, float *o)
{
    float a = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2];
    float b = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2];
    float c = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2];
    o[0] = a; o[1] = b; o[2] = c;
}

static void rot_nb(const float R[3][3], const float *v, float *o)
{
    float a = R[0][0]*v[0] + R[1][0]*v[1] + R[2][0]*v[2];
    float b = R[0][1]*v[0] + R[1][1]*v[1] + R[2][1]*v[2];
    float c = R[0][2]*v[0] + R[1][2]*v[1] + R[2][2]*v[2];
    o[0] = a; o[1] = b; o[2] = c;
}

/* 原始 LSB -> 比力 m/s^2（只用离线常量：减零偏、除标度、乘本地重力） */
static void raw_f_mps2(const volatile v5f_hold_t *h, float *f)
{
    static const float b0[3] = { V5F_ACCEL_BIAS_LSB_X, V5F_ACCEL_BIAS_LSB_Y, V5F_ACCEL_BIAS_LSB_Z };
    uint32_t i;
    for (i = 0u; i < 3u; i++) {
        f[i] = ((float)h->imu.accel_lsb[i] - b0[i]) / g_v5f_accel_lsb_per_g[i] * V5F_EKF_G_MPS2;
    }
}

/* 原始 LSB -> 角速度 rad/s（只用离线常量） */
/* 陀螺的**完整离线模型**：逐轴标度 -> 对称交叉项。顺序与常量与旧链
 * proc_gyro_bias.c 完全一致（那里是 dv[i] -> 交叉 -> 输出，再在 proc_attitude.c 里
 * 做加速度敏感度）。唯一不含的是**在线牵引出来的零偏**，那只由 EKF 自己的 bg 状态承担。
 * ★ 加速度敏感度按项目结论**不做**：实测标量模型不成立
 *   （不补 8.649 度 -> 补了 8.997 度，三条摇晃记录全部变差），
 *   V5F_GYRO_GSENS_* 已被故意置 0。"与旧链同一套离线模型"就包含"它关着的我也不开"。 */
static void raw_w_rads(const volatile v5f_hold_t *h, float *w)
{
    static const float g0[3] = { V5F_GYRO_BIAS_LSB_X, V5F_GYRO_BIAS_LSB_Y, V5F_GYRO_BIAS_LSB_Z };
    float dv[3];
    uint32_t i;

    for (i = 0u; i < 3u; i++) {
        dv[i] = ((float)h->imu.gyro_lsb[i] - g0[i]) / g_v5f_gyro_lsb_per_dps[i];
    }
    /* 对称交叉：dps_out[i] = dps[i] + sum_{j!=i} k_ij * dps[j]（0.15~0.19%） */
    w[0] = (dv[0] + V5F_GYRO_KXY * dv[1] + V5F_GYRO_KXZ * dv[2]) * DEG2RAD;
    w[1] = (dv[1] + V5F_GYRO_KXY * dv[0] + V5F_GYRO_KYZ * dv[2]) * DEG2RAD;
    w[2] = (dv[2] + V5F_GYRO_KXZ * dv[0] + V5F_GYRO_KYZ * dv[1]) * DEG2RAD;
}

/* 误差注入：q <- Exp(dth) (x) q（全局误差约定，一阶半角）
 * ★ dp 的注入**不**看 s_origin_ok：位置本来就是"相对对齐点"，有没有 ENU 原点都成立。
 *   第一版在这里加了 origin_ok 的门，结果没有 GPS 时气压观测修出来的 dp 被丢掉，
 *   滤波器只能拿 b_baro 去吸收，反而把高度环搞成单边。 */
static void ekf_inject(const float *dx)
{
    float dq[4], t[4];
    uint32_t i;

    /* VER=131 修复：dx 是**误差状态**，索引用 EI_*（原来误用 IX_*，
     * ba/bg 各错一格，dx[15] 同时被当成 bg_z 与 bb）。 */
    for (i = 0u; i < 3u; i++) s_x[IX_P  + i] += dx[EI_P  + i];
    for (i = 0u; i < 3u; i++) s_x[IX_V  + i] += dx[EI_V  + i];
    for (i = 0u; i < 3u; i++) s_x[IX_BA + i] += dx[EI_BA + i];
    for (i = 0u; i < 3u; i++) s_x[IX_BG + i] += dx[EI_BG + i];
    /* 钳位：这两个状态没有直接观测，只被弱相关间接推；不钳就能漂到不可能的值
     * （实测 bg_z = -40.7 dps），而 bg 参与积分 -> 会真去转姿态。 */
    for (i = 0u; i < 3u; i++) {
        if (s_x[IX_BA + i] >  V5F_EKF_BA_LIM_MPS2) s_x[IX_BA + i] =  V5F_EKF_BA_LIM_MPS2;
        if (s_x[IX_BA + i] < -V5F_EKF_BA_LIM_MPS2) s_x[IX_BA + i] = -V5F_EKF_BA_LIM_MPS2;
        if (s_x[IX_BG + i] >  V5F_EKF_BG_LIM_RADS) s_x[IX_BG + i] =  V5F_EKF_BG_LIM_RADS;
        if (s_x[IX_BG + i] < -V5F_EKF_BG_LIM_RADS) s_x[IX_BG + i] = -V5F_EKF_BG_LIM_RADS;
    }
    s_x[IX_BB] += dx[EI_BB];
    s_x[IX_MB] += dx[EI_MB];   /* VER=132 */
    if (s_x[IX_MB] >  V5F_EKF_BM_LIM_DEG * DEG2RAD) s_x[IX_MB] =  V5F_EKF_BM_LIM_DEG * DEG2RAD;
    if (s_x[IX_MB] < -V5F_EKF_BM_LIM_DEG * DEG2RAD) s_x[IX_MB] = -V5F_EKF_BM_LIM_DEG * DEG2RAD;

    dq[0] = 1.0f;
    dq[1] = 0.5f * dx[EI_Q + 0u];
    dq[2] = 0.5f * dx[EI_Q + 1u];
    dq[3] = 0.5f * dx[EI_Q + 2u];
    q_norm(dq);
    q_mul(dq, &s_x[IX_Q], t);
    for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = t[i];
    q_norm(&s_x[IX_Q]);
}

/* o = q (x) v (x) q*：把一个**矢量**按单位四元数旋转（与 q_mul 不是一回事）。
 * 用于把磁力计采样时刻的机体系矢量转到当前机体系。 */
static void q_rot_vec(const float *q, const float *v, float *o)
{
    float u[3], tt[3], c[3];
    u[0] = q[1]; u[1] = q[2]; u[2] = q[3];
    tt[0] = 2.0f*(u[1]*v[2] - u[2]*v[1]);
    tt[1] = 2.0f*(u[2]*v[0] - u[0]*v[2]);
    tt[2] = 2.0f*(u[0]*v[1] - u[1]*v[0]);
    c[0] = u[1]*tt[2] - u[2]*tt[1];
    c[1] = u[2]*tt[0] - u[0]*tt[2];
    c[2] = u[0]*tt[1] - u[1]*tt[0];
    o[0] = v[0] + q[0]*tt[0] + c[0];
    o[1] = v[1] + q[0]*tt[1] + c[1];
    o[2] = v[2] + q[0]*tt[2] + c[2];
}

static void H_zero(uint8_t m)
{
    uint32_t i, j;
    for (i = 0u; i < m; i++) {
        for (j = 0u; j < EKF_N; j++) s_H[i][j] = 0.0f;
    }
}

/* ---------------------------------------------------------------------------
 * 通用观测更新（m = 1/2/3）：H = s_H，R = m x m
 *   P 用标准形式 P <- P - K S K^T（周期末统一对称化 + 对角下限）
 *   返回 0 = 已更新（**含被软加权**：观测确实进去了、只是权重小）；2 = 数值失败。
 * ------------------------------------------------------------------------- */
static uint8_t ekf_update(const float *R, uint8_t m, const float *r,
                          float nis_max, float *nis_out, uint8_t *rej,
                          uint16_t inj_mask, float k_cap)
{
    float S[3][3], Si[3][3];
    float det, nis = 0.0f, s;
    float s_soft = 1.0f;   /* ★VER=86 chi2 软加权的实际 R 放大倍数 */
    uint32_t i, j, k;

    for (i = 0u; i < EKF_N; i++) {
        for (j = 0u; j < m; j++) {
            s = 0.0f;
            for (k = 0u; k < EKF_N; k++) s += s_Pn[i][k] * s_H[j][k];
            s_PHt[i][j] = s;
        }
    }
    for (i = 0u; i < m; i++) {
        for (j = 0u; j < m; j++) {
            s = 0.0f;
            for (k = 0u; k < EKF_N; k++) s += s_H[i][k] * s_PHt[k][j];
            S[i][j] = s + R[i*m + j];
        }
    }
    if (m == 1u) {
        if (S[0][0] < 1e-12f) return 2u;
        Si[0][0] = 1.0f / S[0][0];
    } else if (m == 2u) {
        det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
        if (det < 1e-18f) return 2u;
        Si[0][0] =  S[1][1]/det; Si[0][1] = -S[0][1]/det;
        Si[1][0] = -S[1][0]/det; Si[1][1] =  S[0][0]/det;
    } else {
        det = S[0][0]*(S[1][1]*S[2][2] - S[1][2]*S[2][1])
            - S[0][1]*(S[1][0]*S[2][2] - S[1][2]*S[2][0])
            + S[0][2]*(S[1][0]*S[2][1] - S[1][1]*S[2][0]);
        if (det < 1e-18f) return 2u;
        Si[0][0] = (S[1][1]*S[2][2] - S[1][2]*S[2][1])/det;
        Si[0][1] = (S[0][2]*S[2][1] - S[0][1]*S[2][2])/det;
        Si[0][2] = (S[0][1]*S[1][2] - S[0][2]*S[1][1])/det;
        Si[1][0] = (S[1][2]*S[2][0] - S[1][0]*S[2][2])/det;
        Si[1][1] = (S[0][0]*S[2][2] - S[0][2]*S[2][0])/det;
        Si[1][2] = (S[0][2]*S[1][0] - S[0][0]*S[1][2])/det;
        Si[2][0] = (S[1][0]*S[2][1] - S[1][1]*S[2][0])/det;
        Si[2][1] = (S[0][1]*S[2][0] - S[0][0]*S[2][1])/det;
        Si[2][2] = (S[0][0]*S[1][1] - S[0][1]*S[1][0])/det;
    }
    for (i = 0u; i < m; i++) {
        s = 0.0f;
        for (j = 0u; j < m; j++) s += Si[i][j] * r[j];
        nis += r[i] * s;
    }
    if (nis_out) *nis_out = nis;
    if (nis > nis_max) {
        /* 软加权，不硬剔：Si /= (nis/nis_max)，等价 R_eff = R*(nis/nis_max)。
         * 为什么不能硬剔：剔一次不改 P -> S 不变 -> nis 永远超限 -> 环永久锁死
         * （实测三次；最近一次是 VER=20 里 nis_baro p50=11.92、53.5% 的帧被剔、
         *  p_z 漂到 ±287 m 而 b_baro 冻在 0）。极端野值权重趋 0，仍然挡得住。 */
        float sc = nis / nis_max;
        if (sc > V5F_EKF_CHI2_SC_MAX) sc = V5F_EKF_CHI2_SC_MAX;
        for (i = 0u; i < m; i++) {
            for (j = 0u; j < m; j++) Si[i][j] /= sc;
        }
        s_soft = sc;                     /* ★VER=86 S_eff = soft*S */
        s_chi2_soft = 1u;
        if (rej && *rej < 250u) (*rej)++;
    } else if (rej) {
        *rej = 0u;
    }

    for (i = 0u; i < EKF_N; i++) {
        for (j = 0u; j < m; j++) {
            s = 0.0f;
            for (k = 0u; k < m; k++) s += s_PHt[i][k] * Si[k][j];
            s_K[i][j] = s;
        }
    }
    /* ★ 注入掩码：每道观测只能动它有物理资格动的状态。
     * 把 K 里不该动的行清零，dx 与 P 的修正就同时受限 —— 这叫"观测在子空间里作用"。
     * 为什么必须做：K = P H^T S^-1 会通过协方差交叉项把观测注入到 H 里根本没有的
     * 状态上。实测 VER=29：气压观测一恢复，就经 P[8][2] 往偏航注入，bg_z 顶到
     * ±10 dps 钳位、偏航以 10 dps 转飞（-156 度 vs 旧链 +4 度）。 */
    for (i = 0u; i < EKF_N; i++) {
        if (!(inj_mask & (uint16_t)(1u << i))) {
            for (j = 0u; j < m; j++) s_K[i][j] = 0.0f;
        }
    }
    /* ★ 增益上限 k_cap：K 的每一项取绝对值上限。关键：dx 与下面的 P 修正用的是
     * **同一个 K**，所以状态与协方差始终一致 —— 这正是上次"限幅"犯的错
     * （只回滚状态、不回滚 P，形成正反馈，把闪现变成了恒速漂移）。 */
    /* ★VER=85 去掉 K 的逐元素钳位（用上报数据离线回放验证）：
     * 钳 K 后协方差收缩退化成 K^2*S（与 P 无关），P 因此有地板：
     *   实测 sigma_yaw 卡在 54~68 度（回放逐段误差 <=1.1 度）、
     *   sigma_tilt 卡在 4.4~17 度；不钳时分别收敛到 2.2 度与 ~3.3 度。
     * 现在：K 保持最优 -> P -= K S K' 就是精确的 (I-KH)P；
     *        限步改由"钳新息"承担：|r_k| <= k_cap。K<=1 时 dx=K*r 幅度不超过 k_cap，
     *        所以 k_cap 的物理含义仍是"单次最大修正量"，各调用点取值含义不变。 */
    /* ★VER=86 恢复 K 的逐元素钳位（限步）。
     * VER=85 改成钳新息 + 用"最优 K"的公式更新 P 是根本错误：
     * 实际只提取了极小一部分信息，P 却按全部信息收缩 -> 过度自信。
     * 实测：sigma_yaw 钉在 2.500 度下界、sigma_tilt 0.21 度，
     *       而 |新息| p90 27.6 / max 33.8 度 -> 增益塌陷、姿态自由漂。
     * 正确不变量：P 必须对应"实际用掉的增益"。 */
    if (k_cap > 0.0f) {
        for (i = 0u; i < EKF_N; i++) {
            for (j = 0u; j < m; j++) {
                if (s_K[i][j] >  k_cap) s_K[i][j] =  k_cap;
                if (s_K[i][j] < -k_cap) s_K[i][j] = -k_cap;
            }
        }
    }
    /* dx = K r（r 不钳：新息是"我错多少"的唯一信息，钳了就看不见误差） */
    for (i = 0u; i < EKF_N; i++) {
        s = 0.0f;
        for (k = 0u; k < m; k++) s += s_K[i][k] * r[k];
        s_dx[i] = s;
    }
    /* s_KS = K * S_eff（S_eff = soft*S，chi2 软加权时保持精确） */
    for (i = 0u; i < EKF_N; i++) {
        for (j = 0u; j < m; j++) {
            s = 0.0f;
            for (k = 0u; k < m; k++) s += s_K[i][k] * S[k][j];
            s_KS[i][j] = s * s_soft;
        }
    }
    /* s_KHP = K (H P) = K * PHt' */
    for (i = 0u; i < EKF_N; i++) {
        for (j = 0u; j < EKF_N; j++) {
            s = 0.0f;
            for (k = 0u; k < m; k++) s += s_K[i][k] * s_PHt[j][k];
            s_KHP[i][j] = s;
        }
    }
    /* Joseph: P <- P - KHP - KHP' + K S_eff K'
     * K 取最优时恰好退化为 P - K S K'（=原公式），
     * K 被钳时则是对该 K 精确的后验协方差 -> 不会过度自信。 */
    for (i = 0u; i < EKF_N; i++) {
        for (j = 0u; j < EKF_N; j++) {
            s = 0.0f;
            for (k = 0u; k < m; k++) s += s_KS[i][k] * s_K[j][k];
            s_Pn[i][j] += s - s_KHP[i][j] - s_KHP[j][i];
        }
    }
    ekf_inject(s_dx);
    return 0u;
}

/* ===========================================================================
 * 各道观测（一道一函数，各自只占阶段机的一帧）
 * ========================================================================= */

/* M6 重力/倾斜（3 维，秩 2） */
static void ekf_m6_tilt(const volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)
{
    float R[9], r[3], f[3], Rt[3][3], pr[3], fb[3], up[3], ex[3], M[3][3], Sx[3][3];
    float e2, tang;
    uint8_t st;
    uint32_t i, j, k;

    if (!V5F_EKF_TILT_EN || !gate->ekf_tilt || !h->imu.acc_valid) return;
    q_to_R(&s_x[IX_Q], Rt);
    raw_f_mps2(h, f);
    for (i = 0u; i < 3u; i++) f[i] -= s_x[IX_BA + i];   /* 观测模型的比力要去掉零偏残差 */
    e2 = sqrtf(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    if (e2 <= 1e-6f) return;
    fb[0] = f[0]/e2; fb[1] = f[1]/e2; fb[2] = f[2]/e2;
    up[0] = 0.0f; up[1] = 0.0f; up[2] = 1.0f;
    rot_nb(Rt, up, pr);                        /* 预测：静止时"上"在机体系的表示 */
    /* 切平面投影：r = (I - pr pr^T)(fb - pr)。径向分量对倾角没有信息，不投影就等于
     * 把"比力模长残差"当倾角误差灌进滤波器。 */
    ex[0] = fb[0] - pr[0]; ex[1] = fb[1] - pr[1]; ex[2] = fb[2] - pr[2];
    tang = ex[0]*pr[0] + ex[1]*pr[1] + ex[2]*pr[2];
    for (i = 0u; i < 3u; i++) r[i] = ex[i] - tang*pr[i];
    /* H = (I - pr pr^T) R^T [up]x ；d(pred)/d(dth) = R^T [up]x */
    Sx[0][0] = 0.0f;   Sx[0][1] = -up[2]; Sx[0][2] =  up[1];
    Sx[1][0] = up[2];  Sx[1][1] = 0.0f;   Sx[1][2] = -up[0];
    Sx[2][0] = -up[1]; Sx[2][1] = up[0];  Sx[2][2] = 0.0f;
    for (i = 0u; i < 3u; i++) {
        for (j = 0u; j < 3u; j++) {
            float s2 = 0.0f;
            for (k = 0u; k < 3u; k++) s2 += Rt[k][i] * Sx[k][j];
            M[i][j] = s2;
        }
    }
    H_zero(3u);
    for (i = 0u; i < 3u; i++) {
        for (j = 0u; j < 3u; j++) {
            s_H[i][6u + j] = M[i][j]
                - pr[i]*(pr[0]*M[0][j] + pr[1]*M[1][j] + pr[2]*M[2][j]);
        }
    }
    for (i = 0u; i < 9u; i++) R[i] = 0.0f;
    /* ★VER=89 诚实 R：把离心项 a_c = w^2*ARM 造成的比力方向偏差写进 R，而不是靠把
     *   门收到 2 dps 去回避它(那样实测 grav_ok 只开 33%、磁暂停 54%)。小角度下
     *   atan(x)=x，故 eac(rad) = level_dps^2*DEG2RAD^2*ARM/g，钳 0.2 rad 防呆。
     *   level_dps 是 128 帧(16 ms)窗均值，瞬时 w 可能更高，所以这是**下界**估计。 */
    {
        float wr  = h->stat.level_dps * DEG2RAD;
        float eac = wr * wr * V5F_EKF_TILT_AC_ARM_M / V5F_EKF_G_MPS2;
        float sg;
        if (eac > 0.2f) eac = 0.2f;
        sg = V5F_EKF_TILT_SIG_RAD + eac;
        R[0] = R[4] = R[8] = sg * sg;
    }
    {
        /* ★ 单周期姿态修正上限（= 重力环的 tau 下限，约 8 s）。
         *   没有它，R=(0.5 度)^2 配 349 Hz 满增益更新，闭环带宽约 100 Hz，
         *   加计的方向噪声（0.127 度）几乎原样进姿态 —— 实测 ekf_q 全程抖，
         *   而旧链 Mahony（tau=5 s，带宽 0.03 Hz）稳。加计/磁的方向误差是
         *   **相关**的（DC 偏置、安装残差），不是白噪声，必须先限带宽。 */
        float q_before[4];
        uint32_t n;
        for (n = 0u; n < 4u; n++) q_before[n] = s_x[IX_Q + n];
        st = ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[3], &s_rej[0], 0x7FC0u, V5F_EKF_TILT_K_MAX);
        {
            float qc[4], dq[4], ang, cap = V5F_EKF_TILT_STEP_MAX_DEG * DEG2RAD;
            qc[0] = q_before[0]; qc[1] = -q_before[1];
            qc[2] = -q_before[2]; qc[3] = -q_before[3];
            q_mul(&s_x[IX_Q], qc, dq);        /* 相对旋转，小角度 */
            ang = 2.0f * sqrtf(dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
            if (ang > cap && ang > 1e-9f) {
                float sc = cap / ang;          /* 线性回缩（小角度等价于 slerp） */
                float qn[4];
                for (n = 0u; n < 4u; n++) {
                    qn[n] = q_before[n] + (s_x[IX_Q + n] - q_before[n]) * sc;
                }
                for (n = 0u; n < 4u; n++) s_x[IX_Q + n] = qn[n];
                q_norm(&s_x[IX_Q]);
            }
        }
    }
    s_tilt_prx = pr[0]; s_tilt_pry = pr[1]; s_tilt_prz = pr[2];   /* ★VER=50 */
    if (st == 0u) {
        s_gate_bits |= V5F_EKF_GB_TILT;
        s_tilt_dqx = s_dx[EI_Q + 0] * RAD2DEG;
        s_tilt_dqy = s_dx[EI_Q + 1] * RAD2DEG;
        s_tilt_dqz = s_dx[EI_Q + 2] * RAD2DEG;
    }
}

/* M5 软 ZUPT（3 维） */
static void ekf_m5_zupt(const volatile v5f_proc_gate_t *gate)
{
    float R[9], r[3], lam, sg;
    uint8_t st;
    uint32_t i;

    if (!gate->ekf_zupt) return;
    lam = (float)gate->vel_soft * (1.0f/255.0f);
    if (lam < 0.05f) lam = 0.05f;
    sg = V5F_EKF_ZUPT_SIG_MPS / lam;
    H_zero(3u);
    s_H[0][3] = 1.0f; s_H[1][4] = 1.0f; s_H[2][5] = 1.0f;
    for (i = 0u; i < 9u; i++) R[i] = 0.0f;
    R[0] = R[4] = R[8] = sg * sg;
    for (i = 0u; i < 3u; i++) r[i] = -s_x[IX_V + i];
    st = ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[1], &s_rej[1], 0x0E38u, 0.05f);
    if (st == 0u) s_gate_bits |= V5F_EKF_GB_ZUPT;
}

/* M3 气压高度（1 维，同时观测 p_z 与 b_baro） */
static void ekf_m3_baro(const volatile v5f_proc_gate_t *gate)
{
    float R[1], r[1], bnow, pnow, bold, pold;
    uint8_t st;

    if (!gate->ekf_baro || !s_baro_new) return;
    bnow = s_baro_h;
    pnow = s_x[IX_P + 2];
    if (s_bh_fill >= (uint8_t)V5F_EKF_BARO_HP_N) {
        /* 差值观测：z = 气压高度在窗口内的变化，预测 = p_z 在窗口内的变化。
         * 慢漂在差值里抵消；H 只挂 dp_z（**不含 b_baro**），所以绝对基准只由
         * GPS 高度 M2 提供 —— 这正是"气压只补瞬态"的意思。 */
        bold = s_bh_ring[s_bh_idx];
        pold = s_bp_ring[s_bh_idx];
        H_zero(1u);
        s_H[0][2] = 1.0f;
        R[0] = 2.0f * V5F_EKF_BARO_SIG_M * V5F_EKF_BARO_SIG_M
             + V5F_EKF_BARO_OLD_VAR_M2;
        r[0] = (bnow - bold) - (pnow - pold);
        {
            float bb_save = s_x[IX_BB];       /* 更新前先存 b_baro */
            uint32_t k2;
            /* ★ 掩码 = 0x0024：dp_z + dv_z，**不含 b_baro**。正确边界是：
         *   · dv_z 合法：气压是位置量，位置-速度的耦合来自动力学模型
         *     （P[p_z][v_z] 由 F 建立），位置观测据此修垂直速度是标准 Kalman 行为，
         *     不是凭空造信息。必须放开，否则垂直速度只能靠积分一路漂。
         *   · b_baro 不合法：它是**绝对**基准，只有 GPS 高度 M2 有资格定它；
         *     没有 M2 时它没有观测者（每周期归零已强制它保持惰性）。
         *   · 姿态/零偏一律不许：气压对姿态没有任何信息。
         *   注：VER=30 的速度发散真凶是被切掉的 ba（泄漏无处可去），不是这一项。 */
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2], &s_rej[2], 0x0024u, 0.02f);
            /* ★ 没有 GPS 高度时 b_baro 必须**完全惰性**。
             *   差分观测的 H 不含 b_baro，但 ekf_update 里 K[15] = Pn[15][2]*Si：
             *   只要 P[15][2] 被传播/更新弄成非零，b_baro 就会被拖走，而 P 的更新
             *   又加深这个相关 -> 正反馈。实测 VER=27 漂到 -116.2 m，pbb 被压到 0，
             *   协方差被带坏后偏航环一起失效（总转角 -156.9 vs 旧链 +0.88）。
             *   "气压只补瞬态"就意味着：没有绝对基准时它既不被动、也不动人。 */
            s_x[IX_BB] = bb_save;
            for (k2 = 0u; k2 < EKF_N; k2++) {
                s_Pn[15][k2] = 0.0f;
                s_Pn[k2][15] = 0.0f;
            }
            s_Pn[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
            s_P[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
            for (k2 = 0u; k2 < EKF_N; k2++) {
                s_P[15][k2] = 0.0f;
                s_P[k2][15] = 0.0f;
            }
            s_P[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
        }
        if (st == 0u) s_gate_bits |= V5F_EKF_GB_BARO;
    }
    /* 入环（无论本次是否更新都要走，保证窗口长度恒定） */
    s_bh_ring[s_bh_idx] = bnow;
    s_bp_ring[s_bh_idx] = pnow;
    s_bh_idx = (uint8_t)((s_bh_idx + 1u) % (uint8_t)V5F_EKF_BARO_HP_N);
    if (s_bh_fill < (uint8_t)V5F_EKF_BARO_HP_N) s_bh_fill++;
}

/* M7 磁偏航（1 维，只挂第 8 项） */
/* ===========================================================================
 * VER=123 地磁→EKF 的**两个入口**（互斥：每个观测帧只调其中一个）
 *   入口 A：重力法向量平面投影 -> 只修正姿态角（偏航）
 *   入口 B：矢量方式（e1 偏航 + e2 倾斜）
 * 判据 s_grav_err（重力矢量误差估计，单位 1e-6 deg）见 ekf_m7_mag 末尾。
 * ========================================================================= */

/* ---- 入口 A：把地磁矢量投影到以重力为法向量的平面，只用平面内的航向角残差 ------
 * 原文取自 VER=103 之前的实现（bak_src/V5F/User/src/proc_ekf.c.bak_v103_magvec），
 * 那一版里这是**地磁唯一的融合方式**、经过实机验证 => 残差/H/掩码直接复用：
 *   r = wrap_pi(thm - thp)   测量 - 预测（xm 参考轴下的平面内角）
 *   H 只有偏航行（s_H[0][8] = 1）
 *   注入掩码 0x0100：只开偏航状态，roll/pitch 的 K 被清零 = 只修正姿态角
 *   R = sigma_h^2 + (tanI*sigma_tilt)^2：倾角不确定度会污染平面内航向；
 *     重力有效时 sigma_tilt 极小，R 自动退化到 sigma_h^2。 */
/* VER=130 静止态牵引速度：由本帧实际 R_yaw 反解 P88 地板，使闭环 K = dt/tau 恒成立。
 * 只在静止（s_yaw_fast）时生效；否则回到 V5F_EKF_YAW_P_MIN（= VER=129 行为）。 */
static void ekf_yaw_pmin_static(float r_yaw)
{
    float k = V5F_MAG_EPOCH_DT_S / V5F_MAG_STATIC_TAU_S;
    if (V5F_MAG_STATIC_TAU_EN != 0u && s_yaw_fast != 0u && k > 0.0f && k < 1.0f) {
        float pf = r_yaw * (k / (1.0f - k));
        s_yaw_p_min = (pf > V5F_EKF_YAW_P_MIN) ? pf : V5F_EKF_YAW_P_MIN;
    } else {
        s_yaw_p_min = V5F_EKF_YAW_P_MIN;
    }
}

static uint8_t ekf_mag_entry_plane(float thm, float thp)
{
    float R1[1], r1[1], sig_h, sig_t, dl;

    r1[0] = wrap_pi(thm - thp);
    sig_h = V5F_EKF_MAG_WHITE_DEG;   /* VER=132: 只用白噪，偏置交给 b_m */
    sig_t = sqrtf(s_P[6][6] + s_P[7][7]) * RAD2DEG;
    if (sig_t > V5F_EKF_MAG_PLANE_TILT_CAP_DEG) sig_t = V5F_EKF_MAG_PLANE_TILT_CAP_DEG;
    dl = V5F_EKF_DIP_TAN * (sig_t * DEG2RAD);
    R1[0] = (sig_h * DEG2RAD) * (sig_h * DEG2RAD) + dl * dl;
    ekf_yaw_pmin_static(R1[0]);   /* VER=130 */

    H_zero(1u);
    s_H[0][EI_Q + 2u] = 1.0f;
    s_H[0][EI_MB] = 1.0f;   /* VER=132 偏置与偏航同系数 */
    return ekf_update(R1, 1u, r1, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                      0x0100u, V5F_EKF_MAG_VEC_K_MAX);
}

/* ---- 入口 B：矢量方式（VER=103 起的现实现，一字未改）：e1 偏航 + e2 倾斜 ---------- */
static uint8_t ekf_mag_entry_vec(const float *r2v, const float *RRv)
{
    ekf_yaw_pmin_static(RRv[0]);   /* VER=130 */
    return ekf_update(RRv, 2u, r2v, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                      0x01C0u, V5F_EKF_MAG_VEC_K_MAX);
}

static void ekf_m7_mag(const volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)
{
    float rg[2], Rt[3][3], Bn[3], mf[3], ab[3], mh[3], xv[3], crs3[3];
    float fp[3], mh2[3], crs4[3], b0v[3], up_nav[3];
    float bb[3], e1[3], e2[3], r2v[2], RRv[4];   /* VER=103 真牵引用 */
    float fhb2, ci, b0x, b0y, b0z;
    float dpar, mhn, xvn, thm, thp, psih, dp2, mh2n, an2, cc;
    uint8_t st;
    uint32_t i;

    /* ★VER=89 先清零：本函数有 8 条提前返回路径(量纲守卫/几何死点/mh2 守卫/位移死区/
     *   倾角参考暂停/样本未更新…)，旧版只在其中 2 条里清零，其余把上一周期的值留进
     *   遥测 —— 实测 mag_dqz 因此长期显示一个并不存在的常量注入(约 0.1745 度/周期，
     *   0.5 s 累积 60.9 度)，据此曾误判"磁在剧烈段注入 143 度/s"。
     *   清零放在入口门之前：门关时"本周期磁修正量"本来就是 0，这是事实不是补丁。 */
    s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;
    /* VER=130 静止态牵引判定：默认关闭（含门关时的早退路径） */
    s_yaw_p_min = V5F_EKF_YAW_P_MIN;
    {
        float wm = sqrtf(h->imu.gyro_dps[0] * h->imu.gyro_dps[0]
                       + h->imu.gyro_dps[1] * h->imu.gyro_dps[1]
                       + h->imu.gyro_dps[2] * h->imu.gyro_dps[2]);
        s_yaw_fast = (uint8_t)((wm < V5F_MAG_DIST_WMAX_DPS) && (s_mn_ok != 0u));
    }
    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;
    s_mag_hold = 0u;                    /* ★VER=84 */
    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
    /* ★VER=72 量纲守卫 */
    {
        float f2 = mf[0]*mf[0] + mf[1]*mf[1] + mf[2]*mf[2];
        if (f2 < 0.25f || f2 > 2.25f) {
            if (s_rej[3] < 250u) s_rej[3]++;
            s_gate_bits |= V5F_EKF_GB_CHI2;
            return;
        }
    }
    fhb2 = h->mag.f[0]*h->mag.f[0] + h->mag.f[1]*h->mag.f[1];
    s_mag_fhb = sqrtf(fhb2);
    /* ★VER=45 补样本陈旧 */
    {
        float am = sqrtf(s_mag_dth[0]*s_mag_dth[0] + s_mag_dth[1]*s_mag_dth[1]
                       + s_mag_dth[2]*s_mag_dth[2]);
        if (am > 1e-4f) {
            float half = 0.5f * am;
            float sc = sinf(half) / am;
            float dq[4];
            float tmp[3];
            dq[0] = cosf(half);
            dq[1] = -sc * s_mag_dth[0];
            dq[2] = -sc * s_mag_dth[1];
            dq[3] = -sc * s_mag_dth[2];
            q_rot_vec(dq, mf, tmp);
            mf[0] = tmp[0]; mf[1] = tmp[1]; mf[2] = tmp[2];
        }
    }
    ci  = 1.0f / sqrtf(1.0f + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN);
    b0x = ci * sinf(V5F_MAG_DECL_RAD);
    b0y = ci * cosf(V5F_MAG_DECL_RAD);
    b0z = -V5F_EKF_DIP_TAN * ci;
    /* ★VER=84 R 含倾角不确定度项：磁新息对倾角误差的灵敏度 =
     * tan(dip) = V5F_EKF_DIP_TAN = 2.08，所以 sigma_tilt 会以 2.08 倍混进偏航新息。 */

    /* ---- 几何诊断（导航系水平二维残差；定义不变，供日志与整角对齐用） ---- */
    q_to_R(&s_x[IX_Q], Rt);
    rot_bn(Rt, mf, Bn);
    s_mag_bh = sqrtf(Bn[0]*Bn[0] + Bn[1]*Bn[1]);
    s_mag_vx = Bn[0]; s_mag_vy = Bn[1];
    s_mag_v0x = b0x; s_mag_v0y = b0y;
    rg[0] = Bn[0] - b0x;
    rg[1] = Bn[1] - b0y;
    s_mag_rx = rg[0];
    s_mag_ry = rg[1];
    /* ★VER=81 已删除几何死点 BH_MIN：它的触发条件是"重力法向与磁北重合"
     * （即到了真磁极），本纬度磁倾 64.3 度、水平分量恒 0.4333，不可能发生。 */

    /* ---- ★VER=80 重力法向直接由**已知姿态**解出 ----
     * a_up = R(q_hat)^T . z_nav = 机体系里的"上"。
     * 不再用加计（运动时它饱和，实测 |a| 到 16.8 g，方向是垃圾），
     * 也不再自己构造 R_tilt。测量与预测都建立在这同一根 a_up 上，
     * 于是倾角误差在两边一致地出现，没有"测量用加计/预测用姿态"的不对称。 */
    up_nav[0] = 0.0f; up_nav[1] = 0.0f; up_nav[2] = 1.0f;
    rot_nb(Rt, up_nav, ab);
    /* ★VER=81 不再设"倾角可信"门：投影轴取的是**姿态自己**的重力法向
     * a_up = R(q_hat)^T z_nav（多路径融合的最优值），测量与预测都已投到该平面内，
     * 两向量同处一个平面，失配只能是绕 a_up 的旋转 = 偏航错。倾角错在这一步
     * 已经被投影消掉了，不需要再来一道门。 */
    /* 诊断：姿态的"上"与加计方向夹角（度数）—— 观测可不可信的直接读数 */
    an2 = sqrtf(h->imu.accel_g[0]*h->imu.accel_g[0]
              + h->imu.accel_g[1]*h->imu.accel_g[1]
              + h->imu.accel_g[2]*h->imu.accel_g[2]);
    if (an2 > 1e-6f) {
        cc = (ab[0]*h->imu.accel_g[0] + ab[1]*h->imu.accel_g[1]
            + ab[2]*h->imu.accel_g[2]) / an2;
        if (cc >  1.0f) cc =  1.0f;
        if (cc < -1.0f) cc = -1.0f;
        s_mag_cmp_amn = acosf(cc) * RAD2DEG;
    }

    /* 测量磁场在法平面内的分量 */
    dpar = mf[0]*ab[0] + mf[1]*ab[1] + mf[2]*ab[2];
    mh[0] = mf[0] - dpar*ab[0];
    mh[1] = mf[1] - dpar*ab[1];
    mh[2] = mf[2] - dpar*ab[2];
    mhn = sqrtf(mh[0]*mh[0] + mh[1]*mh[1] + mh[2]*mh[2]);
    s_mag_cmp_mhn = mhn;
    /* 参考：机体 x 轴在同一平面内的投影 */
    xv[0] = 1.0f - ab[0]*ab[0];
    xv[1] =      - ab[0]*ab[1];
    xv[2] =      - ab[0]*ab[2];
    xvn = sqrtf(xv[0]*xv[0] + xv[1]*xv[1] + xv[2]*xv[2]);
    if (mhn < 1e-3f || xvn < 1e-3f) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
    crs3[0] = ab[1]*mh[2] - ab[2]*mh[1];
    crs3[1] = ab[2]*mh[0] - ab[0]*mh[2];
    crs3[2] = ab[0]*mh[1] - ab[1]*mh[0];
    thm = atan2f((crs3[0]*xv[0] + crs3[1]*xv[1] + crs3[2]*xv[2]) / (mhn*xvn),
                 (mh[0]*xv[0] + mh[1]*xv[1] + mh[2]*xv[2]) / (mhn*xvn));

    /* 预测磁场：同一姿态、同一根 a_up */
    b0v[0] = b0x; b0v[1] = b0y; b0v[2] = b0z;
    rot_nb(Rt, b0v, fp);
    dp2 = fp[0]*ab[0] + fp[1]*ab[1] + fp[2]*ab[2];
    mh2[0] = fp[0] - dp2*ab[0];
    mh2[1] = fp[1] - dp2*ab[1];
    mh2[2] = fp[2] - dp2*ab[2];
    mh2n = sqrtf(mh2[0]*mh2[0] + mh2[1]*mh2[1] + mh2[2]*mh2[2]);
    if (mh2n < 1e-3f) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
    crs4[0] = ab[1]*mh2[2] - ab[2]*mh2[1];
    crs4[1] = ab[2]*mh2[0] - ab[0]*mh2[2];
    crs4[2] = ab[0]*mh2[1] - ab[1]*mh2[0];
    thp = atan2f((crs4[0]*xv[0] + crs4[1]*xv[1] + crs4[2]*xv[2]) / (mh2n*xvn),
                 (mh2[0]*xv[0] + mh2[1]*xv[1] + mh2[2]*xv[2]) / (mh2n*xvn));
    s_mag_cmp_thm = thm * RAD2DEG;
    s_mag_cmp_thp = thp * RAD2DEG;
    {
        float qw = s_x[IX_Q], qx = s_x[IX_Q+1], qy = s_x[IX_Q+2], qz = s_x[IX_Q+3];
        psih = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
        s_mag_yawpre = psih * RAD2DEG;
    }

    /* H 只有偏航项 */
    H_zero(1u);
    s_H[0][EI_Q + 2u] = 1.0f;
    s_H[0][EI_MB] = 1.0f;   /* VER=132 偏置与偏航同系数 */

    /* ★VER=62 开机窗内一次精确角整角修正（用几何残差 rg） */
    if (s_boot_t < V5F_EKF_MAG_ANCHOR_TS) {
        float n2 = b0x * b0x + b0y * b0y;
        float dpsi = 0.0f, e0, e1, h2;
        float qb[4], dq[4], qt[4];
        uint32_t i2;
        e0 = sqrtf(rg[0] * rg[0] + rg[1] * rg[1]);
        if (n2 > 1e-6f) {
            float crs = b0x * rg[1] - b0y * rg[0];
            float dt2 = b0x * rg[0] + b0y * rg[1] + n2;
            dpsi = atan2f(crs, dt2);
        }
        if (dpsi >  3.14159265f) dpsi =  3.14159265f;
        if (dpsi < -3.14159265f) dpsi = -3.14159265f;
        for (i2 = 0u; i2 < 4u; i2++) qb[i2] = s_x[IX_Q + i2];
        h2 = 0.5f * dpsi;
        dq[0] = cosf(h2); dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = sinf(h2);
        q_mul(dq, &s_x[IX_Q], qt);
        for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qt[i2];
        q_norm(&s_x[IX_Q]);
        q_to_R(&s_x[IX_Q], Rt);
        rot_bn(Rt, mf, Bn);
        {
            float rx2 = Bn[0] - b0x, ry2 = Bn[1] - b0y;
            e1 = sqrtf(rx2 * rx2 + ry2 * ry2);
        }
        if (e1 > e0) {
            for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qb[i2];
            h2 = -0.5f * dpsi;
            dq[0] = cosf(h2); dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = sinf(h2);
            q_mul(dq, &s_x[IX_Q], qt);
            for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qt[i2];
            q_norm(&s_x[IX_Q]);
            dpsi = -dpsi;
        }
        s_mag_anchor = 1u;
        s_mag_used = 1u;
        s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = dpsi * RAD2DEG;
        s_gate_bits |= V5F_EKF_GB_MAG;
        rg[0] = 0.0f; rg[1] = 0.0f;
        s_mag_rx = 0.0f; s_mag_ry = 0.0f;
        s_mag_r = 0.0f;
        return;
    }

    /* ---- VER=103 真牵引：机体系残差（不做任何姿态投影）-------------------- */
    {
        float n1, c1, sg2, ddip, conf, Sx[3][3];
        float gn_b[3];
        uint32_t ax2;
        bb[0] = Rt[0][0]*b0x + Rt[1][0]*b0y + Rt[2][0]*b0z;
        bb[1] = Rt[0][1]*b0x + Rt[1][1]*b0y + Rt[2][1]*b0z;
        bb[2] = Rt[0][2]*b0x + Rt[1][2]*b0y + Rt[2][2]*b0z;
        n1 = sqrtf(bb[0]*bb[0] + bb[1]*bb[1] + bb[2]*bb[2]);
        if (n1 < 1e-6f) return;
        bb[0] /= n1; bb[1] /= n1; bb[2] /= n1;
        c1 = mf[0]*bb[0] + mf[1]*bb[1] + mf[2]*bb[2];
        /* r = P_⊥ m^ = m^ - (m^·b^)b^  （等于 (I - b^b^T)(m^ - b^)）
         * 注意不能写成 (m^ - b^) - (m^·b^)b^：那多减了一个 b^，
         * 虽然再投影到 ⊥b^ 后数值被消掉，但 H 就不再是它的 Jacobian（不自洽）。 */
        /* ⊥b^ 平面正交基：e1 = normalize(z^ x b^)，e2 = b^ x e1 */
        /* ---- VER=107 面内基：竖直基准 = **重力矢量**（不假设夹具水平）----
         * e1 = normalize(gn × b^_b)：绕重力旋转 = 航向变化，它在 ⊥b^ 面内的运动方向。
         *   其**方向**只取决于 (gn, b^_b) 所在平面的法向 => 与磁倾角模型误差无关
         *   （实测表观 dip 随环境/位置变 2~3 度，所以偏航通道必须与 dip 解耦）。
         * e2 = b^_b × e1：另一个面内方向，携带**倾斜**信息。 */
        {
            float ag = sqrtf(h->imu.accel_g[0]*h->imu.accel_g[0]
                           + h->imu.accel_g[1]*h->imu.accel_g[1]
                           + h->imu.accel_g[2]*h->imu.accel_g[2]);
            if (ag < 1e-3f) return;
            gn_b[0] = h->imu.accel_g[0] / ag;
            gn_b[1] = h->imu.accel_g[1] / ag;
            gn_b[2] = h->imu.accel_g[2] / ag;
        }
        e1[0] = gn_b[1]*bb[2] - gn_b[2]*bb[1];
        e1[1] = gn_b[2]*bb[0] - gn_b[0]*bb[2];
        e1[2] = gn_b[0]*bb[1] - gn_b[1]*bb[0];
        n1 = sqrtf(e1[0]*e1[0] + e1[1]*e1[1] + e1[2]*e1[2]);
        if (n1 < 1e-4f) {                     /* 重力与磁场近平行（不该发生）：退回 z^ 基准 */
            e1[0] = -bb[1]; e1[1] = bb[0]; e1[2] = 0.0f;
            n1 = sqrtf(e1[0]*e1[0] + e1[1]*e1[1]);
            if (n1 < 1e-6f) { e1[0] = 1.0f; e1[1] = 0.0f; e1[2] = 0.0f; n1 = 1.0f; }
        }
        e1[0] /= n1; e1[1] /= n1; e1[2] /= n1;
        e2[0] = bb[1]*e1[2] - bb[2]*e1[1];
        e2[1] = bb[2]*e1[0] - bb[0]*e1[2];
        e2[2] = bb[0]*e1[1] - bb[1]*e1[0];
        r2v[0] = e1[0]*mf[0] + e1[1]*mf[1] + e1[2]*mf[2];   /* 航向敏感分量 */
        r2v[1] = e2[0]*mf[0] + e2[1]*mf[1] + e2[2]*mf[2];   /* 倾斜敏感分量 */
        s_mag_r  = atan2f(sqrtf(r2v[0]*r2v[0] + r2v[1]*r2v[1]), c1) * RAD2DEG;
        s_mag_rx = r2v[0] * RAD2DEG;
        s_mag_ry = r2v[1] * RAD2DEG;
        /* H_i = +e_i^T [b^_b]x R^T（标准形式 H = dh/dx；基在当前估计处冻结） */
        H_zero(2u);
        Sx[0][0] = 0.0f;   Sx[0][1] = -bb[2]; Sx[0][2] =  bb[1];
        Sx[1][0] = bb[2];  Sx[1][1] = 0.0f;   Sx[1][2] = -bb[0];
        Sx[2][0] = -bb[1]; Sx[2][1] = bb[0];  Sx[2][2] = 0.0f;
        for (ax2 = 0u; ax2 < 3u; ax2++) {
            float w0, w1, w2;
            w0 = Sx[0][0]*Rt[ax2][0] + Sx[0][1]*Rt[ax2][1] + Sx[0][2]*Rt[ax2][2];
            w1 = Sx[1][0]*Rt[ax2][0] + Sx[1][1]*Rt[ax2][1] + Sx[1][2]*Rt[ax2][2];
            w2 = Sx[2][0]*Rt[ax2][0] + Sx[2][1]*Rt[ax2][1] + Sx[2][2]*Rt[ax2][2];
            s_H[0][IX_Q + ax2] = e1[0]*w0 + e1[1]*w1 + e1[2]*w2;
            s_H[1][IX_Q + ax2] = e2[0]*w0 + e2[1]*w1 + e2[2]*w2;
        }
        s_H[0][EI_MB] = s_H[0][EI_Q + 2u];   /* VER=132 导航竖直轴偏置 */
        /* ---- 倾斜权的置信度：两者都用**重力**做竖直基准 ----
         * dip_m = 90 - angle(m^, gn)（与姿态无关、与夹具是否水平无关）；
         * ddip = |dip_m - 模型倾角|；环境畸变/模型失配越大 -> c 越小 -> 倾斜行 R 越大。
         * 干净环境 ddip->0 => c->1 => 磁的倾斜信息按 sigma_yaw 正常使用。 */
        {
            float cm = mf[0]*gn_b[0] + mf[1]*gn_b[1] + mf[2]*gn_b[2];
            float dip_m, dip_0;
            if (cm >  1.0f) cm =  1.0f;
            if (cm < -1.0f) cm = -1.0f;
            /* 纾佸満鏈濅笅鏃?angle(m^,gn) = 90 + I锛屾墍浠?dip_m = angle - 90锛? +I锛夛紱
             * 鍐欐垚 90 - angle 浼氬緱 -57 搴︼紝浣夸俊搴︽案杩滃帇鍒颁笅闄?*/
            dip_m = acosf(cm) * RAD2DEG - 90.0f;
            dip_0 = atanf(V5F_EKF_DIP_TAN) * RAD2DEG;
            ddip  = fabsf(dip_m - dip_0);
            conf  = 1.0f / (1.0f + (ddip / V5F_EKF_MAG_TILT_DDIP_REF_DEG)
                                   * (ddip / V5F_EKF_MAG_TILT_DDIP_REF_DEG));
            if (conf < V5F_EKF_MAG_TILT_CONF_MIN) conf = V5F_EKF_MAG_TILT_CONF_MIN;
        }
        sg2 = V5F_EKF_MAG_VEC_SIG_DEG * DEG2RAD;
        RRv[0] = sg2 * sg2;                       /* 航向行：固定 R（S3 实测磁残差 p50 1.7~2.15 deg） */
        RRv[1] = 0.0f; RRv[2] = 0.0f;
        {   /* VER=118: 倾斜行同样用固定常数 R，不再按 conf 抬 R */
            float st = V5F_EKF_MAG_VEC_SIG_TILT_DEG;
            RRv[3] = (st * DEG2RAD) * (st * DEG2RAD);
            s_mag_rs = (float)s_grav_err * 1.0e-6f;   /* VER=123 上报：重力矢量误差估计 deg（0=重力有效）*/
            (void)conf;
        }
    }

    /* ---- VER=103 共用门：残差上限 + 同一磁样本不重复更新 -------------------
     * **不再有"运动中禁磁"**：合法性由外门给（gate->ekf_mag_yaw = |mag_norm-1| < MAG_ERR_LIM，只看模）；
     * 这里只剩残差太大(>R_MAX)拒一次、以及同一个磁样本不重复更新（磁 ~190Hz vs 帧 335Hz）。 */
    if (s_mag_r > V5F_EKF_MAG_R_MAX_DEG) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
    {
        uint32_t icm = g_shm ? g_shm->ist.hdr.cnt : 0u;
        if (icm == s_mag_cnt_upd) {
            s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;
            return;
        }
        s_mag_cnt_upd = icm;
    }

    /* ---------------- 模式 1：真牵引（2 维向量量测） ---------------- */
    /* ---- VER=123 两个入口互斥：由"重力矢量误差估计"决定用哪一个 ------------------
     * s_grav_err <= THR(2 deg)：重力矢量误差很低 -> 入口 A（投影到重力法平面，
     *                            只修正姿态角，roll/pitch 完全不参与）
     * s_grav_err >  THR       ：重力长期没牵引、倾角只能靠陀螺 -> 入口 B（矢量方式）
     * 阈值比较用微度整数；不加回差（判据不含 EKF 的 P 反馈，不存在极限环）。 */
    if (s_grav_err > (uint32_t)(V5F_EKF_MAG_FF_THR_DEG * 1.0e6f)) {
        st = ekf_mag_entry_vec(r2v, RRv);      /* 入口 B：矢量方式 */
        if (st == 0u) s_mag_mode = 2.0f;       /* VER=124 上报：矢量入口 */
    } else {
        st = ekf_mag_entry_plane(thm, thp);    /* 入口 A：重力法平面投影，只修正姿态角 */
        if (st == 0u) s_mag_mode = 1.0f;       /* VER=124 上报：投影入口 */
    }
    if (st == 0u) {
        s_gate_bits |= V5F_EKF_GB_MAG;
        s_mag_used = 1u;
        s_mag_dqx = s_dx[EI_Q + 0] * RAD2DEG;
        s_mag_dqy = s_dx[EI_Q + 1] * RAD2DEG;
        s_mag_dqz = s_dx[EI_Q + 2] * RAD2DEG;
    } else {
        s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;
    }

}

/* M1 水平位置（2 维，按 V5F_EKF_POS_PERIOD_S 降频）+ M2 高度（1 维） */
static void ekf_m1m2_gps(const volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)
{
    float R[4], r[2], rb[2], rw;
    uint8_t st;
    int32_t dla, dlo;

    if (!s_gps_new) return;

    if (gate->ekf_gps_pos && s_origin_ok && s_pos_wait >= V5F_EKF_POS_PERIOD_S) {
        dla = h->gps_rmc.lat_e7 - s_lat0_e7;
        dlo = h->gps_rmc.lon_e7 - s_lon0_e7;
        rb[0] = (float)dlo * 1e-7f * s_m_per_deg_lon;      /* 东 */
        rb[1] = (float)dla * 1e-7f * s_m_per_deg_lat;      /* 北 */
        H_zero(2u);
        s_H[0][0] = 1.0f; s_H[1][1] = 1.0f;
        rw = 1.0f;
        if (h->gps_gsv.snr_n > 0u && h->gps_gsv.snr_avg_dbhz > 1.0f) {
            float ratio = V5F_EKF_C_N0_REF_DBHZ / h->gps_gsv.snr_avg_dbhz;
            rw = ratio * ratio;
            if (rw < 1.0f) rw = 1.0f;
            if (rw > 9.0f) rw = 9.0f;
        }
        R[0] = V5F_GPS_POS_R_H_M * V5F_GPS_POS_R_H_M * rw;
        R[1] = 0.0f; R[2] = 0.0f; R[3] = R[0];
        r[0] = rb[0] - s_x[IX_P + 0u];
        r[1] = rb[1] - s_x[IX_P + 1u];
        st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[0], &s_rej[4], 0x0E3Fu, 0.05f);
        if (st == 0u) { s_gate_bits |= V5F_EKF_GB_GPS_POS; s_pos_wait = 0.0f; }
    }

    /* GPS 高度必须与位置一起降频：它和水平位置是**同一条接收机解算**出来的，
     * 误差同样是 5 s 以上相关时间的慢随机游走（水平实测 lag1 自相关 0.999），
     * 按每个历元喂就是让 R=(5.5 m)^2 名不副实、偷偷给垂直通道超配权重 ——
     * 与气压原来那个错完全同类。 */
    if (gate->ekf_gps_alt && s_alt_wait >= V5F_EKF_POS_PERIOD_S) {
        H_zero(1u);
        s_H[0][2] = 1.0f;
        R[0] = V5F_GPS_ALT_R_M * V5F_GPS_ALT_R_M;
        r[0] = h->gps_gga.alt_m - s_x[IX_P + 2];
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, NULL, &s_rej[4], 0x8024u, 0.05f);
        if (st == 0u) { s_gate_bits |= V5F_EKF_GB_GPS_ALT; s_alt_wait = 0.0f; }
    }
}

/* M4 多普勒速度（2 维：speed 与 course 同源，不许拆成两个观测） */
static void ekf_m4_vel(const volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)
{
    float R[4], r[2], cr, sp;
    uint8_t st;

    if (!gate->ekf_gps_vel || !s_gps_new) return;
    cr = h->gps_rmc.course_deg * DEG2RAD;
    sp = h->gps_rmc.speed_mps;
    H_zero(2u);
    s_H[0][3] = 1.0f; s_H[1][4] = 1.0f;
    R[0] = V5F_GPS_VEL_R_MPS * V5F_GPS_VEL_R_MPS;
    R[1] = 0.0f; R[2] = 0.0f; R[3] = R[0];
    r[0] = sp * sinf(cr) - s_b_dop - s_x[IX_V + 0u];
    r[1] = sp * cosf(cr) - s_b_dop - s_x[IX_V + 1u];
    st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[1], &s_rej[1], 0x0E18u, 0.05f);
    if (st == 0u) s_gate_bits |= V5F_EKF_GB_GPS_VEL;
}

/* ---------------------------------------------------------------------------
 * 标称推进 + 建 F
 * ------------------------------------------------------------------------- */
static void ekf_step_nominal(void)
{
    float R[3][3], dvn[3], an[3], dv[3];
    float qn[4];
    uint32_t i, j;

    q_to_R(&s_x[IX_Q], R);
    for (i = 0u; i < 3u; i++) dv[i] = s_dvb[i] - s_x[IX_BA + i] * s_dt_e;
    rot_bn(R, dv, dvn);
    for (i = 0u; i < 3u; i++) {
        dv[i] = dvn[i] + ((i == 2u) ? (-V5F_EKF_G_MPS2 * s_dt_e) : 0.0f);
    }
    /* ---- 水平速度死区（挡"重力经倾角误差漏进水平加速度"的常值偏置）----
     * 这个偏置 g*dtheta_h 与真实常值加速度在观测上不可分，所以只能按量级取舍：
     * 泄漏造成**无界增长**的速度误差，而被吃掉的真实小加速度是**瞬态**的
     * （悬停时水平加速度本就近 0）。死区随滤波器自己的倾角不确定度正比例缩放，
     * 于是"倾角越不可信 -> 速度通道越不敏感"，自洽。
     * ★ 只改估计，不改协方差：Q_vv 里那一项继续让 sigma_v 诚实增长。 */
    if (V5F_EKF_VDEAD_K > 0.0f) {
        float td = V5F_EKF_VDEAD_K * V5F_EKF_G_MPS2
                 * sqrtf(s_Pn[6][6] + s_Pn[7][7]) * s_dt_e;   /* 本周期对应的 dv 门限 */
        for (i = 0u; i < 2u; i++) {
            if (dv[i] < td && dv[i] > -td) dv[i] = 0.0f;
        }
    }
    /* ---- 路径 B：速度泄漏（只作用于水平两轴）----
     * lambda = K*sigma_tilt，即 tau = 1/(K*sigma_tilt)。姿态可信时 tau 很大
     * （几乎不干预，保住慢速平移精度）；姿态不可信时 tau 变小，把"重力经倾角
     * 误差漏进加速度"造成的**无界**漂移限成有界（稳态误差 = b*tau）。
     * 与死区的本质区别：这里不砍任何单帧信息，只给速度一个与不确定度匹配的漏。 */
    if (V5F_EKF_VLEAK_K > 0.0f) {
        float stg = sqrtf(s_Pn[6][6] + s_Pn[7][7]);      /* 倾角 1sigma，rad */
        float lam = V5F_EKF_VLEAK_K * stg;
        float lam_min = 1.0f / V5F_EKF_VLEAK_TAU_MAX_S;
        if (lam < lam_min) lam = lam_min;
        for (i = 0u; i < 2u; i++) {
            dv[i] -= s_x[IX_V + i] * lam * s_dt_e;
        }
    }
    for (i = 0u; i < 3u; i++) an[i] = dv[i] / s_dt_e;
    for (i = 0u; i < 3u; i++) {
        s_x[IX_P + i] += s_x[IX_V + i] * s_dt_e + 0.5f * an[i] * s_dt_e * s_dt_e;
    }
    for (i = 0u; i < 3u; i++) s_x[IX_V + i] += dv[i];
    for (i = 0u; i < 3u; i++) s_a_nav[i] = an[i];

    /* 姿态：q <- q (x) s_dq。s_dq 是这 16 帧**逐帧复合**出来的四元数增量，
     * 不是把轴角矢量相加（那样三轴同时转动会产生圆锥误差 -> 偏航大误差）。 */
    q_mul(&s_x[IX_Q], s_dq, qn);
    for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = qn[i];
    q_norm(&s_x[IX_Q]);

    /* F = I + A dt */
    for (i = 0u; i < EKF_N; i++) {
        for (j = 0u; j < EKF_N; j++) s_F[i][j] = 0.0f;
        s_F[i][i] = 1.0f;
    }
    for (i = 0u; i < 3u; i++) s_F[EI_P + i][EI_V + i] = s_dt_e;   /* VER=131 */   /* dp' = dv */

    q_to_R(&s_x[IX_Q], R);
    for (i = 0u; i < 3u; i++) {
        for (j = 0u; j < 3u; j++) s_Rn[i][j] = R[i][j];
    }
    {
        float fb[3], fnv[3];
        for (i = 0u; i < 3u; i++) fb[i] = s_dvb[i]/s_dt_e - s_x[IX_BA + i];
        /* |a_lin|（g）：整流误差 ~ k_rect*|a_lin|，作为姿态过程噪声的来源 */
        s_alin_g = sqrtf(fb[0]*fb[0] + fb[1]*fb[1] + fb[2]*fb[2]) / V5F_EKF_G_MPS2;
        rot_bn(R, fb, fnv);
        s_Rf[0][0] = 0.0f;    s_Rf[0][1] = -fnv[2]; s_Rf[0][2] =  fnv[1];
        s_Rf[1][0] = fnv[2];  s_Rf[1][1] = 0.0f;    s_Rf[1][2] = -fnv[0];
        s_Rf[2][0] = -fnv[1]; s_Rf[2][1] = fnv[0];  s_Rf[2][2] = 0.0f;
    }
    for (i = 0u; i < 3u; i++) {
        for (j = 0u; j < 3u; j++) {
            /* VER=131 修复：原来用 IX_*（存储索引）写 F，ba/bg 错位一格，
             * 且 F[.][15] 把气压实偏 bb 串进了姿态。现一律用 EI_*。 */
            s_F[EI_V + i][EI_Q  + j] = -s_Rf[i][j] * s_dt_e;
            s_F[EI_V + i][EI_BA + j] = -s_Rn[i][j] * s_dt_e;
            s_F[EI_Q + i][EI_BG + j] = -s_Rn[i][j] * s_dt_e;
        }
    }
    s_dt_prev = s_dt_e;
    s_F_ok = 1u;
}

/* 协方差传播的第 s_prop_row 行（用上个周期建好的 F 与本周期时长） */
static void ekf_prop_row(void)
{
    uint32_t j, k, row = s_prop_row;
    float sa2, sg2, sq_yaw = 0.0f, qd, dtq = s_dt_prev;

    for (j = 0u; j < EKF_N; j++) {
        float s = 0.0f;
        for (k = 0u; k < EKF_N; k++) s += s_F[row][k] * s_P[k][j];
        s_G[row][j] = s;
    }
    sa2 = V5F_EKF_SIG_A_MPS2 * V5F_EKF_SIG_A_MPS2;
    /* 速度过程噪声也要加上**重力经倾角不确定度的泄漏**：
     * a_nav = R(q)·(f-ba) + g，姿态倾角不确定度 sigma_tilt 会让重力的一部分漏进
     * 水平加速度，a_h,err ≈ g·sigma_tilt。实测 sigma_tilt p50 0.63 度 -> 0.108 m/s^2。
     * 不加这一项，平移时 sigma_vel_h 只长到 0.07 m/s 而真实速度约 1 m/s —— 差 14 倍，
     * 下游会去信一个错了 1 m/s 的速度。（用 s_P 里的倾角方差，即把耦合项算进协方差。） */
    {
        float st2 = s_P[6][6] + s_P[7][7];
        float gst = V5F_EKF_G_MPS2 * sqrtf(st2);
        sa2 += gst * gst;
    }
    /* 姿态过程噪声 = 白噪声 + **加速度整流**。
     * 后者是运动相关的系统项（a_c = w^2*r 恒 >= 0，整流不抵消、单向累积），
     * 只靠 sigma_g 建模会让 sigma_yaw 在剧烈运动里严重偏小（实测差 35 倍）。
     * 不补偿它（标量模型不成立），但必须承认它。 */
    {
        float sr = V5F_EKF_RECT_DPS_PER_G * s_alin_g * DEG2RAD;
        float ss = V5F_EKF_GYRO_KS * s_wmag;   /* VER=118: 标度误差项 -> 方差随时间与总转角单调增长 */
        float sq = V5F_EKF_SIG_G_RADS * V5F_EKF_SIG_G_RADS + sr * sr + ss * ss;
        sg2 = sq;
        /* ---- VER=122 偏航行(row 8)的 Q 按 EKF 标准记账（用户：地磁修正速度用 ekf 标准数学实现）----
         * Q_yaw = ARW^2 + (KS_YAW*|w|*DEG2RAD)^2，并整体 <= V5F_EKF_Q_YAW_MAX。
         *   1) ARW 用 S3 实测随机游走 V5F_EKF_Q_YAW_ARW；原式把 8 kHz 单样本 RMS
         *      (0.1224 dps) 当谱密度用，等效 7.34 deg/rt-h（Q 大 336 倍）；
         *   2) 转动项用 1000 ppm 口径 V5F_EKF_GYRO_KS_YAW 并补上 DEG2RAD
         *      （sq 其它项都是 rad 量纲；原式漏掉 = 放大 1/DEG2RAD^2 = 3283 倍）；
         *   3) Q_MAX 等价于"闭环 tau >= 1 s"，即修正速度 <= 2 dps（误差 <= 2 deg 时）；
         *   4) 倾角行(row 6/7)保持 VER=118/119 原状，不动已验收的通道。 */
        {
            float kry = V5F_EKF_GYRO_KS_YAW * s_wmag * DEG2RAD;
            float ark = V5F_EKF_Q_YAW_ARW;
            sq_yaw = ark * ark + kry * kry;
            if (sq_yaw > V5F_EKF_Q_YAW_MAX) sq_yaw = V5F_EKF_Q_YAW_MAX;
        }
    }
    for (j = 0u; j < EKF_N; j++) {
        float s = 0.0f;
        for (k = 0u; k < EKF_N; k++) s += s_G[row][k] * s_F[j][k];
        if (j == row) {
            qd = V5F_EKF_Q_FLOOR;
            if      (row >= EI_V  && row <= EI_V  + 2u) qd += sa2 * dtq;
            else if (row >= EI_Q  && row <= EI_Q  + 2u) qd += ((row == EI_Q + 2u) ? sq_yaw : sg2) * dtq;
            else if (row >= EI_BA && row <= EI_BA + 2u) qd += V5F_EKF_SIG_BA_RW * V5F_EKF_SIG_BA_RW * dtq;
            else if (row >= EI_BG && row <= EI_BG + 2u) qd += V5F_EKF_SIG_BG_RW * V5F_EKF_SIG_BG_RW * dtq;
            else if (row == EI_BB)                      qd += V5F_EKF_SIG_BARO_RW * V5F_EKF_SIG_BARO_RW * dtq;
            else if (row == EI_MB)                      qd += V5F_EKF_BM_Q_RADS2 * dtq;   /* VER=132 GM 过程 */
            /* ★VER=73 偏航方差下限：不加这一项，M7 的 k_cap 限幅更新会把
             * P[8][8] 指数抽干（削减 ∝ P）-> 偏航增益饿死 -> 环路失效。
             * 见 v5f_tune.h V5F_EKF_Q_YAW_MIN 的实测数据与取值依据。 */
            if (row == EI_Q + 2u) qd += V5F_EKF_Q_YAW_MIN;
            s += qd;
        }
        s_Pn[row][j] = s;
    }
    s_prop_row++;
}

/* 周期末：对称化 + 对角下限 + 派生 1sigma + 把 s_Pn 变成本周期的新 s_P */
static void ekf_finalize(void)
{
    uint32_t i, j;
    for (i = 0u; i < EKF_N; i++) {
        for (j = i; j < EKF_N; j++) {
            float a = 0.5f * (s_Pn[i][j] + s_Pn[j][i]);
            if (i == j && a < V5F_EKF_P_FLOOR) a = V5F_EKF_P_FLOOR;
            /* ★VER=75 偏航方差下限：M7 的 k_cap 限幅更新对 P[8][8] 的削减 ∝ P，
             * 是指数塌陷，实测会被削到 1e-12 使偏航增益归零、环路彻底失去力气
             * （VER=74 录制：静置中残差从 21 度涨到 49 度，P 养回来才拉回）。
             * 下限取磁力计自身测量噪声的平方，保证 K 恒在 k_cap 上限。 */
            if (i == EI_Q + 2u && j == EI_Q + 2u && a < s_yaw_p_min) a = s_yaw_p_min;   /* VER=130 */
            s_Pn[i][j] = a;
            s_Pn[j][i] = a;
        }
    }
    for (i = 0u; i < EKF_N; i++) {
        for (j = 0u; j < EKF_N; j++) s_P[i][j] = s_Pn[i][j];
    }
    s_sig[0] = sqrtf(s_Pn[8][8]) * RAD2DEG;
    s_sig[1] = sqrtf(s_Pn[0][0] + s_Pn[1][1]);
    s_sig[2] = sqrtf(s_Pn[3][3] + s_Pn[4][4]);
    s_sig[3] = sqrtf(s_Pn[6][6] + s_Pn[7][7]) * RAD2DEG;   /* 倾角（水平两轴合成） */
}

static void ekf_publish(volatile v5f_hold_t *h)
{
    uint32_t i;
    for (i = 0u; i < 3u; i++) h->ekf.p[i] = s_x[IX_P + i];
    for (i = 0u; i < 3u; i++) h->ekf.v[i] = s_x[IX_V + i];
    for (i = 0u; i < 4u; i++) h->ekf.q[i] = s_x[IX_Q + i];
    for (i = 0u; i < 3u; i++) h->ekf.ba[i] = s_x[IX_BA + i];
    for (i = 0u; i < 3u; i++) h->ekf.bg[i] = s_x[IX_BG + i] * RAD2DEG;   /* rad/s -> dps */
    h->ekf.b_baro = s_x[IX_BB];
    /* 诊断四件套：s_Pn 是"传播/更新后的工作协方差"，s_P 是"上周期末的"。对齐那一步
     * 只写了 s_P（s_Pn 还是 bss 的 0），所以两者一起报就能判定 P0 有没有落地、
     * 传播有没有把它搬过去。 */
    h->ekf.pzz    = s_Pn[2][2];
    h->ekf.pbb    = s_Pn[15][15];
    h->ekf.p_p_zz = s_P[2][2];
    h->ekf.p_p_bb = s_P[15][15];
    h->ekf.mag_gate   = s_mag_gate;
    h->ekf.mag_used   = s_mag_used;
    h->ekf.mag_mode    = s_mag_mode;   /* VER=124 */
    g_v5f_ekf_mb_deg = s_x[IX_MB] * RAD2DEG;   /* VER=132 */
    h->ekf.mag_bh     = s_mag_bh;
    h->ekf.mag_r_deg  = s_mag_r;
    h->ekf.p_yy       = s_Pn[8][8];
     h->ekf.prop_ok = (uint8_t)s_prop_ok;
     h->ekf.f_ok = (uint8_t)s_F_ok;
      h->ekf.prop_row = s_pub_row;
      h->ekf.stage = s_pub_stage;
     h->ekf.mag_rej = s_rej[3];
      h->ekf.mag_fhb = s_mag_fhb;
       h->ekf.mag_rx = s_mag_rx;
       h->ekf.mag_ry = s_mag_ry;
        h->ekf.mag_vx = s_mag_vx;
        h->ekf.mag_vy = s_mag_vy;
        h->ekf.mag_v0x = s_mag_v0x;
        h->ekf.mag_v0y = s_mag_v0y;
        h->ekf.mag_yawpre = s_mag_yawpre;
       h->ekf.mag_dqx = s_mag_dqx;
       h->ekf.mag_dqy = s_mag_dqy;
       h->ekf.mag_dqz = s_mag_dqz;
       h->ekf.mag_cmp_thm = s_mag_cmp_thm;
       h->ekf.mag_cmp_thp = s_mag_cmp_thp;
       h->ekf.mag_cmp_amn = s_mag_cmp_amn;
       h->ekf.mag_cmp_mhn = s_mag_cmp_mhn;
       h->ekf.mag_age_ms = s_mag_age_ms;
       h->ekf.mag_rs = s_mag_rs;
       h->ekf.tilt_inv_ms = s_tilt_inv_ms;
       h->ekf.mag_hold = s_mag_hold;
       h->ekf.grav_ok = s_grav_ok;
       h->ekf.grav_nis = s_nis[3];
        h->ekf.tilt_dqx = s_tilt_dqx;
        h->ekf.tilt_dqy = s_tilt_dqy;
        h->ekf.tilt_dqz = s_tilt_dqz;
        h->ekf.tilt_prx = s_tilt_prx;
        h->ekf.tilt_pry = s_tilt_pry;
        h->ekf.tilt_prz = s_tilt_prz;
    for (i = 0u; i < 3u; i++) h->ekf.a_nav[i] = s_a_nav[i];
    h->ekf.gate_bits     = s_gate_bits;
    h->ekf.sigma_yaw_deg = s_sig[0];
    h->ekf.sigma_pos_h   = s_sig[1];
    h->ekf.sigma_vel_h   = s_sig[2];
    h->ekf.sigma_tilt_deg = s_sig[3];
    for (i = 0u; i < 5u; i++) h->ekf.nis[i] = s_nis[i];
    h->ekf.aligned   = s_aligned;
    h->ekf.valid     = s_aligned;
    h->ekf.origin_ok = s_origin_ok;
}

/* ===========================================================================
 * 第 7 步：EKF 本体（上游读门）
 * ========================================================================= */
uint8_t v5f_proc_ekf(volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)
{
    float dt, w[3], f[3];
    uint64_t tk;
    uint32_t i, j;

    if (!V5F_EKF_EN) return V5F_PROC_OK;

    tk = h->imu.fresh.drdy_tick;
    if (tk == 0ULL) return V5F_PROC_OK;
    if (s_last_tick == 0ULL) { s_last_tick = tk; return V5F_PROC_OK; }
    if (tk <= s_last_tick) { s_last_tick = tk; return V5F_PROC_OK; }
    dt = (float)(tk - s_last_tick) * 1e-8f;
    s_last_tick = tk;
    if (dt <= 0.0f || dt > V5F_EKF_DT_MAX_S) return V5F_PROC_OK;

    s_t_run += dt;

    /* ---------------- 一次性对齐（四级兜底，保证一定启动） ----------------
     * ★ 教训（VER=26 实测）：原条件挂了 mag.trust 与 is_static。两者的判据都压在
     *   阈值上会抖（mag.trust 里的 |a_lin|<0.05g 实测 p50=0.0487g；is_static 本次
     *   只有 37.4%），没同时落上时 EKF **整个停摆**（gate_bits 恒 0、|ekf_q| 恒 0）。
     *   一次性对齐绝不能挂在一个会抖的门上。 */
    if (!s_aligned) {
        float am = h->imu.accel_g[0]*h->imu.accel_g[0]
                 + h->imu.accel_g[1]*h->imu.accel_g[1]
                 + h->imu.accel_g[2]*h->imu.accel_g[2];
        uint8_t amag_ok, force = 0u, can_align = 0u, self_init = 0u;
        float q0[4];

        amag_ok = (uint8_t)(h->imu.acc_valid
                            && (fabsf(am - 1.0f) < V5F_EKF_ALIGN_AMAG_TOL));
        /* D：连 up_ref_ok 都没来（旧链自己也要"静止 + 1g"才建系）-> EKF 自己建倾角。
         *    用与 proc_attitude 完全同一套最小旋转：axis = u x e_z = (u_y, -u_x, 0)，
         *    q = [1 + u_z, axis] 归一化；u 朝下时退化成绕机体 x 转 180 度。 */
        if (h->att.up_ref_ok) {
            for (i = 0u; i < 4u; i++) q0[i] = h->att.q[i];
            can_align = amag_ok;
        } else if (h->imu.acc_valid) {
            float ux = h->imu.accel_g[0], uy = h->imu.accel_g[1], uz = h->imu.accel_g[2];
            float n2 = ux*ux + uy*uy + uz*uz, w, nx, ny, s;
            if (n2 > 1e-6f) {
                s = 1.0f / sqrtf(n2);
                ux *= s; uy *= s; uz *= s;
                if (uz > -0.999999f) { w = 1.0f + uz; nx = uy; ny = -ux; }
                else                 { w = 0.0f; nx = 1.0f; ny = 0.0f; }
                s = 1.0f / sqrtf(w*w + nx*nx + ny*ny);
                q0[0] = w*s; q0[1] = nx*s; q0[2] = ny*s; q0[3] = 0.0f;
                can_align = amag_ok;
                self_init = 1u;
            }
        }
        /* A/B/C：判静要求逐级放宽 */
        if (can_align && !(h->stat.valid && (h->stat.is_static || h->stat.ac_bypass))) {
            if (s_t_run >= V5F_EKF_ALIGN_TIMEOUT_S) {
                can_align = (uint8_t)(fabsf(am - 1.0f) < V5F_EKF_ALIGN_AMAG_LOOSE);
                force = 1u;
            }
        }
        if (!can_align && h->imu.acc_valid && h->att.up_ref_ok
            && s_t_run >= V5F_EKF_ALIGN_FORCE_S) {
            for (i = 0u; i < 4u; i++) q0[i] = h->att.q[i];
            can_align = 1u; force = 1u;
        }

        if (can_align) {
            float Ri[3][3], mf[3], Bn[3], azi, dpsi, hq[4], qz[4];
            for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = q0[i];
            q_norm(&s_x[IX_Q]);
            /* 磁偏航一次性对齐：只在**稳健模长门**（与 M7 同一判据）通过时才做。
             * 不做也完全可以 —— M7 的 H 只挂偏航项，它自己会把偏航拉到位。 */
            if (s_mn_ok) {
                q_to_R(&s_x[IX_Q], Ri);
                for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
                rot_bn(Ri, mf, Bn);
                azi  = atan2f(Bn[0], Bn[1]);
                /* 符号：h = atan2(Bx,By)，而 q <- Exp(phi z)(x)q 使 h_new = h - phi
                 * （R_z(phi)·(0,1,0) = (-sin phi, cos phi, 0)）；所以 phi = azi - D。 */
                dpsi = wrap_pi(azi - V5F_MAG_DECL_RAD);
                hq[0] = cosf(0.5f*dpsi); hq[1] = 0.0f; hq[2] = 0.0f; hq[3] = sinf(0.5f*dpsi);
                q_mul(hq, &s_x[IX_Q], qz);
                for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = qz[i];
                q_norm(&s_x[IX_Q]);
            }

            for (i = 0u; i < 3u; i++) s_x[IX_P  + i] = 0.0f;
            for (i = 0u; i < 3u; i++) s_x[IX_V  + i] = 0.0f;
            for (i = 0u; i < 3u; i++) s_x[IX_BA + i] = 0.0f;
            for (i = 0u; i < 3u; i++) {   /* VER=118 单向播种追踪器零偏（此后 EKF 独占，不连续注入） */
                s_x[IX_BG + i] = h->imu.bias_ok ? (h->imu.gyro_bias_dps[i] * DEG2RAD) : 0.0f;
            }
            s_x[IX_BB] = 0.0f;
            s_x[IX_MB] = 0.0f;   /* VER=132 复位地磁偏置 */

            for (i = 0u; i < EKF_N; i++) {
                for (j = 0u; j < EKF_N; j++) s_P[i][j] = 0.0f;
            }
            s_P[0][0] = s_P[1][1] = s_P[2][2] = V5F_EKF_P0_POS_M * V5F_EKF_P0_POS_M;
            s_P[3][3] = s_P[4][4] = s_P[5][5] = V5F_EKF_P0_VEL_MPS * V5F_EKF_P0_VEL_MPS;
            s_P[6][6] = s_P[7][7] = V5F_EKF_P0_TILT_RAD * V5F_EKF_P0_TILT_RAD;
            s_P[8][8] = V5F_EKF_P0_YAW_RAD * V5F_EKF_P0_YAW_RAD;
            s_P[9][9] = s_P[10][10] = s_P[11][11] = V5F_EKF_P0_BA_MPS2 * V5F_EKF_P0_BA_MPS2;
            {   /* VER=118 播种 P0_bias：追踪器有效用 0.005 dps，否则用默认 */
                float sbs = h->imu.bias_ok ? V5F_EKF_P0_BG_SEED_RADS : V5F_EKF_P0_BG_RADS;
                s_P[12][12] = s_P[13][13] = s_P[14][14] = sbs * sbs;
            }
            s_P[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
            s_P[EI_MB][EI_MB] = (V5F_EKF_BM_P0_DEG * DEG2RAD)
                              * (V5F_EKF_BM_P0_DEG * DEG2RAD);   /* VER=132 */
            /* s_Pn 还是 bss 的 0；同步成 P0，免得首帧上报假的 0 */
            for (i = 0u; i < EKF_N; i++) {
                for (j = 0u; j < EKF_N; j++) s_Pn[i][j] = s_P[i][j];
            }
            /* 自己建的倾角有误差 -> 把倾角方差开大一点，让 M6 后面收 */
            if (self_init) s_P[6][6] = s_P[7][7] = 0.05f * 0.05f;
            if (self_init) s_Pn[6][6] = s_Pn[7][7] = 0.05f * 0.05f;

            for (i = 0u; i < 3u; i++) { s_dth[i] = 0.0f; s_dvb[i] = 0.0f; }

            s_dq[0] = 1.0f; s_dq[1] = 0.0f; s_dq[2] = 0.0f; s_dq[3] = 0.0f;
            s_dt_e = 0.0f; s_dt_prev = 0.0f;
            s_prop_row = 0u; s_stage = 0u; s_F_ok = 0u;
            s_pos_wait = 0.0f; s_alt_wait = 0.0f; s_baro_wait = 0.0f;
            s_baro_t_last = 0.0f; s_baro_t_seen = 0u;
            s_alin_g = 0.0f; s_sat = 0u;
            s_mag_dth[0] = s_mag_dth[1] = s_mag_dth[2] = 0.0f;
            s_w_int[0] = s_w_int[1] = s_w_int[2] = 0.0f;
            s_w_snap[0] = s_w_snap[1] = s_w_snap[2] = 0.0f;
            s_mag_ts_last = 0ULL;
            s_bh_idx = 0u; s_bh_fill = 0u;
            s_grav_err = 0u; s_ff_n = 0u; s_ff_ok = 0u; s_ff_t = s_t_run;   /* VER=123 切换判据复位 */
            s_mag_anchor = 0u;
            s_boot_t = 0.0f;   /* VER=126: 磁锚定窗从**对齐时刻**起算（原来从上电起算，对齐晚于 3 s 就永远不锚定 -> 上电那次快速牵引丢失） */
            s_mag_dist = 0u; s_md_hold = 0u; s_md_n = 0u;
            s_yaw_p_min = V5F_EKF_YAW_P_MIN; s_yaw_fast = 0u;   /* VER=130 */ s_md_psim = 0.0f; s_md_psig = 0.0f;   /* ★VER=51 重新对齐后允许再做一次快速偏航对齐 */
            for (i = 0u; i < 5u; i++) { s_nis[i] = 0.0f; s_rej[i] = 0u; }
            s_gate_bits = 0u;
            s_sig[0] = V5F_EKF_P0_YAW_RAD * RAD2DEG;
            s_sig[1] = V5F_EKF_P0_POS_M;
            s_sig[2] = V5F_EKF_P0_VEL_MPS;
            s_sig[3] = V5F_EKF_P0_TILT_RAD * RAD2DEG;
            s_a_nav[0] = s_a_nav[1] = s_a_nav[2] = 0.0f;
            s_aligned = 1u;
            s_gate_bits |= V5F_EKF_GB_ALIGN;
            if (force)     s_gate_bits |= V5F_EKF_GB_ALIGN_FORCE;
            ekf_publish(h);
            return V5F_PROC_OK;
        }
        return V5F_PROC_OK;
    }

    /* ---------------- 周期阶段机 ---------------- */
    /* ---------------- 预积分：**每一帧**都要做 ----------------
     * ★ 必须放在阶段判断之外。第一版把它写在 `s_stage < DECIM` 里，于是那 7 个
     *   观测更新帧（stage 16..22）的陀螺/加计数据被整个丢掉 —— 一周期 23 帧里
     *   只积了 16 帧，姿态少转 16/23 = 0.6957。实测一个 92.16 度的台阶 EKF 只跟了
     *   65.57 度，比值 0.7115，与 0.6957 对上。后果是磁环一直在后面追（板子静止时
     *   EKF 偏航还被以 9~12 dps 拽回来补缺口），v/p 同样按 70% 缩放。
     * 必须减估计的陀螺零偏：误差状态里 F 写的是 dth' = -R dbg，就是在说
     *   "标称角速度 = 量测 - bg_hat"。加计那一路本来就是对的（下面 dv 里减 ba*dt_e）。 */
    raw_w_rads(h, w);
    raw_f_mps2(h, f);
    /* ★VER=83 磁陈旧补偿改用磁自己的 DRDY 时间戳。
     * 旧做法在 ist.hdr.cnt 变化时把 s_mag_dth 清零，等价于把"我发现计数器变了"
     * 当成采样时刻：DRDY 到发布/发现的流水线延迟被算成 0，而这份延迟正是
     * "拿旧磁数据修新陀螺仪"的量。现改为：s_w_int 永不归零地积分 w*dt，
     * 磁时间戳一变就快照 s_w_snap，补偿角 = s_w_int(now) - s_w_snap
     * = 从 t_drdy 到现在的真实转角。同时上报磁数据年龄用于判定延迟量级。 */
    {
        uint32_t ic = g_shm ? g_shm->ist.hdr.cnt : 0u;
        uint64_t ts = h->mag.fresh.drdy_tick;
        if (ic != s_ist_last || ts != s_mag_ts_last) {
            float ags = 0.0f;
            s_ist_last = ic;
            s_mag_ts_last = ts;
            /* ---- VER=97：把快照回溯到真正的 DRDY 时刻 ----
             * 原来 s_w_snap = s_w_int(本帧)，等价于把"我方发现新样本的那一帧"当成采样
             * 时刻，DRDY->读出这段延迟被当成 0（原注释已声明）。实测该延迟中位 0.56 ms
             * / p90 0.61 ms（VER=96 的 159-160 两列，10 ns 计数），2000 dps 下就是 1.1
             * 度固定滞后不被补偿。这里把快照按 (tk-ts) 往回推 w*age，s_mag_dth 的弧长
             * 从而覆盖完整的 DRDY->当前帧。 */
            if (ts != 0ULL && tk > ts) {
                uint64_t a10 = tk - ts;
                if (a10 > V5F_MAG_RDAGE_MAX_TICK) a10 = V5F_MAG_RDAGE_MAX_TICK;
                ags = (float)a10 * 1e-8f;
            }
            for (i = 0u; i < 3u; i++) s_w_snap[i] = s_w_int[i] - w[i] * ags;
        }
        s_mag_age_ms = (ts != 0ULL && tk > ts) ? (float)(tk - ts) * 1e-5f : 0.0f;
    }
    /* ★VER=84 倾角参考失效累计（每帧），驱动磁暂停门。
     * 门信号取上一帧算好的 gate->ekf_tilt，0.3s 的门不在乎一帧滞后。 */
    s_grav_ok = gate->ekf_tilt;
    /* VER=123 地磁两入口切换：观测帧之间的 DRDY 帧计数（门控降采样源） */
    s_ff_n++;
    if (gate->ekf_tilt) s_ff_ok++;
    if (gate->ekf_tilt) s_tilt_bad_s = 0.0f;
    else if (s_tilt_bad_s < 10.0f) s_tilt_bad_s += dt;
    s_tilt_inv_ms = s_tilt_bad_s * 1e3f;
    /* ★VER=95 解耦：磁的暂停不再直接跟 gate->ekf_tilt（它被加计污染）。
     *   只在"原本的倾角观测**本可用**（acc_valid 且未削顶 且 |a|^2 在容差内）而倾角门仍拒"
     *   时累计 —— 那才是姿态倾角真的有问题；加计不可用（运动/冲击/削顶）时不累计，
     *   磁的可用性交回场判据（模长/沿重力分量/磁倾/水平分量）+ 连续 1 s 有效才开门。
     *   含义变化：磁在运动中可继续修正，而不再等到停下后才一次性灌入（实测停后 0.5 s
     *   曾灌入 59 度而运动期只有 17 度）。 */
    /* VER=103: 原 s_mag_tilt_bad_s 累计块已删除（它只服务于"运动中禁磁"那条 hold）。 */
    for (i = 0u; i < 3u; i++) {
        if (i == 0u) s_wmag = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);   /* VER=118 */
        s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;
        s_dvb[i] += f[i] * dt;
        s_w_int[i] += w[i] * dt;
        s_mag_dth[i] = s_w_int[i] - s_w_snap[i];
    }
    /* ★ 姿态增量**逐帧复合成四元数**，而不是把轴角矢量相加。
     *   相加只在单轴转动时等于复合；三轴同时转动时丢掉交换子项，表现为绕第三轴的
     *   虚假转动（偏航误差），即一阶圆锥误差 —— 实测"三轴同时转偏航极大误差"，
     *   而单轴转动完全正常。旧链 8 kHz 下每帧复合一次，所以没有这个误差。
     *   用精确半角（sin/cos），不用一阶近似；顺序：机体增量在右侧、按时间右乘。 */
    {
        float wx = w[0] - s_x[IX_BG], wy = w[1] - s_x[IX_BG + 1], wz = w[2] - s_x[IX_BG + 2];
        float dqf[4], qt[4], th2 = (wx*wx + wy*wy + wz*wz) * dt * dt;
        if (th2 > 1e-12f) {
            float th = sqrtf(th2), h = 0.5f * th, sc = sinf(h) / th;
            dqf[0] = cosf(h);
            dqf[1] = sc * wx * dt;
            dqf[2] = sc * wy * dt;
            dqf[3] = sc * wz * dt;
        } else {
            dqf[0] = 1.0f;
            dqf[1] = 0.5f * wx * dt;
            dqf[2] = 0.5f * wy * dt;
            dqf[3] = 0.5f * wz * dt;
        }
        q_norm(dqf);
        q_mul(s_dq, dqf, qt);
        for (i = 0u; i < 4u; i++) s_dq[i] = qt[i];
        q_norm(s_dq);
    }
    s_dt_e += dt;

    if (s_stage < V5F_EKF_DECIM) {
        /* 协方差：本帧传播一行。16 行正好摊在 stage 0..15 这 16 帧里，
         * 到 stage 16 时已经传播完毕，随后 7 帧做观测更新与收尾。 */
        /* ★VER=44 修正自锁：EKF_N(17) > V5F_EKF_DECIM(16)。
         * 原来 16 帧最多推 16 行 -> s_prop_row 永远到不了 EKF_N
         * -> s_prop_ok 恒 0 -> 所有观测(M6/M5/M7/M1M2/M4)与 ekf_finalize()
         *   全部被跳过，EKF 退化成纯陀螺积分器
         *   （实测 VER=44/47/48：mag_r 恒 0、mag_used 0%、chi2 100%、
         *    gate_bits 只有 aligned|step|chi2）。这就是地磁不启动的真因。
         * 前 15 帧仍各推 1 行（ISR 负载均摊），最后一帧把余下的行补完。*/
        if (s_F_ok) {
            if (s_stage == V5F_EKF_DECIM - 1u) {
                while (s_prop_row < EKF_N) ekf_prop_row();
            } else if (s_prop_row < EKF_N) {
                ekf_prop_row();
            }
        }
        s_stage++;
    } else {
        switch (s_stage) {
        case 16u:
            /* 周期末：先把标称状态推进到本周期末端、并建好下一周期的 F。
             * ★ 必须**在观测更新之前**推进：P 已经在上面 16 帧里传播完了
             *   （P_k|k-1 = F P_k-1|k-1 F' + Q），状态若留到更新之后再推进，
             *   状态与协方差就会错开一个周期。 */
            /* ---- VER=123 重力矢量误差估计（每 EKF 观测帧一次）----------------------
             * 门控判断用两观测帧之间的 DRDY 帧降采样：多数有重力牵引 -> 算"有牵引"。
             * 有牵引：重力正在修倾角 -> 比例衰减（位移，无乘法/无溢出）；
             * 无牵引：累加陀螺自己的误差率（1000 ppm*|w| + 2 deg/h）。
             * 饱和加：到 u32 顶停在最大值，不回绕。 */
            {
                float dtw = s_t_run - s_ff_t;
                s_ff_t = s_t_run;
                if (s_ff_n > 0u) {
                    if (s_ff_ok * 2u >= s_ff_n) {
                        s_grav_err -= (s_grav_err >> V5F_EKF_MAG_FF_SHIFT);
                    } else {
                        float inc = (V5F_EKF_GYRO_KS_YAW * s_wmag * RAD2DEG
                                     + V5F_EKF_GYRO_DRIFT_DPS) * dtw * 1.0e6f;
                        uint32_t a = (inc > 0.0f) ? (uint32_t)inc : 0u;
                        if (s_grav_err <= (V5F_EKF_MAG_FF_MAX - a)) {
                            s_grav_err += a;
                        } else {
                            s_grav_err = V5F_EKF_MAG_FF_MAX;   /* 饱和，不回绕 */
                        }
                    }
                    s_ff_n = 0u; s_ff_ok = 0u;
                }
            }
            s_gate_bits = 0u;                  /* 本周期观测记账从这里开始 */
            if (s_sat) { s_gate_bits |= V5F_EKF_GB_SAT; s_sat = 0u; }
            s_chi2_soft = 0u;
            s_mag_used = 0u; s_mag_mode = 0.0f;   /* VER=124 入口标志每观测帧清零 */
            s_prop_ok = (uint8_t)(s_F_ok && s_prop_row >= EKF_N);
            if (s_dt_e > 1e-6f) {
                ekf_step_nominal();
            } else {
                s_F_ok = 0u;
            }
            for (i = 0u; i < 3u; i++) { s_dth[i] = 0.0f; s_dvb[i] = 0.0f; }
            s_dq[0] = 1.0f; s_dq[1] = 0.0f; s_dq[2] = 0.0f; s_dq[3] = 0.0f;
            s_dt_e = 0.0f;
            s_pub_row = (uint8_t)s_prop_row;   /* ★VER=48 先快照再清零 */
            s_prop_row = 0u;
            break;
        case 17u:
            if (s_prop_ok) ekf_m6_tilt(h, gate);
            break;
        case 18u:
            if (s_prop_ok) ekf_m5_zupt(gate);
            break;
        case 19u:
            /* 气压按设计降到 6.1 Hz（press_avg 的噪声相关时间就是 W=32 后的 165 ms）。
             * ★ 号志必须**只在真的尝试了观测时**才清。原来无条件在每次 stage 19 清，
             *   而 M3 只在抽取门允许时才执行 -> 绝大多数周期把新样本白清了，
             *   真正轮到 M3 时号志早已是 0。实测 baro 门只剩 0.34%（应约 1.75%），
             *   而 40 cm 抬高在气压里明确可见（h_baro 5 s 差分 +0.477 m）、
             *   p_z 却只动了 +0.111 m —— 观测被饿死，不是观测不准。 */
            if (s_prop_ok && s_baro_wait >= V5F_EKF_BARO_PERIOD_S) {
                s_baro_wait = 0.0f;
                /* ★ 顺序不能反：ekf_m3_baro 第一行就是
                 *   `if (!gate->ekf_baro || !s_baro_new) return;`
                 *   先清号志再调用 = 每次都在第一行 return。
                 *   VER=26~28 实测 baro 门恒 0.00%、差分观测从未生效，
                 *   就是这两行写反了。 */
                ekf_m3_baro(gate);
                s_baro_new = 0u;               /* 调用之后再消费 */
            }
            if (s_prop_ok) ekf_m7_mag(h, gate);
            break;
        case 20u:
            if (s_prop_ok) ekf_m1m2_gps(h, gate);
            break;
        case 21u:
            if (s_prop_ok) ekf_m4_vel(h, gate);
            s_gps_new = 0u;
            break;
        default:                               /* 22：收尾 */
            if (s_prop_ok) ekf_finalize();
            /* 没有 ENU 原点（= 没有 GPS 高度 M2）时，b_baro 没有任何观测者。
             * 差分观测的 H 不含它，但它仍可能被协方差交叉项间接拖动。既然没人
             * 观测，就每个周期把它压回 0 并清掉它的协方差行/列，绝不让它偷偷长成
             * 一个假的绝对基准（VER=27 实测曾漂到 -116 m）。 */
            if (!s_origin_ok) {
                uint32_t k3;
                s_x[IX_BB] = 0.0f;
                for (k3 = 0u; k3 < EKF_N; k3++) {
                    s_P[k3][15] = 0.0f;
                    s_P[15][k3] = 0.0f;
                    s_Pn[k3][15] = 0.0f;
                    s_Pn[15][k3] = 0.0f;
                }
                s_P[15][15]  = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
                s_Pn[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
            }
            if (s_aligned)   s_gate_bits |= V5F_EKF_GB_ALIGN;
            if (s_origin_ok) s_gate_bits |= V5F_EKF_GB_ORIGIN;
            if (s_chi2_soft) s_gate_bits |= V5F_EKF_GB_CHI2;   /* 本周期有观测被软加权 */
            if (s_gate_bits & ~(V5F_EKF_GB_ALIGN | V5F_EKF_GB_ORIGIN)) {
                s_gate_bits |= V5F_EKF_GB_STEP;
            }
            s_pub_stage = (uint8_t)s_stage;   /* ★VER=48 先快照再清零 */
            s_stage = 0u;
            ekf_publish(h);
            break;
        }
        if (s_stage != 0u) s_stage++;
    }

    /* ---------------- 每帧收尾 ---------------- */
    s_pos_wait  += dt;
    s_alt_wait  += dt;
    s_baro_wait += dt;
    s_boot_t += dt;      /* ★VER=62 开机计时 */
    /* 多普勒直流偏置一阶牵引：门 = 上一帧的 ZUPT 门（外生），与陀螺零偏牵引同构 */
    if (gate->ekf_zupt && h->gps_rmc.status == (uint8_t)'A') {
        float a = dt / V5F_EKF_DOP_TAU_S;
        s_b_dop += (h->gps_rmc.speed_mps - s_b_dop) * a;
    }
    return V5F_PROC_OK;
}

/* ===========================================================================
 * 第 8 步：EKF 各观测的**外部门控**（下游写、上游下一帧读）
 * ========================================================================= */
uint8_t v5f_proc_ekf_gate(volatile v5f_hold_t *h, volatile v5f_proc_gate_t *gate)
{
    static uint32_t s_gga_cnt_last, s_rmc_cnt_last, s_bmp_cnt_last;
    static float    s_press_last;
    static uint8_t  s_started;
    uint32_t gga_cnt, rmc_cnt, bmp_cnt;
    uint8_t  gq_ok, gn_ok, snr_ok, vd_ok, amag_ok;
    float    spac, am2;

    if (!V5F_EKF_EN) return V5F_PROC_OK;

    gga_cnt = g_shm ? g_shm->gps_gga.hdr.cnt : 0u;
    rmc_cnt = g_shm ? g_shm->gps_rmc.hdr.cnt : 0u;
    bmp_cnt = g_shm ? g_shm->bmp.hdr.cnt     : 0u;
    if (!s_started) {
        s_started = 1u;
        s_gga_cnt_last = gga_cnt;
        s_rmc_cnt_last = rmc_cnt;
        s_bmp_cnt_last = bmp_cnt;
    }
    if (gga_cnt != s_gga_cnt_last) { s_gps_new = 1u; s_gga_cnt_last = gga_cnt; }
    if (rmc_cnt != s_rmc_cnt_last) { s_rmc_cnt_last = rmc_cnt; }
    /* 气压：★ 实测 baro.hdr.cnt 与 ist.hdr.cnt **是同一个事件计数器**（恒差 1），
     * 所以它实际按磁力计的 ~187 Hz 递增，不代表气压有新样本。因此这里再加一道
     * "压力值真的变了"的判据，避免 187 次/秒地重复喂同一个旧值。 */
    if (bmp_cnt != s_bmp_cnt_last) {
        float p = h->baro.press_avg_pascal;
        s_bmp_cnt_last = bmp_cnt;
        if (p != s_press_last) {
            s_press_last = p;
            if (p > V5F_EKF_BARO_PA_LO && p < V5F_EKF_BARO_PA_HI) {
                float hnew = V5F_EKF_BARO_M_PER_PA * (V5F_EKF_BARO_PA0 - p);
                /* ★ 这里原来是**死判据**：s_baro_h_last 只在跳变小时更新，而真正被使用的
                 *   s_baro_h 无条件赋值 —— 等于 |dh| 门从来没有拒绝过任何样本。
                 *   现在真的拒绝：跳变或温差超限就整帧丢弃（不更新 s_baro_h、不置号志）。 */
                float dth = fabsf(h->baro.temp_celsius - s_baro_t_last);
                if (s_baro_seen
                    && (fabsf(hnew - s_baro_h_last) >= V5F_EKF_BARO_STEP_MAX_M
                        || (s_baro_t_seen && dth > V5F_EKF_BARO_DT_MAX_C))) {
                    /* 野值 / 温度跳变：丢本样本 */
                } else {
                    s_baro_h_last = hnew;
                    s_baro_h = hnew;
                    s_baro_seen = 1u;
                    s_baro_new  = 1u;
                }
                s_baro_t_last = h->baro.temp_celsius;
                s_baro_t_seen = 1u;
            }
        }
    }

    /* ★ 本项目的铁律"判空查位、勿用值判空"：每个定标字段都要先查它对应的 flags 位
     *   再信它的值。原来 sat_num / hdop 直接用值判 —— 字段一旦没解析出来就是 0，
     *   门会**永久关闭**，GPS 整路静默失效（与 fixq>=2 那类"永远打不开的门"同型）。
     *   现在条件挂在 flags 上：没这个字段就不拿它当判据。
     * ★ fixq >= 1 是**有意偏离设计**（设计写 {2,4}）：近场全部是单点定位 fixq=1，
     *   照设计写就永远打不开；R 已经按 4 m（实测 2D RMS 4.24~5.97）给足了。 */
    gq_ok  = (uint8_t)((h->gps_gga.fix_quality >= 1u)
                    && ((!(h->gps_gga.fresh.flags & SHM_GGA_SV))
                        || (h->gps_gga.sat_num >= V5F_EKF_SV_MIN))
                    && ((!(h->gps_gga.fresh.flags & SHM_GGA_HDOP))
                        || (h->gps_gga.hdop <= V5F_EKF_HDOP_MAX)));
    snr_ok = (uint8_t)((h->gps_gsv.snr_n == 0u)
                    || (h->gps_gsv.snr_avg_dbhz >= V5F_EKF_SNR_MIN_DBHZ));
    gn_ok  = (uint8_t)(h->gps_rmc.status == (uint8_t)'A');
    vd_ok  = (uint8_t)((!(h->gps_gsa.fresh.flags & SHM_GSA_DOP))
                    || (h->gps_gsa.vdop <= V5F_EKF_VDOP_MAX));
    spac   = h->gps_rmc.speed_mps;

    gate->ekf_gps_pos = (uint8_t)(gq_ok && snr_ok && gn_ok
                        && (h->gps_gga.fresh.flags & SHM_GGA_QUALITY)
                        && (h->gps_rmc.fresh.flags & SHM_RMC_POS));
    gate->ekf_gps_alt = (uint8_t)(gate->ekf_gps_pos && vd_ok
                        && (h->gps_gga.fresh.flags & SHM_GGA_ALT));
    /* M4 **不需要**降频：多普勒速度是每个历元独立解算的白噪声（与位置那种
     * 5 s 相关的慢游走不同），实测扣掉直流偏置后 std 0.15 m/s，R=(0.15 m/s)^2 成立。
     * 这条与 M1/M2/M3 的处理不同是**有依据的**，别为了"统一"去给它加降频。 */
    gate->ekf_gps_vel = (uint8_t)(gq_ok && gn_ok && snr_ok
                        && (h->gps_rmc.fresh.flags & SHM_RMC_SPEED)
                        && (h->gps_rmc.fresh.flags & SHM_RMC_COURSE)
                        && (spac > V5F_EKF_VEL_SPEED_MIN_MPS));
    gate->ekf_baro = (uint8_t)(s_t_run >= V5F_EKF_BARO_WARMUP_S
                    && h->baro.temp_celsius > -40.0f
                    && h->baro.temp_celsius < 85.0f);
    am2 = h->imu.accel_g[0]*h->imu.accel_g[0]
        + h->imu.accel_g[1]*h->imu.accel_g[1]
        + h->imu.accel_g[2]*h->imu.accel_g[2];
    /* 加计削顶：|LSB| 到满量程 -> 该帧比力是错的，M6 不许用，且要留下证据 */
    {
        uint8_t sat = 0u;
        uint32_t k;
        for (k = 0u; k < 3u; k++) {
            int32_t v = (int32_t)h->imu.accel_lsb[k];
            if (v >= 32000 || v <= -32000) sat = 1u;
        }
        if (sat) s_sat = 1u;
        /* ★ 与设计的一处**有意偏离**：设计写"长窗净 |w| < 2 dps"（256 ms 窗），
         *   这里用的是 stat.level_dps（128 帧 ≈ 16 ms）。短窗其实更对 —— 要排除转动
         *   是因为离心项 a_c = w^2*r 会淹没重力方向，而 a_c 只在高 w 时才显著，
         *   16 ms 窗抓的正是它；慢转时 a_c 可忽略，不该因此关掉倾斜观测。
         *   独立字段：与 gate->att_tilt 同源判据但各自成位，改一个不动另一个。
         * ★VER=89：门限由 2 dps 改为 V5F_EKF_TILT_WMAX_DPS(61.2 dps，物理预算)，
         *   同时把 a_c 项按 |w| 写进 M6 的 R（见 ekf_m6_tilt）——门只负责粗判，
         *   带宽从**诚实的 R** 出，不再用收紧门限来回避离心项。 */
        gate->ekf_tilt = (uint8_t)(!sat && h->imu.acc_valid
                        && (fabsf(am2 - 1.0f) < V5F_EKF_TILT_AMAG_TOL)
                        /* ★VER=91 回退到原设计的 2 dps（VER=89 曾放宽到 WMAX=61.2 dps）。
                         *   原因：手持运动时加计的误差主项是**线加速度的切向分量**，它**不改变**
                         *   |a| -> 上面那条 |am2-1|<0.06 门对它完全不可见（|a|=1.03 g 时切向可达
                         *   0.25 g ≈ 14 度倾角误差）。放宽门等于让这类被污染的观测进倾角更新，
                         *   实测运动姿态与"运动后漂"因此变差。R 里那个 a_c 项只覆盖离心项，不够。
                         *   代价（已知并接受）：磁更新会像原来一样在运动中常被 s_tilt_bad_s 暂停
                         *   —— 那与"倾角门"是**两个**问题，应另行决定是否解耦，不在这里混。 */
                        && ((h->stat.level_dps < V5F_EKF_TILT_LEV_DPS) || h->stat.ac_bypass));
    }
    /* ★ 门用 mag.ok（只查 |y| 模长一致性 = 外部磁干扰），**不用** mag.trust。
     *   trust 里那个 |a_lin| < 0.05 g 是给旧链加的：旧链的 Bw/psi_true 由旧链姿态算出，
     *   倾角不可信时它们不可信。EKF 的 M7 用的是 mag.f（机体磁场，纯传感器量）
     *   配自己的 q_hat，加速度不污染磁场测量 —— 那个条件对 M7 无效，
     *   而且恰好在陀螺被整流误差污染时把偏航基准关掉（实测剧烈段只开 0.3%）。
     *   运动带来的两条真实误差走 R：倾角耦合 + 样本陈旧（在 ekf_m7_mag 里处理）。 */
    {
        /* ★VER=90 基线**恒定固化在固件里**（= 1.0），彻底不做自愈。
         *   标定把 A/C 归一化成 |f| = 1（见 V5F_MAG_A_INIT），所以安装正确时 |f| 恒为 1；
         *   场次硬铁变化意味着**标定不再匹配安装**，正确做法是重标，而不是让门跟着漂。
         *   旧版自愈（偏离<门限时慢跟随 + 持续偏离超过 V5F_EKF_MAG_NORM_REBASE
         *   ≈2.9 s 就把基线重定到当前值）本意是跟场次常数偏差，但外部铁磁干扰只要
         *   持续 3 s 就会被吸成新基线 -> dev 归零 -> 门重开。
         *   实测 VER=89 夹具 90/180 采集：t=19.0~21.5 |f|=1.52~1.80、磁场方向偏 +17.5 度，
         *   t=24~28.5 |f|=1.11、方向偏 -20 度，两段 mag_used 仍开 46~54%
         *   （|f| 本来就在 col45 mag.mag_norm 上报着），EKF 偏航被拽 17~20 度且**不恢复**
         *   —— 这就是"运动后漂"。改成只与常数 1.0 比以后，上述各段全部关门，
         *   偏航交回陀螺；若整场都开不了门（mag_used=0% 且 col45 明显偏离 1），
         *   说明该安装需要**重新标定**，而不是让固件自己漂。 */
        float dev, mnow = h->mag.mag_norm;
        dev = fabsf(mnow - 1.0f);
        s_mn_ok = (uint8_t)(dev < V5F_MAG_ERR_LIM);
        /* ---- VER=126 地磁失效门（陀螺一致性）：外部磁干扰不许造成"假运动" -----------
         * 实测：设备静止时磁航向路径能跑到 1683°/30 s（瞬时 p99 340 dps），地磁把这股
         * 假运动当真实转动灌进姿态（EKF 偏航被拽 -62.5°）。判据见 v5f_tune.h。
         * 门限按转速放宽 + 削顶/高转速护栏，全部依据 2026-09-22 的新录像。 */
        {
            float wm = sqrtf(h->imu.gyro_dps[0] * h->imu.gyro_dps[0]
                           + h->imu.gyro_dps[1] * h->imu.gyro_dps[1]
                           + h->imu.gyro_dps[2] * h->imu.gyro_dps[2]);
            int32_t lx = h->imu.gyro_lsb[0], ly = h->imu.gyro_lsb[1], lz = h->imu.gyro_lsb[2];
            uint8_t clipped = (uint8_t)((lx >= V5F_MAG_CLIP_LSB || lx <= -V5F_MAG_CLIP_LSB
                                      || ly >= V5F_MAG_CLIP_LSB || ly <= -V5F_MAG_CLIP_LSB
                                      || lz >= V5F_MAG_CLIP_LSB || lz <= -V5F_MAG_CLIP_LSB) ? 1u : 0u);
            s_md_psim += wrap_pi((h->mag.psi_true_deg - s_md_psi) * DEG2RAD);
            s_md_psig += h->imu.gyro_dps[2] * V5F_MAG_DIST_DT_S * DEG2RAD;
            s_md_psi = h->mag.psi_true_deg;
            s_md_n++;
            if (s_md_n >= (uint16_t)V5F_MAG_DIST_WIN_N) {
                float rate = fabsf(s_md_psim - s_md_psig) / V5F_MAG_DIST_WIN_S * RAD2DEG;
                float thr = V5F_MAG_DIST_THR_DPS + V5F_MAG_DIST_KS * wm;   /* VER=127: KS=0 -> 固定 20 dps */
                if (!clipped && wm < V5F_MAG_DIST_WMAX_DPS && rate > thr) {
                    s_md_hold = (uint32_t)V5F_MAG_DIST_HOLD_N;
                }
                s_md_psim = 0.0f; s_md_psig = 0.0f; s_md_n = 0u;
            }
            if (s_md_hold > 0u) { s_md_hold--; s_mag_dist = 1u; } else { s_mag_dist = 0u; }
        }
        gate->ekf_mag_yaw = (uint8_t)(s_mn_ok && (V5F_EKF_YAW_OBS_EN != 0u)
                                      && (s_mag_dist == 0u));   /* VER=126 加干扰门 */
        s_mag_gate = gate->ekf_mag_yaw;      /* 上报：门到底开没开 */
    }
    /* ★ 无 GPS 时 speed 那一路被 flags 旁路掉，门退化成纯 is_static —— 而等效原理下
     *   匀速平动对陀螺与加计都不可见（实测走动时 is_static 可 100% 成立）。
     *   追加**外生**的交流能量判据 e_ac（三轴滑窗方差之和，对直流零偏不可见）：
     *   实测静止 0.048~0.057、运动 0.37~84，门限 0.20。它只能让门更保守。 */
    /* | |a_离线校正| - 1 |：**外生**（原始 LSB + 离线标定常量），且与姿态无关 ——
     * 静止时比力模长恒为 1 g。原来用 a_lin，那是旧链牵引环的在线残差，而那个环自己
     * 的判据就是 |a_lin|，实测卡在极限环里（静置 |a_lin| p50 0.0487 g），
     * ZUPT 门因此只开 52.7%。 */
    {
        static const float b0[3] = { V5F_ACCEL_BIAS_LSB_X, V5F_ACCEL_BIAS_LSB_Y,
                                     V5F_ACCEL_BIAS_LSB_Z };
        float s2 = 0.0f;
        uint32_t k;
        for (k = 0u; k < 3u; k++) {
            float v = ((float)h->imu.accel_lsb[k] - b0[k]) / g_v5f_accel_lsb_per_g[k];
            s2 += v * v;
        }
        amag_ok = (uint8_t)(h->imu.acc_valid
                            && (fabsf(sqrtf(s2) - 1.0f) < V5F_EKF_ZUPT_AMAG_TOL));
    }
    /* ★ 不再要 is_static。实测（VER=22）静止段 is_static 只有 6~8%，而
     *   e_ac<0.2 是 100%、||a_off|-1|<3% 是 99.9% —— 它是唯一的拦路虎。
     *   它的判据是**陀螺直流电平**，剧烈运动后在线牵引冻结在错值上 -> 静止时
     *   DC 仍超阈。后果致命：3.87 g 冲击把速度积到 6.8 m/s，随后静止段 ZUPT 不开
     *   -> 速度不下来 -> Δp 漂 8.95 m、|p_h| 到 15.7 m（而真值是 10×20 cm）。
     *   丢掉它**不损失能力**：直流判据对平滑平移同样瞎（项目实测"60 s 匀速走动
     *   is_static 100%"），真正对平移有效的是 e_ac 与 ||a|-1|，两条都已在门里。 */
    gate->ekf_zupt = (uint8_t)((h->stat.e_ac < V5F_EKF_ZUPT_EAC_MAX)
                    && amag_ok
                    && ((spac < V5F_EKF_ZUPT_SPEED_MAX_MPS)
                        || !(h->gps_rmc.fresh.flags & SHM_RMC_SPEED)));

    if (!s_origin_ok && gate->ekf_gps_pos && h->gps_rmc.lat_e7 != 0) {
        float lat0 = (float)h->gps_rmc.lat_e7 * 1e-7f;
        s_lat0_e7 = h->gps_rmc.lat_e7;
        s_lon0_e7 = h->gps_rmc.lon_e7;
        s_m_per_deg_lat = V5F_EKF_R_EARTH_M * DEG2RAD;
        s_m_per_deg_lon = V5F_EKF_R_EARTH_M * DEG2RAD * cosf(lat0 * DEG2RAD);
        s_origin_ok = 1u;
        s_x[IX_P + 0u] = 0.0f; s_x[IX_P + 1u] = 0.0f;
        s_x[IX_V + 0u] = 0.0f; s_x[IX_V + 1u] = 0.0f;
        s_gate_bits |= V5F_EKF_GB_ORIGIN;
    }

    return V5F_PROC_OK;
}
