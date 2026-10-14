/* 处理链可调常量：集中一处，改这里就够。
 * 全部数值的实测依据见《地磁特性实验记录.md》与《地磁修正设计.md》。
 * 一律不做运行期 exp/sqrt 现算，表查/移位/比较为主。 */
#ifndef PROC_TUNE_H
#define PROC_TUNE_H

/* ---------------- 器件常量 ---------------- */
/* ★ 陀螺逐轴标度（2026-10-13 应用）
 *
 * 标度误差【不在零偏追踪器的估计范围内】（proc_gyro_bias 只估 s_bias），
 * 所以它原样保留在 gyro_dps 里。实测开环偏航闭合误差 15.048°，
 * 施加本组修正后降到 1.874°（8 倍）。
 *
 * 来源：证据链归档_20261013/03_陀螺标度主源_翻滚A_路径29174.bin
 *       方法：PC 端四元数前缀扫描 + 三维网格细扫，double 内核
 *       交叉验证：05_慢转C 独立给出 x/y 逐位相同 (-0.5030, -0.2400)；
 *                04_翻滚B 给出 (-0.5250, -0.3400, -0.0150)
 *       逐轴分辨框：±0.0050% / ±0.0025% / ±0.0050%
 *       ★ A 的解在换用加计标定前后【一字不变】——鲁棒性已验证
 *
 * 施加方式： gyro_dps = (gyro_lsb - s_bias) * K_轴     （K = 0.140 / (1 + s)）
 * 换算：   x: 0.140/0.994970 = 0.140708    s = -0.5030%
 *          y: 0.140/0.997600 = 0.140337    s = -0.2400%
 *          z: 0.140/0.998850 = 0.140161    s = -0.1150%
 *
 * ★ 为何不能只改主路径：proc_chain.c 有 9 处提前返回分支也做
 *   gyro_dps = gyro_lsb * K，必须同源，否则分支间标度不一致。
 */
#define GYRO_DPS_PER_LSB_X      0.140708f
#define GYRO_DPS_PER_LSB_Y      0.140337f
#define GYRO_DPS_PER_LSB_Z      0.140161f

/* 逐轴取值器（供 for(i=0..2) 循环与显式下标共用） */
static inline float gyro_dps_per_lsb(int i)
{
    return (i == 0) ? GYRO_DPS_PER_LSB_X
         : ((i == 1) ? GYRO_DPS_PER_LSB_Y : GYRO_DPS_PER_LSB_Z);
}

/* MMC5983MA 18 位模式：16384 counts/G，零点 131072 counts（手册） */
#define MAG_CNT_PER_G           16384.0f
#define MAG_ZERO_CNT            131072.0f
/* ★★ 实测常量零点（SET-only 口径，2026-10-14）
 * 由四份好录制的【分相球心】均值定出，跨录制离散仅 0.1~0.3%：
 *   000534 [129642.5 131609.2 131315.8]   235545 [129589.8 131538.6 131220.4]
 *   234141 [129668.4 131645.7 131281.3]   233844 [129708.6 131636.1 131456.6]
 * 与名义零点 MAG_ZERO_CNT(131072) 之差即器件零场偏置（约 0.094 G）。
 * ★ 为什么用【常量】而不逐录制拟合：
 *   · 地磁矢量方向在世界系恒定 ⟹ 本就不需要拟合
 *   · "姿态覆盖不足"的录制（如 000813 猫身上静止）会让逐录制拟合病态：
 *     实测该录制逐录制拟合给出 |B|=0.007 G，换常量零点后 0.514 G、散度 0.228%
 * ★ 原有 MAG_DEV_BIAS_*（0.0423 G）与实测差 2.2 倍，且从未被使用/验证，
 *   本次改用实测常量零点；MAG_DEV_BIAS_* 保留但不再作为零点依据。 */
/* ★★ 2026-10-14 更新：测量模式改为【连续采样(CMM_EN)】后，器件零场偏置发生位移。
 *   实测：新模式下的 SET 相球心两次独立会话互证
 *     rec_20261014_025044（多姿态静止，覆盖 118.5°） = [130430.3 131333.5 132385.0]
 *     rec_20261014_025232（剧烈全姿态，覆盖  70.5°，球残差 1.079%） = [130403.7 131411.5 132252.3]
 *     两者相差仅 156 counts（0.6%）⟹ 高度可复现
 *   旧值（触发模式时代） = [129652.4 131607.4 131318.5]，距新值 1214~1348 counts (15~17%)
 *   证据：EKF 认为的世界系磁方向 W=R(q)·z 漂移末态残留 5.33°，
 *         且用户实测"大运动回到原位后偏航残留 ~7°"，与几何估算 atan(1348/9800)=7.8° 吻合。
 *   取两会话均值（下方三行）。旧值保留在注释里以便回退。 */
#define MAG_ZERO_X_CNT          130417.0f   /* 原 129652.4f */
#define MAG_ZERO_Y_CNT          131373.0f   /* 原 131607.4f */
#define MAG_ZERO_Z_CNT          132319.0f   /* 原 131318.5f */
/* 器件零场偏置模长：本次实测 0.04233 G
 * 依据：六面标定面间散布 0.1955%，跨上电 0.07%，与姿态无关 */
#define MAG_DEV_BIAS_G          0.0423f
/* ★ 2026-10-14：器件零场偏置的体坐标方向（实测值，暂不参与计算）。
 *   原为 #define MAG_DEV_BIAS_UX/UY/UZ，唯一使用者是 proc_mag.c 的 s_bias_u[3]，
 *   而 s_bias_u 全仓 0 处引用 ⟹ 常量随之成为死定义，故改为注释保留。
 *   现有实现按【逐相球心】扣零点，未做逐轴偏置方向扣除。
 *   若将来启用逐轴扣除，恢复下面三行并在 proc_mag.c 重建 s_bias_u：
 *     #define MAG_DEV_BIAS_UX         0.52813f
 *     #define MAG_DEV_BIAS_UY        -0.34223f
 *     #define MAG_DEV_BIAS_UZ         0.77715f
 */

/* LSM6DSV16X ±4000 dps 档：0.140 dps/LSB（本板源码 + 手册） */
#define GYRO_DPS_PER_LSB        0.140f
/* ±2g 档：0.061 mg/LSB */
#define ACCEL_G_PER_LSB         0.000488f  /* ±16 g: 0.488 mg/LSB */

/* ---------------- 陀螺零偏追踪 ---------------- */
#define GB_TAU_WORK_TICKS       1200000000ULL   /* tau_work = 12 s（10 ns 计数） */
#define GB_TAU_BOOT_TICKS       100000000ULL    /* tau_boot = 1 s */
#define GB_BOOT_TICKS           (3ULL * GB_TAU_BOOT_TICKS)
#define GB_SNAP_TICKS           2000000ULL      /* 快照粒度 20 ms */
#define GB_HIST_DEPTH           64u             /* 快照深度 = 1.28 s */
#define GB_DT_MAX_TICKS         20000000ULL     /* 单帧 dt 上限 200 ms */
#define GB_ALPHA_WORK_TICK      (1.0f / (float)GB_TAU_WORK_TICKS)
#define GB_ALPHA_BOOT_TICK      (1.0f / (float)GB_TAU_BOOT_TICKS)
/* 零偏初值（counts）：上电即接近零点，避免启动偏置被误判成运动 */
#define GB_BIAS0_LSB_X          (-5.0f)
#define GB_BIAS0_LSB_Y          (-2.0f)
#define GB_BIAS0_LSB_Z          (3.0f)

/* ---------------- 动静判定 ---------------- */
#define DET_W                   122u    /* 判静滑窗帧数（= 128 × 124.66us / 130.3us） */
#define DET_DEB_ON              8u      /* 连续 8 帧超阈 -> 判运动（约 1 ms） */
#define DET_DEB_OFF             800u    /* 连续 800 帧低阈 -> 判静止（约 100 ms） */
#define DET_THR_BASE            0.108f  /* 10 × sigma_s，实测 */
#define DET_THR_OFF_RATIO       0.6f    /* 回判静止阈值 = 0.6 × 当前阈值（迟滞） */
#define DET_THR_NBUCK           20u     /* 阈值表档数 */
#define DET_THR_SHIFT           7u      /* 证据粒度 -> 查表下标 */
#define DET_AC_E_ON             0.20f   /* 交流能量上限 dps^2 */
#define DET_AC_E_TOT            4.00f   /* 总能量上限 dps^2 */
#define DET_AC_DEB_ON           8u
#define DET_ROLLBACK_GRAN       24u     /* 静止->运动边沿回退的粒度数（480 ms） */

/* ---------------- 协方差标量门控 ---------------- */
#define PG_FLOOR_DEG            0.005f  /* 有重力时残余不确定度下限 */
#define PG_CEIL_DEG             2.000f
#define PG_THR_DEG              0.30f   /* 阈值：0.6 × 地磁典型误差 0.5 度 */
#define PG_DECAY                0.00048828f
#define SG_PPM                  0.000246f
#define W_FLOOR_DPS             40.0f   /* 膨胀率的角速度下限 */
#define INFL                    1.0f    /* 膨胀倍数：1.0 = 物理值 */

/* ---------------- 磁修正 ---------------- */
/* ★ 增益上限的来历（实测约束，不是拍的）:
 *   本板单个磁样本的【方向】噪声约 4~8 度（判据: m·g 的 std = 0.073，
 *   换算方向噪声 0.073/sqrt(1-0.84^2) = 0.134 rad = 7.7 度；逐对 |B| std 8.5%）。
 *   修正逐帧施加（7680 Hz），若 K_MAX 取 0.8（准直连），
 *   每帧把 7.7 度的方向噪声灌进姿态 -> 四元数肉眼可见抖动。
 *   K 的含义是"每帧吃掉几成误差"，闭环时间常数 tau = dt/K：
 *     K=0.80 -> tau = 0.13 ms（噪声直通）
 *     K=0.05 -> tau = 2.6 ms
 *     K=0.02 -> tau = 6.5 ms（推荐：噪声被 1/sqrt(7680*K) 充分平均）
 *   取 0.02。 */
#define CORR_K_MIN              0.0f
#define CORR_K_MAX              0.0f
/* 重力环增益：K_GRAV = 每帧吃掉几成倾角误差，闭环时间常数 tau = dt/K
 *   7680 Hz 下:  K=0.02  -> tau = 6.5 ms  带宽 24 Hz   ← 噪声直通，实测抖
 *                K=0.002 -> tau = 65 ms   带宽 2.4 Hz  ← 推荐
 *                K=0.001 -> tau = 130 ms  带宽 1.2 Hz  ← v2 稳态量级
 * 加计方向噪声 0.17 度（逐帧，实测），必须靠限带宽压掉。
 * v2 proc_ekf.c 的教训原文：「K=0.5度的R 配 349 Hz 满增益，闭环带宽约 100 Hz，
 *   加计的方向噪声几乎原样进姿态 —— 实测 ekf_q 全程抖」。
 * 重力环只拉倾斜（roll/pitch），不碰偏航。
 * ★ 加计测的是【比力】，静止时朝上（+up）；本环即用它拉倾斜。 */
#define CORR_K_GRAV             0.002f

/* 重力环转轴符号 GRAV_CORR_SIGN。已实测：+1.0 与 -1.0 都不对，先回滚为 +1.0。
 * 推导上 cr = pb x av 已是正确转轴（把估计的 up 转向实测 a）。
 * 若实测表现为「倾斜后被拉向反方向」，把这里改成 -1.0f 即可，无需改结构。
 * 判据：
 *   静止放置   -> q 的倾角应收敛到真实倾角且不抖
 *   缓慢倾斜   -> 姿态应跟随倾斜方向
 *   松手回平   -> 姿态应回到水平
 */
#define GRAV_CORR_SIGN          1.0f
#define MAG_MIN_PERP            0.02f   /* 水平投影退化下限（G），低于则不修 */
#define MAG_MIN_UNIT            0.10f   /* 单位矢量的模长下限（G）：低于此视为无效，保持上一帧 */
#define MAG_ACC_LIMIT_G         0.03f   /* |a| 偏离 1g 超过此值视为不可信 */

/* ---------------- MMC 卡死检测 ---------------- */
#define MAG_STALL_FRAMES        1536u   /* 连续无新磁数据帧数上限（约 200 ms @7.68kHz） */

/* ---------------- 参考矢量（★ 由录制数据定出，不抄 v2） ----------------
 * 导航系 = 真 ENU：x = 东, y = 北, z = 上（与 v2 proc_ekf.c 第 12 行同约定）
 * 所以磁北的水平分量必须落在【+y 轴】；放在 +x 会让偏航整体偏 90 度。
 *
 * 提取方法（只用录制数据，不含任何假设的 INC/D 值）:
 *   m_i·g_i = b0·z_nav = b0_z  必须恒定（m_i 体坐标磁场, g_i 体坐标重力）
 *   seg45 静止样本 35467 个(75.8%)，实测 m·g 中位 = +0.842180
 *   ⟹ b0_z = +0.842180，等效磁倾角 asin(0.84218) = 57.3711°
 *   ⟹ b0 = (0, +0.53920, +0.84218)
 *
 * ★ 对照: v2 的值 (0.60880, 0, -0.79333) 隐含 b0_z = -0.79333、磁倾角 52.5°，
 *   水平轴在 x、竖直分量为负 —— 与本板实测【符号相反】，曾导致四元数整 90° 翻转。
 *   物理不同，v2 的参考矢量不能借用。
 *
 * 注: m·g 的 std 0.073 反映残余方向噪声与姿态抖动；中位即最优参考。
 *     水平方位是任意常数，被初始对准的偏航一次性吸收。 */
#define MAG_REF_NX              0.0f
#define MAG_REF_NY              0.53920f
#define MAG_REF_NZ              0.84218f

/* ---------------- 地磁修正方式（验收依据，CDC TYPE7 的 mag_fix 字段）----------------
 * ★ 与 cdc_report.h 的 MAG_FIX_* 必须同值；约定 0=失能 / 1=入口A / 2=入口B */
#define MAG_FIX_OFF             0u      /* 失能：本帧未做地磁修正 */
#define MAG_FIX_ENTRY_A         1u      /* 入口A：重力法平面投影，只修偏航（重力参与） */
#define MAG_FIX_ENTRY_B         2u      /* 入口B：3D 矢量（重力未参与，磁独担倾斜） */

/*==============================================================================
 * justfloat 上报模式（二选一，绝不混发 —— 混发两种流会导致上位机无法解析）
 *============================================================================*/
/* 1 = 只发 ATT 帧（16 float = 64 B @ 100 Hz）
 *      给 justfloat / VOFA+ 上位机画姿态波形：
 *        chan0 qw, chan1 qx, chan2 qy, chan3 qz,
 *        chan4 bias_ok, chan5 grav_err_deg, chan6 grav_applied, chan7 mag_corr_en
 *     带宽 6.4 KB/s。
 *
 * 0 = 只发 RAW 帧（19 float = 76 B @ 7670 Hz）
 *      板上每一个输入采样原样上报（陀螺/加计/磁 counts + DRDY 时间戳），
 *      零换算零滤波零抽点，可逐位复现板上链路。
 *     ★ 这个流 justfloat 上位机不兼容（它不是"每帧一组通道"的姿态流）。
 *     带宽 583 KB/s。
 */
#ifndef JF_ATT_ONLY
#define JF_ATT_ONLY             0
#endif

#endif /* PROC_TUNE_H */
