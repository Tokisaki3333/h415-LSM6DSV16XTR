/* 处理链对外接口：四元数姿态 + 陀螺零偏追踪 + 两级地磁修正 + 重力环。
 *
 * 数据流（每 IMU 帧，7.68 kHz）：
 *   ① proc_static_detect  吃上一帧 gyro_dps        -> gate.is_static
 *   ② proc_gyro_bias      吃 gate.is_static        -> gyro_dps[] / bias_dps[]
 *   ③ proc_attitude_prop  用 gyro_dps 积分 dt      -> q
 *   ④ proc_mag_corr       吃 gate.Pg / mag 帧      -> 修正 q
 * 磁帧由 V3F 的 mmc 通道给出（SET/RESET 交替，每两拍凑成一对，1040 Hz）。
 *
 * 全部状态为文件静态，仅被 V5F 处理主循环单线程调用，无需加锁。
 * 时间基准统一用 10 ns 计数，与调用次数无关。 */
#ifndef PROC_H
#define PROC_H

#include <stdint.h>
#include "mipc_shm.h"   /* shared_mem_t：mipc_shm.h 用匿名 typedef，无法前置声明 */

/* 门控状态：proc_static_detect 写，proc_gyro_bias / proc_mag_corr 读 */
typedef struct {
    uint8_t is_static;          /* 1 = 静止 */
    uint8_t rollback_granules;  /* 静止->运动边沿要回退的粒度数 */
} proc_gate_t;

/* 处理链全部输出与中间状态 */
typedef struct {
    /* --- 陀螺 --- */
    float   gyro_lsb[3];        /* 本帧原始 counts */
    float   gyro_dps[3];        /* 零偏校正后 dps */
    float   bias_dps[3];        /* 零偏估计 dps */
    uint8_t bias_ok;            /* 有效追踪时长 >= 3*tau_boot，锁存 */

    /* --- 静止判定 --- */
    float   level_dps;          /* 滑窗均值最大轴 */
    uint8_t is_static;
    uint16_t bias_evidence_gran; /* 累积静止证据（粒度数），供阈值表查表 */

    /* --- 姿态 --- */
    float   q[4];               /* 体->导航 四元数 */
    uint8_t q_valid;

    /* --- 门控 --- */
    float   Pg_deg;             /* 协方差标量（度） */
    float   K;                  /* 修正增益 */
    uint8_t entry_B;            /* 1 = 3D 矢量修正，0 = 仅偏航 */
    uint8_t mag_fix;            /* 地磁修正方式：0=失能 1=入口A 2=入口B（验收依据） */

    /* --- 磁 --- */
    float   mag_g[3];           /* 净场（已扣器件零场偏置），G */
    float   mag_u[3];           /* ★ 净场单位矢量：配对后立刻归一，符号按 SET 拍一致 */
    float   mag_norm_g;
    uint8_t mag_valid;          /* 本帧有可用磁数据 */
    uint8_t mag_stall;          /* 磁通道卡死 */
    uint8_t mag_bad_acc;        /* 本帧 |a| 不可信，不做磁修正 */
    uint8_t grav_applied;       /* 本帧重力环是否真的施加了倾斜修正 */
    float   grav_err_deg;       /* 本帧重力环的新息（估计 up 与实测 a 的夹角），度 */
    uint8_t mag_corr_en;        /* ★ 地磁修正总开关：0=完全隔离磁修正（对照实验用） */

    /* --- 时间戳 --- */
    uint64_t tick;              /* 本帧 10 ns 计数 */
} proc_state_t;

extern proc_state_t     g_proc;
extern proc_gate_t      g_gate;

/* 主循环分步接口，按 §数据流 顺序调用 */
void    proc_chain_init(void);
void    proc_chain_step(const volatile shared_mem_t *shm);

/* 纯外部参考姿态（加计+磁强计，零陀螺）：成功返回 1 */
uint8_t proc_ag_mag_att(const volatile shared_mem_t *shm, float *q_out);

/* 取走本秒 EKF 迭代计时统计（供 main.c 的 TYPE8 上报用） */


/* 各步初始化 */
void    proc_mag_init(void);
void    proc_static_detect_init(void);
void    proc_gyro_bias_init(void);
void    proc_attitude_init(void);
void    proc_mag_corr_init(void);

/* ---------------- 陀螺 + 地磁 ESKF（重力牵引关闭）----------------
 * 状态只有三维姿态误差 [dtheta(3)]；角速度零偏由 proc_gyro_bias 负责，
 * ESKF 不再估计、也不再扣除。
 * 启动即 identity：q0=[1,0,0,0]，v_up/v_mag 直接取启动窗口实测均值。 */
void    proc_eskf_init(void);
void    proc_eskf_step(float dt_s, const float *gyro_dps, const float *gyro_lsb,
                       const float *a_g, const float *mag_u, uint8_t mag_valid,
                       uint8_t new_mag, uint64_t t10ns);
void    proc_eskf_get_q(float *q_out);
void    proc_eskf_clear_qhemi(void);
void    proc_eskf_sync_gproc(void);
void    proc_eskf_get_R(float *R_out);
uint8_t proc_eskf_ready(void);
uint8_t proc_eskf_calib_done(void);
uint8_t proc_eskf_valid(void);
uint8_t proc_eskf_grav_open(void);
/* ★ 持久状态上报（PC 侧复刻初始化必需，见 proc_eskf.c 注释） */
float   proc_eskf_acc_thr(void);          /* 自适应重力残差门限 */
float proc_eskf_grav_w(void);
float proc_eskf_grav_nis(void);
float   proc_eskf_calib_n(void);          /* 启动标定累计样本数 */
void    proc_eskf_p_sym(float *out6);     /* P 的 6 个独立分量 */
uint8_t proc_eskf_calibrating(void);
float   proc_eskf_mag_resid(void);
float   proc_eskf_acc_resid(void);
float   proc_eskf_mag_corr(void);
float   proc_eskf_p_perp(void);
float   proc_eskf_p_par(void);

/* 单独可调用（供调试/单测） */
uint8_t proc_static_detect_step(void);
uint8_t proc_gyro_bias_step(void);
uint8_t proc_mag_step(const volatile shared_mem_t *shm);
uint8_t proc_mag_is_new(void);      /* ★ 本帧是否有新的磁拍（EKF 据此门控地磁观测） */
void    proc_attitude_prop(float dt_s);
void    proc_attitude_align(const float *a_g);
void    proc_gate_step(float dt_s);
void    proc_mag_corr_step(const volatile shared_mem_t *shm, float dt_s);

#endif /* PROC_H */
