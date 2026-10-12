/* 纯外部参考姿态：只用【加计(重力) + 磁强计】两个矢量方向，不碰陀螺
 *
 *   导航系 ENU 右手系: x = 东, y = 北, z = 上
 *     up_b    = normalize(a)                          （假定 a = g）
 *     north_b = normalize(m^ - (m^·up_b) up_b)         （m^ = 上游归一好的单位矢量）
 *     east_b  = up_b × north_b
 *     R = [east_b | north_b | up_b]                   体->导航
 *     q = quat(R)
 *   ★ 陀螺零参与 —— 用于在陀螺无置信度时提供绝对姿态参考。
 *   ★ 倾角由加计决定；偏航由磁场水平分量决定。
 *   ★ 器件近水平且 m 水平分量足够大时才有效，否则返回 0。
 */
#include "proc.h"
#include "mipc_shm.h"
#include "proc_tune.h"
#include <math.h>

static float s_q[4];

/* 3x3 -> 四元数 */
static void mat_to_quat(const float R[3][3], float *q)
{
    float tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0f)
    {
        float s = sqrtf(tr + 1.0f) * 2.0f;
        q[0] = 0.25f * s;
        q[1] = (R[2][1] - R[1][2]) / s;
        q[2] = (R[0][2] - R[2][0]) / s;
        q[3] = (R[1][0] - R[0][1]) / s;
    }
    else if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
    {
        float s = sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = 0.25f * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    }
    else if (R[1][1] > R[2][2])
    {
        float s = sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = 0.25f * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    }
    else
    {
        float s = sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = 0.25f * s;
    }
    {
        float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (n > 1e-12f) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
        else { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; }
    }
}

/* 用加计 + 磁强计算姿态，成功返回 1，q[4] 有效 */
uint8_t proc_ag_mag_att(const volatile shared_mem_t *shm, float *q_out)
{
    float a[3], m[3], up[3], north[3], east[3], R[3][3];
    float an, mn, d, nn;
    uint32_t i;

    a[0] = (float)shm->imu.ax * ACCEL_G_PER_LSB;
    a[1] = (float)shm->imu.ay * ACCEL_G_PER_LSB;
    a[2] = (float)shm->imu.az * ACCEL_G_PER_LSB;
    an = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (an < 1e-3f) return 0u;
    up[0] = a[0]/an; up[1] = a[1]/an; up[2] = a[2]/an;

    if (g_proc.mag_valid == 0u) return 0u;
    /* 上游 proc_mag.c 已归一为「单位 + 符号一致 + 方向一致」的数据流，这里直接用 */
    m[0] = g_proc.mag_u[0]; m[1] = g_proc.mag_u[1]; m[2] = g_proc.mag_u[2];
    mn = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    if (mn < 1e-6f) return 0u;
    m[0]/=mn; m[1]/=mn; m[2]/=mn;

    /* north_b = m - (m·up)up */
    d = m[0]*up[0] + m[1]*up[1] + m[2]*up[2];
    north[0] = m[0] - d*up[0];
    north[1] = m[1] - d*up[1];
    north[2] = m[2] - d*up[2];
    nn = sqrtf(north[0]*north[0] + north[1]*north[1] + north[2]*north[2]);
    if (nn < MAG_MIN_PERP) return 0u;          /* 水平分量太小，退化 */
    north[0]/=nn; north[1]/=nn; north[2]/=nn;

    /* east_b = up × north */
    east[0] = up[1]*north[2] - up[2]*north[1];
    east[1] = up[2]*north[0] - up[0]*north[2];
    east[2] = up[0]*north[1] - up[1]*north[0];

    /* R = [east | north | up] */
    for (i = 0u; i < 3u; i++)
    {
        R[i][0] = east[i];
        R[i][1] = north[i];
        R[i][2] = up[i];
    }
    mat_to_quat(R, s_q);
    for (i = 0u; i < 4u; i++) q_out[i] = s_q[i];
    return 1u;
}