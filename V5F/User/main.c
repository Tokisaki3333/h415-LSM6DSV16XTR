/*******************************************************************************
 * main.c —— V5F 大核
 * 取共享区 -> 姿态处理链 -> justfloat 上报四元数
 *
 * ★ USB CDC 上【只有】justfloat 裸帧，不混任何 TYPE/SEQ/CRC 包。
 *   printf 走 USART1（WCH-Link 那路），与 CDC 是两条独立物理链路。
 *
 *   justfloat 帧 = 0x7F800000, 0x7F800000, 1.0, seq, qw,qx,qy,qz, mag_fix, grav_fix, gyro_ok
 *   10 个 float32 = 40 B/帧，@ 约 100 Hz（7.68 kHz / 77）
 *
 * ★ 本版姿态 = 陀螺零偏追踪 + 直接积分（修正环已关闭，用于单独检查这条链路）。
 *
 * ★ 地磁隔离实验：每 MAG_ISO_PERIOD_S 秒翻转 g_proc.mag_corr_en
 *   （开 5 s / 关 5 s），通道 6 的 mag_fix 直接标注当前状态，
 *   一眼即可对照"只靠陀螺+重力"与"加入地磁"两种姿态。
 *******************************************************************************/
#include "debug.h"
#include "ch32h417.h"
#include "sys_clk.h"
#include "mipc_v5.h"
#include "board_io.h"
#include "mipc_shm.h"
#include "sensor_dma.h"
#include "USBCDC.h"
#include "justfloat.h"
#include "proc.h"
#include "proc_tune.h"

/* ★ 上报模式：QUAT 帧（4 float 四元数 + 帧尾 2×sync = 24 B），IMU 满速率。
 *   本版唯一上报流 —— 【只上报四元数】，帧长恒定 24 B，
 *   绝不混入任何其它数据。帧尾的两次 00 00 80 7F 是唯一标识。 */
#define ATT_DIV             1u      /* 满速率：7.67 kHz 上报 */

int main(void)
{
    uint32_t last_imu_cnt = 0u;
    uint32_t last_mmc_cnt = 0u;
    uint32_t att_div      = 0u;
    uint32_t skip_cnt     = 0u;      /* 主循环跟丢的采样数（cnt 跳号累计） */
    uint32_t prev_imu_cnt = 0u;
    int32_t  mx = 0, my = 0, mz = 0;
    uint8_t  mpol = 0u;
    uint64_t t_next;
    /* ★ 原始磁 counts 上报用：当前拍 + 上一拍（含各自极性），PC 据此重建 SET/RESET 对 */
    int32_t  m_cur_raw[3]  = {0, 0, 0};
    int32_t  m_prev_raw[3] = {0, 0, 0};
    uint8_t  pol_cur = 0u, pol_prev = 0u;
    uint8_t  mmc_seen = 0u, mmc_prev_valid = 0u;

    SystemAndCoreClockUpdate();
    Delay_Init();

    printf("\r\n[V5F] start (justfloat QUAT 24B)\r\n");

    (void)mipc_v5_get_shared();                         /* 须在 RX DMA 使能前 */

    while (g_shm->v3f_cfg_done == 0u) Delay_Ms(1);
    printf("[V5F] shm=%08X, v3f_cfg_done=%u\r\n",
           (unsigned)(uintptr_t)g_shm, (unsigned)g_shm->v3f_cfg_done);

    SPI_DMA_Init();

    printf("LSM6,init_rc=%d\r\n", BOARD_IMU_Init());
    printf("LSM6,enable_rc=%d\r\n", BOARD_IMU_Enable());
    printf("[V5F] Ready. justfloat = QUAT only (4 float/24B @IMU rate)\r\n");

    proc_chain_init();

    usbhs_hid_enable();

    t_next = GetTime64_Us() + 5000000ULL;

    for (;;)
    {
        /* ---- 排空：必须【每轮】调用，绝不能降频 ----
         * _hid_ep_send_one() 每次只装 1 个包，hid_up_flush() 只调它一次，
         * 于是"调用次数"就是"可发出的包数上限"：
         *   125 次/s × 512 B(HS) = 64 KB/s，远低于需要的 644 KB/s —— 一帧都送不出去。
         * 驱动敏感的是【提交传输的频率】，不是【检查端点是否空闲的频率】；
         * 端点忙时 _hid_ep_send_one() 自己返回 0，天然限流。
         * 所以这里每轮都调，入队节拍交由 jf_agg_poll() 控制。 */
        usbhs_hid_poll();

        /* ---- 入队节拍：<=125 Hz（JF_AGG_PERIOD_US=8ms）----
         * 实测 Win10 侧驱动对【提交 USB 传输的频率】敏感：7670 帧/s 逐帧入队时
         * 丢 27% / 大部分；v2 是 602 B/帧，同样 7670 帧/s 但提交次数只有 1/7，不丢。
         * 这里让暂存区累积到 8 ms 才整块入队一次，把提交次数降到 <=125 次/s。 */
        jf_agg_poll();

        /* ---- 保活（0.2 Hz；printf 逐字符忙等，本行 118 字符 @921600 = 1.28 ms）---- */
        if ((int64_t)(t_next - GetTime64_Us()) <= 0)
        {
            printf("V5F,imu=%u,mmc=%u,bmp=%u,jf_drop=%u,agg_tx=%u,enq_fail=%u,ep_sent=%u,ep_busy=%u,ep_empty=%u,ep_kb=%u,pend=%u,skip=%u,start=%u,lag=%u,mrec=%u,mreinit=%u,mrc=%u\r\n",
                   (unsigned)g_shm->imu.hdr.cnt,
                   (unsigned)g_shm->mmc.hdr.cnt,
                   (unsigned)g_shm->bmp.hdr.cnt,
                   (unsigned)g_jf_drop,
                   (unsigned)g_jf_agg_tx,
                   (unsigned)g_jf_enq_fail,
                   (unsigned)g_ep_sent,
                   (unsigned)g_ep_busy,
                   (unsigned)g_ep_empty,
                   (unsigned)(g_ep_bytes / 1024u),
                   (unsigned)jf_agg_pending(),
                   (unsigned)skip_cnt,
                   (unsigned)g_shm->imu_start_cnt,
                   (unsigned)(g_shm->imu_start_cnt - g_shm->imu.hdr.cnt),
                   (unsigned)g_shm->mmc_recover_cnt,    /* 第 1 级：清锁存救回次数 */
                   (unsigned)g_shm->mmc_reinit_cnt,     /* 第 2 级：软件复位次数   */
                   (unsigned)g_shm->mmc_last_reinit_rc);/* 最近 init 返回码       */
            t_next = GetTime64_Us() + 5000000ULL;
        }

        /* ---- 锁存最新的磁原始值（约 1 kHz 更新） ---- */
        if (g_shm->mmc.hdr.cnt != last_mmc_cnt)
        {
            last_mmc_cnt = g_shm->mmc.hdr.cnt;
            __sync_synchronize();
            mx   = g_shm->mmc.mx;
            my   = g_shm->mmc.my;
            mz   = g_shm->mmc.mz;
            mpol = g_shm->mmc.pol;

            /* ★ 原始 counts：先把"当前"挪到"上一拍"，再写入新的当前 */
            if (mmc_seen != 0u)
            {
                m_prev_raw[0] = m_cur_raw[0];
                m_prev_raw[1] = m_cur_raw[1];
                m_prev_raw[2] = m_cur_raw[2];
                pol_prev      = pol_cur;
                mmc_prev_valid = 1u;
            }
            m_cur_raw[0] = mx;
            m_cur_raw[1] = my;
            m_cur_raw[2] = mz;
            pol_cur      = mpol;
            mmc_seen     = 1u;
        }

        /* ---- 逐 IMU 帧：取走板上每个采样并上报 ---- */
        if (g_shm->imu.hdr.cnt != last_imu_cnt)
        {
            uint64_t ts;

            {
                uint32_t step = g_shm->imu.hdr.cnt - prev_imu_cnt;
                if (step > 1u) skip_cnt += (step - 1u);   /* 跳过的采样数 */
                prev_imu_cnt = g_shm->imu.hdr.cnt;
            }
            last_imu_cnt = g_shm->imu.hdr.cnt;
            __sync_synchronize();
            ts = shm_chan_ts_read(&g_shm->imu.hdr);     /* 该采样的 DRDY 原始时间戳 */

            proc_chain_step(g_shm);                     /* 处理链照常逐帧跑，与上报无关 */

            att_div++;

            {
                /* ===== 唯一上报流：FULL 帧（全数据调试，124 B），IMU 满速率 =====
                 * ★ 帧内含 ESKF 实际用到的全部输入与输出：
                 *     q(4) + gyro_dps(3) + accel_g(3) + mag_u(3)
                 *     + flag + t_us + imu_cnt + mmc_cnt + mag_resid + acc_resid
                 *     + mag_raw_cur(3) + mag_raw_prev(3) + flag2
                 *   共 2 sync + 31 float = 124 B，帧长恒定，绝不混发其它类型。
                 * ★ 四元数仍在去掉两个 sync 后的 chan2..chan5，与上位机对齐。
                 * ★ flag : bit0 mag_valid, bit1 new_mag, bit2 grav_open, bit3 ready
                 * ★ flag2: bit0 pol_cur, bit1 pol_prev, bit2 mmc_prev_valid,
                 *          bit3 eskf_ready
                 * ★ 原始 counts 为 MMC5983MA 18 位（零点 131072），单帧一相，
                 *   用 (cur, prev) 两拍即可在 PC 端重建 (m_SET, m_RESET) 对。 */
                (void)att_div;
                {
                    float f = 0.0f, f2 = 0.0f;
                    if (g_proc.mag_valid != 0u)          f += 1.0f;
                    if (proc_mag_is_new() != 0u)         f += 2.0f;
                    if (proc_eskf_grav_open() != 0u)     f += 4.0f;
                    if (proc_eskf_ready() != 0u)         f += 8.0f;

                    if (pol_cur != 0u)                   f2 += 1.0f;
                    if (pol_prev != 0u)                  f2 += 2.0f;
                    if (mmc_prev_valid != 0u)            f2 += 4.0f;
                    if (proc_eskf_ready() != 0u)         f2 += 8.0f;

                    (void)jf_full_push(g_proc.q[0], g_proc.q[1],
                                       g_proc.q[2], g_proc.q[3],
                                       g_proc.gyro_dps[0], g_proc.gyro_dps[1], g_proc.gyro_dps[2],
                                       (float)g_shm->imu.ax * ACCEL_G_PER_LSB,
                                       (float)g_shm->imu.ay * ACCEL_G_PER_LSB,
                                       (float)g_shm->imu.az * ACCEL_G_PER_LSB,
                                       g_proc.mag_u[0], g_proc.mag_u[1], g_proc.mag_u[2],
                                       f,
                                       (float)((double)ts * 1e-1),
                                       (float)(g_shm->imu.hdr.cnt & 0x00FFFFFFu),
                                       (float)(g_shm->mmc.hdr.cnt & 0x00FFFFFFu),
                                       proc_eskf_mag_resid(),
                                       proc_eskf_acc_resid(),
                                       (float)m_cur_raw[0], (float)m_cur_raw[1], (float)m_cur_raw[2],
                                       (float)m_prev_raw[0], (float)m_prev_raw[1], (float)m_prev_raw[2],
                                       f2);
                }
                (void)mx; (void)my; (void)mz; (void)mpol;
            }
        }
    }
}
