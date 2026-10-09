/*******************************************************************************
* File Name          : main.c
* Description        : 主程序（V5F 大核）—— 中断 + DMA 采集架构的收尾侧
*
* 【分工】
*   V3F（小核）: 两个 DRDY 中断（短）+ 主循环轮询气压/GPS
*   V5F（本文件）: 两个 DMA 完成中断（DMA1_Ch2 地磁 / DMA1_Ch4 陀螺）
*
*   两个 DMA 完成中断的实体在 User/src/sensor_dma_rx.c ——
*   它们【不在这里调用】，而是由 NVIC 直接进中断，所以本文件
*   不需要为它们做任何注册，只需要保证：
*     ① 共享区指针 g_shm 已就绪（否则中断里的发布会被 if(g_shm) 跳过）
*     ② V5F 侧不干扰 V3F 的外设
*
* 【本文件必须做的一件事：拿共享区地址】
*   g_shm 由 mipc_v5.c 定义并初始化为 0。若不调用 mipc_v5_get_shared()，
*   g_shm 永远是 NULL，两个 DMA 完成中断里 `if (g_shm)` 会全部跳过 ——
*   表现是"中断在跑但共享区永远不更新"，极难排查。
*   之前没有任何代码调用它，所以这一条是本次必须补上的。
*
* 【V5F 不做的事】
*   - 不碰 SPI1/SPI2：两条总线的初始化、CS、Mux、DMA 通道都由 V3F 配置，
*     两个核同时操作同一外设会互相踩。
*   - 不碰 SDMMC / 气压 I2C / GPS USART：都在 V3F 上。
*******************************************************************************/
#include "debug.h"
#include "hardware.h"
#include "ch32h417.h"
#include "sys_clk.h"        /* GetTime64_Us：mipc_shm.h 只含 stdint.h，不传递此声明 */
#include "mipc_v5.h"
#include "mipc_shm.h"
#include "sensor_dma.h"

int main(void)
{
    SystemAndCoreClockUpdate();
    Delay_Init();

    printf("\r\n[V5F] start\r\n");

    /* 摘掉共享区指针。
     * V3F 在自己启动时就把共享区地址写进了 IPC_MSG0，本函数轮询取回。
     * 这一步必须在任何中断可能触发之前完成 —— 否则 DMA 完成中断里
     * g_shm 还是 NULL，那一帧数据会被静默丢弃。 */
    (void)mipc_v5_get_shared();

    /* 报告进度 2：共享区指针已就绪 */
    mipc_v5_sync(2u);

    /* 等 V3F 把两个器件配好（它置 v3f_cfg_done=1）。
     * 【为什么要等】V3F 配好器件之后才会 arm DRDY 中断，而中断一开就会
     * 开始往共享区发帧。本核必须先确认自己能接收，否则存在
     * "V3F 已在发、V5F 还没就绪"的窗口，那些帧会被 if(g_shm) 丢掉。
     * 带超时：V3F 异常时不至于永久卡死在这里。 */
    {
        uint64_t t_wait = GetTime64_Us();
        while (g_shm->v3f_cfg_done == 0u)
        {
            if ((GetTime64_Us() - t_wait) > 3000000ULL)   /* 3s 超时 */
            {
                printf("[V5F] sync timeout, v3f_cfg_done=0\r\n");
                break;
            }
        }
    }

    /* 报告进度 5：V5F 侧就绪，V3F 可以 arm 中断了。
     * 旧版的 3/4 两步对应"等 ICM42605 配好"与"装 SPI1 DMA"；
     * 本架构里器件配置与 DMA 装载都由 V3F 负责，V5F 没有对应动作，故跳过。 */
    mipc_v5_sync(5u);

    printf("[V5F] shm=%08X, armed\r\n", (unsigned)(uintptr_t)g_shm);

    /*--------------------------------------------------------------------------
     * 主循环：本阶段空转
     *
     * 两个 DMA 完成中断完全由硬件驱动，不需要主循环参与。
     * 后续主运动解析（旧工程 proc_* 八步链的等价物）挂在
     * sensor_dma_rx.c 的 DMA1_Channel4_IRQHandler 里那个标注了
     * "主运动解析的挂载点"的位置 —— 它是任务抢占式的长中断，
     * 运行中可被地磁 DMA 中断（更高抢占级）打断。
     *
     * 这里保留一个低频心跳，便于确认 V5F 活着且共享区连着。
     *------------------------------------------------------------------------*/
    {
        uint32_t last_cnt = 0u;
        uint64_t t_next = GetTime64_Us() + 1000000ULL;

        for (;;)
        {
            if ((int64_t)(t_next - GetTime64_Us()) <= 0)
            {
                uint32_t cnt = g_shm ? g_shm->imu.hdr.cnt : 0u;
                uint32_t dc  = cnt - last_cnt;

                printf("[V5F] hb imu_cnt=%u,d=%u,mmc_cnt=%u\r\n",
                       (unsigned)cnt, (unsigned)dc,
                       (unsigned)(g_shm ? g_shm->mmc.hdr.cnt : 0u));

                last_cnt = cnt;
                t_next = GetTime64_Us() + 1000000ULL;
            }
        }
    }
}
