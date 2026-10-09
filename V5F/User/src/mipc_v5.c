#include "mipc_v5.h"
#include "debug.h"

/* 大核持有的共享区指针（V3F 定义共享区并写入 IPC 消息，大核轮询读取）*/
volatile shared_mem_t *g_shm = 0;

/* step1: 轮询 IPC 拿共享区地址（小核启动大核前已写入），返回 0=成功 */
uint8_t mipc_v5_get_shared(void)
{
    while (!g_shm) {
        uint32_t a = IPC_ReadMSG(IPC_MSG0);
        if (a >= 0x20000000 && a < 0x40000000)
            g_shm = (volatile shared_mem_t *)a;
        Delay_Ms(10);
    }
    g_shm->v5f_progress = 1;   /* step1: 拿到地址 */
    printf("V5F add %X\r\n", (u32)g_shm);
    return 0;
}

/* step2..5: 向小核报告初始化进度（小核等 >=5 才停用串口6、改走大核转发）*/
void mipc_v5_sync(uint8_t step)
{
    g_shm->v5f_progress = step;
}
