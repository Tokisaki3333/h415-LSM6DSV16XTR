#include "mipc_v3.h"

#include <stdarg.h>

/* 共享区存储 + 指针：两端统一以 g_shm（指针）命名，地址经 IPC_MSG0 传给 V5F */
volatile shared_mem_t  g_shm_storage = {0};
volatile shared_mem_t *g_shm = &g_shm_storage;

/* printf 风格发信（小核→大核）：log 通道单缓冲，无乒乓（小核低频、严格顺序）
 * 发布协议：写 data_buf → 屏障 → cnt++（大核读端见 mipc_shm.h 注释）*/
void mipc_v3_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf((char *)g_shm->log.data_buf, sizeof(g_shm->log.data_buf), fmt, ap);
    va_end(ap);
    __sync_synchronize();       /* 数据写完再发布，防大核读到新 cnt 配旧数据 */
    g_shm->log.cnt++;           /* 通知大核：新一帧就绪 */
}

static void IPC_Config(IPC_Channel_TypeDef IPC_CHx, IPC_TxCID_TypeDef IPC_TxCIDx, IPC_RxCID_TypeDef IPC_RxCIDx)
{
    IPC_InitTypeDef IPC_InitStructure = {0};

    IPC_InitStructure.IPC_CH   = IPC_CHx;
    IPC_InitStructure.TxCID    = IPC_TxCIDx;
    IPC_InitStructure.RxCID    = IPC_RxCIDx;
    IPC_InitStructure.TxIER    = ENABLE;
    IPC_InitStructure.RxIER    = ENABLE;
    IPC_InitStructure.AutoEN   = ENABLE;
    IPC_Init(&IPC_InitStructure);
}

uint8_t ipc_comm_init(void)
{
    IPC_DeInit();
    IPC_Config(IPC_CH0, IPC_TxCID1, IPC_RxCID0);
    IPC_CH0_Lock();
    IPC_WriteMSG(IPC_MSG0, (uint32_t)g_shm);   /* 共享区地址 */
    printf("[V3F] ipc addr written\r\n");

    NVIC_WakeUp_V5F(Core_V5F_StartAddr);            // wake up V5
    
    return 0;
}