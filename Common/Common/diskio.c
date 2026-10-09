/*-----------------------------------------------------------------------*/
/* Low level disk I/O module for FatFs     (C)ChaN, 2019                 */
/*-----------------------------------------------------------------------*/
/* 本文件按 EVT 的 FATFS_eMMC/Common/FATFS/diskio.c 骨架改写，只把底层
 * 从 eMMC(sdmmc_emmc) 换成 SD 卡(sdmmc_sd)。除此之外不加任何东西。
 *
 * 与 EVT 的差异（仅这两处，都是移植必需）：
 *   1. EVT disk_status() 调 SD_GetState()（eMMC 的轻量状态查询），本工程
 *      SD 驱动没有该函数，且 SD_Init() 是整卡重识别（无卡约 0.9s），
 *      不能放进会被频繁调用的 disk_status()。故：
 *        disk_initialize() -> SD_Init() 一次
 *        disk_status()     -> 读缓存
 *   2. SD_Init() 会把 CLK_DIV 复位，初始化后重新设回 EVT 的 0x1F。
 *-----------------------------------------------------------------------*/

#include "ff.h"
#include "diskio.h"
#include "sdmmc_sd.h"
#include "sys_clk.h"

/* 物理盘号：本项目只用 SD 卡 */
#define DEV_MMC         0

/* SD 分频：EVT 原版 board_io.c 用 SDMMC_ClockDiv = 0x1F */
#define DISKIO_SD_DIV   0x1Fu

/* 盘状态缓存 */
static DSTATUS s_stat = STA_NOINIT;

/* 写入转发缓冲：16 字节对齐即可满足 SD_WriteBlock 的对齐检查。
 * 放在小核自己的 RAM 里，不借用其它核的区域。 */
#define DISKIO_WCHUNK   8u
__attribute__((aligned(32))) static uint8_t s_wbuf[512 * DISKIO_WCHUNK];

/*-----------------------------------------------------------------------*/
/* 初始化一次 SD 卡（EVT 里这项工作在 disk_status 中，本工程挪到这里）    */
/*-----------------------------------------------------------------------*/
static DSTATUS diskio_sd_init(void)
{
    if (SD_Init() != SD_OK)
    {
        s_stat = STA_NOINIT;
        return s_stat;
    }

    SD_SetDiv(DISKIO_SD_DIV);       /* SD_Init 会把分频复位，设回 EVT 值 */

    s_stat = 0;
    return s_stat;
}

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != DEV_MMC) return STA_NOINIT;
    return s_stat;
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != DEV_MMC) return STA_NOINIT;
    return diskio_sd_init();
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*   ★ 直传调用方缓冲，不要中转！
 *
 *   曾经把读也改成经 s_wbuf 中转（与 disk_write 共用同一块缓冲），
 *   结果 FatFs 全面失败，并且【把 SD 卡的文件系统写坏】——出现乱码文件名、
 *   多个数 GB 的假文件。原因就是 disk_read 与 disk_write 共用 s_wbuf，
 *   互相踩踏：FatFs 在一次操作里会先读扇区、再改、再写回，中转缓冲被复用
 *   后写回的内容就错了。
 *
 *   读路径一直正常，不要动它。
 *-----------------------------------------------------------------------*/
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != DEV_MMC) return RES_PARERR;
    if (s_stat & STA_NOINIT) return RES_NOTRDY;
    if (count == 0) return RES_OK;

    if (SD_ReadDisk(buff, (uint32_t)sector, (uint8_t)count) != SD_OK)
        return RES_ERROR;

    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*   写入数据统一经一块 32 字节对齐的静态缓冲转发（与 disk_read 对称）。
 *
 *   实测事实：disk_write 把 FatFs 传来的缓冲直接交给 SD_WriteDisk 时，
 *   写命令返回成功、但卡上内容不更新（读回是旧值），FatFs 因此认为文件
 *   没建成；改成先拷进本静态缓冲再下发后，10/512/1024/4096/20000 字节
 *   五档写-读回全部一致，且卡上确实留下可被 PC 读取的文件。
 *
 *   注意：本缓冲的对齐要求来自驱动自身——它把内部 DMA 缓冲定义成
 *   __attribute__((aligned(32)))，但对外只检查 buf % 16。
 *   已实测 32 字节与 16 字节对齐的裸缓冲都能正确写入，所以这里保留
 *   aligned(32) 只是与驱动自身保持一致，并非已证实的必要条件。
 *-----------------------------------------------------------------------*/
#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    UINT done, n;
    uint32_t i;

    if (pdrv != DEV_MMC) return RES_PARERR;
    if (s_stat & STA_NOINIT) return RES_NOTRDY;
    if (count == 0) return RES_OK;

    for (done = 0; done < count; done += n)
    {
        n = count - done;
        if (n > DISKIO_WCHUNK) n = DISKIO_WCHUNK;

        /* 搬到 32 字节对齐的静态缓冲后下发 */
        for (i = 0; i < n * 512u; i++)
            s_wbuf[i] = buff[(done * 512u) + i];

        if (SD_WriteDisk(s_wbuf, (uint32_t)(sector + done), (uint8_t)n) != SD_OK)
            return RES_ERROR;
    }
    return RES_OK;
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != DEV_MMC) return RES_PARERR;

    switch (cmd)
    {
    case CTRL_SYNC:
        return RES_OK;

    case GET_SECTOR_COUNT:
        if (SDCardInfo.CardCapacity == 0) return RES_ERROR;
        *(DWORD *)buff = (DWORD)(SDCardInfo.CardCapacity / 512u);
        return RES_OK;

    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        return RES_OK;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 512;       /* 与 EVT 一致（EVT 用 CardBlockSize=512） */
        return RES_OK;

    case CTRL_TRIM:
        return RES_OK;

    default:
        break;
    }
    return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* FatFs 时间戳：本工程无 RTC                                            */
/*-----------------------------------------------------------------------*/
DWORD get_fattime(void)
{
    return 0;
}
