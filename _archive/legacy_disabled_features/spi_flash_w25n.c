#include "spi_flash_w25n.h"
#include "debug.h"
#include "ch32h417.h"

/* W25N01GV（SPI NAND）命令（手册 8.x Instruction 节）*/
#define W25N_CMD_RESET         0xFF   /* 器件复位 */
#define W25N_CMD_JEDECID       0x9F   /* 读 JEDEC ID：EF AA 21 */
#define W25N_CMD_WRITE_ENABLE  0x06
#define W25N_CMD_WRITE_DISABLE 0x04
#define W25N_CMD_READ_STATUS   0x0F   /* 读状态寄存器（后跟 0xA0=SR1）*/
#define W25N_CMD_WRITE_STATUS  0x01
#define W25N_CMD_READ_DATA     0x13   /* 页 → 内部缓存（3 字节页地址）*/
#define W25N_CMD_READ_CACHE    0x03   /* 从缓存读（2 字节列地址）*/
#define W25N_CMD_PROGRAM_LOAD  0x02   /* 载入编程数据到缓存（2 字节列地址）*/
#define W25N_CMD_PROGRAM_EXEC  0x10   /* 缓存 → 页编程执行（3 字节页地址）*/
#define W25N_CMD_BLOCK_ERASE   0xD8   /* 块擦除（3 字节页地址）*/

#define W25N_SR1_ADDR          0xA0   /* SR1：保护寄存器（BP[3:0]/TB/WP-E）*/
#define W25N_SR3_ADDR          0xC0   /* SR3：操作状态寄存器 */
#define W25N_SR3_BUSY          0x01   /* SR3 bit0: 操作忙 */
#define W25N_SR3_WEL           0x02   /* SR3 bit1: 写使能锁存 */
#define W25N_SR3_E_FAIL        0x04   /* SR3 bit2: 擦除失败 */
#define W25N_SR3_P_FAIL        0x08   /* SR3 bit3: 编程失败 */
#define W25N_ID_EXPECT         0xEFAA21UL
#define W25N_PAGE_SIZE         2048   /* 数据区页大小 */

/* CS：PB12（软件控制）*/
#define W25N_CS_LOW()   GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define W25N_CS_HIGH()  GPIO_SetBits(GPIOB, GPIO_Pin_12)

static uint8_t spi2_rw(uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, data);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI2);
}

/* 发完所有字节后等 SPI 总线空闲（TXE 仅发送缓冲空，移位寄存器可能未完成；CS 拉高前必须调用）*/
static void spi_wait_busy(void)
{
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET);
}

/* 读指定状态寄存器：发命令+地址后直接读（规格书 Figure 7，无 dummy 字节）*/
static uint8_t w25n_read_sr(uint8_t addr)
{
    uint8_t sr;
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_READ_STATUS);
    spi2_rw(addr);
    sr = spi2_rw(0xFF);               /* 直接读，无额外 dummy */
    spi_wait_busy();
    W25N_CS_HIGH();
    return sr;
}

/* 读 SR1（保护寄存器）*/
static uint8_t w25n_read_sr1(void)
{
    return w25n_read_sr(W25N_SR1_ADDR);
}

/* 读 SR3（操作状态：BUSY/WEL/失败标志）*/
static uint8_t w25n_read_sr3(void)
{
    return w25n_read_sr(W25N_SR3_ADDR);
}

/* 等内部忙结束：标准轮询 SR3 bit0 BUSY + 20ms 超时（#11；擦除 ≤10ms 留余量）
 * 超时打印并尝试软件复位，避免误判 BUSY=0 提前返回 */
static void w25n_wait_busy(void)
{
    uint32_t timeout = 20000;
    while (timeout--) {
        if (!(w25n_read_sr3() & W25N_SR3_BUSY)) return;
        Delay_Us(1);
    }
    printf("[W25N] wait_busy timeout! BUSY stuck\r\n");
    /* 软件复位尝试解除异常 BUSY 状态 */
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_RESET);
    spi_wait_busy();
    W25N_CS_HIGH();
}

static void w25n_write_enable(void)
{
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_WRITE_ENABLE);
    spi_wait_busy();
    W25N_CS_HIGH();
    while (!(w25n_read_sr3() & W25N_SR3_WEL));   /* 确认 WEL 置位（SR3 bit1）*/
}

/* 16 位页地址 PA[15:0]（MSB 先发）*/
static void w25n_page_addr16(uint32_t page)
{
    spi2_rw((uint8_t)(page >> 8));
    spi2_rw((uint8_t)page);
}

void W25N_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure  = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB | RCC_HB2Periph_GPIOC, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_SPI2, ENABLE);

    /* CS PB12（推挽输出，软件控制）*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Low;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);

    /* SCK PB13(AF5) */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF5);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* MISO PB14(AF5) */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF5);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* MOSI PC3(AF5) */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF5);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;   /* Mode 0：实测全通（Mode 3 下写状态寄存器读回 FF，多字节写错位）；配合 spi_wait_busy */
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_Mode7;  /* 参考 EVT */
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI2, &SPI_InitStructure);
    // SPI_HighSpeedMode_Config(SPI2, SPI_HIGH_SPEED_MODE1, ENABLE);
    SPI_Cmd(SPI2, ENABLE);

    /* 复位器件，回到已知态 */
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_RESET);
    spi_wait_busy();
    W25N_CS_HIGH();
    w25n_wait_busy();
}

uint32_t W25N_ReadJEDECID(void)
{
    uint8_t b[3];
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_JEDECID);
    spi2_rw(0xFF);                  /* 丢弃前导字节（命令发送期间 MISO 无效输出 0x00，实测序列 00 EF AA）*/
    b[0] = spi2_rw(0xFF);           /* EF */
    b[1] = spi2_rw(0xFF);           /* AA */
    b[2] = spi2_rw(0xFF);           /* 21 */
    spi_wait_busy();
    W25N_CS_HIGH();
    return ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
}

void W25N_ReadPage(uint32_t page, uint8_t *buf, uint32_t len)
{
    if (len > W25N_PAGE_SIZE) len = W25N_PAGE_SIZE;

    /* 页 → 内部缓存：13h + 8Dummy(00h) + PA[15:0]（对照例程）*/
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_READ_DATA);
    spi2_rw(0x00);                        /* 8Dummy */
    w25n_page_addr16(page);
    spi_wait_busy();
    W25N_CS_HIGH();
    w25n_wait_busy();                 /* 理论极限：BUSY 轮询等页加载完成，替代固定 1ms */

    /* 从缓存读：03h + CA[15:0](0x0000) + 8Dummy(00h) + 数据（对照例程）*/
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_READ_CACHE);
    spi2_rw(0x00);                        /* CA[15:8] */
    spi2_rw(0x00);                        /* CA[7:0] */
    spi2_rw(0x00);                        /* 8Dummy */
    while (len--) *buf++ = spi2_rw(0xFF);
    spi_wait_busy();
    W25N_CS_HIGH();
}

void W25N_WritePage(uint32_t page, const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    if (len > W25N_PAGE_SIZE) len = W25N_PAGE_SIZE;

    /* 1) 先写使能(0x06)，再载入缓存：02h + CA[15:0](0000h) + 满 2048 字节（不足补 FFh）*/
    w25n_write_enable();
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_PROGRAM_LOAD);
    spi2_rw(0x00);                        /* CA[15:8] */
    spi2_rw(0x00);                        /* CA[7:0] */
    for (i = 0; i < W25N_PAGE_SIZE; i++)
        spi2_rw((i < len) ? buf[i] : 0xFF);
    spi_wait_busy();
    W25N_CS_HIGH();
    /* 载入缓存是同步操作（SPI 传输完成即载入完成），无需延时 */

    /* 2) 编程执行：10h + 8Dummy(00h) + PA[15:0]，等待完成（SR3 BUSY 轮询）*/
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_PROGRAM_EXEC);
    spi2_rw(0x00);                        /* 8Dummy */
    w25n_page_addr16(page);
    spi_wait_busy();
    W25N_CS_HIGH();
    w25n_wait_busy();
}

void W25N_EraseBlock(uint32_t page)
{
    /* 128KB 块擦除：D8h + 8Dummy(00h) + PA[15:0]（对照例程）*/
    w25n_write_enable();
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_BLOCK_ERASE);
    spi2_rw(0x00);                        /* 8Dummy */
    w25n_page_addr16(page);
    spi_wait_busy();
    W25N_CS_HIGH();
    w25n_wait_busy();
}

/* 写状态寄存器：0x01 + SR 地址 + 数据（需 WEL=1）*/
static void w25n_write_sr(uint8_t addr, uint8_t data)
{
    w25n_write_enable();
    W25N_CS_LOW();
    spi2_rw(W25N_CMD_WRITE_STATUS);
    spi2_rw(addr);
    spi2_rw(data);
    spi_wait_busy();
    W25N_CS_HIGH();
    /* 写状态寄存器是配置操作，不置 BUSY，无需等待 */
}

/* 专用解锁：清 SR1 的 BP[3:0]/TB（出厂 0x7C = 全片写保护）；解锁后全片可写，显式调用 */
void W25N_Unprotect(void)
{
    w25n_write_sr(W25N_SR1_ADDR, 0x00);
}

uint8_t W25N_SelfTest(void)
{
    uint8_t wbuf[64], rbuf[64];
    uint32_t id, i;
    uint8_t fail = 0;

    id = W25N_ReadJEDECID();
    printf("[W25N] JEDEC ID: %06lX (expect %06lX)\r\n",
           (unsigned long)id, (unsigned long)W25N_ID_EXPECT);
    if (id != W25N_ID_EXPECT) {
        printf("[W25N] ID FAIL\r\n");
        return 1;
    }

    /* 专用解锁（显式调用）；本自检仅在块 0 实验，其他块不动 */
    W25N_Unprotect();
    printf("[W25N] SR1 after unprotect: %02X SR3: %02X\r\n",
           w25n_read_sr1(), w25n_read_sr3());

    /* 实验（仅块 0）：擦块0 → 写 64 字节（位扫描模式 1<<(i&7)，逐位校验）→ 读回比较 */
    W25N_EraseBlock(0);
    printf("[W25N] SR3 after erase: %02X (WEL=%d BUSY=%d E-FAIL=%d)\r\n",
           w25n_read_sr3(),
           (w25n_read_sr3() >> 1) & 1, w25n_read_sr3() & 1, (w25n_read_sr3() >> 2) & 1);
    for (i = 0; i < sizeof(wbuf); i++) wbuf[i] = (uint8_t)(1u << (i & 7));
    W25N_WritePage(0, wbuf, sizeof(wbuf));
    W25N_ReadPage(0, rbuf, sizeof(rbuf));
    printf("[W25N] page0[0..15]:");
    for (i = 0; i < 16; i++) printf(" %02X", rbuf[i]);
    printf("\r\n");
    for (i = 0; i < sizeof(wbuf); i++) {
        if (rbuf[i] != wbuf[i]) {
            fail = 1;
            printf("[W25N] cmp fail @%lu: %02X!=%02X\r\n",
                   (unsigned long)i, rbuf[i], wbuf[i]);
            break;
        }
    }
    printf("[W25N] %s\r\n", fail ? "ERASE+WRITE+READ FAIL" : "ERASE+WRITE+READ PASS");
    return fail;
}
