/* CDC 上报包：组包 + CRC16 + 入上行环。空间不足时整包丢弃，不会写出半包 */

#include "cdc_report.h"
#include "usbd_compatibility_hid.h"
#include "ch32h417.h"
#include "sys_clk.h"

volatile uint32_t g_cdc_tx_pkts;
volatile uint32_t g_cdc_tx_drop;
volatile uint32_t g_cdc_rx_pkts;

static uint16_t s_seq[8];                       /* 每类型独立序号 */

uint16_t crc16_ccitt(const uint8_t *p, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;
    int      b;

    for (i = 0u; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (b = 0; b < 8; b++)
        {
            if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else               crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void cdc_reset_seq(void)
{
    uint32_t i;

    for (i = 0u; i < 8u; i++) s_seq[i] = 0u;
    g_cdc_tx_pkts = 0u;
    g_cdc_tx_drop = 0u;
    g_cdc_rx_pkts = 0u;
}

uint16_t pkt_build(uint8_t *dst, uint8_t type, uint8_t flags,
                   uint16_t seq, uint64_t ts_10ns,
                   const uint8_t *payload, uint8_t payload_len)
{
    uint32_t o = 0u;
    uint32_t i;
    uint16_t crc;

    dst[o++] = PKT_SYNC0;
    dst[o++] = PKT_SYNC1;
    dst[o++] = type;
    dst[o++] = flags;
    dst[o++] = (uint8_t)(seq);
    dst[o++] = (uint8_t)(seq >> 8);
    for (i = 0u; i < 6u; i++) dst[o++] = (uint8_t)(ts_10ns >> (8u * i));
    dst[o++] = payload_len;
    for (i = 0u; i < payload_len; i++) dst[o++] = payload[i];

    crc = crc16_ccitt(&dst[2], o - 2u);         /* 覆盖 TYPE..净荷，跳过 SYNC */
    dst[o++] = (uint8_t)(crc);
    dst[o++] = (uint8_t)(crc >> 8);

    return (uint16_t)o;
}

uint8_t cdc_send(uint8_t type, uint8_t flags, uint64_t ts_10ns,
                 const uint8_t *payload, uint8_t payload_len)
{
    uint8_t  buf[PKT_HDR_LEN + 32u + PKT_CRC_LEN];
    uint16_t n;

    if (type < 8u)
    {
        n = pkt_build(buf, type, flags, s_seq[type], ts_10ns, payload, payload_len);
        if (hid_up_enqueue(buf, n))
        {
            s_seq[type]++;
            g_cdc_tx_pkts++;
            return 1u;
        }
    }
    g_cdc_tx_drop++;
    return 0u;
}

void cdc_rx_poll(void)
{
    static uint32_t last_cnt = 0u;
    uint32_t cnt;

    hid_cmd_poll();

    cnt = g_cmd_cnt;
    if (cnt != last_cnt)
    {
        uint8_t  pl[PL_ACK];
        uint32_t i;
        union { float f; uint32_t u; } cv;

        g_cdc_rx_pkts += (cnt - last_cnt);
        last_cnt = cnt;
        cv.f = g_cmd_echo;

        pl[0] = (uint8_t)cnt;         pl[1] = (uint8_t)(cnt >> 8);
        pl[2] = (uint8_t)(cnt >> 16); pl[3] = (uint8_t)(cnt >> 24);
        pl[4] = g_cmd_last;
        for (i = 0u; i < 4u; i++) pl[5u + i] = (uint8_t)(cv.u >> (8u * i));

        (void)cdc_send(PKT_TYPE_ACK, PKT_FLAG_TSOK, GetTime64_10Ns(), pl, PL_ACK);
    }
}