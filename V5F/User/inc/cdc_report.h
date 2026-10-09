/* CDC 上报包格式：SYNC(2) TYPE(1) FLAGS(1) SEQ(2) TS(6) LEN(1) 净荷 CRC16(2) */

#ifndef CDC_REPORT_H
#define CDC_REPORT_H

#include <stdint.h>

#define PKT_SYNC0       0xA5u
#define PKT_SYNC1       0x5Au
#define PKT_HDR_LEN     13u
#define PKT_CRC_LEN     2u

#define PKT_TYPE_IMU    1u
#define PKT_TYPE_MMC    2u
#define PKT_TYPE_BMP    3u
#define PKT_TYPE_STAT   4u
#define PKT_TYPE_EVENT  5u
#define PKT_TYPE_ACK    6u

#define PKT_FLAG_RETRY  0x01u
#define PKT_FLAG_TSOK   0x02u

#define PL_IMU          14u
#define PL_MMC          12u
#define PL_BMP          12u
#define PL_STAT         16u
#define PL_ACK           9u

#define PKT_LEN(t)      (PKT_HDR_LEN + (t) + PKT_CRC_LEN)

uint16_t crc16_ccitt(const uint8_t *p, uint32_t len);
uint16_t pkt_build(uint8_t *dst, uint8_t type, uint8_t flags,
                   uint16_t seq, uint64_t ts_10ns,
                   const uint8_t *payload, uint8_t payload_len);
uint8_t cdc_send(uint8_t type, uint8_t flags, uint64_t ts_10ns,
                 const uint8_t *payload, uint8_t payload_len);
void cdc_reset_seq(void);
void cdc_rx_poll(void);

extern volatile uint32_t g_cdc_tx_pkts;
extern volatile uint32_t g_cdc_tx_drop;
extern volatile uint32_t g_cdc_rx_pkts;

#endif