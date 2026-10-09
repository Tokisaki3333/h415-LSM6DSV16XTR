#include "GPS.h"
#include "mipc_shm.h"
#include "board_io.h"    /* GNSS_USART / GNSS_RX_PIN 等板级宏 */
#define GPS_RX_BUFFER_SIZE 4096

/*
 * NMEA 校验和 = '$' 与 '*' 之间所有字节的异或（'$' 本身不参与）。
 * 第一级已判定前导头，各命令前导头（命令名5字节 + ','）的异或结果
 * 直接作为第二级校验和初值，写成常量表达式让编译器预先折叠，
 * 第二级只需从第一个数据字段起继续异或 —— 整条语句每个字节只读一次。
 */
#define GNRMC_HEADER_CS ('G' ^ 'N' ^ 'R' ^ 'M' ^ 'C' ^ ',')
#define GNGGA_HEADER_CS ('G' ^ 'N' ^ 'G' ^ 'G' ^ 'A' ^ ',')
#define GNGSA_HEADER_CS ('G' ^ 'N' ^ 'G' ^ 'S' ^ 'A' ^ ',')
/* GSV 各 talker 变体（GN 组合 + 各星座单独发；它们共用同一个解析器与 CmdId） */
#define GNGSV_HEADER_CS (0x67u)   /* GNGSV */
#define GPGSV_HEADER_CS (0x79u)   /* GPGSV */
#define BDGSV_HEADER_CS (0x68u)   /* BDGSV */
#define GLGSV_HEADER_CS (0x65u)   /* GLGSV */
#define GAGSV_HEADER_CS (0x68u)   /* GAGSV */
#define GQGSV_HEADER_CS (0x78u)   /* GQGSV */
#define GIGSV_HEADER_CS (0x60u)   /* GIGSV */

#define GPS_FIELDS_MAX 24

volatile uint8_t RxBuffer2[GPS_RX_BUFFER_SIZE] = {0};   /* DMA 环形接收缓冲，段边界由长度精确控制 */
uint32_t gps_rx_read_idx = 0;

typedef enum {
    CMD_NONE = 0,
    CMD_RMC,
    CMD_GGA,
    CMD_GSA,
    CMD_GSV
} CmdId;

/* ================= 第二级：通用增量解析状态 =================
 * 语句可能被 DMA 环形缓冲回绕截断在任意位置（缓冲区末尾是常见截断点），
 * 这些状态跨 parse_gps_data 的多次分段调用保持，实现跨段续读。 */
typedef struct {
    uint8_t  st;               /* 0=DATA 1=CS1 2=CS2 3=DONE */
    uint8_t  cs;               /* 校验和（初值已含前半段硬编码异或结果） */
    uint8_t  csrx[2];          /* 收到的两个校验字符 */
    uint8_t  nf;               /* 当前数据字段编号（1 起） */
    uint32_t fabs[GPS_FIELDS_MAX]; /* 各字段在 RxBuffer2 中的绝对下标 */
    uint8_t  flen[GPS_FIELDS_MAX]; /* 各字段长度 */
} GpsParser;

static GpsParser rmc_p, gga_p, gsa_p, gsv_p;

typedef struct {
    const char *name;          /* 命令名（5 字符） */
    uint8_t     cs_init;       /* 命令头校验和初值 */
    GpsParser  *p;
    CmdId       cmd;           /* ★ 该表项归属的命令。同一条命令可有多个
                                * talker 变体（GSV 就是），不能用"表下标+1"推。 */
} CmdDef;

static const CmdDef cmd_table[] = {
    { "GNRMC", GNRMC_HEADER_CS, &rmc_p, CMD_RMC },
    { "GNGGA", GNGGA_HEADER_CS, &gga_p, CMD_GGA },
    { "GNGSA", GNGSA_HEADER_CS, &gsa_p, CMD_GSA },
    /* GSV 各 talker 变体（GN 组合 + 各星座单独发），共用一个解析器与 CmdId */
    { "GNGSV", GNGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GPGSV", GPGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "BDGSV", BDGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GLGSV", GLGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GAGSV", GAGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GQGSV", GQGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GIGSV", GIGSV_HEADER_CS, &gsv_p, CMD_GSV },
};
#define CMD_COUNT (sizeof(cmd_table) / sizeof(cmd_table[0]))

/* ★ CmdId -> 解析器。续读路径用它取解析器，不再假设"表下标 == CmdId-1"
 *   （那个假设在一个命令有多个 talker 变体时就不成立）。 */
static GpsParser *const cmd_parser[] = { NULL, &rmc_p, &gga_p, &gsa_p, &gsv_p };

static CmdId    gps_cmd = CMD_NONE;   /* 当前活跃语句的命令（NONE=空闲） */
static uint64_t gps_clk  = 0;         /* 当前语句到达时间戳（跨段沿用首次识别时刻） */
static uint8_t  gps_talk = 0;         /* 当前 GSV 的 talker 序号 0..6 */

/* 将十六进制字符转换为数值 */
static uint8_t hex2val(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0;
}

/* 取绝对下标处的单个字节（越过缓冲区末尾自动回绕） */
static char buf_at(uint32_t abs_idx)
{
    if (abs_idx >= GPS_RX_BUFFER_SIZE) abs_idx -= GPS_RX_BUFFER_SIZE;
    return (char)RxBuffer2[abs_idx];
}

/* 新语句：重置第二级状态，第一个数据字段从本段起点开始 */
static void parser_reset(GpsParser *p, uint8_t cs_init, uint32_t first_abs)
{
    p->st = 0;
    p->cs = cs_init;
    p->nf = 1;
    memset(p->fabs, 0, sizeof(p->fabs));
    memset(p->flen, 0, sizeof(p->flen));
    p->fabs[1] = first_abs;
}

/* ================= GPS 共享区发布（V3F → V5F） =================
 * 每条校验通过的 RMC/GGA/GSA 语句发布一帧；某字段本帧缺失时对应 flags 位为 0、
 * 数值同时清零（显式空标志，判空一律查 flags 位，勿用值判空）。
 * 速度用 float 传 m/s；其余保持定点整数（lat/lon ×1e7 度、hdop/pdop/vdop ×100、alt cm）：
 * 字串解析天然是整数，且经纬度用 float 精度不够；跨核浮点的取舍见 mipc_shm.h。 */

/* 提取字段文本到栈缓冲（字段短；按绝对下标跨回绕安全） */
static void field_copy(char *dst, uint8_t dst_sz, uint32_t fabs, uint8_t flen)
{
    uint8_t i;
    if (flen >= dst_sz) flen = dst_sz - 1;
    for (i = 0; i < flen; i++) dst[i] = buf_at(fabs + i);
    dst[flen] = '\0';
}

/* "整数[.小数]" → ×10^decimals（空/非法返回 0） */
static int32_t parse_fixed(const char *s, int decimals)
{
    int32_t v = 0;
    int d = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); d++; s++; }
        while (d < decimals) { v *= 10; d++; }
    }
    while (d > decimals) { v /= 10; d--; }
    return v;
}

/* "ddmm.mmmmm"（度分）→ 度 ×1e7，无符号（方向由 N/S/E/W 字段决定） */
static int32_t latlon_to_e7(const char *s)
{
    int32_t ddmm = 0, m;
    int64_t frac = 0, p10 = 1;
    int fd = 0;
    while (*s >= '0' && *s <= '9') { ddmm = ddmm * 10 + (*s - '0'); s++; }
    m = ddmm % 100;                              /* 分 */
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { frac = frac * 10 + (*s - '0'); fd++; s++; }
    }
    while (fd--) p10 *= 10;
    return (int32_t)((int64_t)(ddmm / 100) * 10000000LL
                   + (m * p10 + frac) * 10000000LL / (60LL * p10));
}

/* "x.yyy" 节 → m/s（1 节 = 0.5144444 m/s）：取 3 位小数，float 误差 ~1e-7，远小于接收机 0.05 m/s(1σ) */
static float knots_to_mps(const char *s)
{
    return (float)parse_fixed(s, 3) * 1e-3f * 0.51444444f;
}

/* "x.yy" 度 → 度（float）：RMC 的 course 与 magnetic variation 都是这个格式 */
static float deg_from_nmea(const char *s)
{
    return (float)parse_fixed(s, 2) * 1e-2f;
}

/* RMC 通道发布：先清本通道字段（本帧缺失显式空），再填，最后 ts + cnt++ */
static void gps_publish_rmc(void)
{
    char t[16];
    /* 本帧缺失的字段先显式清空并清 flags 位（0 值本身合法，判空一律查 flags） */
    g_shm->gps_rmc.flags = 0;
    g_shm->gps_rmc.status = 0;      g_shm->gps_rmc.lat_e7 = 0; g_shm->gps_rmc.lon_e7 = 0;
    g_shm->gps_rmc.speed_mps = 0;   g_shm->gps_rmc.date_ddmmyy = 0;
    g_shm->gps_rmc.course_deg = 0;  g_shm->gps_rmc.magvar_deg = 0;

    if (rmc_p.flen[2] > 0) {
        g_shm->gps_rmc.flags |= SHM_RMC_STATUS;
        g_shm->gps_rmc.status = (uint8_t)buf_at(rmc_p.fabs[2]);
    }
    if (rmc_p.flen[3] > 0 && rmc_p.flen[5] > 0) {
        field_copy(t, sizeof t, rmc_p.fabs[3], rmc_p.flen[3]);
        int32_t lat = latlon_to_e7(t);
        if (rmc_p.flen[4] > 0 && buf_at(rmc_p.fabs[4]) == 'S') lat = -lat;
        field_copy(t, sizeof t, rmc_p.fabs[5], rmc_p.flen[5]);
        int32_t lon = latlon_to_e7(t);
        if (rmc_p.flen[6] > 0 && buf_at(rmc_p.fabs[6]) == 'W') lon = -lon;
        g_shm->gps_rmc.flags |= SHM_RMC_POS;
        g_shm->gps_rmc.lat_e7 = lat;
        g_shm->gps_rmc.lon_e7 = lon;
    }
    if (rmc_p.flen[7] > 0) {
        field_copy(t, sizeof t, rmc_p.fabs[7], rmc_p.flen[7]);
        g_shm->gps_rmc.flags |= SHM_RMC_SPEED;
        g_shm->gps_rmc.speed_mps = knots_to_mps(t);
    }
    /* 字段 8：对地航向（真北，0~360 度）。
     * ★ EKF 靠它 + speed_mps 才能组成速度矢量（北/东分量）；
     * 只有 speed_mps 标量时，速度只能约束一个方向。 */
    if (rmc_p.flen[8] > 0) {
        field_copy(t, sizeof t, rmc_p.fabs[8], rmc_p.flen[8]);
        g_shm->gps_rmc.flags |= SHM_RMC_COURSE;
        g_shm->gps_rmc.course_deg = deg_from_nmea(t);
    }
    if (rmc_p.flen[9] > 0) {
        field_copy(t, sizeof t, rmc_p.fabs[9], rmc_p.flen[9]);
        g_shm->gps_rmc.flags |= SHM_RMC_DATE;
        g_shm->gps_rmc.date_ddmmyy = (uint32_t)parse_fixed(t, 0);
    }
    /* 字段 10/11：接收机自报的磁偏角（东正西负）。
     * ★ 这是接收机**内部 WMM** 算出来的，可以和我们离线查的 WMM 值互相校验
     * （本机离线值 -7.53 度）。注意别和 course 混：
     * course 是航向（相对真北），magvar 是磁北相对真北的偏角。 */
    if (rmc_p.flen[10] > 0) {
        float mv;
        field_copy(t, sizeof t, rmc_p.fabs[10], rmc_p.flen[10]);
        mv = deg_from_nmea(t);
        if (rmc_p.flen[11] > 0 && buf_at(rmc_p.fabs[11]) == 'W') mv = -mv;
        g_shm->gps_rmc.flags |= SHM_RMC_MAGVAR;
        g_shm->gps_rmc.magvar_deg = mv;
    }
    shm_chan_ts_write(&g_shm->gps_rmc.hdr, gps_clk);
    g_shm->gps_rmc.hdr.cnt++;
}

/* GGA 通道发布 */
static void gps_publish_gga(void)
{
    char t[16];
    g_shm->gps_gga.flags = 0;
    g_shm->gps_gga.quality = 0; g_shm->gps_gga.sv = 0;
    g_shm->gps_gga.hdop_x100 = 0; g_shm->gps_gga.alt_cm = 0;

    if (gga_p.flen[6] > 0) {
        g_shm->gps_gga.flags |= SHM_GGA_QUALITY;
        g_shm->gps_gga.quality = (uint8_t)(buf_at(gga_p.fabs[6]) - '0');   /* '0'-'6' → 数值 0-6 */
    }
    if (gga_p.flen[7] > 0) {
        field_copy(t, sizeof t, gga_p.fabs[7], gga_p.flen[7]);
        g_shm->gps_gga.flags |= SHM_GGA_SV;
        g_shm->gps_gga.sv = (uint8_t)parse_fixed(t, 0);
    }
    if (gga_p.flen[8] > 0) {
        field_copy(t, sizeof t, gga_p.fabs[8], gga_p.flen[8]);
        g_shm->gps_gga.flags |= SHM_GGA_HDOP;
        g_shm->gps_gga.hdop_x100 = (uint16_t)parse_fixed(t, 2);
    }
    if (gga_p.flen[9] > 0) {
        field_copy(t, sizeof t, gga_p.fabs[9], gga_p.flen[9]);
        g_shm->gps_gga.flags |= SHM_GGA_ALT;
        g_shm->gps_gga.alt_cm = parse_fixed(t, 2);
    }
    shm_chan_ts_write(&g_shm->gps_gga.hdr, gps_clk);
    g_shm->gps_gga.hdr.cnt++;
}

/* GSA 通道发布（各系统一条，最新一条覆盖） */
static void gps_publish_gsa(void)
{
    char t[16];
    g_shm->gps_gsa.flags = 0;

    if (gsa_p.flen[15] > 0 && gsa_p.flen[17] > 0) {
        field_copy(t, sizeof t, gsa_p.fabs[15], gsa_p.flen[15]);
        g_shm->gps_gsa.pdop_x100 = (uint16_t)parse_fixed(t, 2);
        field_copy(t, sizeof t, gsa_p.fabs[17], gsa_p.flen[17]);
        g_shm->gps_gsa.vdop_x100 = (uint16_t)parse_fixed(t, 2);
        g_shm->gps_gsa.flags |= SHM_GSA_DOP;
    }
    shm_chan_ts_write(&g_shm->gps_gsa.hdr, gps_clk);
    g_shm->gps_gsa.hdr.cnt++;
}

/* GSV 累加器：一个轮次通常有 2~3 条消息，每条 4 颗星，只在最后一条发布 */

/* ---- GSV：按 talker 累积（7 个变体各存"最近一轮"）----
 * ★ 不能按时间窗聚合：GSV 跟着定位节拍走（$PCAS02,100 => 10 Hz），一个轮次里 7 个
 *   talker 的报文都落在同一个 100 ms 窗口内，相邻报文间隔只有几毫秒 —— 任何
 *   "间隔 > 阈值才算一轮"的条件都不成立，flush 永不执行（实测 gsv_cnt 恒 0）。
 *   改为结构性判定：msg==1 开一轮，msg==tot 提交该 talker 的一轮；
 *   每提交一次就把所有 talker 的最近一轮**求和**发布，不依赖任何时间阈值。
 * ★ 视野星数（字段 3）只在 msg==1 时取 —— 同一 talker 的多条消息里它是同一个数，
 *   每条都加会重复计数。按 talker 求和才是全天空总数。
 * ★ 某 talker 最近一轮超过 GSV_STALE_TICKS 没更新就从合计里剔除（防残留旧值）。 */
#define GSV_NTALK        7u              /* 与 cmd_table 里 GSV 的 talker 变体数一致 */
#define GSV_STALE_TICKS  2000000000ULL   /* 2 s（10 ns 计数） */

typedef struct {
    uint16_t sum;      /* 本轮：C/N0 之和 */
    uint8_t  view;     /* 本轮：视野星数 */
    uint8_t  n;        /* 本轮：有效 C/N0 星数 */
    uint8_t  min;      /* 本轮：最弱星 */
    uint8_t  have;     /* 本轮已有数据 */
    uint16_t v_sum;    /* 最近完成的一轮 */
    uint8_t  v_view;
    uint8_t  v_n;
    uint8_t  v_min;
    uint8_t  v_ok;
    uint64_t v_clk;    /* 最近完成一轮的时刻 */
} gsv_talk_t;

static gsv_talk_t s_gsv[GSV_NTALK];

/* 把所有 talker 的最近一轮求和后写进共享区 */
static void gps_gsv_publish(void)
{
    uint32_t vsum = 0u;
    uint16_t vview = 0u, vn = 0u, vmin = 0u, ntk = 0u;
    uint8_t  i;

    for (i = 0u; i < GSV_NTALK; i++) {
        if (s_gsv[i].v_ok == 0u) continue;
        if ((gps_clk - s_gsv[i].v_clk) > GSV_STALE_TICKS) {
            s_gsv[i].v_ok = 0u;              /* 过期：剔除 */
            continue;
        }
        vview += s_gsv[i].v_view;
        vsum  += s_gsv[i].v_sum;
        vn    += s_gsv[i].v_n;
        if (s_gsv[i].v_n > 0u && (vmin == 0u || s_gsv[i].v_min < vmin)) {
            vmin = s_gsv[i].v_min;
        }
        ntk++;
    }
    if (vview > 255u) vview = 255u;
    if (vn > 255u) vn = 255u;

    g_shm->gps_gsv.flags       = 0u;
    g_shm->gps_gsv.sats_view   = (uint8_t)vview;
    g_shm->gps_gsv.talkers     = (uint8_t)ntk;
    g_shm->gps_gsv.snr_n       = (uint8_t)vn;
    g_shm->gps_gsv.snr_min     = (uint8_t)vmin;
    g_shm->gps_gsv.snr_avg_x10 = (vn > 0u)
                               ? (uint16_t)(vsum * 10u / (uint32_t)vn) : 0u;
    g_shm->gps_gsv.flags      |= SHM_GSV_SNR;
    shm_chan_ts_write(&g_shm->gps_gsv.hdr, gps_clk);
    g_shm->gps_gsv.hdr.cnt++;
}

static void gps_publish_gsv(void)
{
    char        t[16];
    uint8_t     msg = 0u, tot = 0u, k, tk = gps_talk;
    gsv_talk_t *s;

    if (tk >= GSV_NTALK) return;
    s = &s_gsv[tk];

    if (gsv_p.flen[2] > 0u) {
        field_copy(t, sizeof t, gsv_p.fabs[2], gsv_p.flen[2]);
        msg = (uint8_t)parse_fixed(t, 0);
    }
    if (msg == 1u) {                        /* 该 talker 新一轮 */
        s->sum = 0u; s->n = 0u; s->min = 0u; s->view = 0u;
        if (gsv_p.flen[3] > 0u) {
            field_copy(t, sizeof t, gsv_p.fabs[3], gsv_p.flen[3]);
            s->view = (uint8_t)parse_fixed(t, 0);
        }
        s->have = 1u;
    }
    if (s->have == 0u) return;

    for (k = 0u; k < 4u; k++) {             /* SNR 在字段 7/11/15/19 */
        uint8_t fi = (uint8_t)(7u + 4u * k);
        uint8_t v;
        if (fi >= GPS_FIELDS_MAX || gsv_p.flen[fi] == 0u) continue;
        field_copy(t, sizeof t, gsv_p.fabs[fi], gsv_p.flen[fi]);
        v = (uint8_t)parse_fixed(t, 0);
        if (v == 0u) continue;
        s->sum = (uint16_t)(s->sum + v);
        s->n++;
        if (s->min == 0u || v < s->min) s->min = v;
    }

    if (gsv_p.flen[1] > 0u) {
        field_copy(t, sizeof t, gsv_p.fabs[1], gsv_p.flen[1]);
        tot = (uint8_t)parse_fixed(t, 0);
    }
    if (tot != 0u && msg == tot) {          /* 该 talker 一轮结束：提交 */
        s->v_sum = s->sum;  s->v_view = s->view;
        s->v_n   = s->n;    s->v_min  = s->min;
        s->v_ok  = 1u;      s->v_clk  = gps_clk;
        s->have  = 0u;
    }
    /* 每条 GSV 都发布：这样 gsv_cnt 的节律 == GSV 报文的到达率，可直接读出
     * "接收机到底有没有发 GSV"，不必再靠推断。talkers 仍只统计已完成轮次的 talker。 */
    gps_gsv_publish();
}

/* 按当前活跃命令发布对应语句帧 */
static void gps_publish_frame(void)
{
    switch (gps_cmd) {
        case CMD_RMC: gps_publish_rmc(); break;
        case CMD_GGA: gps_publish_gga(); break;
        case CMD_GSA: gps_publish_gsa(); break;
        case CMD_GSV: gps_publish_gsv(); break;
        default: break;
    }
}

/* ================= 第二级：通用语句解析（增量状态机） =================
 * 校验和初值 = 命令头异或常量（parser_reset 已设），只从逗号后的
 * 第一个数据字段继续读，字段记录与校验和单遍完成。
 * 返回本段已消费的字节数；若到段尾语句仍未到行尾，gps_cmd 保持非
 * CMD_NONE 等待下一段续读。
 */
static uint32_t parse_body(uint8_t *data, uint32_t len, uint32_t abs_base, GpsParser *p)
{
    uint32_t consumed = 0;
    uint32_t cur_abs  = abs_base;
    uint8_t  c;

    while (consumed < len) {
        c = data[consumed];
        switch (p->st) {
            case 0:                          /* DATA：数据字段 */
                if (c == ',') {
                    p->cs ^= c;
                    p->nf++;
                    if (p->nf < GPS_FIELDS_MAX) {
                        p->fabs[p->nf] = cur_abs + 1;
                        p->flen[p->nf] = 0;
                    }
                } else if (c == '*') {
                    p->st = 1;               /* '*' 不参与校验和 */
                } else if (c == '\r' || c == '\n') {
                    p->st = 3;               /* 语句残缺，丢弃 */
                } else {
                    p->cs ^= c;
                    if (p->nf < GPS_FIELDS_MAX) p->flen[p->nf]++;
                }
                break;

            case 1:                          /* CS1：校验和第一字符 */
                if (c == '\r' || c == '\n') {
                    p->st = 3;
                } else {
                    p->csrx[0] = c;
                    p->st = 2;
                }
                break;

            case 2:                          /* CS2：校验和第二字符，判定 */
                if (c == '\r' || c == '\n') {
                    p->st = 3;
                } else {
                    p->csrx[1] = c;
                    if ((uint8_t)((hex2val((char)p->csrx[0]) << 4) |
                                  hex2val((char)p->csrx[1])) == p->cs) {
                        /* 小核改用 mipc_v3_printf 转发串口后，GPS 不再直接 printf（同一 USART 冲突），打印停用 */
                        // print_parsed();
                        gps_publish_frame();
                    }
                    p->st = 3;
                }
                break;

            default:                         /* DONE：等行尾 */
                break;
        }
        consumed++;
        cur_abs++;

        if (p->st == 3 && (c == '\r' || c == '\n')) {
            /* 语句结束（行尾已消费），复位并交还第一级 */
            p->st    = 0;
            gps_cmd  = CMD_NONE;
            return consumed;
        }
    }

    /* 段尾：语句未完成，gps_cmd 保持非 NONE，等下一段续读 */
    return consumed;
}

/* ================= 第一级：语句分派 =================
 * WAIT_START   找 '$'
 * IN_HEADER    读命令头直到第一个 ','，查命令表判定命令类型
 * SKIP_LINE    未命中的命令整行弃用（跳到行尾）
 * 命中后立即分派给对应第二级，从逗号后继续读；
 * 前半段校验和初值由第二级按自身命令类型硬编码获得，无需传递。
 */
void parse_gps_data(uint8_t *data, uint32_t len, uint32_t abs_base, uint64_t clk)
{
    enum {
        WAIT_START = 0,
        IN_HEADER,
        SKIP_LINE
    };

    static uint8_t st      = WAIT_START;
    static uint8_t hdr[6];
    static uint8_t hdr_len = 0;

    uint32_t pos = 0;

    /* 上一条语句跨段未读完：整段交给对应第二级续读，读完再从剩余处继续 */
    if (gps_cmd != CMD_NONE) {
        uint32_t n = parse_body(data, len, abs_base, cmd_parser[gps_cmd]);
        if (gps_cmd != CMD_NONE) return;
        pos = n;
    }

    while (pos < len) {
        uint8_t c = data[pos];

        switch (st) {
            case WAIT_START:
                if (c == '$') {
                    hdr_len = 0;
                    st      = IN_HEADER;
                }
                break;

            case IN_HEADER:
                if (c == ',') {
                    /* 命令头结束：查表命中则分派第二级，未命中整行弃用 */
                    if (hdr_len == 5) {
                        for (uint8_t i = 0; i < CMD_COUNT; i++) {
                            if (memcmp(hdr, cmd_table[i].name, 5) == 0) {
                                gps_cmd = cmd_table[i].cmd;
                                /* GSV 有 7 个 talker 变体（表下标 3~9），必须把序号带下去，
                                 * 否则 7 个 talker 的统计会混在一起。 */
                                gps_talk = (gps_cmd == CMD_GSV) ? (uint8_t)(i - 3u) : 0u;
                                gps_clk = clk;
                                parser_reset(cmd_table[i].p, cmd_table[i].cs_init,
                                             abs_base + pos + 1);
                                uint32_t n = parse_body(data + pos + 1,
                                                        len - (pos + 1),
                                                        abs_base + pos + 1,
                                                        cmd_table[i].p);
                                pos += 1 + n;   /* 跳过逗号和整条语句（已消费到行尾） */
                                st = WAIT_START; /* 复位第一级，等下一行 '$' */
                                if (gps_cmd != CMD_NONE) return;
                                continue;
                            }
                        }
                    }
                    st = SKIP_LINE;
                } else if (c == '\r' || c == '\n') {
                    st = WAIT_START;            /* 残缺语句 */
                } else if (hdr_len < (uint8_t)sizeof(hdr)) {
                    hdr[hdr_len++] = c;
                } else {
                    st = SKIP_LINE;             /* 命令头过长，不可能是已登记命令 */
                }
                break;

            case SKIP_LINE:
                if (c == '\r' || c == '\n') st = WAIT_START;
                break;
        }
        pos++;
    }
}

/* 通过 GNSS_USART TX 向模块发送一条指令（阻塞轮询，命令很短，安全）。
 * 指令需自带 NMEA 校验和，例如 10Hz：GPS_Send_Cmd("$PCAS02,100*1E\r\n"); */
void GPS_Send_Cmd(const char *cmd)
{
    while (*cmd) {
        while ((GNSS_USART->STATR & 0x80) == 0);   /* 等 TXE（发送寄存器空） */
        GNSS_USART->DATAR = (uint8_t)*cmd++;
    }
    while ((GNSS_USART->STATR & 0x40) == 0);       /* 等 TC（发送完成） */
}

void GPS_USART_Init()
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA2, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&GNSS_USART->DATAR);
    DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)RxBuffer2;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize         = GPS_RX_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel2, &DMA_InitStructure);

    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART2, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOD, ENABLE);

    // USART3_RX PB11(AF7)
    GPIO_PinAFConfig(GNSS_RX_PORT, GNSS_RX_SRC, GNSS_USART_AF);
    GPIO_InitStructure.GPIO_Pin   = GNSS_RX_PIN;   /* PD6 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GNSS_RX_PORT, &GPIO_InitStructure);

    // USART3_TX PB10(AF7)
    GPIO_PinAFConfig(GNSS_TX_PORT, GNSS_TX_SRC, GNSS_USART_AF);
    GPIO_InitStructure.GPIO_Pin   = GNSS_TX_PIN;   /* PD5 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GNSS_RX_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(GNSS_USART, &USART_InitStructure);

    DMA_MuxChannelConfig(DMA_MuxChannel10, 88);   /* USART2_RX */
    DMA_Cmd(DMA2_Channel2, ENABLE);
    USART_Cmd(GNSS_USART, ENABLE);
    USART_DMACmd(GNSS_USART, USART_DMAReq_Rx, ENABLE);

    DMA_Cmd(DMA2_Channel2, ENABLE); /* GNSS_USART Rx */

    USART_Cmd(GNSS_USART, ENABLE);


    /* 试：定位更新频率设为 10Hz（100ms）。恢复 1Hz 用 "$PCAS02,1000*2E\r\n" */
    GPS_Send_Cmd("$PCAS02,100*1E\r\n");

    /* 语句输出：RMC/GGA/GSA/GSV 每帧，VTG/ZDA/GLL 关闭（GSV 用于上报 C/N0 信噪比） */
    GPS_Send_Cmd("$PCAS03,1,0,1,1,1,0,0,0,0,0,,,1,1,,,,0*32\r\n");
    // GPS_Send_Cmd("$PCAS03,1,1,1,1,1,1,1,1,1,1,0,0,,,1,1,,,,1*33\r\n");
}

/* 配置指令重发：$PCAS02/$PCAS03 在 GPS_USART_Init() 里只发一次，而那一刻 GPS 模块
 * 自身可能还没启动完（模块启动要 100~500 ms），命令被丢掉，模块回落到默认配置
 * —— 表现就是"有定位、RMC/GGA/GSA 都正常，但 GSV 一条都没有"（实测三次记录 gsv_cnt 恒 0，
 * 而同一条表查找路径上的 GSA 速率完全正常，说明不是命令头不匹配）。
 * 上电后按下面的时刻重发几次即可，重发期间接收机正在运行、一定收得到。
 * 单位 10 ns（与 gps_clk 同基）。 */
static const uint64_t s_cfg_tick[5] = {
    0ULL, 50000000ULL, 100000000ULL, 200000000ULL, 500000000ULL
};                                              /* 0 / 0.5 / 1 / 2 / 5 s */
static uint8_t  s_cfg_n;
static uint64_t s_cfg_t0;

static void gps_cfg_retry(void)
{
    uint64_t now;
    if (s_cfg_n >= 5u) return;
    now = GetTime64_10Ns();
    if (s_cfg_t0 == 0ULL) { s_cfg_t0 = now; if (now == 0ULL) return; }
    if ((now - s_cfg_t0) >= s_cfg_tick[s_cfg_n]) {
        GPS_Send_Cmd("$PCAS02,100*1E\r\n");      /* 10 Hz 定位更新率 */
        GPS_Send_Cmd("$PCAS03,1,0,1,1,1,0,0,0,0,0,,,1,1,,,,0*32\r\n");  /* 开 GSV */
        s_cfg_n++;
    }
}

/* 已解析到的绝对位置（0..GPS_RX_BUFFER_SIZE-1）。
 * 限流时它可以停在一条语句中间 —— 解析状态机支持跨段续读。 */
static uint32_t s_proc_idx  = 0u;
static uint8_t  s_caught_up = 1u;

uint8_t GPS_CaughtUp(void) { return s_caught_up; }

/*
 * 轮询一次：解析 DMA 收到的新数据，返回本次处理的字节数。
 *   max_bytes = 0 -> 不限制，一次处理到底
 *   max_bytes > 0 -> 本次最多处理这么多字节，剩余的留给下次
 *
 * 实测解析约 0.45us/字节，故调用方可按"允许耗时"反推 max_bytes。
 * 限流只影响何时返回；解析结果是增量的，与不限流完全等价。
 */
uint32_t GPS_Check(uint32_t max_bytes)
{
    uint32_t done = 0u;

    gps_cfg_retry();

    for (;;)
    {
        uint32_t wr   = GPS_RX_BUFFER_SIZE - DMA2_Channel2->CNTR;
        uint32_t span = GPS_RX_BUFFER_SIZE - s_proc_idx;   /* 到缓冲末尾的长度 */
        uint32_t chunk;

        if (wr == GPS_RX_BUFFER_SIZE) wr = 0u;
        if (wr == s_proc_idx) break;                       /* 已处理到底 */

        /* 本段 = 从 s_proc_idx 到写指针的最长连续区间 */
        if (wr > s_proc_idx)
        {
            chunk = wr - s_proc_idx;
        }
        else
        {
            chunk = span;                                  /* 跨回绕：先吃到缓冲末尾 */
        }

        /* 按本次剩余预算截断 */
        if (max_bytes != 0u)
        {
            uint32_t room = max_bytes - done;
            if (room == 0u) break;
            if (chunk > room)
            {
                chunk = room;
                /* 切点落在语句中间时，宁可少处理也不切碎：
                 * 单条语句最长约 82 字节。若剩余不足一句，留给下次。 */
                if (room < 96u) break;
            }
        }

        parse_gps_data((uint8_t *)&RxBuffer2[s_proc_idx], chunk, s_proc_idx,
                       GetTime64_10Ns());

        s_proc_idx += chunk;
        if (s_proc_idx >= GPS_RX_BUFFER_SIZE) s_proc_idx -= GPS_RX_BUFFER_SIZE;
        done += chunk;

        /* 不限流就继续吃到空 */
        if (max_bytes == 0u) continue;
    }

    gps_rx_read_idx = s_proc_idx;   /* 旧读索引保持同步 */

    {
        uint32_t wr2 = GPS_RX_BUFFER_SIZE - DMA2_Channel2->CNTR;
        if (wr2 == GPS_RX_BUFFER_SIZE) wr2 = 0u;
        s_caught_up = (wr2 == s_proc_idx) ? 1u : 0u;
    }

    return done;
}
