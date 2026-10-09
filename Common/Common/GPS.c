#include "GPS.h"
#include "mipc_shm.h"
#define GPS_RX_BUFFER_SIZE 4096

/*
 * NMEA У��� = '$' �� '*' ֮�������ֽڵ����'$' ��������룩��
 * ��һ�����ж�ǰ��ͷ��������ǰ��ͷ��������5�ֽ� + ','���������
 * ֱ����Ϊ�ڶ���У��ͳ�ֵ��д�ɳ������ʽ�ñ�����Ԥ���۵���
 * �ڶ���ֻ��ӵ�һ�������ֶ��������� ���� �������ÿ���ֽ�ֻ��һ�Ρ�
 */
#define GNRMC_HEADER_CS ('G' ^ 'N' ^ 'R' ^ 'M' ^ 'C' ^ ',')
#define GNGGA_HEADER_CS ('G' ^ 'N' ^ 'G' ^ 'G' ^ 'A' ^ ',')
#define GNGSA_HEADER_CS ('G' ^ 'N' ^ 'G' ^ 'S' ^ 'A' ^ ',')
/* GSV �� talker ���壨GN ��� + �����������������ǹ���ͬһ���������� CmdId�� */
#define GNGSV_HEADER_CS (0x67u)   /* GNGSV */
#define GPGSV_HEADER_CS (0x79u)   /* GPGSV */
#define BDGSV_HEADER_CS (0x68u)   /* BDGSV */
#define GLGSV_HEADER_CS (0x65u)   /* GLGSV */
#define GAGSV_HEADER_CS (0x68u)   /* GAGSV */
#define GQGSV_HEADER_CS (0x78u)   /* GQGSV */
#define GIGSV_HEADER_CS (0x60u)   /* GIGSV */

#define GPS_FIELDS_MAX 24

volatile uint8_t RxBuffer2[GPS_RX_BUFFER_SIZE] = {0};   /* DMA ���ν��ջ��壬�α߽��ɳ��Ⱦ�ȷ���� */
uint32_t gps_rx_read_idx = 0;

typedef enum {
    CMD_NONE = 0,
    CMD_RMC,
    CMD_GGA,
    CMD_GSA,
    CMD_GSV
} CmdId;

/* ================= �ڶ�����ͨ����������״̬ =================
 * �����ܱ� DMA ���λ�����ƽض�������λ�ã�������ĩβ�ǳ����ضϵ㣩��
 * ��Щ״̬�� parse_gps_data �Ķ�ηֶε��ñ��֣�ʵ�ֿ�������� */
typedef struct {
    uint8_t  st;               /* 0=DATA 1=CS1 2=CS2 3=DONE */
    uint8_t  cs;               /* У��ͣ���ֵ�Ѻ�ǰ���Ӳ����������� */
    uint8_t  csrx[2];          /* �յ�������У���ַ� */
    uint8_t  nf;               /* ��ǰ�����ֶα�ţ�1 �� */
    uint32_t fabs[GPS_FIELDS_MAX]; /* ���ֶ��� RxBuffer2 �еľ����±� */
    uint8_t  flen[GPS_FIELDS_MAX]; /* ���ֶγ��� */
} GpsParser;

static GpsParser rmc_p, gga_p, gsa_p, gsv_p;

typedef struct {
    const char *name;          /* ��������5 �ַ��� */
    uint8_t     cs_init;       /* ����ͷУ��ͳ�ֵ */
    GpsParser  *p;
    CmdId       cmd;           /* �� �ñ����������ͬһ��������ж��
                                * talker ���壨GSV ���ǣ���������"���±�+1"�ơ� */
} CmdDef;

static const CmdDef cmd_table[] = {
    { "GNRMC", GNRMC_HEADER_CS, &rmc_p, CMD_RMC },
    { "GNGGA", GNGGA_HEADER_CS, &gga_p, CMD_GGA },
    { "GNGSA", GNGSA_HEADER_CS, &gsa_p, CMD_GSA },
    /* GSV �� talker ���壨GN ��� + ��������������������һ���������� CmdId */
    { "GNGSV", GNGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GPGSV", GPGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "BDGSV", BDGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GLGSV", GLGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GAGSV", GAGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GQGSV", GQGSV_HEADER_CS, &gsv_p, CMD_GSV },
    { "GIGSV", GIGSV_HEADER_CS, &gsv_p, CMD_GSV },
};
#define CMD_COUNT (sizeof(cmd_table) / sizeof(cmd_table[0]))

/* �� CmdId -> ������������·������ȡ�����������ټ���"���±� == CmdId-1"
 *   ���Ǹ�������һ�������ж�� talker ����ʱ�Ͳ��������� */
static GpsParser *const cmd_parser[] = { NULL, &rmc_p, &gga_p, &gsa_p, &gsv_p };

static CmdId    gps_cmd = CMD_NONE;   /* ��ǰ��Ծ�������NONE=���У� */
static uint64_t gps_clk  = 0;         /* ��ǰ��䵽��ʱ�������������״�ʶ��ʱ�̣� */
static uint8_t  gps_talk = 0;         /* ��ǰ GSV �� talker ��� 0..6 */

/* ��ʮ�������ַ�ת��Ϊ��ֵ */
static uint8_t hex2val(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0;
}

/* ȡ�����±괦�ĵ����ֽڣ�Խ��������ĩβ�Զ����ƣ� */
static char buf_at(uint32_t abs_idx)
{
    if (abs_idx >= GPS_RX_BUFFER_SIZE) abs_idx -= GPS_RX_BUFFER_SIZE;
    return (char)RxBuffer2[abs_idx];
}

/* ����䣺���õڶ���״̬����һ�������ֶδӱ�����㿪ʼ */
static void parser_reset(GpsParser *p, uint8_t cs_init, uint32_t first_abs)
{
    p->st = 0;
    p->cs = cs_init;
    p->nf = 1;
    memset(p->fabs, 0, sizeof(p->fabs));
    memset(p->flen, 0, sizeof(p->flen));
    p->fabs[1] = first_abs;
}

/* ================= GPS ������������V3F �� V5F�� =================
 * ÿ��У��ͨ���� RMC/GGA/GSA ��䷢��һ֡��ĳ�ֶα�֡ȱʧʱ��Ӧ flags λΪ 0��
 * ��ֵͬʱ���㣨��ʽ�ձ�־���п�һ�ɲ� flags λ������ֵ�пգ���
 * �ٶ��� float �� m/s�����ౣ�ֶ���������lat/lon ��1e7 �ȡ�hdop/pdop/vdop ��100��alt cm����
 * �ִ�������Ȼ���������Ҿ�γ���� float ���Ȳ�������˸����ȡ��� mipc_shm.h�� */

/* ��ȡ�ֶ��ı���ջ���壨�ֶζ̣��������±����ư�ȫ�� */
static void field_copy(char *dst, uint8_t dst_sz, uint32_t fabs, uint8_t flen)
{
    uint8_t i;
    if (flen >= dst_sz) flen = dst_sz - 1;
    for (i = 0; i < flen; i++) dst[i] = buf_at(fabs + i);
    dst[flen] = '\0';
}

/* "����[.С��]" �� ��10^decimals����/�Ƿ����� 0�� */
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

/* "ddmm.mmmmm"���ȷ֣��� �� ��1e7���޷��ţ������� N/S/E/W �ֶξ����� */
static int32_t latlon_to_e7(const char *s)
{
    int32_t ddmm = 0, m;
    int64_t frac = 0, p10 = 1;
    int fd = 0;
    while (*s >= '0' && *s <= '9') { ddmm = ddmm * 10 + (*s - '0'); s++; }
    m = ddmm % 100;                              /* �� */
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { frac = frac * 10 + (*s - '0'); fd++; s++; }
    }
    while (fd--) p10 *= 10;
    return (int32_t)((int64_t)(ddmm / 100) * 10000000LL
                   + (m * p10 + frac) * 10000000LL / (60LL * p10));
}

/* "x.yyy" �� �� m/s��1 �� = 0.5144444 m/s����ȡ 3 λС����float ��� ~1e-7��ԶС�ڽ��ջ� 0.05 m/s(1��) */
static float knots_to_mps(const char *s)
{
    return (float)parse_fixed(s, 3) * 1e-3f * 0.51444444f;
}

/* "x.yy" �� �� �ȣ�float����RMC �� course �� magnetic variation ���������ʽ */
static float deg_from_nmea(const char *s)
{
    return (float)parse_fixed(s, 2) * 1e-2f;
}

/* RMC ͨ�����������屾ͨ���ֶΣ���֡ȱʧ��ʽ�գ��������� ts + cnt++ */
static void gps_publish_rmc(void)
{
    char t[16];
    /* ��֡ȱʧ���ֶ�����ʽ��ղ��� flags λ��0 ֵ����Ϸ����п�һ�ɲ� flags�� */
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
    /* �ֶ� 8���Եغ����汱��0~360 �ȣ���
     * �� EKF ���� + speed_mps ��������ٶ�ʸ������/����������
     * ֻ�� speed_mps ����ʱ���ٶ�ֻ��Լ��һ������ */
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
    /* �ֶ� 10/11�����ջ��Ա��Ĵ�ƫ�ǣ�������������
     * �� ���ǽ��ջ�**�ڲ� WMM** ������ģ����Ժ��������߲�� WMM ֵ����У��
     * ����������ֵ -7.53 �ȣ���ע���� course �죺
     * course �Ǻ�������汱����magvar �Ǵű�����汱��ƫ�ǡ� */
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

/* GGA ͨ������ */
static void gps_publish_gga(void)
{
    char t[16];
    g_shm->gps_gga.flags = 0;
    g_shm->gps_gga.quality = 0; g_shm->gps_gga.sv = 0;
    g_shm->gps_gga.hdop_x100 = 0; g_shm->gps_gga.alt_cm = 0;

    if (gga_p.flen[6] > 0) {
        g_shm->gps_gga.flags |= SHM_GGA_QUALITY;
        g_shm->gps_gga.quality = (uint8_t)(buf_at(gga_p.fabs[6]) - '0');   /* '0'-'6' �� ��ֵ 0-6 */
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

/* GSA ͨ����������ϵͳһ��������һ�����ǣ� */
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

/* GSV �ۼ�����һ���ִ�ͨ���� 2~3 ����Ϣ��ÿ�� 4 ���ǣ�ֻ�����һ������ */

/* ---- GSV���� talker �ۻ���7 ���������"���һ��"��----
 * �� ���ܰ�ʱ�䴰�ۺϣ�GSV ���Ŷ�λ�����ߣ�$PCAS02,100 => 10 Hz����һ���ִ��� 7 ��
 *   talker �ı��Ķ�����ͬһ�� 100 ms �����ڣ����ڱ��ļ��ֻ�м����� ���� �κ�
 *   "��� > ��ֵ����һ��"����������������flush ����ִ�У�ʵ�� gsv_cnt �� 0����
 *   ��Ϊ�ṹ���ж���msg==1 ��һ�֣�msg==tot �ύ�� talker ��һ�֣�
 *   ÿ�ύһ�ξͰ����� talker �����һ��**���**�������������κ�ʱ����ֵ��
 * �� ��Ұ�������ֶ� 3��ֻ�� msg==1 ʱȡ ���� ͬһ talker �Ķ�����Ϣ������ͬһ������
 *   ÿ�����ӻ��ظ��������� talker ��Ͳ���ȫ���������
 * �� ĳ talker ���һ�ֳ��� GSV_STALE_TICKS û���¾ʹӺϼ����޳����������ֵ���� */
#define GSV_NTALK        7u              /* �� cmd_table �� GSV �� talker ������һ�� */
#define GSV_STALE_TICKS  2000000000ULL   /* 2 s��10 ns ������ */

typedef struct {
    uint16_t sum;      /* ���֣�C/N0 ֮�� */
    uint8_t  view;     /* ���֣���Ұ���� */
    uint8_t  n;        /* ���֣���Ч C/N0 ���� */
    uint8_t  min;      /* ���֣������� */
    uint8_t  have;     /* ������������ */
    uint16_t v_sum;    /* �����ɵ�һ�� */
    uint8_t  v_view;
    uint8_t  v_n;
    uint8_t  v_min;
    uint8_t  v_ok;
    uint64_t v_clk;    /* ������һ�ֵ�ʱ�� */
} gsv_talk_t;

static gsv_talk_t s_gsv[GSV_NTALK];

/* ������ talker �����һ����ͺ�д�������� */
static void gps_gsv_publish(void)
{
    uint32_t vsum = 0u;
    uint16_t vview = 0u, vn = 0u, vmin = 0u, ntk = 0u;
    uint8_t  i;

    for (i = 0u; i < GSV_NTALK; i++) {
        if (s_gsv[i].v_ok == 0u) continue;
        if ((gps_clk - s_gsv[i].v_clk) > GSV_STALE_TICKS) {
            s_gsv[i].v_ok = 0u;              /* ���ڣ��޳� */
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
    if (msg == 1u) {                        /* �� talker ��һ�� */
        s->sum = 0u; s->n = 0u; s->min = 0u; s->view = 0u;
        if (gsv_p.flen[3] > 0u) {
            field_copy(t, sizeof t, gsv_p.fabs[3], gsv_p.flen[3]);
            s->view = (uint8_t)parse_fixed(t, 0);
        }
        s->have = 1u;
    }
    if (s->have == 0u) return;

    for (k = 0u; k < 4u; k++) {             /* SNR ���ֶ� 7/11/15/19 */
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
    if (tot != 0u && msg == tot) {          /* �� talker һ�ֽ������ύ */
        s->v_sum = s->sum;  s->v_view = s->view;
        s->v_n   = s->n;    s->v_min  = s->min;
        s->v_ok  = 1u;      s->v_clk  = gps_clk;
        s->have  = 0u;
    }
    /* ÿ�� GSV ������������ gsv_cnt �Ľ��� == GSV ���ĵĵ����ʣ���ֱ�Ӷ���
     * "���ջ�������û�з� GSV"�������ٿ��ƶϡ�talkers ��ֻͳ��������ִε� talker�� */
    gps_gsv_publish();
}

/* ����ǰ��Ծ�������Ӧ���֡ */
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

/* ================= �ڶ�����ͨ��������������״̬���� =================
 * У��ͳ�ֵ = ����ͷ�������parser_reset ���裩��ֻ�Ӷ��ź��
 * ��һ�������ֶμ��������ֶμ�¼��У��͵�����ɡ�
 * ���ر��������ѵ��ֽ����������β�����δ����β��gps_cmd ���ַ�
 * CMD_NONE �ȴ���һ��������
 */
static uint32_t parse_body(uint8_t *data, uint32_t len, uint32_t abs_base, GpsParser *p)
{
    uint32_t consumed = 0;
    uint32_t cur_abs  = abs_base;
    uint8_t  c;

    while (consumed < len) {
        c = data[consumed];
        switch (p->st) {
            case 0:                          /* DATA�������ֶ� */
                if (c == ',') {
                    p->cs ^= c;
                    p->nf++;
                    if (p->nf < GPS_FIELDS_MAX) {
                        p->fabs[p->nf] = cur_abs + 1;
                        p->flen[p->nf] = 0;
                    }
                } else if (c == '*') {
                    p->st = 1;               /* '*' ������У��� */
                } else if (c == '\r' || c == '\n') {
                    p->st = 3;               /* ����ȱ������ */
                } else {
                    p->cs ^= c;
                    if (p->nf < GPS_FIELDS_MAX) p->flen[p->nf]++;
                }
                break;

            case 1:                          /* CS1��У��͵�һ�ַ� */
                if (c == '\r' || c == '\n') {
                    p->st = 3;
                } else {
                    p->csrx[0] = c;
                    p->st = 2;
                }
                break;

            case 2:                          /* CS2��У��͵ڶ��ַ����ж� */
                if (c == '\r' || c == '\n') {
                    p->st = 3;
                } else {
                    p->csrx[1] = c;
                    if ((uint8_t)((hex2val((char)p->csrx[0]) << 4) |
                                  hex2val((char)p->csrx[1])) == p->cs) {
                        /* С�˸��� mipc_v3_printf ת�����ں�GPS ����ֱ�� printf��ͬһ USART ��ͻ������ӡͣ�� */
                        // print_parsed();
                        gps_publish_frame();
                    }
                    p->st = 3;
                }
                break;

            default:                         /* DONE������β */
                break;
        }
        consumed++;
        cur_abs++;

        if (p->st == 3 && (c == '\r' || c == '\n')) {
            /* ����������β�����ѣ�����λ��������һ�� */
            p->st    = 0;
            gps_cmd  = CMD_NONE;
            return consumed;
        }
    }

    /* ��β�����δ��ɣ�gps_cmd ���ַ� NONE������һ������ */
    return consumed;
}

/* ================= ��һ���������� =================
 * WAIT_START   �� '$'
 * IN_HEADER    ������ͷֱ����һ�� ','����������ж���������
 * SKIP_LINE    δ���е������������ã�������β��
 * ���к��������ɸ���Ӧ�ڶ������Ӷ��ź��������
 * ǰ���У��ͳ�ֵ�ɵڶ�����������������Ӳ�����ã����贫�ݡ�
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

    /* ��һ�������δ���꣺���ν�����Ӧ�ڶ��������������ٴ�ʣ�ദ���� */
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
                    /* ����ͷ�����������������ɵڶ�����δ������������ */
                    if (hdr_len == 5) {
                        for (uint8_t i = 0; i < CMD_COUNT; i++) {
                            if (memcmp(hdr, cmd_table[i].name, 5) == 0) {
                                gps_cmd = cmd_table[i].cmd;
                                /* GSV �� 7 �� talker ���壨���±� 3~9�����������Ŵ���ȥ��
                                 * ���� 7 �� talker ��ͳ�ƻ����һ�� */
                                gps_talk = (gps_cmd == CMD_GSV) ? (uint8_t)(i - 3u) : 0u;
                                gps_clk = clk;
                                parser_reset(cmd_table[i].p, cmd_table[i].cs_init,
                                             abs_base + pos + 1);
                                uint32_t n = parse_body(data + pos + 1,
                                                        len - (pos + 1),
                                                        abs_base + pos + 1,
                                                        cmd_table[i].p);
                                pos += 1 + n;   /* �������ź�������䣨�����ѵ���β�� */
                                st = WAIT_START; /* ��λ��һ��������һ�� '$' */
                                if (gps_cmd != CMD_NONE) return;
                                continue;
                            }
                        }
                    }
                    st = SKIP_LINE;
                } else if (c == '\r' || c == '\n') {
                    st = WAIT_START;            /* ��ȱ��� */
                } else if (hdr_len < (uint8_t)sizeof(hdr)) {
                    hdr[hdr_len++] = c;
                } else {
                    st = SKIP_LINE;             /* ����ͷ���������������ѵǼ����� */
                }
                break;

            case SKIP_LINE:
                if (c == '\r' || c == '\n') st = WAIT_START;
                break;
        }
        pos++;
    }
}

/* ͨ�� USART3 TX ��ģ�鷢��һ��ָ�������ѯ������̣ܶ���ȫ����
 * ָ�����Դ� NMEA У��ͣ����� 10Hz��GPS_Send_Cmd("$PCAS02,100*1E\r\n"); */
void GPS_Send_Cmd(const char *cmd)
{
    while (*cmd) {
        while ((USART3->STATR & 0x80) == 0);   /* �� TXE�����ͼĴ����գ� */
        USART3->DATAR = (uint8_t)*cmd++;
    }
    while ((USART3->STATR & 0x40) == 0);       /* �� TC��������ɣ� */
}

void GPS_USART_Init()
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA2, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DATAR);
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

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART3, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB, ENABLE);

    // USART3_RX PB11(AF7)
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF7);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3_TX PB10(AF7)
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF7);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Low;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART_InitStructure);

    DMA_Cmd(DMA2_Channel2, ENABLE); /* USART3 Rx */

    USART_Cmd(USART3, ENABLE);

    DMA_MuxChannelConfig(DMA_MuxChannel10, 90);  /* USART3_RX */

    /* �ԣ���λ����Ƶ����Ϊ 10Hz��100ms�����ָ� 1Hz �� "$PCAS02,1000*2E\r\n" */
    GPS_Send_Cmd("$PCAS02,100*1E\r\n");

    /* ��������RMC/GGA/GSA/GSV ÿ֡��VTG/ZDA/GLL �رգ�GSV �����ϱ� C/N0 ����ȣ� */
    GPS_Send_Cmd("$PCAS03,1,0,1,1,1,0,0,0,0,0,,,1,1,,,,0*32\r\n");
    // GPS_Send_Cmd("$PCAS03,1,1,1,1,1,1,1,1,1,1,0,0,,,1,1,,,,1*33\r\n");
}

/* ����ָ���ط���$PCAS02/$PCAS03 �� GPS_USART_Init() ��ֻ��һ�Σ�����һ�� GPS ģ��
 * ������ܻ�û����꣨ģ�����Ҫ 100~500 ms������������ģ����䵽Ĭ������
 * ���� ���־���"�ж�λ��RMC/GGA/GSA ���������� GSV һ����û��"��ʵ�����μ�¼ gsv_cnt �� 0��
 * ��ͬһ�������·���ϵ� GSA ������ȫ������˵����������ͷ��ƥ�䣩��
 * �ϵ�������ʱ���ط����μ��ɣ��ط��ڼ���ջ��������С�һ���յõ���
 * ��λ 10 ns���� gps_clk ͬ������ */
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
        GPS_Send_Cmd("$PCAS02,100*1E\r\n");      /* 10 Hz ��λ������ */
        GPS_Send_Cmd("$PCAS03,1,0,1,1,1,0,0,0,0,0,,,1,1,,,,0*32\r\n");  /* �� GSV */
        s_cfg_n++;
    }
}

void GPS_Check()
{
    gps_cfg_retry();   /* ����ָ���ط������� */

    /* ���㵱ǰдָ��λ�ã�������ƣ� */
    uint32_t wr_idx = GPS_RX_BUFFER_SIZE - DMA2_Channel2->CNTR;
    if (wr_idx == GPS_RX_BUFFER_SIZE) wr_idx = 0;

    if (wr_idx != gps_rx_read_idx) {
        uint64_t clk = GetTime64_10Ns();   /* ��䵽��ʱ�̣���ȫ������ͳһ 10 ns ���� */
        if (wr_idx > gps_rx_read_idx) {
            /* ��������������ȷ���ȴ������д�ڱ���DMA �Ḳ���ڱ�������������ݣ� */
            parse_gps_data((uint8_t*)&RxBuffer2[gps_rx_read_idx],
                           wr_idx - gps_rx_read_idx,
                           gps_rx_read_idx, clk);
        } else {
            /* ���ݻ��ƣ������δ��� */
            uint32_t len1 = GPS_RX_BUFFER_SIZE - gps_rx_read_idx;
            if (len1 > 0) {
                parse_gps_data((uint8_t*)&RxBuffer2[gps_rx_read_idx],
                               len1, gps_rx_read_idx, clk);
            }
            if (wr_idx > 0) {
                parse_gps_data((uint8_t*)&RxBuffer2[0],
                               wr_idx, 0, clk);
            }
        }
        gps_rx_read_idx = wr_idx;  /* ���¶����� */
    }
}
