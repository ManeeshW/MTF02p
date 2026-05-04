#include "mtf01.h"
#include <sys/time.h>

/*
 * MSP2 frame layout (MTF-01 output):
 *
 *   0x24 '$'   preamble 1
 *   0x58 'X'   preamble 2
 *   0x3C '<'   direction (sensor → host)
 *   flags      1 byte  (0x00)
 *   function   2 bytes little-endian
 *   size       2 bytes little-endian  (payload byte count)
 *   payload    <size> bytes
 *   crc        1 byte  CRC8 DVB-S2 over flags..payload
 */

#define MSP2_MAX_PAYLOAD 255

typedef struct {
    uint8_t  status;
    uint8_t  flags;
    uint16_t function;
    uint16_t size;
    uint16_t payload_cnt;
    uint8_t  payload[MSP2_MAX_PAYLOAD];
    uint8_t  crc_accum;
} MSP2_t;

static micolink_range_callback_t       g_range_cb;
static MICOLINK_Stats_t                g_stats;
static MICOLINK_PAYLOAD_RANGE_SENSOR_t g_data;

void micolink_set_range_callback(micolink_range_callback_t cb) { g_range_cb = cb; }
void micolink_get_stats(MICOLINK_Stats_t *out)                 { *out = g_stats; }

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t b)
{
    crc ^= b;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

static void dispatch(const MSP2_t *m)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    g_data.time_ms = (uint32_t)(tv.tv_sec * 1000u + tv.tv_usec / 1000u);

    switch (m->function) {
    case MSP2_SENSOR_RANGEFINDER:
        if (m->size >= 5) {
            int32_t dist;
            memcpy(&dist, &m->payload[1], 4);
            g_data.strength   = m->payload[0];
            g_data.distance   = (dist > 0) ? (uint32_t)dist : 0;
            g_data.dis_status = (dist > 0 && m->payload[0] >= MSP2_RANGE_QUALITY_MIN) ? 1 : 0;
            g_data.precision  = 0;
        }
        break;

    case MSP2_SENSOR_OPTIC_FLOW:
        if (m->size >= 9) {
            int32_t vx, vy;
            memcpy(&vx, &m->payload[1], 4);
            memcpy(&vy, &m->payload[5], 4);
            g_data.flow_quality = m->payload[0];
            g_data.flow_vel_x   = (int16_t)(vx >  32767 ?  32767 : vx < -32768 ? -32768 : vx);
            g_data.flow_vel_y   = (int16_t)(vy >  32767 ?  32767 : vy < -32768 ? -32768 : vy);
            g_data.flow_status  = (m->payload[0] >= MSP2_FLOW_QUALITY_MIN) ? 1 : 0;
        }
        break;

    default:
        return; /* unknown function — skip callback */
    }

    if (g_range_cb)
        g_range_cb(&g_data);
}

void micolink_decode(uint8_t data)
{
    static MSP2_t m;

    switch (m.status) {
    case 0: /* wait for '$' (0x24) */
        if (data == 0x24) m.status++;
        break;

    case 1: /* wait for 'X' (0x58) */
        if (data == 0x58) { m.status++; g_stats.headers_found++; }
        else              m.status = 0;
        break;

    case 2: /* direction byte — accept any, start fresh CRC */
        m.crc_accum = 0;
        m.status++;
        break;

    case 3: /* flags */
        m.flags     = data;
        m.crc_accum = crc8_dvb_s2(m.crc_accum, data);
        m.status++;
        break;

    case 4: /* function low byte */
        m.function  = data;
        m.crc_accum = crc8_dvb_s2(m.crc_accum, data);
        m.status++;
        break;

    case 5: /* function high byte */
        m.function |= (uint16_t)data << 8;
        m.crc_accum = crc8_dvb_s2(m.crc_accum, data);
        m.status++;
        break;

    case 6: /* size low byte */
        m.size      = data;
        m.crc_accum = crc8_dvb_s2(m.crc_accum, data);
        m.status++;
        break;

    case 7: /* size high byte */
        m.size     |= (uint16_t)data << 8;
        m.crc_accum = crc8_dvb_s2(m.crc_accum, data);
        m.payload_cnt = 0;
        if      (m.size == 0)                m.status = 9; /* no payload, go to CRC */
        else if (m.size > MSP2_MAX_PAYLOAD)  m.status = 0; /* too large, discard    */
        else                                 m.status = 8;
        break;

    case 8: /* payload bytes */
        m.payload[m.payload_cnt++] = data;
        m.crc_accum = crc8_dvb_s2(m.crc_accum, data);
        if (m.payload_cnt >= m.size) m.status++;
        break;

    case 9: /* CRC byte */
        if (data == m.crc_accum) {
            g_stats.valid_packets++;
            dispatch(&m);
        } else {
            g_stats.checksum_errors++;
        }
        m.status = 0;
        break;

    default:
        m.status = 0;
        break;
    }
}
