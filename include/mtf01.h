#ifndef MTF01_H
#define MTF01_H

#include <stdint.h>

/* MSP2 function codes sent by the MTF-01 */
#define MSP2_SENSOR_RANGEFINDER  0x1F01
#define MSP2_SENSOR_OPTIC_FLOW   0x1F02

/* Minimum signal quality (0-255) required for status = 1 */
#define MSP2_FLOW_QUALITY_MIN   30
#define MSP2_RANGE_QUALITY_MIN   1

/*
 * Combined sensor data populated from the two MSP2 message types:
 *   rangefinder (0x1F01) → distance, strength, dis_status
 *   optic flow  (0x1F02) → flow_vel_x/y, flow_quality, flow_status
 */
#pragma pack(1)
typedef struct {
    uint32_t time_ms;       /* system time (ms)                       */
    uint32_t distance;      /* distance (mm), 0 = unavailable         */
    uint8_t  strength;      /* rangefinder signal quality 0-255       */
    uint8_t  precision;     /* not available via MSP2 (always 0)      */
    uint8_t  dis_status;    /* 1 = valid, 0 = invalid                 */
    uint8_t  reserved1;
    int16_t  flow_vel_x;    /* optical flow velocity x (raw int)      */
    int16_t  flow_vel_y;    /* optical flow velocity y (raw int)      */
    uint8_t  flow_quality;  /* optical flow quality 0-255             */
    uint8_t  flow_status;   /* 1 = valid, 0 = invalid                 */
    uint16_t reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack()

typedef struct {
    uint32_t headers_found;   /* '$X' preambles seen          */
    uint32_t checksum_errors; /* CRC8 mismatches              */
    uint32_t valid_packets;   /* successfully decoded packets */
} MICOLINK_Stats_t;

/* Called on every successfully decoded packet. Optional — see micolink_get_data(). */
typedef void (*micolink_range_callback_t)(const MICOLINK_PAYLOAD_RANGE_SENSOR_t *payload);

/* Feed raw serial bytes one at a time; calls the callback on each complete packet. */
void micolink_decode(uint8_t data);

/* Register a callback to be invoked on each decoded packet. Pass NULL to disable. */
void micolink_set_range_callback(micolink_range_callback_t cb);

/* Poll the latest decoded data. Returns 1 if data is available, 0 if not yet. */
int  micolink_get_data(MICOLINK_PAYLOAD_RANGE_SENSOR_t *out);

/* Copy decoder statistics (packet counts, error counts) into *out. */
void micolink_get_stats(MICOLINK_Stats_t *out);

/* Reset decoder state, statistics, and stored data. */
void micolink_reset(void);

#endif
