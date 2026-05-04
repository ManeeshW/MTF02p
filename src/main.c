#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/select.h>

#include "zenoh.h"
#include "mtf01.h"
#include "config.h"
#include "serial.h"

static volatile int g_running = 1;
static void on_signal(int sig) { (void)sig; g_running = 0; }

static double now_sec(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1e6;
}

static void print_sensor(const MICOLINK_PAYLOAD_RANGE_SENSOR_t *p, double freq_hz)
{
    printf("\033[2J\033[H");
    printf("=== MTF-01 Sensor  (%.1f Hz) ===\n\n", freq_hz);
    printf("  Rangefinder\n");
    printf("    distance  = %u mm   [status=%u  strength=%u]\n",
           p->distance, p->dis_status, p->strength);
    printf("\n");
    printf("  Optical Flow\n");
    printf("    vel x     = %d cm/s@1m\n", p->flow_vel_x);
    printf("    vel y     = %d cm/s@1m\n", p->flow_vel_y);
    printf("    quality   = %u   [status=%u]\n", p->flow_quality, p->flow_status);
    printf("\n");
    printf("  time_ms     = %u ms\n", p->time_ms);
    fflush(stdout);
}

static int make_json(const MICOLINK_PAYLOAD_RANGE_SENSOR_t *p, char *buf, size_t len)
{
    return snprintf(buf, len,
        "{\"time_ms\":%u,\"distance_mm\":%u,\"dis_status\":%u,\"strength\":%u,"
        "\"flow_vel_x\":%d,\"flow_vel_y\":%d,\"flow_quality\":%u,\"flow_status\":%u}",
        p->time_ms, p->distance, (unsigned)p->dis_status, (unsigned)p->strength,
        p->flow_vel_x, p->flow_vel_y, (unsigned)p->flow_quality, (unsigned)p->flow_status);
}

int main(int argc, char *argv[])
{
    const char *cfg_path = (argc > 1) ? argv[1] : "../config.cfg";

    Config cfg;
    if (config_load(cfg_path, &cfg) != 0)
        if (config_load("config.cfg", &cfg) != 0)
            fprintf(stderr, "Warning: config not found, using defaults\n");

    if (cfg.sensor_hz <= 0) cfg.sensor_hz = 50;

    printf("Port: %s  Baud: %d  Rate: %d Hz  Zenoh: %s\n",
           cfg.port, cfg.baud_rate, cfg.sensor_hz, cfg.zenoh_topic);

    int fd = serial_open(cfg.port, cfg.baud_rate);
    if (fd < 0) {
        fprintf(stderr, "Failed to open serial port '%s'\n", cfg.port);
        return 1;
    }

    /* Zenoh session */
    z_owned_config_t z_cfg;
    z_config_default(&z_cfg);
    z_owned_session_t session;
    if (z_open(&session, z_move(z_cfg), NULL) != Z_OK) {
        fprintf(stderr, "Failed to open Zenoh session\n");
        serial_close(fd);
        return 1;
    }

    z_view_keyexpr_t keyexpr;
    z_view_keyexpr_from_str(&keyexpr, cfg.zenoh_topic);

    z_owned_publisher_t publisher;
    if (z_declare_publisher(z_loan(session), &publisher, z_loan(keyexpr), NULL) != Z_OK) {
        fprintf(stderr, "Failed to declare Zenoh publisher on '%s'\n", cfg.zenoh_topic);
        z_close(z_move(session), NULL);
        serial_close(fd);
        return 1;
    }

    micolink_set_range_callback(NULL);   /* poll style */

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    printf("Streaming on zenoh:%s ... Ctrl+C to quit.\n\n", cfg.zenoh_topic);

    /* select timeout = quarter period so we never block longer than 1/4 packet interval */
    long timeout_us = 1000000L / (cfg.sensor_hz * 4);

    uint8_t buf[256];
    char    json_buf[512];
    double  window_start = now_sec();
    double  last_print   = now_sec();
    uint32_t pkt_window  = 0;
    double   freq_hz     = 0.0;

    MICOLINK_PAYLOAD_RANGE_SENSOR_t data;
    uint32_t last_pkt = 0;

    while (g_running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = timeout_us };

        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) { if (!g_running) break; perror("select"); break; }

        if (ready > 0) {
            int n = serial_read(fd, buf, sizeof(buf));
            if (n < 0) { perror("serial_read"); break; }
            for (int i = 0; i < n; i++)
                micolink_decode(buf[i]);
        }

        /* publish on every new packet — natural sensor frequency */
        MICOLINK_Stats_t st;
        micolink_get_stats(&st);
        if (st.valid_packets != last_pkt) {
            pkt_window += st.valid_packets - last_pkt;
            last_pkt    = st.valid_packets;

            if (micolink_get_data(&data)) {
                int n = make_json(&data, json_buf, sizeof(json_buf));
                if (n > 0 && (size_t)n < sizeof(json_buf)) {
                    z_owned_bytes_t payload;
                    z_bytes_copy_from_buf(&payload, (const uint8_t *)json_buf, (size_t)n);
                    z_publisher_put(z_loan(publisher), z_move(payload), NULL);
                }
            }
        }

        /* frequency tracking (1-second rolling window) */
        double elapsed = now_sec() - window_start;
        if (elapsed >= 1.0) {
            freq_hz      = pkt_window / elapsed;
            pkt_window   = 0;
            window_start = now_sec();
        }

        /* console display at 10 Hz */
        if (now_sec() - last_print >= 0.1) {
            if (micolink_get_data(&data)) {
                print_sensor(&data, freq_hz);
            } else {
                micolink_get_stats(&st);
                printf("\033[2J\033[H");
                printf("=== MTF-01 Sensor ===\n\n");
                printf("  Waiting for packets...\n\n");
                printf("  headers found    = %u\n",  st.headers_found);
                printf("  checksum errors  = %u\n",  st.checksum_errors);
                printf("  valid packets    = %u\n",  st.valid_packets);
                fflush(stdout);
            }
            last_print = now_sec();
        }
    }

    z_undeclare_publisher(z_move(publisher));
    z_close(z_move(session), NULL);
    serial_close(fd);
    printf("\nDone.\n");
    return 0;
}
