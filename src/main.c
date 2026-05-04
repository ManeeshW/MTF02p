#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/select.h>

#include "mtf01.h"
#include "config.h"
#include "serial.h"

static volatile int g_running = 1;

/* frequency tracking */
static uint32_t       g_pkt_in_window = 0;
static struct timeval g_window_start;
static double         g_freq_hz = 0.0;

/* totals */
static uint64_t g_total_bytes   = 0;
static uint64_t g_total_packets = 0;

/* latest decoded payload */
static int                             g_has_data = 0;
static MICOLINK_PAYLOAD_RANGE_SENSOR_t g_latest;

/* capture first 64 raw bytes for diagnostics */
#define HEX_CAP 64
static uint8_t g_hex_buf[HEX_CAP];
static int     g_hex_len = 0;

static void on_signal(int sig) { (void)sig; g_running = 0; }

static void on_range_data(const MICOLINK_PAYLOAD_RANGE_SENSOR_t *p)
{
    struct timeval now;
    gettimeofday(&now, NULL);

    g_pkt_in_window++;
    g_total_packets++;

    double elapsed = (now.tv_sec  - g_window_start.tv_sec) +
                     (now.tv_usec - g_window_start.tv_usec) / 1e6;
    if (elapsed >= 1.0) {
        g_freq_hz       = g_pkt_in_window / elapsed;
        g_pkt_in_window = 0;
        g_window_start  = now;
    }

    g_latest   = *p;
    g_has_data = 1;
}

static void print_data(void)
{
    printf("\033[2J\033[H");
    printf("=== MTF-01 Sensor ===\n\n");

    if (!g_has_data) {
        MICOLINK_Stats_t st;
        micolink_get_stats(&st);

        printf("  Waiting for packets...\n\n");
        printf("  rx bytes         = %llu\n",   (unsigned long long)g_total_bytes);
        printf("  headers (0xEF)   = %u\n",     st.headers_found);
        printf("  checksum errors  = %u\n",     st.checksum_errors);
        printf("  valid packets    = %u\n\n",   st.valid_packets);

        if (g_hex_len > 0) {
            printf("  First %d bytes (raw hex):\n  ", g_hex_len);
            for (int i = 0; i < g_hex_len; i++) {
                printf("%02X ", g_hex_buf[i]);
                if ((i + 1) % 16 == 0 && i + 1 < g_hex_len)
                    printf("\n  ");
            }
            printf("\n");
        }
        return;
    }

    printf("  distance           = %u mm\n",       g_latest.distance);
    printf("  distance strength  = %u\n",           g_latest.strength);
    printf("  distance precision = %u\n",           g_latest.precision);
    printf("  distance status    = %u\n",           g_latest.dis_status);
    printf("  flow velocity x    = %d cm/s@1m\n",  g_latest.flow_vel_x);
    printf("  flow velocity y    = %d cm/s@1m\n",  g_latest.flow_vel_y);
    printf("  flow quality       = %u\n",           g_latest.flow_quality);
    printf("  flow status        = %u\n",           g_latest.flow_status);
    printf("\n");
    printf("  Frequency          = %.1f Hz\n",     g_freq_hz);
    printf("  time_ms            = %u ms\n",        g_latest.time_ms);
    printf("  total packets      = %llu\n",         (unsigned long long)g_total_packets);
    fflush(stdout);
}

int main(int argc, char *argv[])
{
    const char *cfg_path = (argc > 1) ? argv[1] : "../config.cfg";

    Config cfg;
    if (config_load(cfg_path, &cfg) != 0)
        if (config_load("config.cfg", &cfg) != 0)
            fprintf(stderr, "Warning: config not found, using defaults\n");

    printf("Port: %s  Baud: %d\n", cfg.port, cfg.baud_rate);

    int fd = serial_open(cfg.port, cfg.baud_rate);
    if (fd < 0) {
        fprintf(stderr, "Failed to open serial port '%s'\n", cfg.port);
        return 1;
    }

    micolink_set_range_callback(on_range_data);
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    gettimeofday(&g_window_start, NULL);
    struct timeval last_print = g_window_start;

    printf("Streaming... Ctrl+C to quit.\n\n");

    uint8_t buf[256];
    while (g_running) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 250000 };

        int ready = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) { if (!g_running) break; perror("select"); break; }

        if (ready > 0 && FD_ISSET(fd, &rfds)) {
            int n = serial_read(fd, buf, sizeof(buf));
            if (n < 0) { perror("serial_read"); break; }

            /* capture first HEX_CAP bytes for diagnostics */
            if (g_hex_len < HEX_CAP) {
                int copy = n < (HEX_CAP - g_hex_len) ? n : (HEX_CAP - g_hex_len);
                memcpy(g_hex_buf + g_hex_len, buf, copy);
                g_hex_len += copy;
            }

            g_total_bytes += (uint64_t)n;
            for (int i = 0; i < n; i++)
                micolink_decode(buf[i]);
        }

        struct timeval now;
        gettimeofday(&now, NULL);
        double since = (now.tv_sec  - last_print.tv_sec) +
                       (now.tv_usec - last_print.tv_usec) / 1e6;
        if (since >= 0.1) {
            print_data();
            last_print = now;
        }
    }

    serial_close(fd);
    printf("\nDone.\n");
    return 0;
}
