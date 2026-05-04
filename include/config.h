#ifndef CONFIG_H
#define CONFIG_H

#define CONFIG_MAX_PATH 256

typedef struct {
    char port[CONFIG_MAX_PATH];
    int  baud_rate;
    int  sensor_hz;
    char zenoh_topic[CONFIG_MAX_PATH];
} Config;

int config_load(const char *path, Config *cfg);

#endif
