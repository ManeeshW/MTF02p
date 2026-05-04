#ifndef CONFIG_H
#define CONFIG_H

#define CONFIG_MAX_PATH 256

typedef struct {
    char port[CONFIG_MAX_PATH];
    int  baud_rate;
} Config;

int config_load(const char *path, Config *cfg);

#endif
