#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static char *trim(char *s) {
    while (isspace((unsigned char)*s)) s++;
    if (*s == '\0') return s;
    char *end = s + strlen(s) - 1;
    while (end > s && isspace((unsigned char)*end)) end--;
    *(end + 1) = '\0';
    return s;
}

int config_load(const char *path, Config *cfg) {
    strncpy(cfg->port, "/dev/cu.usbserial-TGJLZ4T1", CONFIG_MAX_PATH - 1);
    cfg->port[CONFIG_MAX_PATH - 1] = '\0';
    cfg->baud_rate = 115200;

    FILE *f = fopen(path, "r");
    if (!f) return -1;

    char line[512];
    while (fgets(line, sizeof(line), f)) {
        char *p = trim(line);
        if (*p == '#' || *p == '\0') continue;

        char *eq = strchr(p, '=');
        if (!eq) continue;

        *eq = '\0';
        char *key = trim(p);
        char *val = trim(eq + 1);

        if (strcmp(key, "port") == 0) {
            strncpy(cfg->port, val, CONFIG_MAX_PATH - 1);
            cfg->port[CONFIG_MAX_PATH - 1] = '\0';
        } else if (strcmp(key, "baud_rate") == 0) {
            cfg->baud_rate = atoi(val);
        }
    }

    fclose(f);
    return 0;
}
