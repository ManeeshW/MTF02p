#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

int  serial_open(const char *port, int baud_rate);
int  serial_read(int fd, uint8_t *buf, int len);
void serial_close(int fd);

#endif
