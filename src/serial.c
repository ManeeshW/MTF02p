#include "serial.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

static speed_t baud_to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            fprintf(stderr, "Unsupported baud rate %d, defaulting to 115200\n", baud);
            return B115200;
    }
}

int serial_open(const char *port, int baud_rate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("serial_open: open");
        return -1;
    }

    fcntl(fd, F_SETFL, 0);  // blocking mode

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        perror("serial_open: tcgetattr");
        close(fd);
        return -1;
    }

    speed_t speed = baud_to_speed(baud_rate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    // 8N1, no flow control, raw mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT |
                     PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tty.c_cc[VMIN]  = 1;   // block until at least 1 byte
    tty.c_cc[VTIME] = 1;   // 100 ms inter-byte timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("serial_open: tcsetattr");
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

int serial_read(int fd, uint8_t *buf, int len) {
    return (int)read(fd, buf, (size_t)len);
}

void serial_close(int fd) {
    close(fd);
}
