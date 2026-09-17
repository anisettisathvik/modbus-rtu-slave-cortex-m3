/* Host runner: drives the Modbus slave core over a POSIX serial port.
 *
 *   ./build/host_slave /dev/pts/N [slave_addr]
 *
 * POSIX VTIME has 100 ms granularity, so frames are delimited here by
 * inactivity rather than by a true 3.5-character gap. Frame timing can only be
 * verified on the target, where a hardware timer measures it.
 */
#define _DEFAULT_SOURCE   /* cfmakeraw is a BSD/POSIX extension, hidden by -std=c99 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "modbus_slave.h"

#define REGS 32

static int open_port(const char *path)
{
    struct termios tio;
    int fd = open(path, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open"); return -1; }

    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) { perror("tcgetattr"); close(fd); return -1; }
    cfmakeraw(&tio);
    cfsetispeed(&tio, B19200);
    cfsetospeed(&tio, B19200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;          /* 100 ms inactivity = end of frame */
    if (tcsetattr(fd, TCSANOW, &tio) != 0) { perror("tcsetattr"); close(fd); return -1; }
    return fd;
}

int main(int argc, char **argv)
{
    mb_slave_t  s;
    uint16_t    hold[REGS];
    uint8_t     rx[MB_ADU_MAX], tx[MB_ADU_MAX];
    uint16_t    rx_len, tx_len;
    int         fd, i;
    ssize_t     n;
    uint8_t     addr = 0x11;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <port> [slave_addr_dec]\n", argv[0]);
        return 2;
    }
    if (argc >= 3) addr = (uint8_t)atoi(argv[2]);

    fd = open_port(argv[1]);
    if (fd < 0) return 1;

    mb_slave_init(&s, addr, hold, REGS);
    for (i = 0; i < REGS; i++) hold[i] = (uint16_t)(0x1000 + i);

    fprintf(stderr, "slave 0x%02X up on %s, %d holding registers\n",
            addr, argv[1], REGS);

    for (;;) {
        rx_len = 0;
                /* Accumulate until a read returns nothing: an inactivity gap. */
        for (;;) {
            n = read(fd, &rx[rx_len], (size_t)(MB_ADU_MAX - rx_len));
            if (n > 0) {
                rx_len = (uint16_t)(rx_len + n);
                if (rx_len >= MB_ADU_MAX) break;
            } else if (n == 0) {
                if (rx_len > 0) break;      /* gap after data = frame end */
            } else {
                perror("read"); close(fd); return 1;
            }
        }

        switch (mb_slave_handle(&s, rx, rx_len, tx, &tx_len)) {
        case MB_RESULT_RESPOND:
                        /* On the target this is where DE is asserted and released. */
            if (write(fd, tx, tx_len) < 0) perror("write");
            fprintf(stderr, "  rx %u B -> tx %u B\n", rx_len, tx_len);
            break;
        case MB_RESULT_SILENT:
            fprintf(stderr, "  rx %u B -> silent\n", rx_len);
            break;
        case MB_RESULT_BAD_CRC:
            fprintf(stderr, "  rx %u B -> CRC error (total %u)\n",
                    rx_len, s.cnt_crc_err);
            break;
        case MB_RESULT_BAD_LENGTH:
            fprintf(stderr, "  rx %u B -> length error (total %u)\n",
                    rx_len, s.cnt_len_err);
            break;
        }
    }
}
