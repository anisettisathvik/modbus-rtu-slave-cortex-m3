/* Modbus RTU slave firmware for Cortex-M3 (MPS2-AN385).
 *
 * Runs the same protocol layer verified on the host. This file contains only
 * the frame loop: take a frame delimited by the port layer's gap timer, pass it
 * to the protocol layer, transmit any response.
 */
#include <stdio.h>
#include <string.h>
#include "modbus_slave.h"
#include "modbus_crc.h"
#include "mb_port.h"

uint16_t mb_port_get_frame(uint8_t *dst, uint16_t max_len);
uint32_t mb_port_rx_overruns(void);
void     mb_port_tick_1ms(void);
void     uart_puts(const char *s);

extern volatile uint32_t mb_stat_rx_bytes, mb_stat_frames,
                         mb_stat_t35_expiry, mb_stat_de_asserts;

#define REGS 32
static uint16_t   hold[REGS];
static mb_slave_t slave;

/* SysTick at 1 kHz drives the millisecond counter. */
#define SYST_CSR   (*(volatile uint32_t *)0xE000E010u)
#define SYST_RVR   (*(volatile uint32_t *)0xE000E014u)
#define SYST_CVR   (*(volatile uint32_t *)0xE000E018u)

void SysTick_Handler(void);
void SysTick_Handler(void) { mb_port_tick_1ms(); }

static void systick_init(void)
{
    SYST_RVR = (25000000u / 1000u) - 1u;
    SYST_CVR = 0u;
    SYST_CSR = 0x07u;      /* clock source = core, interrupt enable, enable */
}

/* Drives known frames directly into the protocol layer so the firmware can
 * verify itself on the target with no master attached. */
static int selftest(void)
{
    uint8_t  req[MB_ADU_MAX], resp[MB_ADU_MAX];
    uint16_t rl, n;
    uint16_t crc;
    int failures = 0;
    char line[128];

    #define FRAME(body, len) do {                       \
        memcpy(req, body, len);                          \
        crc = mb_crc16(req, len);                        \
        req[len] = (uint8_t)(crc & 0xFF);                \
        req[(len)+1] = (uint8_t)(crc >> 8);              \
        n = (uint16_t)((len) + 2);                       \
    } while (0)

    {   const uint8_t b[] = {0x11,0x03,0x00,0x00,0x00,0x02};
        FRAME(b, 6);
        if (mb_slave_handle(&slave, req, n, resp, &rl) != MB_RESULT_RESPOND ||
            rl != 9 || mb_crc16(resp, rl) != 0) failures++;
        snprintf(line, sizeof(line), "  read 2 regs .......... %s (%u B)\n",
                 failures ? "FAIL" : "ok", rl);
        uart_puts(line);
    }
    {   const uint8_t b[] = {0x11,0x06,0x00,0x05,0x12,0x34};
        int f0 = failures;
        FRAME(b, 6);
        if (mb_slave_handle(&slave, req, n, resp, &rl) != MB_RESULT_RESPOND ||
            hold[5] != 0x1234) failures++;
        snprintf(line, sizeof(line), "  write single ......... %s\n",
                 (failures > f0) ? "FAIL" : "ok");
        uart_puts(line);
    }
    {   const uint8_t b[] = {0x11,0x03,0x00,0x00,0x00,0x00};
        int f0 = failures;
        FRAME(b, 6);
        if (mb_slave_handle(&slave, req, n, resp, &rl) != MB_RESULT_RESPOND ||
            resp[1] != 0x83 || resp[2] != MB_EX_ILLEGAL_VALUE) failures++;
        snprintf(line, sizeof(line), "  qty=0 exception ...... %s\n",
                 (failures > f0) ? "FAIL" : "ok");
        uart_puts(line);
    }
    {   const uint8_t b[] = {0x11,0x03,0x00,0x00,0x00,0x01};
        int f0 = failures;
        FRAME(b, 6);
        req[n-1] ^= 0xFF;
        if (mb_slave_handle(&slave, req, n, resp, &rl) != MB_RESULT_BAD_CRC) failures++;
        snprintf(line, sizeof(line), "  corrupt CRC dropped .. %s\n",
                 (failures > f0) ? "FAIL" : "ok");
        uart_puts(line);
    }
    {   const uint8_t b[] = {0x00,0x06,0x00,0x0A,0xCA,0xFE};
        int f0 = failures;
        FRAME(b, 6);
        if (mb_slave_handle(&slave, req, n, resp, &rl) != MB_RESULT_SILENT ||
            hold[10] != 0xCAFE) failures++;
        snprintf(line, sizeof(line), "  broadcast silent ..... %s\n",
                 (failures > f0) ? "FAIL" : "ok");
        uart_puts(line);
    }
    return failures;
}

int main(void)
{
    uint8_t  req[MB_ADU_MAX], resp[MB_ADU_MAX];
    uint16_t req_len, resp_len;
    char     line[160];
    uint32_t handled = 0;

    systick_init();
    mb_slave_init(&slave, 0x11, hold, REGS);
    for (int i = 0; i < REGS; i++) hold[i] = (uint16_t)(0x1000 + i);

    mb_port_uart_init(19200);

    uart_puts("\n=== Modbus RTU Slave, Cortex-M3 ===\n");
    uart_puts("MPS2-AN385, UART RX interrupt + hardware t3.5 timer\n");
    uart_puts("protocol layer identical to the host-verified build\n\n");
    uart_puts("self-test:\n");

    const int failures = selftest();
    snprintf(line, sizeof(line), "\nself-test: %s\n",
             failures ? "FAILURES" : "5/5 passed");
    uart_puts(line);

    snprintf(line, sizeof(line),
             "\nt3.5 gap at 19200 baud = %lu core cycles (%lu us)\n",
             (unsigned long)((25000000ul * 385ul) / (19200ul * 10ul)),
             (unsigned long)(((25000000ul * 385ul) / (19200ul * 10ul)) / 25ul));
    uart_puts(line);

    uart_puts("\nlistening on UART0 for Modbus frames...\n");

        /* The loop does not exit: the firmware must remain available for frames
     * arriving at any time after boot. */
    uint32_t idle = 0;
    for (;;) {
        req_len = mb_port_get_frame(req, MB_ADU_MAX);
        if (req_len) {
            const mb_result_t r =
                mb_slave_handle(&slave, req, req_len, resp, &resp_len);
            if (r == MB_RESULT_RESPOND) {
                mb_port_uart_send(resp, resp_len);
            }
            handled++;
            idle = 0;
            snprintf(line, sizeof(line), "  rx %u B -> %s\n", req_len,
                     r == MB_RESULT_RESPOND ? "responded" :
                     r == MB_RESULT_SILENT  ? "silent"    :
                     r == MB_RESULT_BAD_CRC ? "crc error" : "length error");
            uart_puts(line);
        } else if (++idle > 40000000u) {
            idle = 0;
            snprintf(line, sizeof(line),
                     "  [stats] handled=%lu frames=%lu rx_bytes=%lu "
                     "t35=%lu de=%lu overruns=%lu up=%lums\n",
                     (unsigned long)handled,
                     (unsigned long)mb_stat_frames,
                     (unsigned long)mb_stat_rx_bytes,
                     (unsigned long)mb_stat_t35_expiry,
                     (unsigned long)mb_stat_de_asserts,
                     (unsigned long)mb_port_rx_overruns(),
                     (unsigned long)mb_port_millis());
            uart_puts(line);
        }
    }
}
