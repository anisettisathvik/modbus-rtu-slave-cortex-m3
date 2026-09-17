#ifndef MB_PORT_H
#define MB_PORT_H

#include <stdint.h>

/* The complete hardware interface of the Modbus port layer.
 *
 * Implemented once per target: port/mb_port_cm3.c for Cortex-M, and
 * test/host_slave.c for the POSIX host build.
 */

/* UART at the given baud, 8E1. Modbus RTU's default framing is 8E1; 8N1 is
 * legal only with two stop bits. */
void mb_port_uart_init(uint32_t baud);

/* RS-485 half duplex: DE/RE high to transmit, low to receive. The driver must
 * not be released until the final stop bit has left the shift register --
 * transmission-complete, not transmit-buffer-empty. */
void mb_port_rs485_tx_enable(void);
void mb_port_rs485_rx_enable(void);

/* Blocking send of a complete ADU. Returns only after transmission completes. */
void mb_port_uart_send(const uint8_t *buf, uint16_t len);

/* Start or restart the 3.5-character inter-frame gap timer. Above 19200 baud
 * the spec fixes this at 1.750 ms; below, it is 3.5 * 11 bit times. The RX
 * interrupt restarts it per byte; its expiry delimits the frame. */
void mb_port_t35_restart(void);
void mb_port_t35_stop(void);

/* Free-running millisecond tick, for timeouts and latency measurement. */
uint32_t mb_port_millis(void);

#endif /* MB_PORT_H */
