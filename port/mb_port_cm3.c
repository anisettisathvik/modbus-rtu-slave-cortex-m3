/* Modbus RTU port layer for Cortex-M, CMSDK peripherals (MPS2-AN385).
 *
 * The only file in the project that touches hardware. Implements the three
 * properties that are timing behaviour rather than protocol behaviour: a UART
 * RX interrupt feeding a ring buffer, a hardware timer measuring the
 * 3.5-character inter-frame gap, and RS-485 DE/RE direction control.
 */

#include <stdint.h>
#include <string.h>
#include "mb_port.h"

/* CMSDK UART0 carries the Modbus bus. The debug console uses semihosting
 * rather than a second UART: sharing a UART between protocol traffic and log
 * output corrupts frames in a way indistinguishable from line noise.
 * UART1 (0x40005000) is not backed on this machine model. */
#define UARTMB_BASE     0x40004000u
#define UART_DATA       (*(volatile uint32_t *)(UARTMB_BASE + 0x00))
#define UART_STATE      (*(volatile uint32_t *)(UARTMB_BASE + 0x04))
#define UART_CTRL       (*(volatile uint32_t *)(UARTMB_BASE + 0x08))
#define UART_INTSTATUS  (*(volatile uint32_t *)(UARTMB_BASE + 0x0C))
#define UART_BAUDDIV    (*(volatile uint32_t *)(UARTMB_BASE + 0x10))

#define UART_STATE_TX_FULL   (1u << 0)
#define UART_STATE_RX_FULL   (1u << 1)
#define UART_CTRL_TX_EN      (1u << 0)
#define UART_CTRL_RX_EN      (1u << 1)
#define UART_CTRL_RX_INT_EN  (1u << 3)
#define UART_INT_RX          (1u << 1)

/* ---- CMSDK dual timer 0, used for the t3.5 gap --------------------------- */
#define TIMER0_BASE     0x40000000u
#define TIMER_CTRL      (*(volatile uint32_t *)(TIMER0_BASE + 0x00))
#define TIMER_VALUE     (*(volatile uint32_t *)(TIMER0_BASE + 0x04))
#define TIMER_RELOAD    (*(volatile uint32_t *)(TIMER0_BASE + 0x08))
#define TIMER_INTSTATUS (*(volatile uint32_t *)(TIMER0_BASE + 0x0C))
#define TIMER_CTRL_EN       (1u << 0)
#define TIMER_CTRL_INT_EN   (1u << 3)

/* ---- GPIO used as the RS-485 DE/RE pin ----------------------------------- */
#define GPIO_DE         (*(volatile uint32_t *)0x40010004u)

/* ---- NVIC ---------------------------------------------------------------- */
#define NVIC_ISER0      (*(volatile uint32_t *)0xE000E100u)
#define IRQ_UART0RX     0
#define IRQ_TIMER0      8

#define SYSTEM_CLOCK_HZ 25000000u

/* RX ring buffer. Single producer (ISR), single consumer (task), power-of-two
 * size, one index owned by each side: the producer only advances head, the
 * consumer only advances tail, and 32-bit aligned access is atomic on
 * Cortex-M, so no critical section is required. A mutex here would be slower
 * and would risk priority inversion against an ISR. */
#define RX_RING_SIZE 512u        /* power of two */
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head = 0u;   /* written by ISR only  */
static volatile uint32_t rx_tail = 0u;   /* written by task only */
static volatile uint32_t rx_overruns = 0u;

/* frame state, owned by the ISR and read by the task */
static volatile uint8_t  frame_ready = 0u;
static volatile uint32_t frame_len   = 0u;
static volatile uint32_t t35_ticks   = 0u;
static volatile uint32_t millis_ctr  = 0u;

/* diagnostics */
volatile uint32_t mb_stat_rx_bytes   = 0u;
volatile uint32_t mb_stat_frames     = 0u;
volatile uint32_t mb_stat_t35_expiry = 0u;
volatile uint32_t mb_stat_de_asserts = 0u;

/* Above 19200 baud the spec fixes the inter-frame gap at 1.750 ms. Below, it
 * is 3.5 character times of 11 bit times each (start, 8 data, parity, stop),
 * i.e. 38.5 bit times. */
static uint32_t t35_reload_for(uint32_t baud)
{
    if (baud > 19200u) {
        return (SYSTEM_CLOCK_HZ / 1000000u) * 1750u;      /* 1.750 ms */
    }
    /* 3.5 chars * 11 bits = 38.5 bit times */
    return (uint32_t)(((uint64_t)SYSTEM_CLOCK_HZ * 385u) / ((uint64_t)baud * 10u));
}
static uint32_t g_t35_reload = 0u;

/* ---- mb_port.h implementation -------------------------------------------- */

void mb_port_uart_init(uint32_t baud)
{
    /* CMSDK divisor; a real STM32 would be USARTx->BRR from the reference
     * manual's oversampling formula. */
    UART_BAUDDIV = (SYSTEM_CLOCK_HZ / baud) < 16u ? 16u : (SYSTEM_CLOCK_HZ / baud);

    rx_head = rx_tail = 0u;
    frame_ready = 0u;
    frame_len = 0u;

    g_t35_reload = t35_reload_for(baud);

    UART_CTRL = UART_CTRL_TX_EN | UART_CTRL_RX_EN | UART_CTRL_RX_INT_EN;
    NVIC_ISER0 |= (1u << IRQ_UART0RX) | (1u << IRQ_TIMER0);

    mb_port_rs485_rx_enable();
}

void mb_port_rs485_tx_enable(void)
{
    GPIO_DE = 1u;
    mb_stat_de_asserts++;
    /* Transceiver driver-enable settling time. Without it the first bit of the
 * first byte is clipped, appearing at the far end as an intermittent CRC
 * failure. */
    for (volatile int i = 0; i < 8; i++) { }
}

void mb_port_rs485_rx_enable(void)
{
    GPIO_DE = 0u;
}

void mb_port_uart_send(const uint8_t *buf, uint16_t len)
{
    mb_port_rs485_tx_enable();

    for (uint16_t i = 0; i < len; i++) {
        while (UART_STATE & UART_STATE_TX_FULL) { }
        UART_DATA = buf[i];
    }

    /* Wait for the shift register to empty, not the holding register. Releasing
 * DE on transmit-buffer-empty truncates the final stop bit on the wire. The
 * CMSDK UART exposes no transmission-complete flag, so the equivalent wait is
 * one character time after the last byte is accepted. */
    while (UART_STATE & UART_STATE_TX_FULL) { }
    for (volatile int i = 0; i < 200; i++) { }

    mb_port_rs485_rx_enable();
}

void mb_port_t35_restart(void)
{
    TIMER_CTRL      = 0u;
    TIMER_INTSTATUS = 1u;
    TIMER_RELOAD    = g_t35_reload;
    TIMER_VALUE     = g_t35_reload;
    TIMER_CTRL      = TIMER_CTRL_EN | TIMER_CTRL_INT_EN;
}

void mb_port_t35_stop(void)
{
    TIMER_CTRL = 0u;
}

uint32_t mb_port_millis(void)
{
    return millis_ctr;
}

/* ---- interrupt handlers -------------------------------------------------- */

void UART0_Handler(void);
void UART0_Handler(void)
{
    UART_INTSTATUS = UART_INT_RX;          /* write 1 to clear */

    while (UART_STATE & UART_STATE_RX_FULL) {
        const uint8_t b = (uint8_t)UART_DATA;
        const uint32_t next = (rx_head + 1u) & (RX_RING_SIZE - 1u);

        if (next == rx_tail) {
            rx_overruns++;                 /* consumer fell behind: drop */
        } else {
            rx_ring[rx_head] = b;
            rx_head = next;
            mb_stat_rx_bytes++;
        }
        /* Every received byte restarts the gap timer. RTU has no framing
 * character: the timer's expiry is the only frame delimiter. */
        mb_port_t35_restart();
    }
}

void TIMER0_Handler(void);
void TIMER0_Handler(void)
{
    TIMER_INTSTATUS = 1u;
    TIMER_CTRL = 0u;

    t35_ticks++;
    mb_stat_t35_expiry++;

    /* Silence for 3.5 character times: the ring now holds exactly one frame. */
    if (rx_head != rx_tail) {
        frame_len   = (rx_head - rx_tail) & (RX_RING_SIZE - 1u);
        frame_ready = 1u;
        mb_stat_frames++;
    }
}

/* Returns the length of a complete frame, or 0. Copies out of the ring so the
 * ISR can continue filling it while the protocol layer runs. */
uint16_t mb_port_get_frame(uint8_t *dst, uint16_t max_len);
uint16_t mb_port_get_frame(uint8_t *dst, uint16_t max_len)
{
    if (!frame_ready) return 0u;

    uint32_t n = frame_len;
    if (n > max_len) n = max_len;

    for (uint32_t i = 0; i < n; i++) {
        dst[i] = rx_ring[(rx_tail + i) & (RX_RING_SIZE - 1u)];
    }
    rx_tail     = (rx_tail + frame_len) & (RX_RING_SIZE - 1u);
    frame_ready = 0u;
    frame_len   = 0u;
    return (uint16_t)n;
}

uint32_t mb_port_rx_overruns(void);
uint32_t mb_port_rx_overruns(void) { return rx_overruns; }

void mb_port_tick_1ms(void);
void mb_port_tick_1ms(void) { millis_ctr++; }
