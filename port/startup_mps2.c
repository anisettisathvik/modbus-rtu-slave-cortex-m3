/* Cortex-M3 startup for the ARM MPS2-AN385: vector table, reset handler, and
 * C runtime initialisation.
 *
 * On reset the core reads the initial main stack pointer from 0x00 and the
 * reset vector from 0x04 before executing any instruction.
 */

#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);

/* SVC, PendSV and SysTick must appear in the vector table for an RTOS
 * scheduler to run. */
void SVC_Handler(void)     __attribute__((weak));
void PendSV_Handler(void)  __attribute__((weak));
void SysTick_Handler(void) __attribute__((weak));
void UART0_Handler(void)   __attribute__((weak));
void TIMER0_Handler(void)  __attribute__((weak));
void UART1_Handler(void)   __attribute__((weak));

void Reset_Handler(void);
static void Default_Handler(void);
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler_Fault")));

void Default_Handler_Fault(void);

__attribute__((section(".isr_vector"), used))
void (* const g_vectors[])(void) = {
    (void (*)(void))&_estack,   /*  0  initial stack pointer */
    Reset_Handler,              /*  1  reset                 */
    Default_Handler,            /*  2  NMI                   */
    Default_Handler_Fault,      /*  3  HardFault             */
    Default_Handler_Fault,      /*  4  MemManage             */
    Default_Handler_Fault,      /*  5  BusFault              */
    Default_Handler_Fault,      /*  6  UsageFault            */
    0, 0, 0, 0,                 /*  7-10 reserved            */
    SVC_Handler,                /* 11  SVCall   -> FreeRTOS  */
    Default_Handler,            /* 12  DebugMon              */
    0,                          /* 13  reserved              */
    PendSV_Handler,             /* 14  PendSV   -> FreeRTOS  */
    SysTick_Handler,            /* 15  SysTick  -> FreeRTOS  */
    /* external interrupts, IRQ0 upward */
    UART0_Handler,              /* 16  IRQ0  UART0 RX  (Modbus bus)  */
    0, 0, 0, 0, 0, 0, 0,        /* 17-23 IRQ1..IRQ7                  */
    TIMER0_Handler,             /* 24  IRQ8  dual timer 0            */
};

void Reset_Handler(void)
{
    uint32_t *src, *dst;

    /* .data is stored in flash and must be copied to RAM before use. */
    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; ) *dst++ = *src++;

    /* .bss must be zeroed: nothing else provides the C guarantee on bare metal. */
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0u;

    (void)main();
    for (;;) { }
}

static void Default_Handler(void)
{
    for (;;) { }
}

void Default_Handler_Fault(void)
{
    /* On a safety controller the output must be cut before halting. The pin is
 * driven directly rather than through any layer that may itself have faulted. */
    volatile uint32_t *gpio_out = (volatile uint32_t *)0x40010004u;
    *gpio_out = 0u;
    for (;;) { }
}

/* Weak defaults, overridden by strong definitions elsewhere in the image. */
__attribute__((weak)) void SVC_Handler(void)     { for (;;) { } }
__attribute__((weak)) void PendSV_Handler(void)  { for (;;) { } }
__attribute__((weak)) void SysTick_Handler(void) { }
__attribute__((weak)) void UART0_Handler(void)   { }
__attribute__((weak)) void TIMER0_Handler(void)  { }
__attribute__((weak)) void UART1_Handler(void)   { }
