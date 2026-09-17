/* Debug console over ARM semihosting.
 *
 * SYS_WRITE0 (operation 0x04) with a NUL-terminated string; BKPT 0xAB traps to
 * the debugger or emulator. Uses no peripheral, so it cannot collide with the
 * Modbus UART.
 */
#include <stddef.h>

void uart_puts(const char *s);
void uart_puts(const char *s)
{
    __asm volatile (
        "mov r0, #0x04\n"
        "mov r1, %0\n"
        "bkpt #0xAB\n"
        :
        : "r" (s)
        : "r0", "r1", "memory");
}

/* newlib syscall stubs required to link the printf family on bare metal. */
int  _close(int f)                { (void)f; return -1; }
int  _fstat(int f, void *st)      { (void)f; (void)st; return 0; }
int  _isatty(int f)               { (void)f; return 1; }
int  _lseek(int f, int p, int w)  { (void)f; (void)p; (void)w; return 0; }
int  _read(int f, char *b, int l) { (void)f; (void)b; (void)l; return 0; }
int  _write(int f, const char *b, int l) { (void)f; (void)b; return l; }
void _exit(int c)                 { (void)c; for (;;) { } }
int  _kill(int p, int s)          { (void)p; (void)s; return -1; }
int  _getpid(void)                { return 1; }
extern char end;
static char *heap = 0;
void *_sbrk(int incr)
{
    char *prev;
    if (heap == 0) heap = &end;
    prev = heap; heap += incr; return prev;
}
