/* libFuzzer harness: coverage-guided arbitrary bytes into the frame parser,
 * checked against the invariants in mb_invariants.h.
 *
 *   make fuzz
 *   ./build/fuzz_slave corpus/ -max_total_time=600
 *
 * A crash is written to a file and replays deterministically:
 *   ./build/fuzz_slave crash-<hash>
 */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "modbus_slave.h"
#include "mb_invariants.h"

#define REGS 32u

static mb_slave_t g_slave;
static uint16_t   g_hold[REGS];
static int        g_ready = 0;

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size);

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    if (!g_ready) {
        mb_slave_init(&g_slave, 0x11u, g_hold, REGS);
        g_ready = 1;
    }
    if (size > MB_ADU_MAX) size = MB_ADU_MAX;

        /* An invariant failure traps, so libFuzzer records it as a crash with a
     * reproducer. */
    if (mb_drive_and_check(&g_slave, data, (uint16_t)size) != 0) {
        fprintf(stderr, "INVARIANT VIOLATED on %zu-byte input\n", size);
        __builtin_trap();
    }
    return 0;
}
