#ifndef MB_INVARIANTS_H
#define MB_INVARIANTS_H

#include <stdint.h>
#include <string.h>
#include "modbus_slave.h"
#include "modbus_crc.h"

/* Invariants checked after every Modbus transaction, regardless of input.
 *
 * Both the constrained-random generator and the fuzzer drive stimulus through
 * mb_drive_and_check(), so a violation is caught whichever engine produced it.
 */

#define MB_CANARY_LEN  32u
#define MB_CANARY_BYTE 0x5Au

typedef struct {
    const char *name;
    uint32_t    checks;
    uint32_t    failures;
} mb_prop_t;

enum {
    P_RESP_LEN_BOUND = 0,
    P_RESP_CRC_SELF_CONSISTENT,
    P_RESP_ADDR_IS_OURS,
    P_BROADCAST_NEVER_REPLIES,
    P_FOREIGN_NEVER_REPLIES,
    P_EXCEPTION_SHAPE,
    P_NO_BUFFER_OVERFLOW,
    P_EXCEPTION_IS_ATOMIC,
    P_RESP_LEN_MINIMUM,
    P_COUNTERS_MONOTONIC,
    MB_PROP_COUNT
};

static mb_prop_t mb_props[MB_PROP_COUNT] = {
    { "resp_len never exceeds 256",            0, 0 },
    { "response CRC is self-consistent",       0, 0 },
    { "response carries our unit id",          0, 0 },
    { "broadcast never produces a reply",      0, 0 },
    { "foreign address never gets a reply",    0, 0 },
    { "exception frame is exactly 5 bytes",    0, 0 },
    { "no write past the response buffer",     0, 0 },
    { "rejected request leaves regs untouched",0, 0 },
    { "any reply is at least 4 bytes",         0, 0 },
    { "counters only ever increase",           0, 0 }
};

static void mb_prop_check(int id, int condition)
{
    mb_props[id].checks++;
    if (!condition) {
        mb_props[id].failures++;
    }
}

/* Drive one ADU through the slave and check every invariant.
 * Returns non-zero if any invariant was violated. */
static int mb_drive_and_check(mb_slave_t *s, const uint8_t *req, uint16_t req_len)
{
    uint8_t  resp[MB_ADU_MAX + MB_CANARY_LEN];
    uint16_t resp_len = 0;
    uint16_t regs_before[256];
    uint16_t n_regs = s->holding_count;
    uint32_t responses_before = s->cnt_responses;
    mb_result_t r;
    uint32_t i;
    int failures_before = 0, failures_after = 0;

    for (i = 0; i < MB_PROP_COUNT; i++) failures_before += (int)mb_props[i].failures;

    if (n_regs > 256u) n_regs = 256u;
    memcpy(regs_before, s->holding, (size_t)n_regs * sizeof(uint16_t));

    memset(resp, MB_CANARY_BYTE, sizeof(resp));

    r = mb_slave_handle(s, req, req_len, resp, &resp_len);

    /* canary: the core must never touch the tail of the buffer */
    {
        int intact = 1;
        for (i = MB_ADU_MAX; i < MB_ADU_MAX + MB_CANARY_LEN; i++) {
            if (resp[i] != MB_CANARY_BYTE) { intact = 0; break; }
        }
        mb_prop_check(P_NO_BUFFER_OVERFLOW, intact);
    }

    mb_prop_check(P_COUNTERS_MONOTONIC, s->cnt_responses >= responses_before);

    if (r == MB_RESULT_RESPOND) {
        mb_prop_check(P_RESP_LEN_BOUND, resp_len <= MB_ADU_MAX);
        mb_prop_check(P_RESP_LEN_MINIMUM, resp_len >= 4u);

        if (resp_len >= 4u && resp_len <= MB_ADU_MAX) {
            /* CRC-ing a frame including its own CRC yields zero */
            mb_prop_check(P_RESP_CRC_SELF_CONSISTENT, mb_crc16(resp, resp_len) == 0u);
            mb_prop_check(P_RESP_ADDR_IS_OURS, resp[0] == s->addr);

            if (resp[1] & 0x80u) {
                /* exception: addr + fc|0x80 + code + CRC = 5 bytes, and the
                 * register file must be exactly as it was. A partially applied
                 * write that then reports an error is the worst possible
                 * outcome in an industrial controller. */
                mb_prop_check(P_EXCEPTION_SHAPE, resp_len == 5u);
                mb_prop_check(P_EXCEPTION_IS_ATOMIC,
                    memcmp(regs_before, s->holding,
                           (size_t)n_regs * sizeof(uint16_t)) == 0);
            }
        }

        /* address 0 is broadcast: a reply would collide with every other slave */
        if (req_len >= 1u && req[0] == MB_ADDR_BROADCAST) {
            mb_prop_check(P_BROADCAST_NEVER_REPLIES, 0);
        }
        if (req_len >= 1u && req[0] != MB_ADDR_BROADCAST && req[0] != s->addr) {
            mb_prop_check(P_FOREIGN_NEVER_REPLIES, 0);
        }
    } else {
        if (req_len >= 1u && req[0] == MB_ADDR_BROADCAST) {
            mb_prop_check(P_BROADCAST_NEVER_REPLIES, 1);
        }
        if (req_len >= 1u && req[0] != MB_ADDR_BROADCAST && req[0] != s->addr) {
            mb_prop_check(P_FOREIGN_NEVER_REPLIES, 1);
        }
    }

    for (i = 0; i < MB_PROP_COUNT; i++) failures_after += (int)mb_props[i].failures;
    return failures_after - failures_before;
}

#endif /* MB_INVARIANTS_H */
