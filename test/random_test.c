/* Constrained-random verification of the Modbus RTU slave.
 *
 * Generates weighted-random legal and illegal frames, checks the invariants in
 * mb_invariants.h after every transaction, and samples the coverage model in
 * mb_coverage.h to determine when the input space has been covered.
 *
 *   ./build/random_test [iterations] [seed]
 *
 * The seed is printed and accepted on the command line so any failure replays
 * exactly.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "modbus_slave.h"
#include "modbus_crc.h"
#include "mb_invariants.h"
#include "mb_coverage.h"

#define SLAVE_ADDR 0x11u
#define REGS       32u

/* xorshift32: deterministic and seedable, with no dependence on the host
 * libc RNG. */
static uint32_t rng_state = 1u;
static uint32_t rnd(void)
{
    uint32_t x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}
static uint32_t rnd_range(uint32_t n) { return n ? (rnd() % n) : 0u; }

/* ---- constrained-random frame construction ------------------------------ */

static uint16_t gen_frame(uint8_t *buf)
{
    uint16_t len = 0;
    uint32_t pick;

        /* Addressing weighted toward this slave so most stimulus reaches the
     * protocol decode path. */
    pick = rnd_range(100u);
    if (pick < 75u)      buf[0] = SLAVE_ADDR;
    else if (pick < 90u) buf[0] = MB_ADDR_BROADCAST;
    else                 buf[0] = (uint8_t)(1u + rnd_range(200u));
    if (buf[0] == SLAVE_ADDR && pick >= 90u) buf[0] = 0x22u;

    /* constraint: function code, mostly supported */
    pick = rnd_range(100u);
    if (pick < 35u)      buf[1] = MB_FC_READ_HOLDING;
    else if (pick < 60u) buf[1] = MB_FC_WRITE_SINGLE;
    else if (pick < 85u) buf[1] = MB_FC_WRITE_MULTIPLE;
    else                 buf[1] = (uint8_t)rnd_range(256u);

    if (buf[1] == MB_FC_WRITE_MULTIPLE) {
        uint32_t qty, bc, i;
                /* Quantity biased onto the legal boundary and just past it. */
        pick = rnd_range(100u);
        if (pick < 60u)      qty = 1u + rnd_range(8u);
        else if (pick < 75u) qty = MB_WRITE_MULTI_QTY_MAX;          /* boundary */
        else if (pick < 85u) qty = MB_WRITE_MULTI_QTY_MAX + 1u;     /* just over */
        else                 qty = rnd_range(300u);

        {               /* Start address biased onto the register-file edges. */
            uint32_t start;
            pick = rnd_range(100u);
            if (pick < 25u)      start = 0u;
            else if (pick < 50u) start = rnd_range(REGS);
            else if (pick < 65u) start = REGS - qty;                /* exact end */
            else if (pick < 80u) start = REGS;                      /* just over */
            else if (pick < 90u) start = 0xFFFFu;                   /* wrap */
            else                 start = rnd_range(0x10000u);
            buf[2] = (uint8_t)(start >> 8); buf[3] = (uint8_t)start;
        }
        buf[4] = (uint8_t)(qty >> 8); buf[5] = (uint8_t)qty;

                /* byte_count usually consistent, occasionally not. */
        bc = (rnd_range(100u) < 85u) ? (qty * 2u) : rnd_range(256u);
                /* header(7) + bc + crc(2) must fit MB_ADU_MAX. */
        if (bc > MB_ADU_MAX - 9u) bc = MB_ADU_MAX - 9u;
        buf[6] = (uint8_t)bc;
        for (i = 0; i < bc; i++) buf[7u + i] = (uint8_t)rnd_range(256u);
        len = (uint16_t)(7u + bc);
    } else {
        uint32_t start, qty;
        pick = rnd_range(100u);
        if (pick < 25u)      start = 0u;
        else if (pick < 45u) start = rnd_range(REGS);
        else if (pick < 60u) start = REGS - 1u;                     /* last */
        else if (pick < 75u) start = REGS;                          /* over */
        else if (pick < 85u) start = 0xFFFFu;                       /* wrap */
        else                 start = rnd_range(0x10000u);

        pick = rnd_range(100u);
        if (pick < 10u)      qty = 0u;                              /* illegal */
        else if (pick < 55u) qty = 1u + rnd_range(8u);
        else if (pick < 70u) qty = MB_READ_QTY_MAX;                 /* boundary */
        else if (pick < 80u) qty = MB_READ_QTY_MAX + 1u;            /* just over */
        else if (pick < 90u) qty = REGS - (start < REGS ? start : 0u);
        else                 qty = rnd_range(0x10000u);

        buf[2] = (uint8_t)(start >> 8); buf[3] = (uint8_t)start;
        buf[4] = (uint8_t)(qty >> 8);   buf[5] = (uint8_t)qty;
        len = 6u;
    }

        /* Length mutation applied before the CRC is computed. Truncating after the
     * CRC always invalidates it, so the frame dies at the CRC gate and the
     * per-function length checks are never reached. A frame that is CRC-valid
     * but the wrong length for its function code requires this path. */
    if (rnd_range(100u) < 12u) {
        uint32_t d = rnd_range(3u);
        if (rnd_range(2u) && len > 3u) {
            len = (uint16_t)(len - 1u - d);       /* short body */
            if (len < 2u) len = 2u;
        } else {
            uint32_t k;
            for (k = 0; k <= d && len < MB_ADU_MAX - 3u; k++) {
                buf[len++] = (uint8_t)rnd_range(256u);  /* padded body */
            }
        }
    }

    /* CRC: mostly correct, sometimes corrupted, to exercise the reject path */
    {
        uint16_t crc = mb_crc16(buf, len);
        buf[len]     = (uint8_t)(crc & 0xFFu);
        buf[len + 1] = (uint8_t)(crc >> 8);
        len = (uint16_t)(len + 2u);
        if (rnd_range(100u) < 8u) {
            buf[rnd_range(len)] ^= (uint8_t)(1u << rnd_range(8u)); /* single bit flip */
        }
    }

    /* occasionally truncate, to exercise malformed-ADU handling */
    if (rnd_range(100u) < 5u && len > 2u) {
        len = (uint16_t)(1u + rnd_range(len));
    }

        /* Over-long ADU. The upper length guard is defensive: a correct caller
     * cannot exceed MB_ADU_MAX because that is the receive buffer size. The
     * caller's buffer is oversized to make this case well-defined. */
    if (rnd_range(1000u) < 8u) {
        len = (uint16_t)(MB_ADU_MAX + 1u + rnd_range(32u));
    }
    return len;
}

int main(int argc, char **argv)
{
    mb_slave_t s;
    uint16_t   hold[REGS];
    uint8_t    req[MB_ADU_MAX + 64];   /* headroom for the over-long ADU case */
    uint8_t    resp[MB_ADU_MAX];
    uint16_t   resp_len = 0, req_len;
    uint32_t   iters = 200000u, i;
    uint32_t   seed  = 0xC0FFEEu;
    int        violations = 0, total_violations = 0;
    double     cov;

    if (argc >= 2) iters = (uint32_t)strtoul(argv[1], NULL, 0);
    if (argc >= 3) seed  = (uint32_t)strtoul(argv[2], NULL, 0);
    rng_state = seed ? seed : 1u;

    printf("\n==============================================\n");
    printf(" Constrained-random verification\n");
    printf(" iterations = %u   seed = 0x%08X\n", iters, seed);
    printf("==============================================\n");

    mb_slave_init(&s, SLAVE_ADDR, hold, REGS);
    mb_cov_reset();

    for (i = 0; i < iters; i++) {
        mb_result_t r;
        req_len = gen_frame(req);

        violations = mb_drive_and_check(&s, req, req_len);
        if (violations) {
            printf("\n  !! invariant violated at iteration %u, seed 0x%08X\n",
                   i, seed);
            printf("     frame:");
            { uint16_t k; for (k = 0; k < req_len; k++) printf(" %02X", req[k]); }
            printf("\n     reproduce: ./build/random_test %u 0x%08X\n", i + 1u, seed);
            total_violations += violations;
            if (total_violations > 5) break;
        }

        /* replay for the coverage sample */
        r = mb_slave_handle(&s, req, req_len, resp, &resp_len);
        mb_cov_sample(&s, req, req_len, r, resp, resp_len);
    }

    cov = mb_cov_report();

    printf("\n  Assertions\n");
    for (i = 0; i < MB_PROP_COUNT; i++) {
        printf("    %-38s %8u checks  %s\n",
               mb_props[i].name, mb_props[i].checks,
               mb_props[i].failures ? "FAILED" : "ok");
    }

    printf("\n  Slave counters\n");
    printf("    frames accepted %u | crc errors %u | length errors %u\n",
           s.cnt_frames_ok, s.cnt_crc_err, s.cnt_len_err);
    printf("    exceptions %u | responses %u | broadcasts %u\n",
           s.cnt_exceptions, s.cnt_responses, s.cnt_broadcast);

    printf("\n----------------------------------------------\n");
    if (total_violations == 0 && cov >= 100.0) {
        printf("  PASS  %u frames, 0 violations, coverage closed\n", iters);
    } else if (total_violations == 0) {
        printf("  PASS  %u frames, 0 violations, coverage %.1f%% (not closed)\n",
               iters, cov);
    } else {
        printf("  FAIL  %d invariant violations\n", total_violations);
    }
    printf("----------------------------------------------\n\n");
    return total_violations ? 1 : 0;
}
