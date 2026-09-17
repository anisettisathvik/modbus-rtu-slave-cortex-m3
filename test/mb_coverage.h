#ifndef MB_COVERAGE_H
#define MB_COVERAGE_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "modbus_slave.h"

/* Functional coverage model for the Modbus slave.
 *
 * Coverpoints for function code, outcome, addressing and register-range
 * boundary, plus the function-code x outcome cross. Structural coverage shows
 * which statements ran; this shows which behaviours were exercised, and so is
 * what determines when the random stimulus is complete.
 */

/* ---- coverpoint: function code ---------------------------------------- */
enum { CP_FC_03, CP_FC_06, CP_FC_10, CP_FC_UNSUPPORTED, CP_FC_N };
static const char *cp_fc_name[CP_FC_N] = {
    "fc=0x03 read", "fc=0x06 write1", "fc=0x10 writeN", "fc unsupported"
};

/* ---- coverpoint: outcome ------------------------------------------------ */
enum { CP_OUT_OK, CP_OUT_EX01, CP_OUT_EX02, CP_OUT_EX03,
       CP_OUT_SILENT, CP_OUT_CRC, CP_OUT_LEN, CP_OUT_N };
static const char *cp_out_name[CP_OUT_N] = {
    "normal reply", "exc 01 function", "exc 02 address", "exc 03 value",
    "silent drop", "crc error", "length error"
};

/* ---- coverpoint: addressing --------------------------------------------- */
enum { CP_ADDR_OURS, CP_ADDR_BCAST, CP_ADDR_FOREIGN, CP_ADDR_N };
static const char *cp_addr_name[CP_ADDR_N] = {
    "addressed to us", "broadcast", "another slave"
};

/* ---- coverpoint: register range boundary -------------------------------- */
enum { CP_RNG_LOW, CP_RNG_MID, CP_RNG_LAST, CP_RNG_OVER, CP_RNG_WRAP, CP_RNG_N };
static const char *cp_rng_name[CP_RNG_N] = {
    "first register", "interior", "exactly the last register",
    "past the end", "start+qty wraps 16-bit"
};

typedef struct {
    uint32_t fc[CP_FC_N];
    uint32_t out[CP_OUT_N];
    uint32_t addr[CP_ADDR_N];
    uint32_t rng[CP_RNG_N];
    uint32_t cross_fc_out[CP_FC_N][CP_OUT_N];  /* the cross that matters */
    uint32_t samples;
} mb_cov_t;

static mb_cov_t mb_cov;

static void mb_cov_reset(void) { memset(&mb_cov, 0, sizeof(mb_cov)); }

static void mb_cov_sample(const mb_slave_t *s,
                          const uint8_t *req, uint16_t req_len,
                          mb_result_t r, const uint8_t *resp, uint16_t resp_len)
{
    int fc_bin, out_bin;
    uint8_t fc;

    mb_cov.samples++;

    /* addressing */
    if (req_len >= 1u) {
        if (req[0] == MB_ADDR_BROADCAST)   mb_cov.addr[CP_ADDR_BCAST]++;
        else if (req[0] == s->addr)        mb_cov.addr[CP_ADDR_OURS]++;
        else                               mb_cov.addr[CP_ADDR_FOREIGN]++;
    }

    /* function code */
    fc = (req_len >= 2u) ? req[1] : 0xFFu;
    switch (fc) {
    case MB_FC_READ_HOLDING:   fc_bin = CP_FC_03; break;
    case MB_FC_WRITE_SINGLE:   fc_bin = CP_FC_06; break;
    case MB_FC_WRITE_MULTIPLE: fc_bin = CP_FC_10; break;
    default:                   fc_bin = CP_FC_UNSUPPORTED; break;
    }
    mb_cov.fc[fc_bin]++;

    /* outcome */
    switch (r) {
    case MB_RESULT_BAD_CRC:    out_bin = CP_OUT_CRC; break;
    case MB_RESULT_BAD_LENGTH: out_bin = CP_OUT_LEN; break;
    case MB_RESULT_SILENT:     out_bin = CP_OUT_SILENT; break;
    case MB_RESULT_RESPOND:
    default:
        if (resp_len >= 3u && (resp[1] & 0x80u)) {
            switch (resp[2]) {
            case MB_EX_ILLEGAL_FUNCTION: out_bin = CP_OUT_EX01; break;
            case MB_EX_ILLEGAL_ADDRESS:  out_bin = CP_OUT_EX02; break;
            case MB_EX_ILLEGAL_VALUE:    out_bin = CP_OUT_EX03; break;
            default:                     out_bin = CP_OUT_EX01; break;
            }
        } else {
            out_bin = CP_OUT_OK;
        }
        break;
    }
    mb_cov.out[out_bin]++;
    mb_cov.cross_fc_out[fc_bin][out_bin]++;

    /* register range boundaries — only meaningful for the addressed functions */
    if (req_len >= 6u && (fc == MB_FC_READ_HOLDING ||
                          fc == MB_FC_WRITE_SINGLE ||
                          fc == MB_FC_WRITE_MULTIPLE)) {
        uint32_t start = ((uint32_t)req[2] << 8) | req[3];
        uint32_t qty   = (fc == MB_FC_WRITE_SINGLE)
                       ? 1u : (((uint32_t)req[4] << 8) | req[5]);
        uint32_t end   = start + qty;

        if (end > 0xFFFFu && start <= 0xFFFFu && qty > 0u) {
            mb_cov.rng[CP_RNG_WRAP]++;
        } else if (start == 0u) {
            mb_cov.rng[CP_RNG_LOW]++;
        } else if (end == (uint32_t)s->holding_count) {
            mb_cov.rng[CP_RNG_LAST]++;
        } else if (end > (uint32_t)s->holding_count) {
            mb_cov.rng[CP_RNG_OVER]++;
        } else {
            mb_cov.rng[CP_RNG_MID]++;
        }
    }
}

/* ---- reporting ---------------------------------------------------------- */

static void cov_line(const char *label, uint32_t hits)
{
    printf("    %-34s %8u   %s\n", label, hits, hits ? "HIT" : "-- MISS --");
}

/* Returns coverage as a percentage of bins hit. The cross deliberately
 * excludes bins that are unreachable by construction. */
static double mb_cov_report(void)
{
    int i, j, total = 0, hit = 0;

    printf("\n  Functional coverage, %u samples\n", mb_cov.samples);

    printf("\n  coverpoint function_code\n");
    for (i = 0; i < CP_FC_N; i++) {
        cov_line(cp_fc_name[i], mb_cov.fc[i]);
        total++; hit += (mb_cov.fc[i] > 0);
    }

    printf("\n  coverpoint outcome\n");
    for (i = 0; i < CP_OUT_N; i++) {
        cov_line(cp_out_name[i], mb_cov.out[i]);
        total++; hit += (mb_cov.out[i] > 0);
    }

    printf("\n  coverpoint addressing\n");
    for (i = 0; i < CP_ADDR_N; i++) {
        cov_line(cp_addr_name[i], mb_cov.addr[i]);
        total++; hit += (mb_cov.addr[i] > 0);
    }

    printf("\n  coverpoint register_range\n");
    for (i = 0; i < CP_RNG_N; i++) {
        cov_line(cp_rng_name[i], mb_cov.rng[i]);
        total++; hit += (mb_cov.rng[i] > 0);
    }

    printf("\n  cross function_code x outcome\n");
    printf("    %-18s", "");
    for (j = 0; j < CP_OUT_N; j++) printf("%9.9s", cp_out_name[j]);
    printf("\n");
    for (i = 0; i < CP_FC_N; i++) {
        printf("    %-18s", cp_fc_name[i]);
        for (j = 0; j < CP_OUT_N; j++) {
                        /* Unreachable by construction: an unsupported function code can
             * only yield exception 01, a silent drop, or a bus error. */
            int reachable = 1;
            if (i == CP_FC_UNSUPPORTED &&
                (j == CP_OUT_OK || j == CP_OUT_EX02 || j == CP_OUT_EX03)) {
                reachable = 0;
            }
            /* fc 0x06 cannot produce exception 03: it has no quantity field */
            if (i == CP_FC_06 && j == CP_OUT_EX03) reachable = 0;
            /* a supported fc never yields exception 01 */
            if (i != CP_FC_UNSUPPORTED && j == CP_OUT_EX01) reachable = 0;

            if (!reachable) {
                printf("%9s", ".");
            } else {
                printf("%9u", mb_cov.cross_fc_out[i][j]);
                total++; hit += (mb_cov.cross_fc_out[i][j] > 0);
            }
        }
        printf("\n");
    }

    printf("\n  bins hit: %d / %d = %.1f%%\n",
           hit, total, 100.0 * (double)hit / (double)total);
    return 100.0 * (double)hit / (double)total;
}

#endif /* MB_COVERAGE_H */
