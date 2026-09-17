#include <stdio.h>
#include <string.h>
#include "modbus_slave.h"
#include "modbus_crc.h"

static int pass = 0, fail = 0;

static void check(const char *name, int ok)
{
    if (ok) { pass++; printf("  PASS  %s\n", name); }
    else    { fail++; printf("  FAIL  %s\n", name); }
}

/* build an ADU from a body, appending a correct CRC */
static uint16_t frame(uint8_t *out, const uint8_t *body, uint16_t len)
{
    uint16_t crc;
    memcpy(out, body, len);
    crc = mb_crc16(out, len);
    out[len]     = (uint8_t)(crc & 0xFF);
    out[len + 1] = (uint8_t)(crc >> 8);
    return (uint16_t)(len + 2);
}

#define REGS 16

int main(void)
{
    mb_slave_t s;
    uint16_t   hold[REGS];
    uint8_t    req[MB_ADU_MAX], resp[MB_ADU_MAX];
    uint16_t   rl, n;
    mb_result_t r;

    printf("\n=== CRC-16/MODBUS ===\n");
    {
        /* the canonical vector every Modbus implementation is checked against:
         * "123456789" -> 0x4B37 */
        const uint8_t v[] = "123456789";
        uint16_t c = mb_crc16(v, 9);
        printf("  check value = 0x%04X (expect 0x4B37)\n", c);
        check("CRC check value", c == 0x4B37);

        /* a real request: slave 1, read 2 regs from 0 */
        const uint8_t q[] = {0x01,0x03,0x00,0x00,0x00,0x02};
        c = mb_crc16(q, 6);
        printf("  crc(01 03 00 00 00 02) = 0x%04X -> on wire %02X %02X\n",
               c, c & 0xFF, c >> 8);
                /* The numeric CRC is 0x0BC4; documentation and packet captures show
         * "C4 0B" because RTU transmits the CRC low byte first, unlike every
         * other field in the frame. */
        check("CRC value is 0x0BC4", c == 0x0BC4);
        check("wire order is C4 0B",
              (c & 0xFF) == 0xC4 && (c >> 8) == 0x0B);
    }

    printf("\n=== 0x03 Read Holding Registers ===\n");
    mb_slave_init(&s, 0x11, hold, REGS);
    hold[0] = 0xAE41; hold[1] = 0x5652; hold[2] = 0x4340;
    {
        const uint8_t body[] = {0x11,0x03,0x00,0x00,0x00,0x03};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("read 3 regs responds", r == MB_RESULT_RESPOND);
        check("response length 11", rl == 11);
        check("byte count = 6", resp[2] == 6);
        check("reg0 big-endian AE 41", resp[3] == 0xAE && resp[4] == 0x41);
        check("reg2 big-endian 43 40", resp[7] == 0x43 && resp[8] == 0x40);
        check("response CRC valid", mb_crc16(resp, rl) == 0);
    }
    {   /* quantity 0 is illegal */
        const uint8_t body[] = {0x11,0x03,0x00,0x00,0x00,0x00};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("qty=0 -> exception", r == MB_RESULT_RESPOND && resp[1] == 0x83);
        check("qty=0 -> ILLEGAL_VALUE", resp[2] == MB_EX_ILLEGAL_VALUE);
    }
    {   /* qty 126 exceeds the 125 register read limit */
        const uint8_t body[] = {0x11,0x03,0x00,0x00,0x00,0x7E};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("qty=126 -> ILLEGAL_VALUE",
              r == MB_RESULT_RESPOND && resp[2] == MB_EX_ILLEGAL_VALUE);
    }
    {   /* start 14 + qty 4 = 18 > 16 registers */
        const uint8_t body[] = {0x11,0x03,0x00,0x0E,0x00,0x04};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("overrun -> ILLEGAL_ADDRESS",
              r == MB_RESULT_RESPOND && resp[2] == MB_EX_ILLEGAL_ADDRESS);
    }
    {   /* start 0xFFFF + qty 2 must not wrap to a valid range */
        const uint8_t body[] = {0x11,0x03,0xFF,0xFF,0x00,0x02};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("start+qty overflow -> ILLEGAL_ADDRESS",
              r == MB_RESULT_RESPOND && resp[2] == MB_EX_ILLEGAL_ADDRESS);
    }
    {   /* boundary: last register exactly */
        const uint8_t body[] = {0x11,0x03,0x00,0x0F,0x00,0x01};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("start=15 qty=1 is legal", r == MB_RESULT_RESPOND && resp[1] == 0x03);
    }

    printf("\n=== 0x06 Write Single Register ===\n");
    {
        const uint8_t body[] = {0x11,0x06,0x00,0x05,0x12,0x34};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("write single responds", r == MB_RESULT_RESPOND);
        check("register updated", hold[5] == 0x1234);
        check("response echoes request", rl == 8 && memcmp(resp, req, 8) == 0);
    }
    {
        const uint8_t body[] = {0x11,0x06,0x00,0x20,0x12,0x34};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("write out of range -> ILLEGAL_ADDRESS",
              r == MB_RESULT_RESPOND && resp[1] == 0x86 &&
              resp[2] == MB_EX_ILLEGAL_ADDRESS);
    }

    printf("\n=== 0x10 Write Multiple Registers ===\n");
    {
        const uint8_t body[] = {0x11,0x10,0x00,0x08,0x00,0x02,0x04,
                                0xDE,0xAD,0xBE,0xEF};
        n = frame(req, body, 11);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("write multiple responds", r == MB_RESULT_RESPOND);
        check("both registers written",
              hold[8] == 0xDEAD && hold[9] == 0xBEEF);
        check("response is 8 bytes", rl == 8);
        check("response echoes start+qty",
              resp[3] == 0x08 && resp[5] == 0x02);
    }
    {   /* byte count says 4 but qty says 3 */
        const uint8_t body[] = {0x11,0x10,0x00,0x08,0x00,0x03,0x04,
                                0xDE,0xAD,0xBE,0xEF};
        n = frame(req, body, 11);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("byte_count != 2*qty -> ILLEGAL_VALUE",
              r == MB_RESULT_RESPOND && resp[2] == MB_EX_ILLEGAL_VALUE);
    }
    {   /* declared byte count does not match the bytes actually present */
        const uint8_t body[] = {0x11,0x10,0x00,0x08,0x00,0x02,0x04,
                                0xDE,0xAD};
        n = frame(req, body, 9);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("truncated payload -> BAD_LENGTH", r == MB_RESULT_BAD_LENGTH);
    }

    printf("\n=== error and addressing paths ===\n");
    {   /* corrupt one CRC byte */
        const uint8_t body[] = {0x11,0x03,0x00,0x00,0x00,0x01};
        n = frame(req, body, 6);
        req[n - 1] ^= 0xFF;
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("bad CRC -> discarded", r == MB_RESULT_BAD_CRC);
    }
    {   /* correct CRC, wrong slave */
        const uint8_t body[] = {0x22,0x03,0x00,0x00,0x00,0x01};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("other slave -> silent", r == MB_RESULT_SILENT);
    }
    {   /* unsupported function 0x2B */
        const uint8_t body[] = {0x11,0x2B,0x00,0x00,0x00,0x01};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("unknown fc -> ILLEGAL_FUNCTION",
              r == MB_RESULT_RESPOND && resp[1] == 0xAB &&
              resp[2] == MB_EX_ILLEGAL_FUNCTION);
    }
    {   /* runt frame */
        uint8_t tiny[3] = {0x11, 0x03, 0x00};
        r = mb_slave_handle(&s, tiny, 3, resp, &rl);
        check("3-byte frame -> BAD_LENGTH", r == MB_RESULT_BAD_LENGTH);
    }
    {   /* broadcast write: must act, must not answer */
        const uint8_t body[] = {0x00,0x06,0x00,0x0A,0xCA,0xFE};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("broadcast write is silent", r == MB_RESULT_SILENT);
        check("broadcast write took effect", hold[10] == 0xCAFE);
    }
    {   /* broadcast read is meaningless */
        const uint8_t body[] = {0x00,0x03,0x00,0x00,0x00,0x01};
        n = frame(req, body, 6);
        r = mb_slave_handle(&s, req, n, resp, &rl);
        check("broadcast read is silent", r == MB_RESULT_SILENT);
    }

    printf("\n=== counters ===\n");
    printf("  frames_ok=%u crc_err=%u len_err=%u exceptions=%u "
           "responses=%u broadcast=%u\n",
           s.cnt_frames_ok, s.cnt_crc_err, s.cnt_len_err,
           s.cnt_exceptions, s.cnt_responses, s.cnt_broadcast);
    check("one CRC error counted", s.cnt_crc_err == 1);
    check("two broadcasts counted", s.cnt_broadcast == 2);

    printf("\n---------------------------------------\n");
    printf("  %d passed, %d failed\n", pass, fail);
    printf("---------------------------------------\n\n");
    return fail == 0 ? 0 : 1;
}
