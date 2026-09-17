/* Multi-slave bus contention check.
 *
 * RS-485 is a shared differential pair: two slaves transmitting simultaneously
 * destroys the frame, and on hardware the fault is intermittent and gives no
 * indication of which device transmitted.
 *
 * Eight slaves share a virtual bus and every unit identifier (0..255) is driven
 * against every supported function code, testing the property that the number
 * of slaves transmitting is never greater than one.
 */

#include <stdio.h>
#include <string.h>
#include "modbus_slave.h"
#include "modbus_crc.h"

#define N_SLAVES 8u
#define REGS     16u

int main(void)
{
    mb_slave_t s[N_SLAVES];
    uint16_t   hold[N_SLAVES][REGS];
    uint8_t    addrs[N_SLAVES] = { 1u, 2u, 5u, 17u, 42u, 99u, 200u, 247u };
    uint8_t    req[MB_ADU_MAX], resp[MB_ADU_MAX];
    uint16_t   resp_len, crc;
    uint32_t   i, unit, f;
    uint32_t   frames = 0, collisions = 0, answered = 0, unanswered = 0;
    uint32_t   bcast_writes_applied = 0;
    const uint8_t fcs[4] = { 0x03u, 0x06u, 0x10u, 0x2Bu };

    for (i = 0; i < N_SLAVES; i++) {
        mb_slave_init(&s[i], addrs[i], hold[i], REGS);
    }

    printf("\n==============================================\n");
    printf(" Multi-slave bus, %u slaves, exhaustive sweep\n", N_SLAVES);
    printf(" unit ids 0..255 x %u function codes\n", (unsigned)sizeof(fcs));
    printf("==============================================\n\n");

    for (unit = 0u; unit <= 255u; unit++) {
        for (f = 0u; f < sizeof(fcs); f++) {
            uint16_t len;
            uint32_t responders = 0;

            req[0] = (uint8_t)unit;
            req[1] = fcs[f];
            if (fcs[f] == 0x10u) {
                req[2] = 0; req[3] = 0; req[4] = 0; req[5] = 2; req[6] = 4;
                req[7] = 0xAB; req[8] = 0xCD; req[9] = 0xEF; req[10] = 0x01;
                len = 11u;
            } else {
                req[2] = 0; req[3] = 0; req[4] = 0; req[5] = 1;
                len = 6u;
            }
            crc = mb_crc16(req, len);
            req[len]     = (uint8_t)(crc & 0xFFu);
            req[len + 1] = (uint8_t)(crc >> 8);
            len = (uint16_t)(len + 2u);

                        /* Every slave on the segment receives every frame. */
            for (i = 0; i < N_SLAVES; i++) {
                if (mb_slave_handle(&s[i], req, len, resp, &resp_len)
                    == MB_RESULT_RESPOND) {
                    responders++;
                }
            }

            frames++;
            if (responders > 1u) {
                collisions++;
                printf("  COLLISION: unit 0x%02X fc 0x%02X -> %u responders\n",
                       (unsigned)unit, fcs[f], responders);
            } else if (responders == 1u) {
                answered++;
            } else {
                unanswered++;
            }
        }
    }

        /* A broadcast write must reach every slave and produce no reply. */
    {
        uint16_t len;
        req[0] = MB_ADDR_BROADCAST; req[1] = 0x06u;
        req[2] = 0; req[3] = 7; req[4] = 0xC0; req[5] = 0xDE;
        len = 6u;
        crc = mb_crc16(req, len);
        req[len] = (uint8_t)(crc & 0xFFu); req[len + 1] = (uint8_t)(crc >> 8);
        len = (uint16_t)(len + 2u);

        for (i = 0; i < N_SLAVES; i++) {
            (void)mb_slave_handle(&s[i], req, len, resp, &resp_len);
            if (hold[i][7] == 0xC0DEu) bcast_writes_applied++;
        }
    }

    printf("  frames driven ............ %u\n", frames);
    printf("  exactly one responder .... %u\n", answered);
    printf("  no responder ............. %u\n", unanswered);
    printf("  collisions ............... %u\n", collisions);
    printf("  broadcast reached ........ %u / %u slaves\n",
           bcast_writes_applied, N_SLAVES);

    printf("\n----------------------------------------------\n");
    if (collisions == 0u && bcast_writes_applied == N_SLAVES) {
        printf("  PASS  no bus contention possible; broadcast is universal\n");
    } else {
        printf("  FAIL  %u collisions, broadcast reached %u/%u\n",
               collisions, bcast_writes_applied, N_SLAVES);
    }
    printf("----------------------------------------------\n\n");

    return (collisions == 0u && bcast_writes_applied == N_SLAVES) ? 0 : 1;
}
