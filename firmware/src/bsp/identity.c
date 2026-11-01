/* Startup SDK UID read only. Factory ISP572 V1.0 RAM I/O helpers are unchanged.
 * This is not an erase/write API. Match the factory dispatcher save/restore
 * around the same CH572_ROM_UID_ADDR read, including its safe-access sequence. */
#include "ch572/ch572.h"
#include "ch572/flash_io.h"

void BSP_ReadIdentity(uint8_t *data) {
    FLASH_ReadUid(data);

    /* The UID contains six factory bytes followed by
     * the little-endian sum of their three 16-bit words, modulo 65536. */
    uint16_t check = (data[0] | (uint16_t)data[1] << 8) + (data[2] | (uint16_t)data[3] << 8) +
                     (data[4] | (uint16_t)data[5] << 8);
    data[6] = check;
    data[7] = check >> 8;
}
