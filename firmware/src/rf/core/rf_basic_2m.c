/* Fixed Basic2M profile, preserving the generic driver's ordered MMIO accesses.
 * No cached hardware state and no additional SRAM objects. */
#include "rf/core/rf_core.h"
#include "ch572/attributes.h"
#include "rf/core/rf_basic_2m.h"

__RF_RAM(RFIP_SetRx2m) bStatus_t RFIP_SetRx2m(const rfipRx_t *p) {
    if (!p)
        return 1;
    if (!p->rxDMA)
        return 2;

    /* Stop the previous operation before changing channel and DMA ownership. */
    RF_LLE16(6) = 4;
    if (RF_LLE(0) & 3)
        RF_LLE(0) |= 8;
    RF_BB(0)       &= 0xfffcffffu;
    RF_BB(0)        = (RF_BB(0) & 0xfffcffffu) | 0x20000;
    RF_RFEND(8)    |= 0x330000;
    RF_LLE(0x50)    = 0xf01d8;
    RF_RFEND(0x2c) &= ~2u;
    RF_BB(0)        = (RF_BB(0) & ~0x7fu) | (p->frequency & 0x7f);
    RF_BB(0x60)     = p->accessAddress;
    RF_BB(0x64)     = p->accessAddressEx;
    RF_BB(0x58)     = p->crcInit;
    RF_BB(0x5c)     = (RF_BB(0x5c) & 0xff000000u) | (p->crcPoly & 0xffffff);
    RF_LLE(0x7c)    = p->rxDMA;
    RF_LLE(0x64)    = 0;
    RF_LLE(8)       = 0x20000;

    /* Fixed Basic2M receive format; calibration remains a live input. */
    RF_BB(0)       &= 0xfffffcffu;
    RF_RFEND(0x50)  = (RF_RFEND(0x50) & ~31u) | (gTuneFilter2M & 31);
    RF_BB(0x24)    &= 0xf9ffffffu;
    RF_BB(0x14)     = (RF_BB(0x14) & ~63u) | 13;
    RF_BB8(0x1e)    = 127;
    RF_BB(0x1c)     = (RF_BB(0x1c) & ~1023u) | 0xa0;
    RF_receiveWait  = (p->rxMaxLen + 3u) * 8 + 60;
    RF_LLE(0)       = 1;
    RF_status       = 1;
    if (p->timeOut) {
        /* Preserve the original mask/fence/clear/program/restore sequence. */
        uint32_t mask = RF_LLE(0xc);

        RF_LLE(0xc) = mask & ~0x20000u;
        __asm__ volatile("fence.i" ::: "memory");
        RF_LLE(8)    = 0x20000;
        RF_LLE(0x64) = p->timeOut;
        RF_LLE(0xc)  = mask;
    }
    return 0;
}

__RF_RAM(RFIP_StartTx2m) bStatus_t RFIP_StartTx2m(const rfipTx_t *p) {
    /* Keep the stop, power and settling sequence, including its PLL gap. */
    RF_LLE(0)      |= 8;
    RF_BB(0x34)     = (RF_BB(0x34) & 0x1ffffffu) | ((uint32_t)(p->txPowerVal | 0x40) << 25);
    RF_LLE16(6)     = p->waitTime;
    RF_BB(0x34)     = (RF_BB(0x34) & ~3u) | 1;
    RF_BB(0)       &= 0xfffcffffu;
    RF_BB(0)        = (RF_BB(0) & 0xfffcffffu) | 0x20000;
    RF_RFEND(8)    |= 0x330000;
    RF_LLE(0x50)    = 0xf02d8;
    RF_RFEND(0x2c) &= ~2u;
    RF_BB(0)        = (RF_BB(0) & ~0x7fu) | (p->frequency & 0x7f);
    if (RF_LLE16(6) > 40) {
        RF_RFEND(4) &= ~0x100u;
        RF_NOPS4();
        RF_RFEND(4) |= 0x100;
    }
    RF_BB(0xc)      = p->accessAddress;
    RF_BB(0x10)     = p->accessAddressEx;
    RF_BB(4)        = p->crcInit;
    RF_BB(8)        = (RF_BB(8) & 0xff000000u) | (p->crcPoly & 0xffffff);
    RF_LLE(0x78)    = p->txDMA;
    RF_RFEND(0x34) &= 0xfffff8ffu;

    /* Basic2M length comes from the DMA header, not p->txLen. */
    uint32_t packet = *(const uint32_t *)(uintptr_t)p->txDMA;

    RF_BB(0)      &= 0xfffffcffu;
    RF_BB(0x24)   &= 0xf9ffffffu;
    uint32_t wait  = RF_LLE16(6);
    uint32_t mask  = RF_LLE(0xc);

    RF_LLE(0xc) = mask & ~0x20000u;
    __asm__ volatile("fence.i" ::: "memory");
    RF_LLE(8)         = 0x20000;
    uint32_t interval = (((packet >> 8) & 255) + 11) * 4;

    RF_LLE(0x64)  = (interval + (wait >> 1) + 44) * 2;
    RF_LLE(0xc)   = mask;
    RF_BB(0)     |= 0x1000000;
    RF_BB(0x34)  &= ~3u;
    RF_LLE(0)     = 2;
    RF_status     = 0x10;
    return 0;
}
