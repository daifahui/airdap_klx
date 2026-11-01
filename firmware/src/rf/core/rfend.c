/* Calibration/control stays in Flash, exactly as in the factory library.
 * Signed divider operands and byte coefficient stores are intentional. */
#include "rf/core/rf_core.h"

uint32_t RFEND_TXCtuneCall3182(int32_t channel) {
    int32_t  delta = (int32_t)RFEND_co2401 - RFEND_co2440, value = (32 - channel) * delta;
    uint32_t packed = 0;

    for (unsigned i = 0; i < 8; i++) {
        packed  = (packed << 4) | (uint32_t)(value / 39);
        value  += delta;
    }
    return packed;
}

uint32_t RFEND_TXCtuneCallx3190(int32_t channel) {
    int32_t  delta = (int32_t)RFEND_co2440 - RFEND_co2480, value = delta * (channel + 7);
    uint32_t packed = 0;

    for (unsigned i = 0; i < 8; i++) {
        packed  = (packed << 4) | (uint32_t)(value / 39);
        value  -= delta;
    }
    return packed;
}

void RFEND_DevInit(void) {
    RF_RFEND(0x28)  = 0x480;
    RF_RFEND(0x3c)  = (RF_RFEND(0x3c) & 0xf8ffffffu) | 0x2000000;
    RF_RFEND(0x3c)  = (RF_RFEND(0x3c) & 0x1fffffffu) | 0x40000000;
    RF_RFEND(0x30) &= 0xfff9ffffu;
    RF_RFEND(0x30)  = (RF_RFEND(0x30) & 0x8fffffffu) | 0x40000000;
    RF_RFEND(0x48) &= 0xfff8ffffu;
    RF_RFEND(0x50)  = (RF_RFEND(0x50) & 0xfffff8ffu) | 0x300;
    RF_RFEND(0x5c) |= 0x70000;
    RF_RFEND(0x5c) |= 0x7000000;

    /* Factory loop: one NOP then decrement/branch,100 iterations. */
    for (unsigned count = 100; count; --count)
        __asm__ volatile("nop" ::: "memory");
}

void RFEND_RXFilter(void) {
    RF_RFEND(0x50) &= ~0x10000u;
    RF_RFEND(8)    |= 0x200000;
    RF_RFEND(0xc)  |= 0x10;
    RF_RFEND(0xc)  &= ~0x10u;
    RF_NOPS4();
    RF_RFEND(0xc) |= 0x10;
    RF_RFEND(4)   |= 0x1000;
    RF_LLE(0x64)   = 100;
    while (RF_LLE(0x64) && !(RF_RFEND(0x9c) & 0x100)) {
    }
    RF_NOPS4();
    uint32_t value = RF_RFEND(0x9c) & 31;

    gTuneFilter     = value;
    RF_RFEND(0x50) |= 0x10000;
    RF_RFEND(0x50)  = (RF_RFEND(0x50) & ~31u) | value;
    RF_RFEND(8)    &= ~0x200000u;
    if (value > 3)
        gTuneFilter2M = value - 3;
}

void RFEND_RXAdc(void) {
    RF_RFEND(0x58) &= ~0x10000u;
    RF_RFEND(8)    |= 0x10000;
    RF_RFEND(0xc)  &= ~0x100u;
    RF_RFEND(4)    &= ~0x10000u;
    RF_RFEND(0xc)  |= 0x100;
    RF_RFEND(4)    |= 0x10000;
}

void RFEND_WaitTune(uint32_t channel) {
    RF_RFEND(4)    &= ~1u;
    RF_RFEND(0x38)  = (RF_RFEND(0x38) & 0xfffe00ffu) | ((channel & 0x1ff) << 8);
    RF_RFEND(4)    |= 1;
    RF_LLE(0x64)    = 8000;
    while (!(RF_RFEND(0x90) & 0x04000000) || !(RF_RFEND(0x90) & 0x02000000)) {
        if (!RF_LLE(0x64))
            return;
    }
}

static uint32_t RFEND_DivideOrMinusOne(int32_t a, int32_t b) {
    return b ? (uint32_t)(a / b) : UINT32_MAX;
}

void RFEND_TXCtune(void) {
    RF_RFEND(4)    &= ~0x100u;
    RF_RFEND(0x28) &= ~0x1000u;
    RF_RFEND(0x2c) &= ~0x10u;
    RF_RFEND(8)    |= 0x20000;
    RF_RFEND(4)    |= 0x10;
    if (RFEND_tuneFlag == 0x1234)
        RFEND_tuneFlag = 0;
    else {
        RFEND_WaitTune(0xbf);
        RFEND_co2401 = RF_RFEND(0x90) & 63;
        RFEND_ga2401 = (RF_RFEND(0x94) >> 10) & 127;
        RFEND_WaitTune(0xe7);
        RFEND_co2480 = RF_RFEND(0x90) & 63;
        RFEND_ga2480 = (RF_RFEND(0x94) >> 10) & 127;
        RFEND_WaitTune(0xd3);
        RFEND_co2440 = RF_RFEND(0x90) & 63;
        RFEND_ga2440 = (RF_RFEND(0x94) >> 10) & 127;
    }
    for (unsigned i = 0; i < 5; i++)
        RF_RFEND(0xa0 + 4 * i) = RFEND_TXCtuneCall3182(i * 8);
    for (unsigned i = 0; i < 5; i++)
        RF_RFEND(0xb4 + 4 * i) = RFEND_TXCtuneCallx3190(i * 8 + 1);
    int32_t dco = (int32_t)RFEND_co2440 - RFEND_co2480, dga = (int32_t)RFEND_ga2440 - RFEND_ga2480;

    RF_RFEND(0xc8) = ((uint32_t)((dco * 42) / 40) << 4) | (uint32_t)((dco * 41) / 40);
    int32_t value  = dga * 10;

    for (unsigned shift = 8; shift < 32; shift += 4) {
        RF_RFEND(0xc8) |= (RFEND_DivideOrMinusOne(value, dco) | 8) << shift;
        value          -= dga;
    }
    RF_RFEND(0xcc) = 0;
    value          = dga * 4;
    for (unsigned shift = 0; shift < 16; shift += 4) {
        RF_RFEND(0xcc) |= (RFEND_DivideOrMinusOne(value, dco) | 8) << shift;
        value          -= dga;
    }
    dga   = (int32_t)RFEND_ga2401 - RFEND_ga2440;
    dco   = (int32_t)RFEND_co2401 - RFEND_co2440;
    value = 0;
    for (unsigned shift = 16; shift < 32; shift += 4) {
        RF_RFEND(0xcc) |= RFEND_DivideOrMinusOne(value, dco) << shift;
        value          += dga;
    }
    value = dga * 4;
    for (unsigned shift = 0; shift < 28; shift += 4) {
        RF_RFEND(0xd0) |= RFEND_DivideOrMinusOne(value, dco) << shift;
        value          += dga;
    }
    RF_RFEND(4)    &= ~0x10u;
    RF_RFEND(4)    &= ~1u;
    RF_RFEND(0x28) |= 0x1000;
    RF_RFEND(0x2c) |= 0x10;
    RF_RFEND(0x38)  = (RF_RFEND(0x38) & ~63u) | (RFEND_co2440 & 63);
    RF_RFEND(0x38)  = (RF_RFEND(0x38) & 0x80ffffffu) | ((uint32_t)(RFEND_ga2440 & 127) << 24);
}
