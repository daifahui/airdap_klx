#include "rf/core/rf_core.h"
#include "ch572/attributes.h"

__RF_RAM(LLE_LibIRQHandler) void LLE_LibIRQHandler(void) {
    void (*process)(uint32_t) = lleProcess;
    uint32_t flags            = RF_LLE(8);

    if (RF_LLE(0xc) & flags) {
        RF_LLE(8) = flags;
        if (process)
            process(RF_LLE(0xc) & flags);
    }
}

__RF_RAM(BB_LibIRQHandler) void BB_LibIRQHandler(void) {
    if (RF_BB(0x40) & 0x40) {
        RF_BB(0x40)  = 0x60;
        RF_LLE(8)    = 0x20000;
        RF_LLE(0x64) = RF_receiveWait;
        RF_status    = 2;
    }
}

__RF_RAM(RFIP_ReadCrc) uint8_t RFIP_ReadCrc(void) {
    return (RF_BB(0x40) >> 5) & 1;
}

__RF_RAM(RFIP_WakeUpRegInit) void RFIP_WakeUpRegInit(void) {
    RF_LLE(4)    |= 0x1e;
    RF_LLE(0x54)  = 0;
    RF_LLE(0xc)   = 0x20003;
    RF_BB(0x40)   = 0xffff;
    RF_BB(0x3c)   = 0x40;
    RF_BB(0x38)   = 0x20018;
}

__RF_RAM(RFIP_SetPHYIdle) void RFIP_SetPHYIdle(void) {
    RF_LLE(0x50) &= ~0x10u;
    if (RF_LLE(0) & 3) {
        RF_LLE(0) |= 8;
        RF_NOPS2();
        RF_LLE(0x50) = (RF_LLE(0x50) & 0xfffff8ffu) | 0x500;
    }
}

void RFIP_Calibration(void) {
    RF_BB(0)     &= 0xfffcffffu;
    RF_BB(0)      = (RF_BB(0) & 0xfffcffffu) | 0x20000;
    RF_RFEND(8)  |= 0x330000;
    RF_LLE(0x50)  = 0xf05d8;
    RFEND_TXCtune();
    RF_RFEND(4) |= 0x100;
    RFEND_RXFilter();
    RFEND_RXAdc();
    RF_LLE(8)     = 0x20000;
    RF_BB(0)      = (RF_BB(0) & 0xfffcffffu) | 0x10000;
    RF_RFEND(8)  &= 0xffcdffffu;
    RF_LLE(0x50)  = 0xf0080;
}

void RFIP_RegInit(void (*process)(uint32_t)) {
    gptrRFENDReg = (volatile uint32_t *)RF_RFEND_BASE;
    gptrLLEReg   = (volatile uint32_t *)RF_LLE_BASE;
    gptrBBReg    = (volatile uint32_t *)RF_BB_BASE;
    RFEND_DevInit();
    RFIP_WakeUpRegInit();
    RFIP_Calibration();
    lleProcess = process;
}
