#include "rf/core/rf_core.h"
#include "ch572/attributes.h"

__RF_RAM(RF_BasicProcess) void RF_BasicProcess(uint32_t flags) {
    uint32_t event;
    uint8_t  id;

    if (flags & 2) {
        RF_LLE(0x64)  = 0;
        RF_LLE(8)     = 0x20000;
        flags        &= ~0x20000u;
        if (gRfConfig.processMask & RF_STATE_TX_FINISH)
            gRfConfig.rfProcessCB(RF_STATE_TX_FINISH, 0);
    }
    if (flags & 1) {
        RF_LLE(0x64) = 0;
        RF_LLE(8)    = 0x20000;
        event        = RFIP_ReadCrc() ? RF_STATE_RX_CRCERR : RF_STATE_RX;
        if (!(gRfConfig.processMask & event))
            return;
        id = 0;
    } else {
        if (!(flags & 0x20000) || !(gRfConfig.processMask & RF_STATE_TIMEOUT))
            return;
        event = RF_STATE_TIMEOUT;
        id    = RF_status;
    }
    gRfConfig.rfProcessCB(event, id);
}

bStatus_t RFRole_BasicInit(rfRoleConfig_t *p) {
    gRfConfig = *p;
    RFIP_RegInit(RF_BasicProcess);
    return 0;
}
