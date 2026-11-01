/* RF stop must retain SRAM execution and MMIO ordering. */
#include "rf/core/rf_core.h"
#include "ch572/attributes.h"

__RF_RAM(RFRole_Stop) bStatus_t RFRole_Stop(void) {
    RFIP_SetPHYIdle();
    return 0;
}
