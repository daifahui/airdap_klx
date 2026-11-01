#ifndef RF_BASIC_2M_H
#define RF_BASIC_2M_H

#include "ch572/ch572rf.h"

/* Private fixed-profile entries for rf_bridge: PHY_MODE_PHY_2M, whitening on,
 * and a direct encoded channel (frequency <=2300000). AA, CRC, DMA, power,
 * settling delay and receive timeout remain live parameters. */

bStatus_t RFIP_SetRx2m(const rfipRx_t *parameters);

bStatus_t RFIP_StartTx2m(const rfipTx_t *parameters);
#endif
