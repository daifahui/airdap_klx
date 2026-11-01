#include "rf/core/rf_core.h"

/* Shared RF controller state and calibration coefficients. */
uint8_t        RFEND_co2401, RFEND_co2440, RFEND_co2480, RFEND_ga2401, RFEND_ga2440, RFEND_ga2480;
uint8_t        gTuneFilter, gTuneFilter2M;
uint16_t       RFEND_tuneFlag;
rfRoleConfig_t gRfConfig;
uint32_t       RF_status, RF_receiveWait;
void (*lleProcess)(uint32_t);
volatile uint32_t *gptrBBReg, *gptrLLEReg, *gptrRFENDReg;
