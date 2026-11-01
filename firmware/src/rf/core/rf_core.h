#ifndef RF_CORE_H
#define RF_CORE_H

#include <stdbool.h>
#include "ch572/attributes.h"
#include "ch572/ch572.h"
#include "ch572/ch572rf.h"
#include "rf/core/rf_regs.h"

#define __RF_RAM(fn) __HIGH_CODE_SECTION(#fn) __NO_INLINE
/* Preserve real timing gaps, absent from decompiler output. */
#define RF_NOPS2() __asm__ volatile("nop; nop" ::: "memory")
#define RF_NOPS4() __asm__ volatile("nop; nop; nop; nop" ::: "memory")

extern uint8_t  RFEND_co2401, RFEND_co2440, RFEND_co2480, RFEND_ga2401, RFEND_ga2440, RFEND_ga2480;
extern uint8_t  gTuneFilter, gTuneFilter2M;
extern uint16_t RFEND_tuneFlag;
extern rfRoleConfig_t gRfConfig;
extern uint32_t       RF_status, RF_receiveWait;
extern void (*lleProcess)(uint32_t);
extern volatile uint32_t *gptrBBReg, *gptrLLEReg, *gptrRFENDReg;

void RF_BasicProcess(uint32_t flags);

void RFIP_SetPHYIdle(void);

void RFIP_RegInit(void (*process)(uint32_t));

void RFEND_DevInit(void);

void RFEND_TXCtune(void);

void RFEND_RXFilter(void);

void RFEND_RXAdc(void);
#endif
