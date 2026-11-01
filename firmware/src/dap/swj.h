/* SPDX-License-Identifier: Apache-2.0 */

#ifndef SWJ_H
#define SWJ_H

#include <stdint.h>
#include <stdbool.h>
#include "dap/gpio_timing.h"

enum swj_backend {
    SWJ_GPIO_SWD,
    SWJ_SPI_SWD,
    SWJ_GPIO_JTAG,
    SWJ_NONE = 255
};

extern volatile uint32_t SWJ_delayLoops[GPIO_TIMING_COUNT];
extern uint32_t          SWJ_requestedHz, SWJ_actualHz;
extern enum swj_backend  SWJ_backend;
extern uint8_t           SWJ_spiDiv;
extern uint32_t          SWJ_spiBudgetCycles;
extern uint32_t          SWJ_spiTimeouts;

uint8_t  SWD_GpioTransfer(uint32_t, const uint8_t *, uint8_t *);
uint8_t  SWD_SpiTransfer(uint32_t, const uint8_t *, uint8_t *);
void     SWD_GpioSequence(uint32_t, const uint8_t *, uint8_t *);
void     SWD_SpiSequence(uint32_t, const uint8_t *, uint8_t *);
void     JTAG_GpioSequence(uint32_t, const uint8_t *, uint8_t *);
void     JTAG_GpioIr(uint32_t);
uint32_t JTAG_GpioIdCode(void);
void     JTAG_GpioAbort(uint32_t);
uint8_t  JTAG_GpioTransfer(uint32_t, uint32_t *);

void SWJ_Init(void);

bool SWJ_Clock(uint32_t hz);

void SWJ_Port(uint8_t port);

void SWJ_Off(void);

uint32_t GPIO_Shift(uint32_t tx, uint32_t bits, uint32_t driveMask, uint32_t sampleMask);

/* gpio_shift accepts 0..32 LSB-first bits. Drive: 0, TDI or DIO;
 * sample: 0, TDO or DIO. SET/CLR writes affect only the selected pins. */

uint32_t SWJ_GpioInput(uint32_t mask);

void SWJ_GpioOutput(uint32_t mask, uint32_t bit);

void SWJ_DioEnable(bool on);

/* Resident Flash recovery; it calls only the resident GPIO engine. */

uint8_t SWD_Recover(uint32_t req, uint32_t ack);
#endif
