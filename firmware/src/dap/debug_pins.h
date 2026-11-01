/* SPDX-License-Identifier: Apache-2.0 */

#ifndef DEBUG_PINS_H
#define DEBUG_PINS_H

/* Include after ch572.h. These helpers are inlined into SRAM callers. */
#include "bsp/board.h"

#define PIN_DEBUG (PIN_CLK | PIN_DIO | PIN_TDI | PIN_TDO | PIN_RESET)

static inline void SWJ_DebugPinsSetup(unsigned port) {
    uint32_t output = PIN_CLK | PIN_DIO | (port == 2 ? PIN_TDI : 0);
    uint32_t input  = PIN_TDO | PIN_RESET | (port == 1 ? PIN_TDI : 0);
    uint32_t pullup = PIN_DIO | PIN_RESET | (port == 2 ? PIN_TDI | PIN_TDO : 0);

    /* Release unused pins first. PD_DRV means output drive strength when DIR=1;
     * clearing it selects 5 mA and prevents opposing pulls when DIR changes. */
    R32_PA_DIR    &= ~input;
    R32_PA_PD_DRV &= ~PIN_DEBUG;
    R32_PA_PU      = (R32_PA_PU & ~PIN_DEBUG) | pullup;
    R32_PA_CLR     = PIN_RESET; /* Open-drain emulation: latch always remains zero. */
    R32_PA_SET     = output & ~PIN_CLK;
    R32_PA_DIR    |= output & ~PIN_CLK;
    R32_PA_SET     = PIN_CLK;
    R32_PA_DIR    |= PIN_CLK; /* Data is driven HIGH before the clock can rise. */
}
#endif
