/* SPDX-License-Identifier: Apache-2.0 */

#ifndef GPIO_TIMING_H
#define GPIO_TIMING_H

#include "dap/dap_config.h"

/* Measured QingKe half-period cycles with highcode at CPU_CLOCK=100 MHz.
 * TX is five cycles, RX is six: the shared SHIFT delay uses the shorter TX
 * phase so both respect the requested maximum. GPIO reads and taken branches
 * are not single-cycle instructions. Raw captures: docs/dap_timing.md. */
#define GPIO_SHIFT_HALF_BASE  5
#define GPIO_JTAG_HALF_BASE   8
#define GPIO_CLOCK_HALF_BASE  4
#define GPIO_SHIFT_DELAY_BASE GPIO_SHIFT_HALF_BASE
#define GPIO_JTAG_DELAY_BASE  GPIO_JTAG_HALF_BASE
#define GPIO_CLOCK_DELAY_BASE GPIO_CLOCK_HALF_BASE
#define GPIO_ACK_HALF_BASE    6
#define GPIO_ACK_DELAY_BASE   GPIO_ACK_HALF_BASE
#define GPIO_DELAY_STEP       6

#ifndef __ASSEMBLER__
#include <stdint.h>

#define GPIO_TIMING_SHIFT 0
#define GPIO_TIMING_JTAG  1
#define GPIO_TIMING_CLOCK 2
#define GPIO_TIMING_ACK   3
#define GPIO_TIMING_COUNT 4

static inline uint32_t GPIO_DelayLoops(uint32_t half, uint32_t fastBase, uint32_t delayBase) {
    uint32_t compensation =
        GPIO_LOW_EXTRA_NOPS < GPIO_HIGH_EXTRA_NOPS ? GPIO_LOW_EXTRA_NOPS : GPIO_HIGH_EXTRA_NOPS;
    if (half <= fastBase + compensation) {
        return 0;
    }

    /* Counter setup and the final non-taken branch cancel in the measured
     * cycle budget. Round upwards; no delay is added when the fast path fits. */
    uint32_t remaining = half > delayBase + compensation ? half - delayBase - compensation : 0;
    uint32_t loops     = (remaining + GPIO_DELAY_STEP - 1) / GPIO_DELAY_STEP;

    return loops ? loops : 1;
}

static inline uint32_t GPIO_BitCycles(uint32_t fastBase, uint32_t delayBase, uint32_t loops) {
    uint32_t base = loops ? delayBase : fastBase;

    return 2 * base + GPIO_LOW_EXTRA_NOPS + GPIO_HIGH_EXTRA_NOPS + 2 * GPIO_DELAY_STEP * loops;
}
#endif
#endif
