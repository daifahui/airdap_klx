#ifndef CLOCK_TRANSITION_H
#define CLOCK_TRANSITION_H

#include <stdbool.h>
#include "ch572/attributes.h"
#include <stdint.h>

/* Only an IRQ-locked foreground owner may arm this SDK sampler. HSE and
 * PLL must already be on: that path has at most two CLK writes. Cold boot
 * and ISP calls leave the pointer NULL and never touch the timebase. */
#define CLOCK_TRACE_CAPACITY 2U

typedef struct {
    uint32_t before, after, cfg;
} clock_sample_t;

typedef struct {
    clock_sample_t samples[CLOCK_TRACE_CAPACITY];
    uint32_t       count; /* CAPACITY + 1 reports overflow, without writing past it. */
} clock_trace_t;

extern clock_trace_t *volatile CLOCK_sdkTrace;

static __ALWAYS_INLINE bool CLOCK_CfgValid(uint8_t cfg) {
    uint8_t div = cfg & 0x1fU;

    if ((cfg & 0xe0U) == 0xc0U) {
        return cfg == 0xc0U;
    }
    if (cfg & 0x20U) {
        return false;
    }
    return (cfg & 0xc0U) == 0x40U ? (!div || div >= 6U) : div != 1U;
}

/* Caller has validated source/divisor. Preserve the old divisor while
 * leaving PLL; install the final PLL divisor before entering PLL. */
static __ALWAYS_INLINE uint8_t CLOCK_CfgBridge(uint8_t old, uint8_t next) {
    if ((old & 0xc0U) == 0x40U && !(next & 0x40U)) {
        return (next & 0xc0U) | (old & 0x1fU);
    }
    if ((next & 0xc0U) == 0x40U && !(old & 0x40U)) {
        return (old & 0xc0U) | (next & 0x1fU);
    }
    return old;
}
#endif
