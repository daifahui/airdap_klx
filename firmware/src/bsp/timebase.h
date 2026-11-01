#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdint.h>
#include "ch572/attributes.h"

#define TIMEBASE_UNITS_PER_MS     2400000U
#define TIMEBASE_UNITS_PER_US     2400U
#define TIMEBASE_UNITS_PER_SECOND 2400000000U

typedef struct {
    uint32_t counter;
    uint32_t phase; /* Fractions of 1/2.4 GHz, always less than one ms. */
} timebase_t;

/* Exact source/divisor accounting, including fractional-MHz HSE/N.
 * Decode DIV=0 as /32. Caller excludes LSI and illegal divisors. */
static __ALWAYS_INLINE uint32_t TIME_CfgUnits(uint8_t cfg) {
    uint32_t n = cfg & 0x1fU;

    if (!n) {
        n = 32U;
    }
    return n * ((cfg & 0xc0U) == 0x40U ? 4U : 75U);
}

static __ALWAYS_INLINE void TIME_AdvanceCfg(timebase_t *t, uint32_t *ms, uint32_t counter,
                                            uint8_t cfg) {
    uint32_t delta = counter - t->counter;
    uint32_t n     = cfg & 0x1fU;

    if (!n) {
        n = 32U;
    }
    uint32_t sourceCycles = (cfg & 0xc0U) == 0x40U ? 600000U : 32000U;
    uint32_t units        = n * ((cfg & 0xc0U) == 0x40U ? 4U : 75U);
    uint32_t whole        = delta / sourceCycles;
    uint32_t phase        = t->phase + (delta - whole * sourceCycles) * units;

    /* phase < 79,200,000 even for UINT32_MAX delta and /32. */
    *ms        += whole * n + phase / TIMEBASE_UNITS_PER_MS;
    t->counter  = counter;
    t->phase    = phase % TIMEBASE_UNITS_PER_MS;
}

/* The bracketing CLK-store cycles are charged to the old source. This
 * saturating budget bounds that convention's error if each boundary cycle
 * belongs to either the old or new clock. It is not a settling-time proof. */
static __ALWAYS_INLINE uint32_t TIME_BoundaryBudget(uint32_t budget, uint32_t delta, uint8_t oldCfg,
                                                    uint8_t newCfg) {
    uint32_t oldUnits   = TIME_CfgUnits(oldCfg);
    uint32_t newUnits   = TIME_CfgUnits(newCfg);
    uint32_t difference = oldUnits > newUnits ? oldUnits - newUnits : newUnits - oldUnits;

    if (difference && delta > (UINT32_MAX - budget) / difference) {
        return UINT32_MAX;
    }
    return budget + delta * difference;
}
#endif
