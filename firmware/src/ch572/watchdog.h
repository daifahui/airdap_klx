#ifndef CH572_WATCHDOG_H
#define CH572_WATCHDOG_H

#include "ch572/ch572sfr.h"
#include "ch572/attributes.h"

static __ALWAYS_INLINE void WDOG_Feed(void) {
    R8_WDOG_COUNT = 0;
}
#endif
