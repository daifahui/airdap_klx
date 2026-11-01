/* Minimal CH572 platform interface. Hardware definitions retain WCH provenance.
 * Protected-register windows preserve the verified SDK key/fence/IRQ sequence.
 * See src/ch572/LICENSE. */

#ifndef CH572_PLATFORM_H
#define CH572_PLATFORM_H

#include <stdint.h>
#include "ch572/attributes.h"
#include <string.h>
#include "app_config.h"

#include "ch572/ch572sfr.h"
#include "ch572/core_riscv.h"
#include "ch572/ch572_clock.h"

#define SAFEOPERATE __asm__ volatile("fence.i")

#define CH572_ROM_UID_ADDR 0x3F018U

/* These paired macros share a saved IRQ state and must bracket one SAM window.
 * Keep both key writes and fences, and close the window before opening another. */
#define CH572_SAFE_ACCESS_ENABLE()                                                                 \
    do {                                                                                           \
        volatile uint32_t irqState = __risc_v_disable_irq();                                       \
        SAFEOPERATE;                                                                               \
        R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;                                                     \
        R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;                                                     \
        SAFEOPERATE;

#define CH572_SAFE_ACCESS_DISABLE()                                                                \
    R8_SAFE_ACCESS_SIG = 0;                                                                        \
    __risc_v_enable_irq(irqState);                                                                 \
    SAFEOPERATE;                                                                                   \
    }                                                                                              \
    while (0)
#endif
