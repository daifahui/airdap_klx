#ifndef CH572_DFU_ENTRY_H
#define CH572_DFU_ENTRY_H

#include <stdint.h>
#include "ch572/attributes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Non-ISR, no-return entry. The preloaded SRAM adapter stops old DMA and
 * restores safe Flash clocks before calling the boot-resident DFU entry.
 * The application stack remains valid until CLK_SetSysClock returns;
 * dfu_enter then replaces SRAM, vectors, stack and global-pointer state.
 * USB_DfuEnterAtVector accepts an active writable absolute-vector table. */
void USB_DfuEnter(void) __NO_RETURN;

void USB_DfuEnterAtVector(uintptr_t activeVectorBase) __NO_RETURN;

#ifdef __cplusplus
}
#endif
#endif
