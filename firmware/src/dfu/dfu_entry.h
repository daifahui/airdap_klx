#ifndef CH572_DFU_ENTRY_H
#define CH572_DFU_ENTRY_H

#include <stdint.h>
#include "ch572/attributes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Complete LOCAL/HOST firmware uses src/dfu/usb_dfu_entry.S: a smaller resident
 * adapter sharing the existing SRAM clock closure BEFORE any SRAM overwrite.
 * The valid non-ISR caller stack is required until CLK_SetSysClock returns. That
 * adapter/startup/linker is selected by the main build; do not add both APIs.
 * For an independent external application, use the self-contained gate below.
 *
 * Link the exact build-generated dfu_app_entry.S into application .highcode.
 * The startup must copy .highcode and include dfu_startup_hook.inc AFTER BSS
 * clear, before main. It automatically copies the exact gate from the tail.
 * There is no preparation work at the later call site. Include the generated
 * dfu_app_entry_assert.ld in the application's SECTIONS block to check placement.
 * A missing boot preload fails closed instead of jumping to uninitialized RAM.
 *
 * Non-ISR call; no return. First assembly instructions mask CPU interrupts.
 * Code is preloaded SRAM, stack/gp independent, including the U/M gateway,
 * old DMA stop and HSE/PLL recovery. The old application stack is valid until
 * ECALL discards its automatic context; no old stack is used during copying.
 * All old application SRAM may be replaced. Never call before startup copy.
 *
 * Default entry uses application's linked _vector_base. If the application
 * relocates the active absolute-vector table, pass its actual writable SRAM
 * base to USB_DfuEnterAtVector(). PMP must permit this gate, table writes,
 * peripheral access and tail execution. Calling from ongoing Flash-controller
 * transactions/debug-halted Flash is outside the reentrant takeover contract.
 * An already-pending NMI/reset cannot be made atomic with this call.
 * User-mode CSR800 masking needs the core IE_REMAP_EN setting;
 * PFIC channels are also masked before the ECALL privilege handoff.
 *
 * HSE/PLL may be off: the caller itself must still be executable from SRAM.
 * Merely placing this callee in SRAM cannot make a Flash caller runnable.
 * Formal tail remains OFFLINE ONLY while protected-region writes are forbidden.
 */

void USB_DfuEnter(void) __NO_RETURN;

void USB_DfuEnterAtVector(uintptr_t activeVectorBase) __NO_RETURN;

#ifdef __cplusplus
}
#endif
#endif
