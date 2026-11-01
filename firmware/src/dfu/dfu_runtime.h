#ifndef CH572_DFU_RUNTIME_H
#define CH572_DFU_RUNTIME_H

#include "usb/usb_descriptors.h"
#include <stdint.h>
#include <stdbool.h>

#define DFU_RUNTIME_SETUP_UNHANDLED (-1)
#define DFU_RUNTIME_SETUP_STALL     (-2)
#define DFU_RUNTIME_SETUP_STATUS_IN (-3)

/* Runtime DFU follows the DAP/CDC interfaces in the application descriptor. */
#define USB_DFU_RUNTIME_DESCRIPTORS                                                                \
    USB_INTERFACE_DESC_SIZE, USB_DESC_INTERFACE, USB_DFU_RUNTIME_INTERFACE, USB_ALT_DEFAULT, 0,    \
        USB_CLASS_APPLICATION, USB_DFU_SUBCLASS, USB_DFU_PROTOCOL_RUNTIME, 0, USB_DFU_DESC_SIZE,   \
        USB_DESC_DFU_FUNCTIONAL,                                                                   \
        USB_DFU_ATTR_DOWNLOAD | USB_DFU_ATTR_UPLOAD | USB_DFU_ATTR_WILL_DETACH,                    \
        (USB_DFU_DETACH_TIMEOUT_MS & 0xFF), (USB_DFU_DETACH_TIMEOUT_MS >> 8),                      \
        (USB_DFU_TRANSFER_SIZE & 0xFF), (USB_DFU_TRANSFER_SIZE >> 8), (USB_DFU_BCD_1_1 & 0xFF),    \
        (USB_DFU_BCD_1_1 >> 8)

/* Positive results are response byte counts; negative results use the
 * DFU_RUNTIME_SETUP_* codes. Data is returned through DFU_runtimeResponse.
 * Cancel on a new SETUP or USB reset; commit only after the status IN ACK. */
extern uint8_t DFU_runtimeResponse[USB_DFU_STATUS_SIZE];

int DFU_RuntimeSetup(uint8_t type, uint8_t req, uint16_t value, uint16_t index, uint16_t length);

void DFU_RuntimeCancelControl(void);

/* True only when this ACK commits DETACH; freeze mode changes at that point. */

bool DFU_RuntimeStatusComplete(void);

/* Foreground only. No effect until DETACH's status stage has completed.
 * A committed DETACH calls USB_DfuEnter() and never returns. */

void DFU_RuntimePoll(void);
#endif
