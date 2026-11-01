#include "app_config.h"
#include "usb/usb_requests.h"

#if APP_DFU_BUILD
#include "usb/usb_descriptors.h"
#endif

#include "dfu/dfu_runtime.h"
#include "dfu/dfu_entry.h"
static volatile uint8_t DFU_detachPending, DFU_detachReady;
uint8_t                 DFU_runtimeResponse[USB_DFU_STATUS_SIZE];

void DFU_RuntimeCancelControl(void) {
    DFU_detachPending = 0;
}

int DFU_RuntimeSetup(uint8_t type, uint8_t req, uint16_t value, uint16_t index, uint16_t length) {
    if (index != USB_DFU_RUNTIME_INTERFACE ||
        (type != USB_RT_OUT_CLASS_INTERFACE && type != USB_RT_IN_CLASS_INTERFACE))
        return DFU_RUNTIME_SETUP_UNHANDLED;
    if (type == USB_RT_OUT_CLASS_INTERFACE && req == USB_DFU_DETACH && !length) {
        /* wValue is the host detach timeout; device detaches itself because
         * bitWillDetach=1. Never jump in this SETUP/ISR callback. */
        (void)value;
        DFU_detachPending = 1;
        return DFU_RUNTIME_SETUP_STATUS_IN;
    }
    uint8_t state = (DFU_detachPending || DFU_detachReady) ? USB_DFU_APP_DETACH : USB_DFU_APP_IDLE;

    if (type == USB_RT_IN_CLASS_INTERFACE && req == USB_DFU_GETSTATUS && !value &&
        length == USB_DFU_STATUS_SIZE) {
        for (unsigned i = 0; i < USB_DFU_STATUS_SIZE; i++)
            DFU_runtimeResponse[i] = 0;
        DFU_runtimeResponse[USB_DFU_STATE_OFFSET] = state;
        return USB_DFU_STATUS_SIZE;
    }
    if (type == USB_RT_IN_CLASS_INTERFACE && req == USB_DFU_GETSTATE && !value &&
        length == USB_DFU_STATE_SIZE) {
        DFU_runtimeResponse[0] = state;
        return USB_DFU_STATE_SIZE;
    }
    return DFU_RUNTIME_SETUP_STALL;
}

bool DFU_RuntimeStatusComplete(void) {
    if (DFU_detachPending) {
        DFU_detachPending = 0;
        DFU_detachReady   = 1;
        return true;
    }
    return false;
}

void DFU_RuntimePoll(void) {
    if (DFU_detachReady) {
        USB_DfuEnter();
    }
}
