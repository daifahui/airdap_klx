/* USB FS device control, CMSIS-DAP v1/v2 and CDC ACM. */

#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "usb/usb_device.h"
#include "usb/usb_descriptors.h"

#if USB_DFU_ENABLE
#include "dfu/dfu_runtime.h"
#endif

#include "bsp/board.h"
#include "mode/mode_image.h"
#include "runtime.h"

#if APP_HAS_RF_DAP
#include "rf/rf_dap.h"
#endif

#include "mode/mode_memory.h"

#if USB_HAS_DAP
#include "dap/dap_config.h"
#include "dap/swj.h"
#include "dap/dap.h"
#include "dap/dap_queue.h"
#endif

#include "uart/uart_io.h"

#if APP_HAS_RF
#include "rf/rf_bridge.h"

#if APP_HAS_RF_UART
#include "rf/rf_uart.h"
#endif
#endif

/* EP4 notification DMA follows EP0 at +64; it is never a data ring. */

#if (APP_HAS_DAP && !APP_HAS_RF)
#define __USB_POLL_CODE __HIGH_CODE
#else
#define __USB_POLL_CODE
#endif

#if APP_HAS_RF_UART
#define __CDC_FOREGROUND_CODE
#else
#define __CDC_FOREGROUND_CODE __HIGH_CODE
#endif

#define EP0_STORAGE_SIZE   128U
#define CDC_LINE_IDLE      0U
#define CDC_LINE_PENDING   1U
#define CDC_LINE_TIMED_OUT 2U

volatile uint8_t USB_configured, USB_suspended;
volatile uint8_t USB_sleepPending;

static uint8_t USB_ep0[EP0_STORAGE_SIZE] __ALIGNED(EP0_STORAGE_SIZE);

#if USB_HAS_DAP
/* Bulk EP2 OUT and EP3 IN point directly into separate owned queue buffers.
 * EP1 HID retains independent OUT/IN banks and the copying report path. */
static uint8_t USB_ep1[128] __ALIGNED(128) __SECTION(MODE_LOCAL_SECTION("ep1"));

#if !APP_COMBINED_HOST
static dap_queue_t DAP_queue __ALIGNED(DAP_QUEUE_ALIGNMENT)
    __SECTION(MODE_LOCAL_SECTION("dap_queue"));
#endif

_Static_assert(__alignof__(DAP_queue) >= DAP_QUEUE_ALIGNMENT, "transaction window alignment");

static struct {
    uint32_t         held_at, hid_held_at;
    volatile uint8_t in_busy, executing, held_length, hid_held_length;
    uint8_t          held_next, hid_idle, hid_control;

} DAP_transport __SECTION(MODE_LOCAL_SECTION("dap_transport"));

/* The DMA bank retains one unexpected regular OUT while all advertised slots
 * are occupied. Keep OUT open for TransferAbort until that bank is occupied. */
#endif

/* CDC receive data IN DMA bank. */
static uint8_t USB_ep6[USB_FS_PACKET_SIZE] __ALIGNED(USB_FS_PACKET_SIZE)
    __SECTION(MODE_LOCAL_SECTION("ep6"));
static uint8_t USB_ep5[USB_FS_PACKET_SIZE] __ALIGNED(USB_FS_PACKET_SIZE)
    __SECTION(MODE_LOCAL_SECTION("ep5"));
static volatile uint8_t CDC_inBusy, CDC_notifyBusy, CDC_dtr, CDC_notifyPending;

#if APP_HAS_RF_UART
/* EP5's accepted DMA packet owns this bank until the reliable RF queue takes
 * it. Other DAP producers may consume capacity after the earlier ACK grant. */
static uint8_t           CDC_rfOutHeld;
static volatile uint32_t CDC_rfOutCancelled;
#endif

static volatile uint8_t  CDC_zlpPending;
static volatile uint8_t  CDC_rxWaiting;
static volatile uint32_t CDC_rxStarted;
static volatile uint32_t CDC_zlpStarted;
static uint16_t          CDC_errorPending;

#if APP_HAS_RF_UART
/* EP4 owns a generation snapshot until its IN ACK. No second payload queue. */
static volatile uint32_t CDC_overrunNotified, CDC_overrunArmed;
#endif

#if !APP_HAS_RF_UART
/* Authorized after STATUS ACK, independent of the next SETUP scratch. */
static uart_line_t      CDC_linePending;
static uint32_t         CDC_lineStarted;
static volatile uint8_t CDC_lineState;
#endif

static volatile uint8_t USB_haltedIn, USB_haltedOut;

enum ep0_phase {
    EP0_IDLE,
    EP0_DATA_IN,
    EP0_STATUS_OUT,
    EP0_STATUS_IN,
    EP0_LINE_OUT,
    EP0_LINE_STATUS,
    EP0_DTR_STATUS,
    EP0_BREAK_STATUS,
    EP0_LINE_WAIT,
    EP0_DTR_WAIT,
    EP0_BREAK_WAIT,
    EP0_HID_REPORT_OUT
};
static enum ep0_phase USB_controlPhase;
static const uint8_t *USB_controlData;
static uint16_t       USB_controlRemaining;
static uint8_t        USB_controlZlp, USB_pendingAddress = 0xFF;
static uint32_t       CDC_controlWaitAt;
static uint8_t        USB_smallReply[8] __ALIGNED(4);

#if BSP_HAS_TARGET_POWER
static uint8_t USB_powerPending = UINT8_MAX;
#endif

static uint8_t  MODE_pending              = UINT8_MAX;
static uint8_t  MODE_configurationPending = UINT8_MAX;
static uint32_t MODE_statusAt;

#if (APP_HAS_RF_UART)
static void CDC_OverrunCancel(void) {
    R8_UEP4_CTRL        |= UEP_T_RES_NAK;
    R8_UEP4_T_LEN        = 0;
    CDC_notifyBusy       = 0;
    CDC_notifyPending    = 1;
    CDC_errorPending     = 0;
    CDC_overrunNotified = CDC_overrunArmed = 0;
}
#endif

#if USB_HAS_DAP
static void USB_HidCancelControl(void) {
    if ((APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) && DAP_transport.hid_control) {
        DAP_transport.hid_control     = 0;
        DAP_transport.hid_held_length = 0;
    }
}
#endif

static void USB_Ep0Stall(void) {
#if APP_HAS_RF_UART
    RF_UartHoldControls(false);
#endif

#if USB_HAS_DAP
    USB_HidCancelControl();
#endif

#if BSP_HAS_TARGET_POWER
    USB_powerPending = UINT8_MAX;
#endif

    MODE_pending              = UINT8_MAX;
    MODE_configurationPending = UINT8_MAX;
    MODE_AppModeHoldAuto(false);
    USB_controlPhase = EP0_IDLE;
    R8_UEP0_CTRL     = RB_UEP_T_TOG | RB_UEP_R_TOG | UEP_T_RES_STALL | UEP_R_RES_STALL;
}

static void USB_Ep0Status(void) {
    USB_controlPhase = EP0_STATUS_IN;
    R8_UEP0_T_LEN    = 0;
    R8_UEP0_CTRL     = RB_UEP_T_TOG | RB_UEP_R_TOG | UEP_T_RES_ACK | UEP_R_RES_NAK;
}

static void USB_Ep0CdcWait(enum ep0_phase phase) {
    /* Legal back-to-back class requests wait for the existing RF command.
     * No new UART side effect is committed until this status IN is ACKed. */
    USB_controlPhase  = phase;
    CDC_controlWaitAt = BSP_ms;
    R8_UEP0_T_LEN     = 0;
    R8_UEP0_CTRL      = RB_UEP_T_TOG | RB_UEP_R_TOG | UEP_T_RES_NAK | UEP_R_RES_NAK;
}

static void CDC_ControlPoll(void) {
    if (USB_controlPhase != EP0_LINE_WAIT && USB_controlPhase != EP0_DTR_WAIT &&
        USB_controlPhase != EP0_BREAK_WAIT) {
        return;
    }

#if !UART_RX_PACKET_RING
    uint32_t usbEnabled = PFIC_GetStatusIRQ(USB_IRQn);

    PFIC_DisableIRQ(USB_IRQn);
#endif

    enum ep0_phase phase = USB_controlPhase;
    if ((phase == EP0_LINE_WAIT || phase == EP0_DTR_WAIT || phase == EP0_BREAK_WAIT) &&
        !(R8_USB_INT_FG & (RB_UIF_TRANSFER | RB_UIF_BUS_RST | RB_UIF_SUSPEND))) {
        bool ready;

#if APP_HAS_RF_UART
        if (phase == EP0_LINE_WAIT) {
            uart_line_t line = {
                (((uint32_t)USB_smallReply[0]) | ((uint32_t)USB_smallReply[1] << 8) |
                 ((uint32_t)USB_smallReply[2] << 16) | ((uint32_t)USB_smallReply[3] << 24)),
                USB_smallReply[4], USB_smallReply[5], USB_smallReply[6]};
            ready = RF_UartLineReady(&line);
        } else {
            ready = RF_UartControlReady();
        }
#else
        ready = CDC_lineState != CDC_LINE_PENDING;
#endif

#if APP_HAS_RF_UART
        ready = ready && !CDC_rfOutHeld;
#endif

        ready = ready && !BSP_stopRequested && !(BSP_ClockState() & 2U);
        if (ready) {
            USB_Ep0Status();
            USB_controlPhase = phase == EP0_LINE_WAIT  ? EP0_LINE_STATUS
                               : phase == EP0_DTR_WAIT ? EP0_DTR_STATUS
                                                       : EP0_BREAK_STATUS;
        } else if (BSP_ms - CDC_controlWaitAt >= 5000U) {
            USB_Ep0Stall();
        }
    }

#if !UART_RX_PACKET_RING
    if (usbEnabled) {
        PFIC_EnableIRQ(USB_IRQn);
    }
#endif
}

#if !APP_HAS_RF_UART
static void CDC_LinePoll(void) {
    if (CDC_lineState != CDC_LINE_PENDING) {
        return;
    }
    if (BSP_stopRequested || !(APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL)) {
        CDC_lineState = CDC_LINE_IDLE;
        UART_ConfigureCancel();
        return;
    }
    if (UART_ConfigureLine(&CDC_linePending)) {
        CDC_lineState = CDC_LINE_IDLE;
    } else if (BSP_ms - CDC_lineStarted >= 5000U) {
        CDC_lineState = CDC_LINE_TIMED_OUT;
        UART_ConfigureCancel();
    }
}
#endif

static void USB_Ep0Next(void) {
    unsigned n =
        USB_controlRemaining > USB_FS_PACKET_SIZE ? USB_FS_PACKET_SIZE : USB_controlRemaining;
    if (n) {
        memcpy(USB_ep0, USB_controlData, n);
    }
    USB_controlData      += n;
    USB_controlRemaining -= n;
    R8_UEP0_T_LEN         = n;
    R8_UEP0_CTRL =
        (R8_UEP0_CTRL & ~(MASK_UEP_T_RES | MASK_UEP_R_RES)) | UEP_T_RES_ACK | UEP_R_RES_NAK;
}

static void USB_Ep0Read(const uint8_t *p, uint16_t n, uint16_t requested) {
    if (!requested) {
        USB_controlPhase = EP0_STATUS_OUT;
        R8_UEP0_CTRL     = RB_UEP_R_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        return;
    }
    USB_controlZlp       = (n < requested && !(n & 63));
    USB_controlData      = p;
    USB_controlRemaining = n > requested ? requested : n;
    USB_controlPhase     = EP0_DATA_IN;
    USB_Ep0Next();
}

static volatile uint8_t *USB_EndpointControl(uint8_t address) {
    switch (address) {
#if USB_HAS_DAP
        case DAP_BULK_OUT_EP:
            return &R8_UEP2_CTRL;
        case DAP_BULK_IN_EP:
            return &R8_UEP3_CTRL;
#endif

        case CDC_OUT_EP:
            return &R8_UEP5_CTRL;
        case CDC_IN_EP:
            return &R8_UEP6_CTRL;
        case CDC_NOTIFY_EP:
            return &R8_UEP4_CTRL;

#if USB_HAS_DAP
        case DAP_HID_OUT_EP:
        case DAP_HID_IN_EP:
            return &R8_UEP1_CTRL;
#endif

        default:
            return 0;
    }
}

static __NO_INLINE void USB_ResetEndpoints(void) {
    USB_haltedIn = USB_haltedOut = 0;

#if USB_HAS_DAP
    R8_UEP2_CTRL  = UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP1_CTRL  = UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP3_CTRL  = UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP2_T_LEN = R8_UEP1_T_LEN = R8_UEP3_T_LEN = 0;
    if (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) {
        memset(&DAP_transport, 0, sizeof(DAP_transport));
        memset(USB_ep1 + USB_HID_REPORT_SIZE, 0, USB_HID_REPORT_SIZE);
    }
#endif

    R8_UEP5_CTRL  = UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP6_CTRL  = UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP4_CTRL  = UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP5_T_LEN = R8_UEP6_T_LEN = R8_UEP4_T_LEN = 0;

#if APP_HAS_RF_UART
    CDC_rfOutCancelled += CDC_rfOutHeld;
    CDC_rfOutHeld       = 0;
#endif

    CDC_inBusy = CDC_notifyBusy = CDC_dtr = CDC_zlpPending = CDC_rxWaiting = 0;
    CDC_notifyPending                                                      = 1;

#if APP_HAS_RF_UART
    CDC_OverrunCancel();
#endif
}

static void USB_DataPause(void) {
    /* These endpoints use ACK/NAK/STALL; OR preserves an existing halt. */

#if USB_HAS_DAP
    R8_UEP2_CTRL |= UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP1_CTRL |= UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP3_CTRL |= UEP_T_RES_NAK;
#endif

    R8_UEP5_CTRL |= UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP6_CTRL |= UEP_T_RES_NAK;
    R8_UEP4_CTRL |= UEP_T_RES_NAK;
}

static void USB_StopIsr(void) {
    /* Revoke hardware grants before publishing stop. A resume may arrive
     * before foreground cleanup; new OUTs must not be ACKed into that old
     * generation and then discarded by USB_Flush. */
    USB_DataPause();
    BSP_stopRequested = 1;
    BSP_ReleaseTarget();

#if USB_HAS_DAP
    if (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) {
#if APP_COMBINED_HOST
        RF_DapAbort(0);
#else
        DAP_transferAbort = 1;
#endif
    }
#endif
}

#if USB_HAS_DAP
/* Keep source bookkeeping in Flash: the resident ISR only dispatches tokens. */
static __NO_INLINE void DAP_Receive(uint8_t source, uint8_t n) {
    volatile uint8_t *ctrl = source == DAP_SOURCE_HID ? &R8_UEP1_CTRL : &R8_UEP2_CTRL;
    uint8_t          *packet =
        source == DAP_SOURCE_HID ? USB_ep1 : (&DAP_queue)->data[(&DAP_queue)->bulk_rx];
    if (!USB_configured || USB_suspended || USB_sleepPending || BSP_stopRequested ||
        !BSP_workReady) {
        *ctrl = (*ctrl & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
        return;
    }

    if (n > DAP_PACKET_SIZE || (source == DAP_SOURCE_HID && n != USB_HID_REPORT_SIZE)) {
        USB_StopIsr();
    } else if (n && packet[0] == ID_DAP_TransferAbort) {
        /* Abort is out of band and has no response. A second client must not
         * abort the first client's active transfer or consume a queue slot. */

#if APP_COMBINED_HOST
        RF_DapAbort(source);
#else
        if (DAP_transport.executing == source) {
            DAP_transferAbort = 1;
        }
#endif
    } else if (n && (

                        (source == DAP_SOURCE_HID ? DAP_transport.held_length
                                                  : DAP_transport.hid_held_length) ||
                        !(source == DAP_SOURCE_HID
                              ? DAP_QueuePushSource(&DAP_queue, packet, n, BSP_ms, source)
                              : DAP_QueueAcceptSource(&DAP_queue, n, BSP_ms, DAP_SOURCE_BULK)))) {
        if (!DAP_transport.held_length && !DAP_transport.hid_held_length) {
            DAP_transport.held_next = source;
        }
        if (source == DAP_SOURCE_HID) {
            DAP_transport.hid_held_at     = BSP_ms;
            DAP_transport.hid_held_length = n;
        } else {
            DAP_transport.held_at     = BSP_ms;
            DAP_transport.held_length = n;
        }
    }
    if (source == DAP_SOURCE_BULK) {
        R16_UEP2_DMA = (uint16_t)(uintptr_t)(&DAP_queue)->data[(&DAP_queue)->bulk_rx];
    }
    if (BSP_stopRequested ||

        (source == DAP_SOURCE_HID ? DAP_transport.hid_held_length : DAP_transport.held_length)) {
        *ctrl = (*ctrl & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
    }
}
#endif

/* Keep the infrequent request decoder in Flash instead of inlining it into
 * the SRAM interrupt handler. Bulk transfers need the short SRAM path. */
static __NO_INLINE void USB_Ep0Setup(void) {
#if USB_DFU_ENABLE
    DFU_RuntimeCancelControl();
#endif

#if APP_HAS_RF_UART
    RF_UartHoldControls(false);
#endif

#if USB_HAS_DAP
    USB_HidCancelControl();
#endif

#if BSP_HAS_TARGET_POWER
    USB_powerPending = UINT8_MAX;
#endif

    MODE_pending              = UINT8_MAX;
    MODE_configurationPending = UINT8_MAX;
    MODE_AppModeHoldAuto(false);

    /* SETUP_ACT guarantees eight request bytes; RX_LEN retains the previous
     * receive length on CH572 and must not be used to validate SETUP. */
    uint8_t  type = USB_ep0[USB_SETUP_TYPE_OFFSET], req = USB_ep0[USB_SETUP_REQUEST_OFFSET];
    uint16_t value     = (((uint16_t)USB_ep0[USB_SETUP_VALUE_OFFSET]) |
                      ((uint16_t)USB_ep0[USB_SETUP_VALUE_OFFSET + 1] << 8)),
             index     = (((uint16_t)USB_ep0[USB_SETUP_INDEX_OFFSET]) |
                      ((uint16_t)USB_ep0[USB_SETUP_INDEX_OFFSET + 1] << 8)),
             length    = (((uint16_t)USB_ep0[USB_SETUP_LENGTH_OFFSET]) |
                       ((uint16_t)USB_ep0[USB_SETUP_LENGTH_OFFSET + 1] << 8));
    USB_pendingAddress = 0xFF;
    USB_controlPhase   = EP0_IDLE;
    USB_controlZlp     = 0;
    R8_UEP0_CTRL       = RB_UEP_T_TOG | RB_UEP_R_TOG | UEP_T_RES_NAK | UEP_R_RES_NAK;

#if BSP_HAS_TARGET_POWER
    if ((type & USB_REQ_TYP_MASK) == USB_REQ_TYP_VENDOR && req == USB_POWER_REQUEST) {
        if (type == USB_RT_IN_VENDOR_DEVICE && !value && !index && length == 1) {
            USB_smallReply[0] = BSP_TargetPowerEnabled();
            USB_Ep0Read(USB_smallReply, 1, length);
            return;
        }
        if (type == USB_RT_OUT_VENDOR_DEVICE && !index && !length && USB_configured) {
            USB_powerPending = value != 0;
            USB_Ep0Status();
            return;
        }
        USB_Ep0Stall();
        return;
    }
#endif

    if ((type & USB_REQ_TYP_MASK) == USB_REQ_TYP_VENDOR && req == USB_MODE_REQUEST) {
        if (type == USB_RT_IN_VENDOR_DEVICE && !value && !index && length == 2) {
            app_mode_status_t status;

            MODE_AppModeSnapshot(&status);
            USB_smallReply[0] =
                status.source == APP_MODE_AUTOMATIC || status.current == APP_WORK_NONE
                    ? USB_MODE_AUTO
                    : status.current + USB_MODE_LOCAL;
            USB_smallReply[1] = 0;
            USB_Ep0Read(USB_smallReply, 2, length);
            return;
        }
        uint8_t mode = value == USB_MODE_AUTO ? APP_WORK_AUTO : value - USB_MODE_LOCAL;

        if (type == USB_RT_OUT_VENDOR_DEVICE && !index && !length && value <= USB_MODE_RF_TARGET &&
            USB_configured && MODE_AppModeValidate(mode) == APP_MODE_ERROR_NONE) {
            MODE_pending  = mode;
            MODE_statusAt = BSP_ms;
            MODE_AppModeHoldAuto(true);
            USB_Ep0Status();
            return;
        }
        USB_Ep0Stall();
        return;
    }

#if USB_DFU_ENABLE
    if (USB_configured) {
        int result = DFU_RuntimeSetup(type, req, value, index, length);

        if (result != DFU_RUNTIME_SETUP_UNHANDLED) {
            if (result == DFU_RUNTIME_SETUP_STATUS_IN)
                USB_Ep0Status();
            else if (result > 0)
                USB_Ep0Read(DFU_runtimeResponse, result, length);
            else
                USB_Ep0Stall();
            return;
        }
    }
#endif

    if (type == USB_RT_IN_STANDARD_DEVICE && req == USB_REQ_GET_DESCRIPTOR) {
        uint16_t       n = 0;
        const uint8_t *p = USB_Descriptor(value >> 8, value & 255, index, &n);

        if (p) {
            USB_Ep0Read(p, n, length);
            return;
        }
    } else if (type == USB_RT_IN_STANDARD_INTERFACE && req == USB_REQ_GET_DESCRIPTOR) {
        uint16_t       n = 0;
        const uint8_t *p = USB_HidDescriptor(value >> 8, value & 255, index, &n);

        if (p) {
            USB_Ep0Read(p, n, length);
            return;
        }
    } else if ((type == USB_RT_IN_VENDOR_DEVICE || type == USB_RT_IN_VENDOR_INTERFACE) &&
               req == USB_OS_VENDOR_CODE &&
               ((index == USB_MS_OS10_COMPAT_INDEX && type == USB_RT_IN_VENDOR_DEVICE &&
                 value == 0) ||
                (index == USB_MS_OS10_PROPERTIES_INDEX && type == USB_RT_IN_VENDOR_INTERFACE &&
                 value < USB_INTERFACE_COUNT) ||
                (index == USB_MS_OS20_SET_INDEX && type == USB_RT_IN_VENDOR_DEVICE &&
                 value == 0))) {
        uint16_t       n = 0;
        const uint8_t *p = USB_OsDescriptor(index, value, &n);

        if (p) {
            USB_Ep0Read(p, n, length);
            return;
        }
    } else if (type == USB_RT_IN_VENDOR_DEVICE && req == USB_WEBUSB_VENDOR_CODE &&
               value == USB_WEBUSB_LANDING_PAGE && index == USB_WEBUSB_GET_URL_INDEX) {
        uint16_t       n = 0;
        const uint8_t *p = USB_WebusbDescriptor(value, &n);

        if (p) {
            USB_Ep0Read(p, n, length);
            return;
        }
    } else if (type == USB_RT_OUT_STANDARD_DEVICE && req == USB_REQ_SET_ADDRESS &&
               value < USB_ADDRESS_LIMIT && !index && !length && !USB_configured) {
        USB_pendingAddress = value;
        USB_Ep0Status();
        return;
    } else if (type == USB_RT_OUT_STANDARD_DEVICE && req == USB_REQ_SET_CONFIGURATION &&
               value <= USB_CONFIG_DEFAULT && !index && !length) {
        USB_StopIsr();
        USB_ResetEndpoints();
        USB_configured = value;
        MODE_AppModeNoteUsbUnconfigured(); /* old ACK is invalid immediately */
        MODE_configurationPending = value;
        USB_Ep0Status();
        return;
    } else if (type == USB_RT_IN_STANDARD_DEVICE && req == USB_REQ_GET_CONFIGURATION && !value &&
               !index && length == 1) {
        USB_smallReply[0] = USB_configured;
        USB_Ep0Read(USB_smallReply, 1, length);
        return;
    } else if (type == USB_RT_IN_STANDARD_INTERFACE && req == USB_REQ_GET_INTERFACE && !value &&
               index < USB_INTERFACE_COUNT && length == 1 && USB_configured) {
        USB_smallReply[0] = 0;
        USB_Ep0Read(USB_smallReply, 1, length);
        return;
    } else if (type == USB_RT_OUT_STANDARD_INTERFACE && req == USB_REQ_SET_INTERFACE && !value &&
               index < USB_INTERFACE_COUNT && !length && USB_configured) {
        USB_Ep0Status();
        return;
    } else if (req == USB_REQ_GET_STATUS && !value && length == USB_STATUS_SIZE) {
        USB_smallReply[0] = USB_smallReply[1] = 0;
        if ((type == USB_RT_IN_STANDARD_DEVICE && index == 0) ||
            (type == USB_RT_IN_STANDARD_INTERFACE && USB_configured &&
             index < USB_INTERFACE_COUNT)) {
            USB_Ep0Read(USB_smallReply, 2, length);
            return;
        }
        if (type == USB_RT_IN_STANDARD_ENDPOINT && !(index & USB_EP_INDEX_RESERVED_MASK)) {
            if (index == 0 || index == USB_EP_IN) {
                USB_Ep0Read(USB_smallReply, 2, length);
                return;
            }
            if (USB_configured && USB_EndpointControl(index)) {
                USB_smallReply[0] = !!((index & USB_EP_IN ? USB_haltedIn : USB_haltedOut) &
                                       (1U << (index & USB_EP_NUMBER_MASK)));
                USB_Ep0Read(USB_smallReply, 2, length);
                return;
            }
        }
    } else if (type == USB_RT_OUT_STANDARD_ENDPOINT &&
               (req == USB_REQ_CLEAR_FEATURE || req == USB_REQ_SET_FEATURE) && !value && !length &&
               USB_configured && !(index & USB_EP_INDEX_RESERVED_MASK)) {
        volatile uint8_t *ctrl = USB_EndpointControl(index);

        if (ctrl) {
            uint8_t bit = 1U << (index & USB_EP_NUMBER_MASK);

            if (index & USB_EP_IN) {
                if (req == USB_REQ_SET_FEATURE) {
                    USB_haltedIn |= bit;
                    *ctrl         = (*ctrl & ~MASK_UEP_T_RES) | UEP_T_RES_STALL;
                } else {
                    USB_haltedIn &= ~bit;
                    *ctrl         = (*ctrl & ~(MASK_UEP_T_RES | RB_UEP_T_TOG)) | UEP_T_RES_NAK;

#if USB_HAS_DAP
                    if ((index == DAP_BULK_IN_EP || index == DAP_HID_IN_EP) &&
                        (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL)) {
                        uint8_t source = index == DAP_BULK_IN_EP ? DAP_SOURCE_BULK : DAP_SOURCE_HID;

                        DAP_QueueRetrySource(&DAP_queue, source);
                        DAP_transport.in_busy &= ~(1U << source);
                    }
#endif

                    /* Single TX banks retain their data across a halt. */
                    if ((APP_HAS_RF_UART ? BSP_workMode == APP_WORK_RF_HOST
                                         : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL)) &&
                        ((index == CDC_IN_EP && CDC_inBusy) ||
                         (index == CDC_NOTIFY_EP && CDC_notifyBusy))) {
                        *ctrl = (*ctrl & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
                    }
                }
            } else {
                if (req == USB_REQ_SET_FEATURE) {
                    USB_haltedOut |= bit;
                    *ctrl          = (*ctrl & ~MASK_UEP_R_RES) | UEP_R_RES_STALL;
                } else {
                    USB_haltedOut &= ~bit;
                    *ctrl          = (*ctrl & ~(MASK_UEP_R_RES | RB_UEP_R_TOG)) | UEP_R_RES_NAK;
                }
            }
            USB_Ep0Status();
            return;
        }
    }

#if USB_HAS_DAP
    if (index == DAP_HID_IF && USB_configured &&
        (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL)) {
        if (type == USB_RT_IN_CLASS_INTERFACE && req == USB_HID_GET_REPORT &&
            value == USB_HID_REPORT_INPUT && length) {
            /* Return the current input report without consuming a transaction.
             * DAP responses are delivered through the originating HID IN. */
            USB_Ep0Read(USB_ep1 + USB_HID_REPORT_SIZE, USB_HID_REPORT_SIZE, length);
            return;
        }
        if (type == USB_RT_OUT_CLASS_INTERFACE && req == USB_HID_SET_REPORT &&
            value == USB_HID_REPORT_OUTPUT && length == USB_HID_REPORT_SIZE &&
            !DAP_transport.hid_held_length && !BSP_stopRequested && BSP_workReady) {
            DAP_transport.hid_control = 1;
            R8_UEP1_CTRL              = (R8_UEP1_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
            USB_controlPhase          = EP0_HID_REPORT_OUT;
            R8_UEP0_CTRL              = RB_UEP_R_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
            return;
        }
        if (type == USB_RT_IN_CLASS_INTERFACE && req == USB_HID_GET_IDLE && !value && length == 1) {
            USB_smallReply[0] = DAP_transport.hid_idle;
            USB_Ep0Read(USB_smallReply, 1, length);
            return;
        }
        if (type == USB_RT_OUT_CLASS_INTERFACE && req == USB_HID_SET_IDLE && !(value & 255) &&
            !length) {
            DAP_transport.hid_idle = value >> 8;
            USB_Ep0Status();
            return;
        }
        if (type == USB_RT_IN_CLASS_INTERFACE && req == USB_HID_GET_PROTOCOL && !value &&
            length == 1) {
            USB_smallReply[0] =
                USB_HID_PROTOCOL_REPORT; /* Report protocol; the interface is non-boot. */
            USB_Ep0Read(USB_smallReply, 1, length);
            return;
        }
        if (type == USB_RT_OUT_CLASS_INTERFACE && req == USB_HID_SET_PROTOCOL &&
            value == USB_HID_PROTOCOL_REPORT && !length) {
            USB_Ep0Status();
            return;
        }
    }
#endif

    if (index == CDC_CONTROL_IF && USB_configured &&
        (APP_HAS_RF_UART
             ? BSP_workMode == APP_WORK_RF_HOST && BSP_workReady
             : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) && BSP_LocalSelected())) {
        if (type == USB_RT_IN_CLASS_INTERFACE && req == USB_CDC_GET_LINE_CODING && !value &&
            length == USB_CDC_LINE_CODING_SIZE) {
            const uart_line_t *line =

#if APP_HAS_RF_UART
                RF_UartHostLine();
#else
                &UART_line;
#endif

            uint32_t baud = line->baud;

            for (unsigned i = 0; i < 4; i++) {
                USB_smallReply[i] = baud >> (8 * i);
            }
            USB_smallReply[USB_CDC_STOP_OFFSET]      = line->stop;
            USB_smallReply[USB_CDC_PARITY_OFFSET]    = line->parity;
            USB_smallReply[USB_CDC_DATA_BITS_OFFSET] = line->bits;
            USB_Ep0Read(USB_smallReply, USB_CDC_LINE_CODING_SIZE, length);
            return;
        }
        if (type == USB_RT_OUT_CLASS_INTERFACE && req == USB_CDC_SET_LINE_CODING && !value &&
            length == USB_CDC_LINE_CODING_SIZE) {
#if APP_HAS_RF_UART
            RF_UartHoldControls(true);
#endif

            USB_controlPhase  = EP0_LINE_OUT;
            R8_UEP5_CTRL     |= UEP_R_RES_NAK;
            R8_UEP0_CTRL      = RB_UEP_R_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
            return;
        }
        if (type == USB_RT_OUT_CLASS_INTERFACE && req == USB_CDC_SET_CONTROL_LINE && !length &&
            !(value & ~USB_CDC_CONTROL_MASK)) {
#if APP_HAS_RF_UART
            RF_UartHoldControls(true);
            R8_UEP5_CTRL |= UEP_R_RES_NAK;
#endif

            USB_smallReply[0] = !!(value & USB_CDC_CONTROL_DTR);
            if (BSP_stopRequested || (BSP_ClockState() & 2U)) {
                USB_Ep0CdcWait(EP0_DTR_WAIT);
                return;
            }

#if !APP_HAS_RF_UART
            if (CDC_lineState == CDC_LINE_PENDING) {
                USB_Ep0CdcWait(EP0_DTR_WAIT);
                return;
            }
#endif

#if APP_HAS_RF_UART
            if (CDC_rfOutHeld || !RF_UartControlReady()) {
                USB_Ep0CdcWait(EP0_DTR_WAIT);
                return;
            }
#endif

            USB_Ep0Status();
            USB_controlPhase = EP0_DTR_STATUS;
            return;
        }
        if (type == USB_RT_OUT_CLASS_INTERFACE && req == USB_CDC_SEND_BREAK && !length) {
#if APP_HAS_RF_UART
            RF_UartHoldControls(true);
            R8_UEP5_CTRL |= UEP_R_RES_NAK;
#endif

            USB_smallReply[0] = value;
            USB_smallReply[1] = value >> 8;
            if (BSP_stopRequested || (BSP_ClockState() & 2U)) {
                USB_Ep0CdcWait(EP0_BREAK_WAIT);
                return;
            }

#if !APP_HAS_RF_UART
            if (CDC_lineState == CDC_LINE_PENDING) {
                USB_Ep0CdcWait(EP0_BREAK_WAIT);
                return;
            }
#endif

#if APP_HAS_RF_UART
            if (CDC_rfOutHeld || !RF_UartControlReady()) {
                USB_Ep0CdcWait(EP0_BREAK_WAIT);
                return;
            }
#endif

            USB_Ep0Status();
            USB_controlPhase = EP0_BREAK_STATUS;
            return;
        }
    }
    USB_Ep0Stall();
}

static __NO_INLINE void USB_Ep0In(bool setupPending) {
    if (USB_controlPhase == EP0_DATA_IN) {
        if (setupPending) {
            /* The new request aborts this read. Do not overwrite its SETUP
             * bytes with an unused continuation of the previous response. */
            USB_controlPhase = EP0_IDLE;
            return;
        }
        R8_UEP0_CTRL ^= RB_UEP_T_TOG;
        if (USB_controlRemaining) {
            USB_Ep0Next();
        } else if (USB_controlZlp) {
            USB_controlZlp = 0;
            USB_Ep0Next();
        } else {
            USB_controlPhase = EP0_STATUS_OUT;
            R8_UEP0_CTRL     = RB_UEP_R_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        }
    } else if (USB_controlPhase == EP0_STATUS_IN || USB_controlPhase == EP0_LINE_STATUS ||
               USB_controlPhase == EP0_DTR_STATUS || USB_controlPhase == EP0_BREAK_STATUS) {
#if USB_DFU_ENABLE
        if (!setupPending) {
            if (DFU_RuntimeStatusComplete())
                MODE_AppModeFreeze();
        }
#endif

        if (USB_controlPhase == EP0_LINE_STATUS) {
            uart_line_t line = {
                (((uint32_t)USB_smallReply[0]) | ((uint32_t)USB_smallReply[1] << 8) |
                 ((uint32_t)USB_smallReply[2] << 16) | ((uint32_t)USB_smallReply[3] << 24)),
                USB_smallReply[4], USB_smallReply[5], USB_smallReply[6]};

#if APP_HAS_RF_UART
            if (!RF_UartLineSubmit(&line)) {
                RF_UartCommitFailed();
            }
#else
            if (CDC_lineState == CDC_LINE_PENDING) {
                BSP_Fault();
            }
            CDC_linePending = line;
            CDC_lineStarted = BSP_ms;
            CDC_lineState   = CDC_LINE_PENDING;
            BSP_ClockQueue(true);
#endif
        } else if (USB_controlPhase == EP0_DTR_STATUS) {
            CDC_dtr = USB_smallReply[0];

#if APP_HAS_RF_UART
            if (!RF_UartControlSubmit(CDC_dtr, 0)) {
                RF_UartCommitFailed();
            }
#else
            UART_Open(CDC_dtr);
#endif

            CDC_notifyPending = 1;
        } else if (USB_controlPhase == EP0_BREAK_STATUS) {
#if APP_HAS_RF_UART
            if (!RF_UartControlSubmit(CDC_dtr, (((uint16_t)USB_smallReply[0]) |
                                                ((uint16_t)USB_smallReply[1] << 8)))) {
                RF_UartCommitFailed();
            }
#else
            UART_Break((((uint16_t)USB_smallReply[0]) | ((uint16_t)USB_smallReply[1] << 8)));
#endif
        }

#if APP_HAS_RF_UART
        RF_UartHoldControls(false);
#endif

#if BSP_HAS_TARGET_POWER
        if (USB_powerPending != UINT8_MAX) {
            BSP_TargetPower(USB_powerPending != 0);
            USB_powerPending = UINT8_MAX;
        }
#endif

#if USB_HAS_DAP
        if ((APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) &&
            DAP_transport.hid_control == 2) {
            DAP_transport.hid_control = 0;
            DAP_Receive(DAP_SOURCE_HID, USB_HID_REPORT_SIZE);
        }
#endif

        if (MODE_configurationPending == 1 && !setupPending) {
            MODE_AppModeNoteUsbConfigured(BSP_ms);
        }
        MODE_configurationPending = UINT8_MAX;
        if (MODE_pending != UINT8_MAX) {
            uint32_t sequence;
            uint8_t  mode = MODE_pending;

            MODE_pending = UINT8_MAX;
            if (MODE_AppModeCommit(mode, &sequence) == APP_MODE_ERROR_NONE) {
                app_mode_status_t status;

                MODE_AppModeSnapshot(&status);
                if (status.phase == APP_MODE_PENDING) {
                    USB_StopIsr();
                } else if (status.phase == APP_MODE_READY) {
                    BSP_workReady = 1;
                }
            }
            MODE_AppModeHoldAuto(false);
        }
        if (USB_pendingAddress != 0xFF) {
            R8_USB_DEV_AD = USB_pendingAddress;
        }
        USB_pendingAddress = 0xFF;

#if BSP_HAS_TARGET_POWER
        USB_powerPending = UINT8_MAX;
#endif

        USB_controlPhase = EP0_IDLE;
        R8_UEP0_CTRL     = UEP_R_RES_ACK | UEP_T_RES_NAK;
    }
}

static __NO_INLINE void USB_Ep0Out(void) {
#if USB_HAS_DAP
    if (USB_controlPhase == EP0_HID_REPORT_OUT) {
        if (R8_USB_RX_LEN != USB_HID_REPORT_SIZE ||
            !(APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) || BSP_stopRequested) {
            USB_Ep0Stall();
            return;
        }
        runtime_word_t       *dst = (runtime_word_t *)USB_ep1;
        const runtime_word_t *src = (const runtime_word_t *)USB_ep0;

        for (unsigned i = 0; i < 16; ++i) {
            dst[i] = src[i];
        }
        __asm__ volatile("" ::: "memory");
        DAP_transport.hid_control = 2;
        USB_Ep0Status();
        return;
    }
#endif

    if (USB_controlPhase == EP0_LINE_OUT) {
        if (R8_USB_RX_LEN != 7) {
            USB_Ep0Stall();
            return;
        }
        uart_line_t line = {(((uint32_t)USB_ep0[0]) | ((uint32_t)USB_ep0[1] << 8) |
                             ((uint32_t)USB_ep0[2] << 16) | ((uint32_t)USB_ep0[3] << 24)),
                            USB_ep0[USB_CDC_STOP_OFFSET], USB_ep0[USB_CDC_PARITY_OFFSET],
                            USB_ep0[USB_CDC_DATA_BITS_OFFSET]};
        if (!

#if APP_HAS_RF_UART
            RF_UartLineValid(&line)
#else
            UART_LineValid(&line, NULL, NULL)
#endif

        ) {
            USB_Ep0Stall();
            return;
        }

        /* This 7-byte request uses the existing EP0 reply scratch until ACK.
         * A new SETUP replaces the phase before using that scratch again. */
        memcpy(USB_smallReply, USB_ep0, 7);
        if (BSP_stopRequested || (BSP_ClockState() & 2U)) {
            USB_Ep0CdcWait(EP0_LINE_WAIT);
            return;
        }

#if APP_HAS_RF_UART
        if (CDC_rfOutHeld || !RF_UartLineReady(&line)) {
            USB_Ep0CdcWait(EP0_LINE_WAIT);
            return;
        }
#else
        if (CDC_lineState == CDC_LINE_PENDING) {
            USB_Ep0CdcWait(EP0_LINE_WAIT);
            return;
        }
#endif

        USB_Ep0Status();
        USB_controlPhase = EP0_LINE_STATUS;
        return;
    }
    if (USB_controlPhase == EP0_STATUS_OUT && R8_USB_RX_LEN == 0) {
        USB_controlPhase = EP0_IDLE;
        R8_UEP0_CTRL     = UEP_R_RES_ACK | UEP_T_RES_NAK;
    } else {
        USB_Ep0Stall();
    }
}

bool USB_Flush(void) {
#if APP_COMBINED_HOST
    if (RF_dapReceiving) {
        return false;
    }
#endif

    uint32_t irq = BSP_IrqSave();

    USB_DataPause();
    uint32_t snapshot = R32_USB_STATUS;

    /* A foreground stop from an older transaction must not cancel a newly
     * accepted EP0 mode/power/report request before its status ACK. */
    bool cdcWait = false;

    /* A new legal CDC request may arrive before old SET_CONFIGURATION stop
     * cleanup. Its unacknowledged resident scratch survives that cleanup;
     * foreground then permits the new status ACK. Never cancel an accepted
     * new request merely because an older stop had not been serviced yet. */
    cdcWait = USB_controlPhase == EP0_LINE_WAIT || USB_controlPhase == EP0_DTR_WAIT ||
              USB_controlPhase == EP0_BREAK_WAIT;
    if ((USB_controlPhase != EP0_IDLE && !cdcWait) ||
        !(snapshot & ((uint32_t)RB_UMS_SIE_FREE << 8)) || ((snapshot >> 16) & RB_UIF_TRANSFER)) {
        BSP_IrqRestore(irq);
        return false;
    }

#if BSP_HAS_TARGET_POWER
    USB_powerPending = UINT8_MAX;
#endif

#if APP_HAS_RF_USB
    if (BSP_workMode == APP_WORK_RF_HOST || BSP_workMode == APP_WORK_RF_TARGET) {
#if APP_COMBINED_HOST
        /* Boot RF discovery is independent of USB reset/configuration; there
         * is no business session yet. Preserve its valid current OFFER. */
        if (!MODE_AppModeBootScanning() && !USB_sleepPending)
#endif

            RF_BridgeFlush();
    }

#if APP_HAS_RF_UART
    CDC_OverrunCancel();
    RF_UartReset();
#endif
#endif

#if USB_HAS_DAP
    /* Stop hardware ownership before resetting the pool or changing DMA. */
    R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(MASK_UEP_R_RES | MASK_UEP_T_RES)) |
                   ((USB_haltedOut & (1U << DAP_BULK_OUT_EP)) ? UEP_R_RES_STALL : UEP_R_RES_NAK) |
                   UEP_T_RES_NAK;
    R8_UEP3_CTRL =
        (R8_UEP3_CTRL & ~(MASK_UEP_R_RES | MASK_UEP_T_RES)) | UEP_R_RES_NAK |
        ((USB_haltedIn & (1U << DAP_BULK_IN_EP_NUMBER)) ? UEP_T_RES_STALL : UEP_T_RES_NAK);
    R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(MASK_UEP_R_RES | MASK_UEP_T_RES)) |
                   ((USB_haltedOut & (1U << DAP_HID_EP_NUMBER)) ? UEP_R_RES_STALL : UEP_R_RES_NAK) |
                   ((USB_haltedIn & (1U << DAP_HID_EP_NUMBER)) ? UEP_T_RES_STALL : UEP_T_RES_NAK);
    if (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) {
#if APP_COMBINED_HOST
        RF_DapReset();
#else
        DAP_QueueInit(&DAP_queue);
#endif

        R16_UEP2_DMA          = (uint16_t)(uintptr_t)DAP_queue.data[DAP_queue.bulk_rx];
        R16_UEP3_DMA          = (uint16_t)(uintptr_t)DAP_queue.data[DAP_queue.spare];
        DAP_transport.in_busy = DAP_transport.executing = DAP_transport.held_length =
            DAP_transport.hid_held_length               = 0;
        DAP_transport.hid_control                       = 0;

        memset(USB_ep1 + USB_HID_REPORT_SIZE, 0, USB_HID_REPORT_SIZE);

#if APP_HAS_DAP
        DAP_transferAbort   = 0;
        DAP_data.debug_port = 0;
        SWJ_Off();
#endif
    }
#endif

#if !APP_HAS_RF_UART
    if (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) {
        UART_Stop();
    }
#endif

#if !APP_HAS_RF_UART
    CDC_lineState = CDC_LINE_IDLE;
    if (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) {
        UART_ConfigureCancel();
    } else {
        BSP_ClockQueue(false);
    }
#endif

#if APP_HAS_RF_UART
    CDC_rfOutCancelled += CDC_rfOutHeld;
    CDC_rfOutHeld       = 0;
#endif

    /* Host drivers keep their open-port DTR across suspend. A bus reset or
     * mode switch still clears it through the ordinary lifecycle path. */
    if (!USB_sleepPending)
        CDC_dtr = 0;
    CDC_inBusy = CDC_notifyBusy = CDC_zlpPending = CDC_rxWaiting = 0;
    CDC_errorPending                                             = 0;
    CDC_notifyPending                                            = 1;

    /* Suspend and route changes are not a USB bus reset. Preserve DATA toggles
     * and halt state; SET_CONFIGURATION and BUS_RST reset them in the ISR. */

#if USB_HAS_DAP
    /* DAP DMA and control were quiesced before releasing pool ownership. */
#endif

    R8_UEP5_CTRL = (R8_UEP5_CTRL & ~(MASK_UEP_R_RES | MASK_UEP_T_RES)) |
                   ((USB_haltedOut & (1U << CDC_OUT_EP)) ? UEP_R_RES_STALL : UEP_R_RES_NAK) |
                   UEP_T_RES_NAK;
    R8_UEP6_CTRL = (R8_UEP6_CTRL & ~MASK_UEP_T_RES) |
                   ((USB_haltedIn & (1U << CDC_IN_EP_NUMBER)) ? UEP_T_RES_STALL : UEP_T_RES_NAK);
    R8_UEP4_CTRL =
        (R8_UEP4_CTRL & ~MASK_UEP_T_RES) |
        ((USB_haltedIn & (1U << CDC_NOTIFY_EP_NUMBER)) ? UEP_T_RES_STALL : UEP_T_RES_NAK);
    BSP_IrqRestore(irq);
    return true;
}

void USB_ResumeData(void) {
#if !APP_HAS_RF_UART
    /* Divisor/remap and the restored HCLK were retained; no implicit new
     * SET_LINE_CODING is required from macOS for an already open port. */
    UART_Open(CDC_dtr);
#endif

    CDC_notifyPending = 1;
    PFIC_SetPendingIRQ(USB_IRQn); /* The owner reopens the retained endpoints. */
}

void USB_Init(void) {
    /* A role handoff does not reset peripherals. Reset the old SIE before
     * rebuilding its endpoint/DMA state, including its automatic toggles. */
    R8_USB_CTRL   = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
    R8_USB_CTRL   = 0;
    R8_UEP4_1_MOD = 0;
    R8_UEP2_3_MOD = 0;
    R8_UEP567_MOD = 0;
    R16_UEP0_DMA  = (uint16_t)(uintptr_t)USB_ep0;

#if USB_HAS_DAP
    R8_UEP4_1_MOD = RB_UEP1_RX_EN | RB_UEP1_TX_EN;
    R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP3_TX_EN;
    R16_UEP1_DMA  = (uint16_t)(uintptr_t)USB_ep1;
    DAP_QueueInit(&DAP_queue);
    R16_UEP2_DMA = (uint16_t)(uintptr_t)DAP_queue.data[DAP_queue.bulk_rx];
    R16_UEP3_DMA = (uint16_t)(uintptr_t)DAP_queue.data[DAP_queue.spare];
#endif

    R8_UEP567_MOD |= RB_UEP5_RX_EN | RB_UEP6_TX_EN;
    R8_UEP4_1_MOD |= RB_UEP4_TX_EN;
    R16_UEP5_DMA   = (uint16_t)(uintptr_t)USB_ep5;
    R16_UEP6_DMA   = (uint16_t)(uintptr_t)USB_ep6;
    USB_ResetEndpoints();
    R8_UEP0_CTRL  = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_USB_DEV_AD = 0;
    R8_USB_INT_FG = 0xFF;
    R8_USB_CTRL   = RB_UC_INT_BUSY | RB_UC_DMA_EN;
    if (!MODE_AppModePreDisconnect())
        R8_USB_CTRL |= RB_UC_DEV_PU_EN;
    R16_PIN_ALTERNATE |= RB_PIN_USB_EN;
    R16_PIN_ALTERNATE &= ~RB_UDP_PU_EN;
    R8_UDEV_CTRL       = RB_UD_PD_DIS | RB_UD_PORT_EN;
    R8_USB_INT_EN      = RB_UIE_BUS_RST | RB_UIE_SUSPEND | RB_UIE_TRANSFER;
    PFIC_EnableIRQ(USB_IRQn);
}

static __CDC_FOREGROUND_CODE void CDC_OutPoll(void) {
#if !UART_RX_PACKET_RING
    uint32_t irq = BSP_IrqSave();
#endif

#if APP_HAS_RF_UART
    if (CDC_rfOutHeld) {
        if (BSP_stopRequested || !RF_UartSubmit(USB_ep5, CDC_rfOutHeld)) {
            R8_UEP5_CTRL = (R8_UEP5_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;

#if !UART_RX_PACKET_RING
            BSP_IrqRestore(irq);
#endif

            return;
        }
        CDC_rfOutHeld = 0;
    }
    bool ready = (APP_HAS_RF_UART ? BSP_workMode == APP_WORK_RF_HOST && BSP_workReady
                                  : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) &&
                                        BSP_LocalSelected()) &&
                 USB_controlPhase != EP0_LINE_STATUS && USB_controlPhase != EP0_DTR_STATUS &&
                 USB_controlPhase != EP0_BREAK_STATUS && USB_controlPhase != EP0_LINE_WAIT &&
                 USB_controlPhase != EP0_DTR_WAIT && USB_controlPhase != EP0_BREAK_WAIT &&
                 RF_UartHostReady();
#else
    bool ready = BSP_LocalSelected() && !BSP_stopRequested &&
                 UART_TxSpace() >= USB_FS_PACKET_SIZE && CDC_lineState != CDC_LINE_PENDING &&
                 USB_controlPhase != EP0_LINE_OUT && USB_controlPhase != EP0_LINE_STATUS &&
                 USB_controlPhase != EP0_LINE_WAIT && USB_controlPhase != EP0_DTR_WAIT &&
                 USB_controlPhase != EP0_BREAK_WAIT;

    /* UART RX cannot be paused on this two-wire connector. Give its USB IN
     * traffic priority before accepting more OUT packets on the same FS bus. */

#if UART_RX_PACKET_RING
    ready = ready && UART_RxPacketPressure() < UART_RX_SIZE / 2;
#else
    ready = ready && UART_RxAvailable() < UART_RX_SIZE / 2;
#endif
#endif

    R8_UEP5_CTRL = (R8_UEP5_CTRL & ~MASK_UEP_R_RES) | (ready ? UEP_R_RES_ACK : UEP_R_RES_NAK);

#if !UART_RX_PACKET_RING
    BSP_IrqRestore(irq);
#endif
}

static __CDC_FOREGROUND_CODE void CDC_InAck(volatile uint8_t *control) {
    uint32_t irq = BSP_IrqSave();
    *control     = (*control & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
    BSP_IrqRestore(irq);
}

static __CDC_FOREGROUND_CODE void CDC_InPoll(void) {
    /* Caller excludes USB re-entry; the higher-priority UART producer can
     * continue draining its FIFO while this consumer copies a USB packet. */
    if (

#if APP_HAS_RF_UART
        !(APP_HAS_RF_UART
              ? BSP_workMode == APP_WORK_RF_HOST && BSP_workReady
              : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) && BSP_LocalSelected()) ||
#endif

        !USB_configured || USB_suspended || BSP_stopRequested || !BSP_workReady) {
        return;
    }

#if (UART_RX_PACKET_RING && !UART_RX_PACKET_RING)
    /* Once per millisecond, including FIFO-only tails below ready threshold.
     * Reuse byte-ring wait fields; ISR remains the sole RBR consumer. */
    uint32_t now = BSP_ms;

    if (!CDC_rxWaiting || (uint32_t)(now - CDC_rxStarted) >= 1U) {
        CDC_rxStarted = now;
        CDC_rxWaiting = 1;
        UART_RxPacketPollFifo();
    }
#endif

    if (CDC_inBusy && !(USB_haltedIn & (1U << CDC_IN_EP_NUMBER))) {
        R8_UEP6_CTRL = (R8_UEP6_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
    }
    if (!CDC_inBusy && !(USB_haltedIn & (1U << CDC_IN_EP_NUMBER)) && USB_configured &&
        !USB_suspended && !BSP_stopRequested) {
#if APP_HAS_RF_UART
        /* Reverse payload stays in the reliable queue until this USB ACK.
         * Drain it even while a line/control result waits behind that data. */
        unsigned n = RF_UartHostRead(USB_ep6, USB_FS_PACKET_SIZE);

        if (n || (CDC_zlpPending && BSP_ms - CDC_zlpStarted >= CDC_RX_FLUSH_MS)) {
            CDC_zlpPending = n == USB_FS_PACKET_SIZE;
            CDC_zlpStarted = BSP_ms;
            CDC_inBusy     = 1;
            R8_UEP6_T_LEN  = n;
            R8_UEP6_CTRL   = (R8_UEP6_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        }
#elif UART_RX_PACKET_RING
        const uint8_t *dma = USB_ep6; /* ZLP owns no queue slot. */
        unsigned       n   = UART_RxPacketPeek(&dma);

        if (n || (!UART_RxAvailable() && CDC_zlpPending &&
                  (uint32_t)(BSP_ms - CDC_zlpStarted) >= CDC_RX_FLUSH_MS)) {
            CDC_zlpPending = n == USB_FS_PACKET_SIZE;
            CDC_zlpStarted = BSP_ms;
            CDC_inBusy     = 1;
            R16_UEP6_DMA   = (uint16_t)(uintptr_t)dma;
            R8_UEP6_T_LEN  = n;
            __asm__ volatile("" ::: "memory");
            R8_UEP6_CTRL = (R8_UEP6_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        }
#else
        unsigned available = UART_RxAvailable();

        if (available && !CDC_rxWaiting) {
            CDC_rxStarted = UART_RxStarted();
            CDC_rxWaiting = 1;
        }
        bool ready = available >= UART_CDC_IN_PACKET_LIMIT ||
                     (available && BSP_ms - CDC_rxStarted >= CDC_RX_FLUSH_MS);
        bool terminate = !available && CDC_zlpPending && BSP_ms - CDC_zlpStarted >= CDC_RX_FLUSH_MS;

        if (ready || terminate) {
            unsigned n = UART_RxRead(USB_ep6, UART_CDC_IN_PACKET_LIMIT);

            CDC_rxWaiting  = 0;
            CDC_zlpPending = (n == USB_FS_PACKET_SIZE);
            CDC_zlpStarted = BSP_ms;
            CDC_inBusy     = 1;
            R8_UEP6_T_LEN  = n;

            /* Keep copy/metadata/T_LEN before exposing the packet to SIE. */
            __asm__ volatile("" ::: "memory");
            R8_UEP6_CTRL = (R8_UEP6_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        }
#endif
    }
}

#if APP_HAS_RF_USB
bool RF_BridgeTransportReset(void) {
    /* Runtime has RF control/discovery only until routed DAP/CDC is integrated.
     * Do not consume a completion owned by EP0 or the local transports. */

#if APP_HAS_RF_UART
    R8_UEP5_CTRL |= UEP_R_RES_NAK;
    R8_UEP6_CTRL |= UEP_T_RES_NAK;
#endif

#if APP_COMBINED_HOST
    R8_UEP1_CTRL |= UEP_R_RES_NAK | UEP_T_RES_NAK;
    R8_UEP2_CTRL |= UEP_R_RES_NAK;
    R8_UEP3_CTRL |= UEP_T_RES_NAK;
#endif

    uint32_t snapshot = R32_USB_STATUS;
    bool ready = USB_controlPhase == EP0_IDLE && (snapshot & ((uint32_t)RB_UMS_SIE_FREE << 8)) &&
                 !((snapshot >> 16) & (RB_UIF_TRANSFER | RB_UIF_BUS_RST | RB_UIF_SUSPEND));

#if APP_COMBINED_HOST
    ready = ready && !RF_dapReceiving;
    if (ready) {
        RF_DapReset();
        memset(&DAP_transport, 0, sizeof(DAP_transport));
        R8_UEP1_T_LEN = R8_UEP3_T_LEN = 0;
        R16_UEP2_DMA                  = (uint16_t)(uintptr_t)DAP_queue.data[DAP_queue.bulk_rx];
        R16_UEP3_DMA                  = (uint16_t)(uintptr_t)DAP_queue.data[DAP_queue.spare];
    }
#endif

#if APP_HAS_RF_UART
    if (ready) {
        CDC_rfOutCancelled += CDC_rfOutHeld;
        CDC_rfOutHeld       = 0;
        CDC_inBusy = CDC_zlpPending = CDC_rxWaiting = 0;
        R8_UEP6_T_LEN                               = 0;
        CDC_OverrunCancel();
        RF_UartReset();
    }
#endif

#if RF_DAP_TARGET
    if (ready) {
        RF_DapReset();
    }
#endif

    return ready;
}
#endif

static __NO_INLINE void USB_ModePoll(void) {
    /* Ordinary data/DAP work does not enter the mode handoff critical section.
     * ISR publication clears readiness or posts an action before the next pass.
     * Only RF owners need to restart a cleared discovery session. */
    if (BSP_workReady && !MODE_AppModeActionPending() && !MODE_AppModeAutoDiscovery()

#if APP_HAS_RF_USB
        && RF_BridgeDiscovering()
#endif

    ) {
        return;
    }
    uint32_t          irq = BSP_IrqSave();
    app_mode_action_t action;

#if APP_HAS_RF_USB
    if ((!1 || BSP_workMode == APP_WORK_RF_HOST || BSP_workMode == APP_WORK_RF_TARGET) &&
        RF_BridgePaired()) {
        MODE_AppModeNotePeerMatched(RF_BridgePairedAt());
    }
#endif

    MODE_AppModePoll(BSP_ms);
    BSP_workReady = MODE_AppModeReady();

    /* EP0 DMA and its control reply remain resident. Do not replace an active
     * class transaction or consume an IRQ completion belonging to the old bank. */
    if (USB_controlPhase != EP0_IDLE || (R8_USB_INT_FG & RB_UIF_TRANSFER)) {
        BSP_IrqRestore(irq);
        return;
    }
    if (MODE_AppModeActionPending()) {
        USB_DataPause();
        if (!USB_Flush()) {
            BSP_IrqRestore(irq);
            return;
        }
    }
    if (!MODE_AppModeTakeAction(&action)) {
#if APP_HAS_RF_USB
        if (BSP_workMode == APP_WORK_RF_HOST || BSP_workMode == APP_WORK_RF_TARGET) {
            if (!RF_BridgeDiscovering()) {
                RF_BridgeDiscover(BSP_DiscoveryNonce());
            }
        }
#endif

        BSP_IrqRestore(irq);
        return;
    }

    /* Same foreground owns SWD and this handoff. EP0 is idle and USB_Flush
     * has relinquished all old DMA/pool owners above. */
    if (action.target == BSP_workMode) {
        /* Cold HOST bank already contains the temporary scan working set. */
        MODE_AppModeFinish(action.sequence, true, 0);
        BSP_workReady = MODE_AppModeReady();
    } else
        MODE_ImageRequest(action.target);
    BSP_IrqRestore(irq);
    return;
}

#if USB_HAS_DAP
#if (APP_HAS_DAP && !APP_HAS_RF && DAP_PACKET_SIZE == USB_FS_PACKET_SIZE)
/* Bulk DMA already contains one complete packet. The usual data handoff
 * stays in SRAM; HID, Abort and retained/full-queue handling stay in Flash. */
static __NO_INLINE __HIGH_CODE void DAP_ReceiveBulk(uint8_t n) {
    const uint8_t *packet = DAP_queue.data[DAP_queue.bulk_rx];

    if (USB_configured && !USB_suspended && !BSP_stopRequested && BSP_workReady && n &&
        packet[0] != ID_DAP_TransferAbort && !DAP_transport.held_length &&
        !DAP_transport.hid_held_length &&
        DAP_QueueAcceptSource(&DAP_queue, n, BSP_ms, DAP_SOURCE_BULK)) {
        R16_UEP2_DMA = (uint16_t)(uintptr_t)DAP_queue.data[DAP_queue.bulk_rx];
        return;
    }
    DAP_Receive(DAP_SOURCE_BULK, n);
}
#endif

static __NO_INLINE void DAP_Complete(uint8_t source) {
    volatile uint8_t *ctrl = source == DAP_SOURCE_HID ? &R8_UEP1_CTRL : &R8_UEP3_CTRL;
    *ctrl                  = (*ctrl & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
    if (DAP_transport.in_busy & (1U << source)) {
        {
            *ctrl ^= RB_UEP_T_TOG;
        }
        DAP_QueueRetireSource(&DAP_queue, source);
        DAP_transport.in_busy &= ~(1U << source);
    }
}

static __NO_INLINE void DAP_HeldPoll(void) {
    uint8_t source = DAP_transport.held_next == DAP_SOURCE_HID ? DAP_SOURCE_HID : DAP_SOURCE_BULK;

    for (unsigned i = 0; i < 2; ++i) {
        volatile uint8_t *n =
            source == DAP_SOURCE_HID ? &DAP_transport.hid_held_length : &DAP_transport.held_length;
        uint32_t time =
            source == DAP_SOURCE_HID ? DAP_transport.hid_held_at : DAP_transport.held_at;
        if (*n &&

            (source == DAP_SOURCE_HID
                 ? DAP_QueuePushSource(&DAP_queue, USB_ep1, *n, time, source)
                 : DAP_QueueAcceptSource(&DAP_queue, *n, time, DAP_SOURCE_BULK))) {
            if (source == DAP_SOURCE_BULK) {
                R16_UEP2_DMA = (uint16_t)(uintptr_t)(&DAP_queue)->data[(&DAP_queue)->bulk_rx];
            }
            *n                      = 0;
            DAP_transport.held_next = source == DAP_SOURCE_HID ? DAP_SOURCE_BULK : DAP_SOURCE_HID;
        }
        source = source == DAP_SOURCE_HID ? DAP_SOURCE_BULK : DAP_SOURCE_HID;
    }
}

/* HID is limited to one 64-byte report per millisecond. Keep its packing
 * in Flash and within the USB owner; bulk needs neither copy nor padding. */
static __NO_INLINE void DAP_HidReport(const uint8_t *reply, uint8_t n) {
    memcpy(USB_ep1 + USB_HID_REPORT_SIZE, reply, n);
    memset(USB_ep1 + USB_HID_REPORT_SIZE + n, 0, USB_HID_REPORT_SIZE - n);
}

#if APP_COMBINED_HOST
/* The USB owner arbitrates both producers. Keep this queue bookkeeping in
 * Flash so the fix does not push the resident ISR across an RF-bank boundary. */
static __NO_INLINE void USB_RfPublish(void) {
    bool halted = USB_haltedOut & (1U << CDC_OUT_EP);

    if (CDC_rfOutHeld && !halted) {
        CDC_OutPoll();
    }
    if (!CDC_rfOutHeld || halted) {
        RF_DapTransmit();
    }
}
#endif

/* Keep the banked runtime's qualified resident boundary. Its endpoint owner
 * still runs serialized by USB IRQ; only its code storage is in Flash. */
static void DAP_UsbPoll(void) {
#if APP_COMBINED_HOST
    if (!RF_BridgeActive()) {
        R8_UEP1_CTRL |= UEP_R_RES_NAK;
        R8_UEP2_CTRL |= UEP_R_RES_NAK;
        return;
    }
#endif

    if (DAP_transport.held_length || DAP_transport.hid_held_length) {
        DAP_HeldPoll();
    }

#if APP_COMBINED_HOST
    USB_RfPublish();
#endif

    /* Scan once for both sources; an absent HID client does not need its own
     * full-window search on every bulk completion and foreground kick. */
    uint32_t executed = DAP_queue.executed;

    __asm__ volatile("" ::: "memory");
    for (uint32_t id = DAP_queue.retired; id != executed; ++id) {
        dap_slot_meta_t *meta   = &DAP_queue.meta[id & (DAP_PACKET_COUNT - 1)];
        uint8_t          source = meta->source;
        unsigned inEp = source == DAP_SOURCE_HID ? DAP_HID_EP_NUMBER : DAP_BULK_IN_EP_NUMBER;

        if (meta->state == SLOT_RESPONSE && !(DAP_transport.in_busy & (1U << source)) &&
            !(USB_haltedIn & (1U << inEp))) {
            volatile uint8_t *ctrl   = source == DAP_SOURCE_HID ? &R8_UEP1_CTRL : &R8_UEP3_CTRL;
            volatile uint8_t *length = source == DAP_SOURCE_HID ? &R8_UEP1_T_LEN : &R8_UEP3_T_LEN;
            const uint8_t    *reply  = DAP_queue.data[meta->buffer_index];
            uint8_t           n      = meta->response_length;

            meta->state = SLOT_TRANSPORT_IN;
            if (source == DAP_SOURCE_HID) {
                DAP_HidReport(reply, n);
                n = USB_HID_REPORT_SIZE;
            } else {
                R16_UEP3_DMA = (uint16_t)(uintptr_t)reply;
            }
            *length                = n;
            DAP_transport.in_busy |= 1U << source;
            *ctrl                  = (*ctrl & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
            if (DAP_transport.in_busy == ((1U << DAP_SOURCE_BULK) | (1U << DAP_SOURCE_HID))) {
                break;
            }
        }
    }

    /* An already-ACKed OUT needs no register write. Only resume a NAK grant
     * after configuration or after its retained packet has been published. */
    if (!DAP_transport.held_length &&

        !(USB_haltedOut & (1U << DAP_BULK_OUT_EP)) &&
        (R8_UEP2_CTRL & MASK_UEP_R_RES) == UEP_R_RES_NAK) {
        R8_UEP2_CTRL &= ~MASK_UEP_R_RES;
    }
    if (!DAP_transport.hid_held_length && !DAP_transport.hid_control &&
        !(USB_haltedOut & (1U << DAP_HID_EP_NUMBER)) &&
        (R8_UEP1_CTRL & MASK_UEP_R_RES) == UEP_R_RES_NAK) {
        R8_UEP1_CTRL &= ~MASK_UEP_R_RES;
    }
}
#endif

#if (USB_HAS_DAP && !APP_COMBINED_HOST)
/* Execute ready DAP commands from SRAM; cold EP0, mode and CDC maintenance
 * stay in USB_Poll in Flash. False propagates an active stop to its caller. */
static __NO_INLINE __HIGH_CODE bool DAP_CommandPoll(void) {
    /* Amortize cooperative housekeeping across a ready command window.
     * IRQ owners keep moving FIFO/DMA data throughout each command. Stop
     * after one elapsed millisecond (or one queue window) so CDC control, mode
     * changes and FIFO-only tail assistance retain a foreground turn.
     * BSP_DapEnter arbitrates anew at every whole-command boundary. */

    /* HCLK stays fixed inside this foreground batch; a pending clock
     * request prevents the next BSP_DapEnter. Measure elapsed cycles so
     * a nearby millisecond tick cannot truncate an otherwise ready batch. */
    uint32_t started = SysTick->CNTL;
    uint32_t budget  = BSP_clockHz / 1000U;

    for (unsigned batch = 0; batch < DAP_PACKET_COUNT; ++batch) {
        uint8_t n = 0;

        if (DAP_transport.held_length || DAP_transport.hid_held_length) {
            PFIC_SetPendingIRQ(USB_IRQn);
        }
        uint8_t *resp;

        /* The foreground owns command execution. USB ISR owns admission,
         * endpoint registers and retirement; it can run throughout a command.
         * Hosts submit one valid command within the advertised packet capacity. */
        const uint8_t *p = NULL;

        if (BSP_DapEnter()) {
            p = DAP_QueueBegin(&DAP_queue, &n, &resp);
            if (!p) {
                BSP_DapLeave();
            }
        }
        if (p) {
            bool management = p[0] == ID_DAP_Info || p[0] == ID_DAP_Connect ||
                              p[0] == ID_DAP_Disconnect || p[0] == ID_DAP_HostStatus;
            if (!management && !BSP_LocalSelected()) {
                resp[0] = ID_DAP_Invalid;
                n       = 1;
            } else if ((p[0] == ID_DAP_SWD_Sequence && DAP_data.debug_port != DAP_PORT_SWD) ||
                       (p[0] == ID_DAP_JTAG_Sequence && DAP_data.debug_port != DAP_PORT_JTAG)) {
                resp[0] = p[0];
                resp[1] = DAP_ERROR;
                n       = 2;
            } else {
                /* Clear before publishing the active source. An Abort IRQ
                 * after publication remains visible to the current handler. */
                DAP_transferAbort = 0;

                DAP_transport.executing =
                    (p[0] == ID_DAP_Transfer || p[0] == ID_DAP_TransferBlock ||
                     (DAP_ATOMIC_COMMANDS && p[0] == ID_DAP_ExecuteCommands))
                        ? (&DAP_queue)->meta[(&DAP_queue)->executed & (DAP_PACKET_COUNT - 1)].source
                        : 0;

                uint16_t responseLength = DAP_ProcessCommand(p, resp);

                DAP_transport.executing = 0;
                n                       = responseLength;
            }

            if (BSP_stopRequested) {
                BSP_DapLeave();
                return false;
            }
            DAP_QueueFinish(&DAP_queue, n);
            BSP_DapLeave();
        }
        PFIC_SetPendingIRQ(USB_IRQn);
        if (!p || DAP_transferAbort || (uint32_t)(SysTick->CNTL - started) >= budget ||
            DAP_queue.executed == DAP_queue.produced) {
            break;
        }
    }
    return true;
}
#endif

__USB_POLL_CODE void USB_Poll(void) {
    bool selectionPending = MODE_pending != UINT8_MAX;

    if (selectionPending && BSP_ms - MODE_statusAt >= 1000U) {
        uint32_t irq          = BSP_IrqSave();
        bool     stillPending = MODE_pending != UINT8_MAX;

        if (stillPending && BSP_ms - MODE_statusAt >= 1000U) {
            USB_Ep0Stall();
        }
        BSP_IrqRestore(irq);
    }
    USB_ModePoll();
    if (!USB_configured || USB_suspended || BSP_stopRequested || !BSP_workReady) {
        return;
    }

#if !UART_RX_PACKET_RING
    CDC_ControlPoll();
#endif

#if !APP_HAS_RF_UART
    CDC_LinePoll();
#endif

#if (USB_HAS_DAP && !APP_COMBINED_HOST)
    if (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) {
        if (!DAP_CommandPoll()) {
            return;
        }
    }
#endif

    if ((APP_HAS_RF_UART
             ? BSP_workMode == APP_WORK_RF_HOST && BSP_workReady
             : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) && BSP_LocalSelected())) {
#if !APP_HAS_RF_UART
#if UART_RX_PACKET_RING
        /* Packet/FIFO deadlines are IRQ-owned. With DAP active, amortize the
         * millisecond LED/BREAK/close/error/notification housekeeping rather
         * than repeating it between every command. CDC line commits above
         * remain independent of this maintenance cadence. */
        if (APP_HAS_DAP && CDC_rxWaiting && (uint32_t)(BSP_ms - CDC_rxStarted) < 1U) {
            return;
        }
#endif

        UART_Poll();
#endif

#if UART_RX_PACKET_RING
        /* Main never owns the data endpoints. One bounded tick assist covers
         * FIFO-only tails and partial slots when no full-slot IRQ kick occurs. */
        uint32_t now = BSP_ms;

        if (!CDC_rxWaiting || (uint32_t)(now - CDC_rxStarted) >= 1U) {
            CDC_rxStarted = now;
            CDC_rxWaiting = 1;
            UART_RxPacketPollFifo();
            PFIC_SetPendingIRQ(USB_IRQn);
        }
#else
        PFIC_DisableIRQ(USB_IRQn);
        CDC_InPoll();

#if !UART_RX_PACKET_RING
        PFIC_EnableIRQ(USB_IRQn);
#endif

        /* In APP2 owner-only mode, halt check and endpoint update share the
         * same USB exclusion window. UART may grow TX space or RX pressure. */
        if (!(USB_haltedOut & (1U << CDC_OUT_EP))) {
            CDC_OutPoll();
        }

#if UART_RX_PACKET_RING
        PFIC_EnableIRQ(USB_IRQn);
#endif
#endif /* UART_RX_PACKET_RING */

#if !APP_HAS_RF_UART
        CDC_errorPending |= UART_TakeSerialErrors();
#endif

#if APP_HAS_RF_UART
        /* Only USB can reset or ACK this owner; UART/RF IRQs stay serviceable. */
        PFIC_DisableIRQ(USB_IRQn);
        uint32_t generation     = RF_UartOverrunGeneration();
        bool     overrunPending = generation != CDC_overrunNotified;
#endif

        if (USB_configured && !BSP_stopRequested &&
            (APP_HAS_RF_UART
                 ? BSP_workMode == APP_WORK_RF_HOST && BSP_workReady
                 : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) && BSP_LocalSelected()) &&
            !CDC_notifyBusy && !(USB_haltedIn & (1U << CDC_NOTIFY_EP_NUMBER)) &&
            (CDC_notifyPending || CDC_errorPending

#if APP_HAS_RF_UART
             || overrunPending
#endif

             )) {
            runtime_word_t *header = (runtime_word_t *)(USB_ep0 + USB_FS_PACKET_SIZE);

            header[0] = (USB_RT_IN_CLASS_INTERFACE | (USB_CDC_NOTIFY_SERIAL_STATE << 8));
            header[1] = CDC_CONTROL_IF | (USB_CDC_SERIAL_STATE_SIZE << 16);
            uint16_t state =
                CDC_errorPending | (CDC_dtr ? (USB_CDC_STATE_DCD | USB_CDC_STATE_DSR) : 0);

#if APP_HAS_RF_UART
            if (overrunPending) {
                state |= USB_CDC_STATE_OVERRUN;
            }
#endif

            *(runtime_half_t *)(USB_ep0 + USB_FS_PACKET_SIZE + USB_CDC_NOTIFICATION_HEADER_SIZE) =
                state;
            CDC_errorPending  = 0;
            CDC_notifyPending = 0;
            CDC_notifyBusy    = 1;

#if APP_HAS_RF_UART
            CDC_overrunArmed = generation;
#endif

            R8_UEP4_T_LEN = USB_CDC_NOTIFICATION_HEADER_SIZE + USB_CDC_SERIAL_STATE_SIZE;
            CDC_InAck(&R8_UEP4_CTRL);
        }

#if APP_HAS_RF_UART
        PFIC_EnableIRQ(USB_IRQn);
#endif
    }
}

/* Reset is infrequent and has no FIFO service deadline. Keep the lifecycle
 * work in Flash; UART RX may preempt it. The ISR clears the event last. */
static __NO_INLINE void USB_ResetEvent(void) {
#if USB_DFU_ENABLE
    DFU_RuntimeCancelControl();
#endif

#if BSP_HAS_TARGET_POWER
    USB_powerPending = UINT8_MAX;
#endif

    USB_configured = USB_suspended = USB_sleepPending = 0;
    MODE_AppModeNoteUsbUnconfigured();
    MODE_pending              = UINT8_MAX;
    MODE_configurationPending = UINT8_MAX;
    MODE_AppModeHoldAuto(false);
    USB_pendingAddress = 0xFF;
    USB_controlPhase   = EP0_IDLE;
    R8_USB_DEV_AD      = 0;
    USB_StopIsr();
    USB_ResetEndpoints();
    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
}

__INTERRUPT __HIGH_CODE void USB_IRQHandler(void) {
    uint8_t flags = R8_USB_INT_FG;

    if (flags & RB_UIF_BUS_RST) {
        USB_ResetEvent();
        R8_USB_INT_FG = RB_UIF_BUS_RST | RB_UIF_TRANSFER;
        return;
    }
    if (flags & RB_UIF_SUSPEND) {
        USB_suspended = !!(R8_USB_MIS_ST & RB_UMS_SUSPEND);
        if (USB_suspended) {
            USB_sleepPending = 1; /* Resume cannot erase pending owner cleanup. */
            USB_StopIsr();
        }
        R8_USB_INT_FG = RB_UIF_SUSPEND;
    }
    if (flags & RB_UIF_TRANSFER) {
        uint8_t status = R8_USB_INT_ST, ep = status & MASK_UIS_ENDP,
                token = status & MASK_UIS_TOKEN;

        /* A non-idle token is a pending completion, even with SETUP_ACT.
         * CH572 requires clearing that completion before clearing SETUP. */
        if (ep == USB_CONTROL_EP && token == UIS_TOKEN_IN) {
            USB_Ep0In(!!(status & RB_UIS_SETUP_ACT));
        } else if (ep == USB_CONTROL_EP && token == UIS_TOKEN_OUT && (status & RB_UIS_TOG_OK)) {
            /* A new SETUP replaces the EP0 buffer and aborts old OUT data. */
            if (!(status & RB_UIS_SETUP_ACT)) {
                USB_Ep0Out();
            }
        }

#if USB_HAS_DAP
        else if ((APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) &&
                 (ep == DAP_BULK_OUT_EP || ep == DAP_HID_EP_NUMBER) && token == UIS_TOKEN_OUT &&
                 (status & RB_UIS_TOG_OK)) {
            if (ep == DAP_HID_EP_NUMBER) {
                R8_UEP1_CTRL ^= RB_UEP_R_TOG;
            } else {
                R8_UEP2_CTRL ^= RB_UEP_R_TOG;
            }

#if (APP_HAS_DAP && !APP_HAS_RF && DAP_PACKET_SIZE == USB_FS_PACKET_SIZE)
            if (ep == DAP_BULK_OUT_EP) {
                DAP_ReceiveBulk(R8_USB_RX_LEN);
            } else
#endif

            {
                DAP_Receive(ep == DAP_BULK_OUT_EP ? DAP_SOURCE_BULK : DAP_SOURCE_HID,
                            R8_USB_RX_LEN);
            }
        } else if ((APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) &&
                   (ep == DAP_BULK_IN_EP_NUMBER || ep == DAP_HID_EP_NUMBER) &&
                   token == UIS_TOKEN_IN) {
            DAP_Complete(ep == DAP_BULK_IN_EP_NUMBER ? DAP_SOURCE_BULK : DAP_SOURCE_HID);
        }
#endif

        /* Pending mode/stop gates new traffic, but the old bank still owns
         * its completed tokens until the SIE barrier releases that bank. */
        else if ((APP_HAS_RF_UART ? BSP_workMode == APP_WORK_RF_HOST
                                  : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL)) &&
                 ep == CDC_OUT_EP && token == UIS_TOKEN_OUT && (status & RB_UIS_TOG_OK)) {
            {
#if APP_HAS_RF_UART
                /* Revoke the grant and advance this accepted packet's toggle
                 * in one register update, before releasing INT_BUSY. */
                R8_UEP5_CTRL =
                    ((R8_UEP5_CTRL ^ RB_UEP_R_TOG) & ~MASK_UEP_R_RES) |
                    ((USB_haltedOut & (1U << CDC_OUT_EP)) ? UEP_R_RES_STALL : UEP_R_RES_NAK);
#else
#if UART_RX_PACKET_RING
                /* This completed packet consumes the old capacity grant. */
                if (!(USB_haltedOut & (1U << CDC_OUT_EP))) {
                    R8_UEP5_CTRL = (R8_UEP5_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
                }
#endif

                R8_UEP5_CTRL ^= RB_UEP_R_TOG;
#endif

#if APP_HAS_RF_UART
                /* CDC_OutPoll below commits this same DMA bank or keeps it
                 * NAKed; do not duplicate the RF publication in the ISR. */
                CDC_rfOutHeld = R8_USB_RX_LEN;
#else
                if (!UART_TxSubmit(USB_ep5, R8_USB_RX_LEN)) {
                    USB_StopIsr();
                }
#endif

#if !UART_RX_PACKET_RING
                if (!(USB_haltedOut & (1U << CDC_OUT_EP))) {
                    CDC_OutPoll();
                }
#endif
            }
        } else if ((APP_HAS_RF_UART ? BSP_workMode == APP_WORK_RF_HOST
                                    : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL)) &&
                   ep == CDC_IN_EP_NUMBER && token == UIS_TOKEN_IN) {
            R8_UEP6_CTRL = (R8_UEP6_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
            if (CDC_inBusy) {
#if APP_HAS_RF_UART
                RF_UartHostRetire(R8_UEP6_T_LEN);
#endif

#if UART_RX_PACKET_RING
                UART_RxPacketRetire(R8_UEP6_T_LEN);
#endif

                R8_UEP6_CTRL ^= RB_UEP_T_TOG;
            }
            CDC_inBusy = 0;

#if !UART_RX_PACKET_RING
            /* Keep a queued full packet moving within this USB frame. */
            CDC_InPoll();
            if (!(USB_haltedOut & (1U << CDC_OUT_EP))) {
                CDC_OutPoll();
            }
#endif
        } else if ((APP_HAS_RF_UART ? BSP_workMode == APP_WORK_RF_HOST
                                    : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL)) &&
                   ep == CDC_NOTIFY_EP_NUMBER && token == UIS_TOKEN_IN) {
            R8_UEP4_CTRL = (R8_UEP4_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
            if (CDC_notifyBusy) {
                R8_UEP4_CTRL ^= RB_UEP_T_TOG;

#if APP_HAS_RF_UART
                CDC_overrunNotified = CDC_overrunArmed;
#endif
            }
            CDC_notifyBusy = 0;
        }
        if (token != MASK_UIS_TOKEN) {
            R8_USB_INT_FG = RB_UIF_TRANSFER;
        }
        if (R8_USB_INT_ST & RB_UIS_SETUP_ACT) {
            USB_Ep0Setup();
            R8_USB_INT_FG = RB_UIF_TRANSFER;
        }
    }

#if USB_HAS_DAP
    /* Both real completions and foreground software kicks enter this owner.
     * Leave a newly pending hardware event for its next ISR, without clearing
     * it merely to service a kick. Pool reset remains a stopped-bus operation. */
    if (!(R8_USB_INT_FG & (RB_UIF_TRANSFER | RB_UIF_BUS_RST | RB_UIF_SUSPEND)) &&
        (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) && USB_configured && !USB_suspended &&
        !USB_sleepPending && !BSP_stopRequested && BSP_workReady) {
        DAP_UsbPoll();
    }
#endif

#if UART_RX_PACKET_RING
    /* Hardware completions and SETUP keep their existing clear order. A new
     * pending event belongs to the next ISR; never clear it to service a kick.
     * flags==0 is a software wake and must reach this same owner service. */
    if (!(R8_USB_INT_FG & (RB_UIF_TRANSFER | RB_UIF_BUS_RST | RB_UIF_SUSPEND)) &&
        (APP_HAS_RF_UART
             ? BSP_workMode == APP_WORK_RF_HOST && BSP_workReady
             : (APP_COMBINED_HOST || BSP_workMode == APP_WORK_LOCAL) && BSP_LocalSelected()) &&
        USB_configured && !USB_suspended && !USB_sleepPending && !BSP_stopRequested &&
        BSP_workReady) {
        CDC_ControlPoll();
        CDC_InPoll();
        if (!(USB_haltedOut & (1U << CDC_OUT_EP))) {
            CDC_OutPoll();
        }
    }
#endif
}
