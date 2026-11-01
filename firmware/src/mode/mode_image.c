/* Each role owns its entire RAM working set. Only this 64-byte mailbox survives
 * a handoff; executable loader and role startup remain in Flash throughout. */
#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "ch572/watchdog.h"
#include "mode/app_mode.h"
#include "bsp/board.h"
#include "mode/mode_image.h"

#if APP_HAS_USB
#include "usb/usb_device.h"
#endif

#if APP_HAS_RF_USB
#include "rf/rf_bridge.h"
#include "rf/rf_uart.h"
#endif

#if !APP_COMBINED_HOST
#include "uart/uart_io.h"
#endif

#define MODE_IMAGE_MAGIC 0x324B424DU

#define MODE_IMAGE_ROLE                                                                            \
    (APP_COMBINED_TARGET ? APP_WORK_RF_TARGET                                                      \
     : APP_COMBINED_HOST ? APP_WORK_RF_HOST                                                        \
                         : APP_WORK_LOCAL)
static uint32_t MODE_detachedAt;
static uint8_t  MODE_nextRole;
static bool     MODE_replacing;

extern void MODE_ImageTrap(void) __NO_RETURN;

bool MODE_ImageWarm(void) {
    return MODE_mailbox.magic == MODE_IMAGE_MAGIC && MODE_mailbox.role == MODE_IMAGE_ROLE;
}

void MODE_ImageBoot(void) {
    if (MODE_ImageWarm()) {
        MODE_AppModeRestore(MODE_mailbox.controller);
        BSP_ms = MODE_mailbox.milliseconds;

#if !BSP_USB_DONGLE
        app_mode_status_t status;

        MODE_AppModeSnapshot(&status);
        if (status.target == APP_WORK_AUTO)
            MODE_AppModeInit(BSP_ms, APP_MODE_ALL, APP_BOOT_USB_WINDOW_MS);
#endif
    } else {
        MODE_AppModeInit(BSP_ms, APP_MODE_ALL, APP_BOOT_USB_WINDOW_MS);
    }
    MODE_AppModeNoteUsbUnconfigured(); /* new bank has no current USB ACK */
    MODE_mailbox.magic = 0;
    BSP_workMode       = MODE_IMAGE_ROLE;
    BSP_workReady      = 0;
}

void MODE_ImageFinish(bool success) {
    if (!success)
        MODE_AppModeBootRadioFailed();
    app_mode_status_t status;

    MODE_AppModeSnapshot(&status);
    if (status.phase == APP_MODE_SWITCHING) {
        MODE_AppModeFinish(status.sequence, success, APP_MODE_ERROR_START_FAILED);
    }
    BSP_workReady = MODE_AppModeReady();

#if !BSP_USB_DONGLE
    if (!success && MODE_IMAGE_ROLE != APP_WORK_LOCAL) {
        MODE_ImageRequest(APP_WORK_LOCAL);
    }
#endif
}

void MODE_ImageRequest(uint8_t role) {
#if BSP_USB_DONGLE
    if (role != APP_WORK_RF_HOST)
        return;
#endif

    if (MODE_replacing || role > APP_WORK_AUTO) {
        return;
    }
    BSP_workReady = 0;

#if APP_HAS_RF_USB
    RF_BridgeStop();
#endif

#if !APP_COMBINED_HOST
    UART_Stop();
#endif

    BSP_ReleaseTarget();
    BSP_UsbDisconnect();
    MODE_nextRole   = role;
    MODE_detachedAt = BSP_ms;
    MODE_replacing  = true;
}

bool MODE_ImagePoll(void) {
    if (!MODE_replacing) {
#if APP_COMBINED_HOST
        if (MODE_AppModePreDisconnect()) {
            uint32_t irq = BSP_IrqSave();

            if (MODE_AppModeStartScan(BSP_ms)) {
                /* Single t0 is the controller deadline origin; RF Start was
                 * calibrated while detached, no blocking work between these. */
                R8_USB_CTRL |= RB_UC_DEV_PU_EN;
                RF_BridgeDiscover(BSP_DiscoveryNonce());
            }
            BSP_IrqRestore(irq);
            return true;
        }
#endif

        return false;
    }
    if ((uint32_t)(BSP_ms - MODE_detachedAt) < MODE_USB_DETACH_MS) {
        return true;
    }

    /* Every old peripheral owner has stopped, the current foreground command
     * returned, and EP0 status completed before request. No RAM access/call is
     * allowed after the role startup starts replacing the working set. */
    MODE_AppModeSave(MODE_mailbox.controller);

#if BSP_USB_DONGLE
    MODE_mailbox.role = APP_WORK_RF_HOST;
#else
    MODE_mailbox.role = MODE_nextRole == APP_WORK_AUTO ? APP_WORK_RF_HOST : MODE_nextRole;
#endif

    ++MODE_mailbox.discardedTransitions;
    __risc_v_disable_irq();
    MODE_mailbox.milliseconds = BSP_ms;
    MODE_mailbox.magic        = MODE_IMAGE_MAGIC;

    /* Keep WTR armed across ECALL, RAM replacement and initialization.
     * Give the handoff a fresh window; the new bank reloads/arms at entry. */
    WDOG_Feed();
    SysTick->CTLR = 0;
    SysTick->SR   = 0;
    PFIC->IRER[0] = UINT32_MAX;
    PFIC->IRER[1] = UINT32_MAX;
    PFIC->IPRR[0] = UINT32_MAX;
    PFIC->IPRR[1] = UINT32_MAX;

    /* The ECALL bridge returns to a Flash machine continuation before it
     * enters the original SDK startup. GPIO, including target LDO_EN, never
     * goes through reset. Old DMA/IRQ owners have already stopped above. */
    MODE_ImageTrap();
}
