/* Compile-time feature selection and cooperative foreground tasks. */

#include "ch572/ch572.h"
#include "ch572/watchdog.h"
#include "bsp/board.h"

#if (APP_HAS_USB && USB_DFU_ENABLE)
#include "dfu/dfu_runtime.h"
#endif

#include "mode/mode_image.h"
#include "mode/mode_memory.h"

#if APP_HAS_DAP
#include "dap/dap_config.h"
#include "dap/swj.h"
#include "dap/dap.h"
#endif

#include "uart/uart_io.h"

#if APP_HAS_USB
#include "usb/usb_device.h"
#endif

#if APP_HAS_RF
#include "rf/rf_bridge.h"

#if APP_HAS_RF_UART
#include "rf/rf_uart.h"
#endif

#if APP_HAS_RF_DAP
#include "rf/rf_dap.h"
#endif
#endif

int main(void) {
    /* Reset-only WTR is already armed by each bank's assembly entry. Feed
     * only at completed foreground stages; hardware waits remain bounded by
     * overflow (335.5 ms at 100 MHz), never by an ISR feeding indefinitely. */
    WDOG_Feed();

#if RF_TARGET_NO_USB
    /* ROM may have used USB before restarting the application. */
    BSP_UsbDisconnect();
#endif

#if BSP_HAS_TARGET_POWER
    /* Enable target power before UID/boot-bank setup; no external pull-up. */
    if (!MODE_ImageWarm())
        BSP_TargetPower(true);
#endif

    MODE_MemoryBoot();
    WDOG_Feed();         /* boot overlay is ready */
    BSP_UsbDisconnect(); /* detach before role initialization/calibration */
    BSP_Init();
    WDOG_Feed(); /* clock, UID and timebase initialized */
    MODE_ImageBoot();

#if APP_HAS_DAP
    SWJ_Init();
    DAP_Setup();
#endif

#if !APP_HAS_RF_UART
    UART_Init();
#endif

#if APP_HAS_USB
    USB_Init();
#endif

    WDOG_Feed(); /* local peripherals ready; RF calibration gets a fresh window */

#if APP_HAS_RF
#if !APP_HAS_USB
    BSP_workReady = 0;
    bool ready    = RF_BridgeStart(RF_ROLE_TARGET);

    BSP_workReady = ready;

#if APP_HAS_RF_UART
    RF_UartInit(RF_ROLE_TARGET);
#endif

#if RF_DAP_TARGET
    RF_DapReset();
#endif

    RF_BridgeDiscover(BSP_DiscoveryNonce());
#else
    bool ready = RF_BridgeStart(RF_ROLE_HOST);

    RF_UartInit(RF_ROLE_HOST);
    RF_DapReset();
    if (!MODE_AppModePreDisconnect())
        RF_BridgeDiscover(BSP_DiscoveryNonce());
#endif
#endif

#if APP_HAS_RF_USB
    MODE_ImageFinish(ready);
#else
    MODE_ImageFinish(true);
#endif

    WDOG_Feed(); /* initialization completed; retain entry protection */
    while (1) {
        WDOG_Feed(); /* Foreground progress; never a periodic ISR feed. */

#if (APP_HAS_USB && USB_DFU_ENABLE)
        DFU_RuntimePoll(); /* Only foreground, after DETACH status ACK. */
#endif

        BSP_Poll();
        if (MODE_ImagePoll()) {
            continue;
        }

#if APP_HAS_USB
        if (BSP_stopRequested) {
            uint32_t irq = BSP_IrqSave();

            if (USB_Flush()) {
                BSP_ReleaseTarget();
                BSP_stopRequested = 0;
            }
            BSP_IrqRestore(irq);
        }
        USB_Poll();
#endif

#if APP_HAS_RF
        /* USB owns stop requests. Clearing one here could lose a late ISR
         * request between the USB task and this foreground RF task. */

        if (BSP_workMode == RF_ROLE_HOST || BSP_workMode == RF_ROLE_TARGET) {
#if APP_HAS_RF_UART
            /* Observe ISR-owned readiness at the foreground admission boundary. */
            (void)BSP_workReady;
            if (!BSP_stopRequested

#if APP_HAS_USB
                && !USB_suspended && !USB_sleepPending
#endif

            ) {
#endif

                RF_BridgePoll();

#if APP_COMBINED_TARGET
                if (!MODE_peerSleeping) {
#endif

#if APP_HAS_RF_UART
                    RF_UartPoll();
#endif

#if APP_HAS_RF_DAP
                    RF_DapPoll(false);
#endif

#if RF_TX_PREFILL
                    RF_BridgePrefill();
#endif

#if (RF_REPLY_BURST > 1 && APP_COMBINED_TARGET)
                    RF_BridgeReplyPrefill();
#endif

#if APP_COMBINED_TARGET
                }
#endif
            }

#if APP_HAS_RF_UART
        }
#endif
#endif

#if APP_HAS_USB
        if (USB_sleepPending && USB_configured && !BSP_stopRequested &&
            (R8_USB_CTRL & RB_UC_DEV_PU_EN))
            USB_SuspendPoll();
#endif
    }
}
