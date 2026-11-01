/* Bus suspend: stop owners, HALT in a reused SRAM bank, then restore clocks
 * before any old Flash/ISR code runs. No BSS, heap or additional payload pool. */
#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "ch572/watchdog.h"
#include "bsp/board.h"
#include "usb/usb_device.h"
#include "ch572/flash_io.h"
#include "mode/mode_memory.h"

#if APP_HAS_DAP
#include "dap/dap.h"
#include "dap/swj.h"
#endif

#if APP_HAS_RF_USB
#include "rf/rf_bridge.h"
#include "rf/rf_uart.h"
#include "rf/rf_dap.h"
#endif

#define __USB_SLEEP_CODE __NO_INLINE __SECTION(".usb_sleep_ram")

extern uint8_t __usb_sleep_load[], __usb_sleep_start[], __usb_sleep_end[];

/* Copy the complete SDK unlock/write/lock sequence into the same stopped
 * SRAM bank as USB_SleepRam. Factoring the repeated sequence keeps the closure inside that bank
 * without changing safe-access fences or IRQ state. */
static __USB_SLEEP_CODE void USB_SleepWrite8(volatile uint8_t *reg, uint8_t value) {
    CH572_SAFE_ACCESS_ENABLE();
    *reg = value;
    CH572_SAFE_ACCESS_DISABLE();
}

/* Matches the EVT HALT sequence. Caller has IRQs locked, SysTick stopped,
 * every data/DMA owner retired and Flash reset. Calls stay in this bank except
 * for resident CLK_SetSysClock until the saved HCLK/Flash configuration is restored. */
static __USB_SLEEP_CODE void USB_SleepRam(uint8_t clock) {
    uint8_t  wake = R8_SLP_WAKE_CTRL, power = R8_SLP_POWER_CTRL;
    uint8_t  hf = R8_HFCK_PWR_CTRL, tune = R8_XT32M_TUNE;
    uint8_t  pll = R8_PLL_CONFIG, bat = R8_BAT_DET_CTRL;
    uint8_t  off0 = R8_SLP_CLK_OFF0, off1 = R8_SLP_CLK_OFF1;
    uint16_t pins    = R16_PIN_ALTERNATE;
    uint32_t control = PFIC->SCTLR;

    WDOG_Feed();

    /* The SIE's ordinary pullup control must not disappear with its clock. */
    R16_PIN_ALTERNATE = pins | RB_PIN_USB_EN | RB_UDP_PU_EN;
    USB_SleepWrite8(&R8_SLP_CLK_OFF0, off0 | RB_SLP_CLK_UART | RB_SLP_CLK_TMR | RB_SLP_CLK_CMP);
    USB_SleepWrite8(&R8_SLP_CLK_OFF1, (off1 | RB_SLP_CLK_BLE | RB_SLP_CLK_I2C | RB_SLP_CLK_PWMX |
                                       RB_CLK_OFF_AESCCM | RB_SLP_CLK_SPI) &
                                          ~RB_SLP_CLK_USB);
    USB_SleepWrite8(&R8_SLP_WAKE_CTRL, RB_WAKE_EV_MODE | RB_SLP_USB_WAKE);
    USB_SleepWrite8(&R8_SLP_POWER_CTRL, power & ~RB_WAKE_DLY_MOD); /* EVT 3584-cycle wake */
    USB_SleepWrite8(&R8_BAT_DET_CTRL, 0);
    USB_SleepWrite8(&R8_XT32M_TUNE, (tune & 0xfc) | 3);
    USB_SleepWrite8(&R8_HFCK_PWR_CTRL, hf & ~RB_CLK_XT32M_KEEP); /* XT_FORCE_EN = 0 */
    USB_SleepWrite8(&R8_PLL_CONFIG, pll | (1U << 5));
    R8_FLASH_CTRL = 4;

    /* WFE + SEVONPEND retains a resume edge while global IRQs are masked.
     * No Flash handler is admitted until the old clock has been restored. */
    PFIC->SCTLR = control | (1U << 2) | (1U << 3) | (1U << 4);

    /* Resume/reset may have arrived after foreground admission. The USB
     * wake source remains enabled; an already pending IRQ also wakes WFI. */
    while (R8_USB_MIS_ST & RB_UMS_SUSPEND) {
        PFIC_ClearPendingIRQ(USB_IRQn);
        if (!(R8_USB_MIS_ST & RB_UMS_SUSPEND) ||
            (R8_USB_INT_FG & (RB_UIF_BUS_RST | RB_UIF_TRANSFER)))
            break;
        WDOG_Feed();
        __asm__ volatile("wfi\n nop\n nop" ::: "memory");
    }
    USB_SleepWrite8(&R8_PLL_CONFIG, pll);
    CLK_SetSysClock((SYS_CLKTypeDef)clock);
    USB_SleepWrite8(&R8_SLP_CLK_OFF0, off0);
    USB_SleepWrite8(&R8_SLP_CLK_OFF1, off1);
    USB_SleepWrite8(&R8_HFCK_PWR_CTRL, hf);
    USB_SleepWrite8(&R8_XT32M_TUNE, tune);
    USB_SleepWrite8(&R8_BAT_DET_CTRL, bat);
    USB_SleepWrite8(&R8_SLP_WAKE_CTRL, wake);
    USB_SleepWrite8(&R8_SLP_POWER_CTRL, power);
    PFIC->SCTLR       = control;
    R16_PIN_ALTERNATE = pins;
    WDOG_Feed();
}

void USB_SuspendPoll(void) {
    /* USB_Flush already released endpoints/pins and cancelled old requests.
     * A whole DAP command has returned before the main loop can get here. */
    uint32_t admission = BSP_IrqSave();

    if (!USB_sleepPending) {
        BSP_IrqRestore(admission);
        return;
    }
    USB_sleepPending = 0; /* A later suspend posts its own pending event. */
    bool ready       = BSP_workReady;

    BSP_workReady = 0;
    BSP_IrqRestore(admission);

#if APP_HAS_RF_USB
    if (USB_suspended)
        (void)RF_BridgePrepareSuspend();
    RF_BridgeStop();
#endif

    /* A very short suspend may already have resumed while old DMA owners
     * were draining. Restore their services without cycling target power. */
    if (USB_suspended) {
        bool powered = BSP_TargetPowerEnabled();

        BSP_TargetPower(false);
        R32_PA_CLR = PIN_LED1 | PIN_LED2;

        /* Load the startup Flash controller code for its software reset;
         * then reuse that same stopped bank for the HALT closure. */
        MODE_MemoryBoot();
        FLASH_Reset();
        memcpy(__usb_sleep_start, __usb_sleep_load,
               (unsigned)(__usb_sleep_end - __usb_sleep_start));
        __asm__ volatile("fence.i" ::: "memory");
        uint32_t irq = BSP_IrqSave();

        BSP_TimeSuspend();
        USB_SleepRam(R8_CLK_SYS_CFG);
        BSP_TimeResume();
        USB_suspended = !!(R8_USB_MIS_ST & RB_UMS_SUSPEND);

#if APP_HAS_DAP
        SWJ_backend = SWJ_NONE; /* the sleeping bank replaced the execution code */
#endif

        BSP_IrqRestore(irq);
        BSP_TargetPower(powered);
    }

#if APP_HAS_RF_USB
    /* Sleep is a session boundary. Old ACKs/responses cannot consume new
     * data; restore this HOST's role without changing its USB address. */
    ready = RF_BridgeStart(RF_ROLE_HOST);
    RF_UartReset(); /* keep the host-confirmed baud and DTR across sleep */
    RF_DapReset();
    if (!MODE_AppModePreDisconnect())
        RF_BridgeDiscover(BSP_DiscoveryNonce());
#endif

    BSP_workReady = ready;
    USB_ResumeData();
}
