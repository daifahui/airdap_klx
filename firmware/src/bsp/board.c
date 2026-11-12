/* Board clock, pin ownership, route cancellation and LED state. */

#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "ch572/watchdog.h"
#include "bsp/board.h"
#include "mode/mode_image.h"

#if APP_COMBINED_TARGET
#include "mode/app_mode.h"
#endif

#include "bsp/led_state.h"
#include "dap/debug_pins.h"

#if APP_HAS_RF_USB
#include "rf/rf_bridge.h"
#endif

#if APP_HAS_DAP
#include "dap/dap.h"
#endif

#if APP_HAS_RF_UART
/* Cold control and LED expiry yield RAM to the physical RF receive path. */
#define __BSP_CONTROL_CODE
#else
#define __BSP_CONTROL_CODE __HIGH_CODE
#endif

volatile uint32_t BSP_ms;
volatile uint8_t  BSP_stopRequested;
volatile uint8_t  BSP_workMode;
volatile uint8_t  BSP_workReady = 1;
char              BSP_serial[BSP_SERIAL_LENGTH + 1U];

#if APP_COMBINED_TARGET
static uint32_t MODE_targetLinkCheckedAt;
#endif

static volatile led_state_t BSP_leds[2];
static volatile uint32_t    BSP_ledSample;

__BSP_CONTROL_CODE void BSP_ReleaseTarget(void) {
#if !BSP_USB_DONGLE
    {
        R32_PA_DIR    &= ~PIN_TARGET;
        R32_PA_PU     &= ~PIN_TARGET;
        R32_PA_PD_DRV &= ~PIN_TARGET;
    }
    R8_SPI_CTRL_MOD = 0;
    R8_UART_IER     = 0;
    R32_PA_CLR      = PIN_RESET;
#endif
}

void BSP_UsbDisconnect(void) {
    R8_USB_INT_EN = 0;
    PFIC_DisableIRQ(USB_IRQn);
    R8_USB_CTRL        = 0;
    R8_UDEV_CTRL       = 0;
    R16_PIN_ALTERNATE &= ~(RB_UDP_PU_EN | RB_PIN_USB_EN);
    R32_PA_DIR        &= ~3U;
    R32_PA_PU         &= ~3U;
    R32_PA_PD_DRV     &= ~3U;
}

void BSP_TargetPower(bool on) {
#if BSP_HAS_TARGET_POWER
    R32_PA_PU     &= ~PIN_LDO_EN;
    R32_PA_PD_DRV &= ~PIN_LDO_EN;
    if (on) {
        R32_PA_SET = PIN_LDO_EN;
    } else {
        R32_PA_CLR = PIN_LDO_EN;
    }

    /* Preload the commanded level before enabling push-pull output. */
    R32_PA_DIR |= PIN_LDO_EN;
#else
    (void)on;
#endif
}

bool BSP_TargetPowerEnabled(void) {
#if BSP_HAS_TARGET_POWER
    return (R32_PA_DIR & PIN_LDO_EN) && (R32_PA_OUT & PIN_LDO_EN);
#else
    return true;
#endif
}

uint64_t BSP_DiscoveryNonce(void) {
    /* Session freshness, not cryptographic randomness. SysTick alone repeats
     * after deterministic software-reset startup. RTC is read only here and
     * is never reloaded by the application/DFU; mix its running counters too.
     * The existing mailbox sequence also distinguishes warm-role attempts. */
    uint32_t high = R32_RTC_CNT_DIV2;
    uint32_t low  = BSP_ms ^ (uint32_t)SysTick->CNT ^ R32_RTC_CNT_LSI;

    low ^= ++MODE_mailbox.binding.reserved;
    for (unsigned i = 0; i < BSP_SERIAL_LENGTH; i++) {
        high = (high << 5) ^ (high >> 27) ^ (uint8_t)BSP_serial[i];
        low  = (low << 3) ^ (low >> 29) ^ (uint8_t)BSP_serial[BSP_SERIAL_LENGTH - 1U - i];
    }
    uint64_t nonce = ((uint64_t)high << 32) | low;

    return nonce ? nonce : 1;
}

void BSP_Init(void) {
    if ((APP_HAS_DAP && !APP_HAS_RF) || APP_HAS_RF_USB) {
        BSP_ledSample = UINT32_MAX;
    }
    BSP_ReleaseTarget();

#if BSP_HAS_TARGET_POWER
    if (!MODE_ImageWarm())
        BSP_TargetPower(true);
#endif

#if APP_HAS_USB
    R16_PIN_ALTERNATE &= ~RB_PIN_DEBUG_EN;
#endif

    CLK_SetHseCapacitance(BSP_HSE_CAP);
    CLK_SetSysClock(CLK_SOURCE_HSE_PLL_100MHz);
    R32_PA_CLR     = PIN_LED1 | PIN_LED2; /* Active-high LEDs stay off at startup. */
    R32_PA_PD_DRV &= ~(PIN_LED1 | PIN_LED2);
    R32_PA_DIR    |= PIN_LED1 | PIN_LED2;
    uint8_t uid[BSP_UID_SIZE] __ALIGNED(4);

    BSP_ReadIdentity(uid);
    const char hex[] = "0123456789ABCDEF";
    for (unsigned i = 0; i < BSP_UID_SIZE; i++) {
        BSP_serial[2 * i]     = hex[uid[i] >> 4];
        BSP_serial[2 * i + 1] = hex[uid[i] & 15];
    }
    BSP_serial[BSP_SERIAL_LENGTH] = 0;
    BSP_TimeInit();

    /* EVT startup writes INTSYSCR=3, leaving PREEMPT[3:2]=0. INESTEN and
     * INESTCR alone cannot permit pre-emption: select bit7 as its priority. */
    uint32_t interruptControl = read_csr(0x804);

    write_csr(0x804, (interruptControl & ~0xCU) | 0x4U);

    /* The eight-byte UART FIFO must pre-empt USB control/data processing. */
    PFIC_SetPriority(UART_IRQn, 0);

#if APP_HAS_RF_USB
    /* CH572 supports two active interrupt levels. UART alone is the higher
     * level, so it may pre-empt RF without requiring a third nested ISR. */
    PFIC_SetPriority(BLEB_IRQn, 1);
    PFIC_SetPriority(BLEL_IRQn, 1);
#endif

    PFIC_SetPriority(USB_IRQn, 1);
    PFIC_SetPriority(SysTick_IRQn, 1);
}

__INTERRUPT __HIGH_CODE void SysTick_Handler(void) {
    BSP_TimeTick();
}

void BSP_DelayUs(uint32_t us) {
    while (us && !BSP_stopRequested) {
        /* Bounded host-requested delays are progress, including several
         * Delay commands in one Atomic request. A stuck wait still expires. */
        WDOG_Feed();
        uint32_t part  = us > 10000U ? 10000U : us;
        uint32_t start = BSP_TimeUs();

        while ((uint32_t)(BSP_TimeUs() - start) < part && !BSP_stopRequested) {
        }
        us -= part;
    }
}

void BSP_LedStatus(unsigned i, uint8_t on) {
    if (i < 2) {
#if !APP_COMBINED_HOST
        BSP_leds[i].status = !!on;
#else
        (void)on;
#endif

        if (APP_HAS_DAP && !APP_HAS_RF) {
            BSP_ledSample = BSP_ms - 1U;
        }
    }
}

void BSP_LedActivity(unsigned i) {
    if (i < 2) {
        LED_SetActivity(&BSP_leds[i], BSP_ms);
        if (APP_HAS_DAP && !APP_HAS_RF) {
            BSP_ledSample = BSP_ms - 1U;
        }
    }
}

__HIGH_CODE bool BSP_LocalSelected(void) {
#if APP_COMBINED_HOST
    /* A different role loads its own image before it can drive target pins. */
    return false;
#elif RF_DAP_TARGET
    return BSP_workMode == RF_ROLE_TARGET && BSP_workReady;
#elif APP_HAS_RF_UART
    /* Only the remote target role may drive this profile's physical UART. */
    return BSP_workMode == RF_ROLE_TARGET && BSP_workReady;
#else
    return BSP_workMode == 0 && BSP_workReady;
#endif
}

static __NO_INLINE void LED_Poll(void) {
#if APP_HAS_RF_USB
    bool host = BSP_workMode == RF_ROLE_HOST;

    if (host && !RF_BridgePaired()) {
        /* Role/discovery ownership stays with RF; LEDs only report the link.
         * A bound Host waiting for its recorded peer gets a slow single pulse
         * instead of the dark "no binding" state. */
        if (!MODE_mailbox.binding.bound) {
            BSP_ledSample = UINT32_MAX;
            R32_PA_CLR    = PIN_LED1 | PIN_LED2;
            return;
        }
        uint32_t now = BSP_ms;

        if (BSP_ledSample == UINT32_MAX) {
            BSP_ledSample = now;
        }
        uint32_t phase = (now - BSP_ledSample) % 1000U;
        uint32_t on    = phase < 100U ? PIN_LED1 : 0;

        R32_PA_SET = on;
        R32_PA_CLR = (PIN_LED1 | PIN_LED2) & ~on;
        return;
    }
    if (host || (BSP_workMode == RF_ROLE_TARGET && !RF_BridgePaired())) {
        /* Reuse the wired LED sample word as this role's pattern origin.
         * Unsigned elapsed time preserves the cycle across BSP_ms wrap. */
        uint32_t now    = BSP_ms;
        uint32_t period = host ? 2000U : 1000U;
        uint32_t phase  = now - BSP_ledSample;

        if (BSP_ledSample == UINT32_MAX) {
            BSP_ledSample = now;
            phase         = 0;
        } else if (phase >= period) {
            phase         %= period;
            BSP_ledSample  = now - phase;
        }
        uint32_t on =
            host ? (phase < 200U ? PIN_LED1 | PIN_LED2 : 0) : (phase < 500U ? PIN_LED1 : PIN_LED2);
        R32_PA_SET = on;
        R32_PA_CLR = (PIN_LED1 | PIN_LED2) & ~on;
        return;
    }

    /* A later disconnect starts again with CONNECT lit. HostStatus and
     * activity state remain intact while the discovery pattern owns GPIO. */
    BSP_ledSample = UINT32_MAX;
#endif

    for (unsigned i = 0; i < 2; i++) {
        /* A stale LED sample is harmless; do not block the UART FIFO to
         * expire an activity indication on every foreground pass. */
        led_state_t snapshot = BSP_leds[i];
        bool        on       = LED_IsOn(&snapshot, BSP_ms);
        uint32_t    pin      = i ? PIN_LED2 : PIN_LED1;

        if (on) {
            R32_PA_SET = pin;
        } else {
            R32_PA_CLR = pin;
        }
    }
}

void BSP_Poll(void) {
#if APP_COMBINED_TARGET
    if (BSP_ms - MODE_targetLinkCheckedAt >= 100U) {
        MODE_targetLinkCheckedAt = BSP_ms;
        if (MODE_AppModeTargetLinkPoll(BSP_ms, RF_BridgePaired())) {
            app_mode_action_t action;

            MODE_AppModeTakeAction(&action);
            MODE_ImageRequest(action.target);
        }
    }
#endif

    /* Wired DAP needs no repeated GPIO writes between millisecond ticks.
     * Status/activity changes invalidate the sample, including an IRQ during
     * LED_Poll. Commit the sample first so that such a change remains due.
     * Keep RF roles on their already qualified foreground policy. */

#if APP_COMBINED_TARGET
    if (MODE_peerSleeping) {
        R32_PA_CLR = PIN_LED1 | PIN_LED2;
        return;
    }
#endif

    if (APP_HAS_DAP && !APP_HAS_RF) {
        uint32_t now = BSP_ms;

        if (now != BSP_ledSample) {
            BSP_ledSample = now;
            LED_Poll();
        }
    } else {
        LED_Poll();
    }
}

void BSP_Fault(void) {
    __asm__ volatile("csrci 0x800,8" ::: "memory");
    BSP_ReleaseTarget();
    while (1) {
        R32_PA_SET = PIN_LED1 | PIN_LED2;
    }
}

__INTERRUPT __HIGH_CODE void HardFault_Handler(void) {
    BSP_Fault();
}
