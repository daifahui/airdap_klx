#ifndef BSP_H
#define BSP_H

#include <stdint.h>
#include "ch572/attributes.h"
#include <stdbool.h>
#include "dap/dap_config.h"
#include "mode/app_mode.h"

#define PIN_CLK           (1U << 5)
#define PIN_DIO           (1U << 7)
#define PIN_TDO           (1U << BSP_TDO_GPIO)
#define PIN_TX            (1U << 2)
#define PIN_RX            (1U << 3)
#define BSP_UART_TX_REMAP 1U
#define BSP_UART_RX_REMAP 1U
#define PIN_TDI           (1U << 10)
#define PIN_LDO_EN        (1U << 4)
#define BSP_RESET_GPIO    6
#define PIN_RESET         (1U << BSP_RESET_GPIO)
#define PIN_LED1          (1U << 9)
#define PIN_LED2          (1U << 8)
#define PIN_TARGET        (PIN_RX | PIN_TX | PIN_TDI | PIN_CLK | PIN_TDO | PIN_DIO | PIN_RESET)
#define BSP_UID_SIZE      8U
#define BSP_SERIAL_LENGTH (2U * BSP_UID_SIZE)

extern volatile uint32_t BSP_ms;
extern volatile uint8_t  BSP_stopRequested;
extern volatile uint8_t  BSP_workMode;
extern volatile uint8_t  BSP_workReady;
extern char              BSP_serial[BSP_SERIAL_LENGTH + 1U];
extern volatile uint32_t BSP_clockHz, BSP_clockEpoch;

/* Cumulative saturated CLK-write boundary uncertainty, in 1/2.4GHz units.
 * Assumes each bracketing counter cycle belongs to the old or new source. */
extern volatile uint32_t BSP_clockUncertaintyUnits;

void BSP_UsbDisconnect(void);

#if DAP_TIMESTAMP_ENABLE
/* Fixed 1 MHz domain across HCLK changes; read-only and leaves IRQs enabled. */

uint32_t BSP_TimeStamp(void);
#endif

void BSP_TimeInit(void);

void BSP_TimeTick(void);

/* HALT excludes stopped-clock time from the active monotonic timebase. */

void BSP_TimeSuspend(void);

void BSP_TimeResume(void);

uint32_t BSP_TimeUs(void);

/* Foreground reservation is mutually exclusive with the complete DAP
 * command, not just its Transfer/TransferBlock portion. Pending UART work
 * gets admission priority over the next queued DAP command. */

void BSP_ClockQueue(bool pending);

bool BSP_ClockAcquire(void);

bool BSP_ClockApply(uint32_t hz);

/* An acquired owner may restore its old plan even after an ISR requests
 * stop. No new peripheral work is admitted by this cancellation path. */

bool BSP_ClockRestore(uint32_t hz);

void BSP_ClockRelease(void);

bool BSP_DapEnter(void);

void BSP_DapLeave(void);

uint8_t BSP_ClockState(void);

void BSP_Init(void);

/* Startup-only SDK 64-bit UID; destination has 8 bytes and four-byte alignment. */

void BSP_ReadIdentity(uint8_t *data);

void BSP_Poll(void);

void BSP_ReleaseTarget(void);

void BSP_LedStatus(unsigned index, uint8_t on);

void BSP_LedActivity(unsigned index);

static __ALWAYS_INLINE uint32_t BSP_IrqSave(void) {
    uint32_t state;

    /* Official WCH startup runs the application in user mode. */
    __asm__ volatile("csrrc %0,0x800,%1" : "=r"(state) : "r"(8) : "memory");
    return state;
}

static __ALWAYS_INLINE void BSP_IrqRestore(uint32_t state) {
    if (state & 8) {
        __asm__ volatile("csrs 0x800,%0" ::"r"(8) : "memory");
    }
}

void BSP_DelayUs(uint32_t us);

void BSP_Fault(void) __NO_RETURN;

bool BSP_LocalSelected(void);

/* MCU-owned CE: startup drives high, off drives low, no pull resistors.
 * This reports the commanded output state, not the target rail voltage. */

void BSP_TargetPower(bool on);

bool BSP_TargetPowerEnabled(void);

uint64_t BSP_DiscoveryNonce(void);
#endif
