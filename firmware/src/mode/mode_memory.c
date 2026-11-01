/* The caller stops bank-owned DMA/IRQs before replacing executable SRAM. */
#include "app_config.h"
#include "mode/mode_memory.h"
#include <stdint.h>
#include <string.h>

#if (APP_HAS_DAP)
extern uint8_t __ov_boot_load[], __ov_boot_start[], __ov_boot_end[];
#endif

#if (APP_HAS_RF_UART && !APP_COMBINED_TARGET)
extern uint8_t __rf_uart_boot_load[], __rf_uart_boot_start[], __rf_uart_boot_end[];
#endif

void MODE_MemoryBoot(void) {
#if (APP_HAS_RF_UART && !APP_COMBINED_TARGET)
    memcpy(__rf_uart_boot_start, __rf_uart_boot_load,
           (unsigned)(__rf_uart_boot_end - __rf_uart_boot_start));
    __asm__ volatile("fence.i" ::: "memory");
#elif (APP_HAS_DAP)
    /* UID access finishes before swj_init loads an execution overlay. */
    memcpy(__ov_boot_start, __ov_boot_load, (unsigned)(__ov_boot_end - __ov_boot_start));
    __asm__ volatile("fence.i" ::: "memory");
#endif
}
