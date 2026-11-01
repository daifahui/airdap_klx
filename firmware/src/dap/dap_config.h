/* SPDX-License-Identifier: Apache-2.0 */

#ifndef DAP_CONFIG_H
#define DAP_CONFIG_H

#include "app_config.h"
#include "usb/usb_protocol.h"

#ifndef __ASSEMBLER__
#include <stdint.h>
#include <string.h>
#include "ch572/attributes.h"
#endif

/* CMSIS-DAP configuration is defined only in this file. */

#define DAP_ATOMIC_COMMANDS  1
#define DAP_TIMESTAMP_ENABLE 0

#if ((DAP_ATOMIC_COMMANDS != 0 && DAP_ATOMIC_COMMANDS != 1) ||                                     \
     (DAP_TIMESTAMP_ENABLE != 0 && DAP_TIMESTAMP_ENABLE != 1))
#error "DAP Atomic Commands and Timestamp switches must be 0 or 1"
#endif

#define DAP_PACKET_COUNT 8U

#if (DAP_PACKET_COUNT < 2 || DAP_PACKET_COUNT > 128 || DAP_PACKET_COUNT & (DAP_PACKET_COUNT - 1))
#error "DAP transaction count must be a power of two, >=2 and <=128"
#endif

#define DAP_PACKET_SIZE 64U

#if DAP_PACKET_SIZE != USB_FS_PACKET_SIZE
#error "DAP_PACKET_SIZE must equal USB_FS_PACKET_SIZE"
#endif

/* Independent response spare and USB OUT DMA bank. */
#define DAP_POOL_BUFFERS (DAP_PACKET_COUNT + 2U)
#define DAP_JTAG_DEV_CNT 8U

#define SWJ_OVERLAY_LIMIT 1536U
#define SWD_SPI_THRESHOLD 1000000U
#define SWJ_MAX_SPI_HZ    25000000U

#if (SWD_SPI_THRESHOLD < 1 || SWJ_MAX_SPI_HZ < 1 || SWJ_MAX_SPI_HZ > (CPU_CLOCK / 2))
#error "SPI threshold and maximum frequency must be valid positive clock rates"
#endif

/* Clock-only fast path: 4 cycles/half; TX/RX/JTAG are slower and are bounded
 * separately by the calibrated GPIO timing model. Tested at 100 MHz HCLK. */
#define SWJ_MAX_GPIO_HZ (CPU_CLOCK / 8U)

#if (SWJ_MAX_GPIO_HZ < 1 || SWJ_MAX_GPIO_HZ > (CPU_CLOCK / 2))
#error "GPIO maximum frequency must be in 1..CPU_CLOCK/2"
#endif

/* Optional additional phase padding, beyond the measured cycle budgets. */
#define GPIO_LOW_EXTRA_NOPS  0
#define GPIO_HIGH_EXTRA_NOPS 0

#if (GPIO_LOW_EXTRA_NOPS < 0 || GPIO_HIGH_EXTRA_NOPS < 0)
#error "GPIO phase compensation cannot be negative"
#endif

/* Wireless DAP scheduling and placement use the same packet configuration. */
#define RF_DAP_POLL_US (APP_COMBINED_HOST ? 200U : 0U)

#if RF_DAP_POLL_US > 1000
#error "RF DAP active polling must be 0..1000 microseconds"
#endif

#define RF_DAP_COMMAND_BATCH (APP_COMBINED_TARGET ? 4 : 1)

#if (RF_DAP_COMMAND_BATCH < 1 || RF_DAP_COMMAND_BATCH > DAP_PACKET_COUNT)
#error "RF_DAP_COMMAND_BATCH must fit the existing DAP queue"
#endif

#ifndef __ASSEMBLER__
#define __STATIC_INLINE       static inline
#define DAP_SWD               1
#define DAP_JTAG              1
#define DAP_DEFAULT_PORT      1U
#define DAP_DEFAULT_SWJ_CLOCK 100000U
#define IO_PORT_WRITE_CYCLES  2U
#define DELAY_FAST_CYCLES     2U
#define DELAY_SLOW_CYCLES     3U

#if DAP_TIMESTAMP_ENABLE
#define TIMESTAMP_CLOCK 1000000U
#define TIMESTAMP_GET() BSP_TimeStamp()
#else
#define TIMESTAMP_CLOCK 0U
#endif

#define DAP_UART                0
#define DAP_UART_USB_COM_PORT   1
#define DAP_UART_RX_BUFFER_SIZE UART_RX_SIZE
#define DAP_UART_TX_BUFFER_SIZE UART_TX_SIZE
#define TARGET_FIXED            0

#define PIN_DELAY_SLOW(n)                                                                          \
    do {                                                                                           \
        volatile uint32_t _d = (n);                                                                \
        while (_d--) {                                                                             \
            __asm__ volatile("nop");                                                               \
        }                                                                                          \
    } while (0)

#define PIN_DELAY_FAST() __asm__ volatile("nop")

void SWJ_ResetOutput(uint32_t bit);

#if DAP_TIMESTAMP_ENABLE
uint32_t BSP_TimeStamp(void);
#endif
#endif /* !__ASSEMBLER__ */
#endif
