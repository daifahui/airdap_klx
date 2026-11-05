#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "flash_layout.h"

/* CH572Q USB Dongle: RF HOST only; full AirDAP remains the default. */

#ifndef BSP_USB_DONGLE
#define BSP_USB_DONGLE 0
#endif

#if (BSP_USB_DONGLE != 0 && BSP_USB_DONGLE != 1)
#error "BSP_USB_DONGLE must be 0 or 1"
#endif

/* Build selectors only. Product parameters below are owned by this file.
 * CMSIS-DAP parameters are defined only in dap_config.h. */

/* Values match the USB mode request parameters in usb/usb_requests.h:
 * USB_MODE_AUTO=0 (composite), USB_MODE_RF_HOST=2, USB_MODE_RF_TARGET=3. */
#define APP_MODE_COMPOSITE 0
#define APP_MODE_RF_HOST   2
#define APP_MODE_RF_TARGET 3

#ifndef APP_MODE
#if BSP_USB_DONGLE
#define APP_MODE APP_MODE_RF_HOST
#else
#define APP_MODE APP_MODE_COMPOSITE
#endif
#endif

/* LOCAL/HOST enable runtime DFU; TARGET has no application USB stack. */
#define USB_DFU_ENABLE (!APP_COMBINED_TARGET)
#define APP_DFU_BUILD  USB_DFU_ENABLE

/* Select a complete firmware image; unrelated drivers are excluded at link time. */
#define APP_COMBINED_TARGET (APP_MODE == APP_MODE_RF_TARGET)
#define APP_COMBINED_HOST   (APP_MODE == APP_MODE_RF_HOST)
#define APP_HAS_RF_DAP      (RF_DAP_TARGET || APP_COMBINED_HOST)
#define RF_FIXED_ROLE       (APP_COMBINED_TARGET ? 2 : APP_COMBINED_HOST ? 1 : 0)

#if (RF_FIXED_ROLE < 0 || RF_FIXED_ROLE > 2)
#error "RF_FIXED_ROLE must be 0 (runtime), 1 (HOST), or 2 (TARGET)"
#endif

/* TARGET uses RF for application traffic and does not link the USB stack. */
#define RF_TARGET_NO_USB APP_COMBINED_TARGET

/* TARGET executes DAP without an application USB stack. */
#define RF_DAP_TARGET APP_COMBINED_TARGET

#if (RF_DAP_TARGET != 0 && RF_DAP_TARGET != 1)
#error "RF_DAP_TARGET must be 0 or 1"
#endif

/* Fixed AirDAP wiring; the Dongle has no target-power output. */
#define BSP_HAS_TARGET_POWER (!BSP_USB_DONGLE)
#define BSP_TDO_GPIO         11

#if (RF_DAP_TARGET && !APP_COMBINED_TARGET)
#error "RF DAP target requires the TARGET bank"
#endif

#if (APP_MODE != APP_MODE_COMPOSITE && !APP_COMBINED_HOST && !APP_COMBINED_TARGET)
#error "Role images require complete LOCAL, HOST or TARGET working sets"
#endif

#if (BSP_USB_DONGLE && (!APP_COMBINED_HOST || !USB_DFU_ENABLE))
#error "USB Dongle requires the banked RF HOST with DFU runtime enabled"
#endif

#define APP_HAS_DAP (APP_MODE == APP_MODE_COMPOSITE || RF_DAP_TARGET)
#define USB_HAS_DAP ((APP_HAS_DAP && !RF_DAP_TARGET) || APP_COMBINED_HOST)

/* The qualified parity-fused fast SPI transfer is the production backend. */
#define APP_HAS_RF_UART (APP_COMBINED_TARGET || APP_COMBINED_HOST)

/* RF typed-stream routing: do not copy a DAP record into the UART parser. */

/* RF placement choices are configured here for each complete role. */

/* HOST radio submission and polling execute from SRAM. */
#define RF_UART_RADIO_TX_RAM APP_COMBINED_HOST
#define RF_POLL_RAM          APP_COMBINED_HOST

#if (RF_POLL_RAM != 0 && RF_POLL_RAM != 1)
#error "RF_POLL_RAM must be 0 or 1"
#endif

#if (RF_UART_RADIO_TX_RAM != 0 && RF_UART_RADIO_TX_RAM != 1)
#error "RF_UART_RADIO_TX_RAM must be 0 or 1"
#endif

#if (RF_UART_RADIO_TX_RAM && !APP_HAS_RF_UART)
#error "RF radio TX placement requires the RF UART image"
#endif

/* Qualified fixed PHY for C RF role banks; LOCAL has no RF core. */

/* Qualified roles may lend an idle RX bank to the half-duplex transmitter. */
#define RF_DMA_SHARED_TX APP_COMBINED_TARGET

#if (RF_DMA_SHARED_TX != 0 && RF_DMA_SHARED_TX != 1)
#error "RF_DMA_SHARED_TX must be 0 or 1"
#endif

#define APP_HAS_RF_USB APP_HAS_RF_UART
#define APP_HAS_RF     APP_HAS_RF_USB

#if (APP_HAS_RF_UART && !APP_COMBINED_TARGET)
#define MODE_LOCAL_SECTION(name) ".bss." name
#define MODE_RF_SECTION(name)    ".rf_uart." name
#else
#define MODE_LOCAL_SECTION(name) ".bss." name
#define MODE_RF_SECTION(name)    ".bss." name
#endif

#define APP_HAS_USB (!RF_TARGET_NO_USB)

#if (RF_TARGET_NO_USB && !RF_DAP_TARGET)
#error "USB-free profile requires a fixed RF TARGET"
#endif

/* Role-image USB teardown before loading another bank. */
#define MODE_USB_DETACH_MS 100U

/* Composite activity briefly inverts the DAP status on the same two LEDs. */
#define BSP_LED_PULSE_MS 30U

#define CPU_CLOCK 100000000U
#define FREQ_SYS  CPU_CLOCK

/* Qualified LOCAL UART ownership; no RF bank owns these packet buffers. */
#define UART_RX_PACKET_RING      (APP_MODE == APP_MODE_COMPOSITE)
#define UART_CDC_IN_PACKET_LIMIT 64
#define UART_RX_SIZE             (APP_COMBINED_TARGET ? 1024U : APP_COMBINED_HOST ? 512U : 256U)
#define UART_TX_SIZE             256U

#if (UART_RX_SIZE < 64 || UART_RX_SIZE & (UART_RX_SIZE - 1))
#error "UART_RX_SIZE must be a power of two, >=64"
#endif

#if (UART_TX_SIZE < 64 || UART_TX_SIZE & (UART_TX_SIZE - 1))
#error "UART_TX_SIZE must be a power of two, >=64"
#endif

#define UART_RX_TRIGGER_BYTES 4U

#if UART_RX_TRIGGER_BYTES == 1
#define UART_RX_TRIGGER_BITS (0U << 6)
#elif UART_RX_TRIGGER_BYTES == 2
#define UART_RX_TRIGGER_BITS (1U << 6)
#elif UART_RX_TRIGGER_BYTES == 4
#define UART_RX_TRIGGER_BITS (2U << 6)
#elif UART_RX_TRIGGER_BYTES == 7
#define UART_RX_TRIGGER_BITS (3U << 6)
#else
#error "UART_RX_TRIGGER_BYTES must be 1, 2, 4 or 7"
#endif

/* Full packets leave immediately; bound the delay of a partial packet. */
#define CDC_RX_FLUSH_MS 1U

#if CDC_RX_FLUSH_MS > 10
#error "CDC_RX_FLUSH_MS must be in 0..10 ms"
#endif

#define UART_MAX_BAUD 1500000U

#if (UART_MAX_BAUD < 300 || UART_MAX_BAUD > (CPU_CLOCK / 8U) || UART_MAX_BAUD > 1500000U)
#error "UART_MAX_BAUD must be in 300..min(CPU_CLOCK/8, 1500000)"
#endif

#define STACK_SIZE  768U
#define BSP_HSE_CAP HSECap_18p
#define USB_VID     0x1209U
#define USB_PID     0x572DU

#define USB_MAX_POWER_MA 100U

#if (USB_MAX_POWER_MA < 2 || USB_MAX_POWER_MA > 500 || USB_MAX_POWER_MA % 2)
#error "USB_MAX_POWER_MA must be an even value from 2 through 500"
#endif

#define RF_ROLE_HOST       1
#define RF_ROLE_TARGET     2
#define RF_DEFAULT_CHANNEL 36U /* RF encoded channel, 2478 MHz. */

#if RF_DEFAULT_CHANNEL > 39
#error "RF_DEFAULT_CHANNEL must be 0..39"
#endif

#define RF_PAIR_ACCESS_ADDRESS 0x71764129U
#define RF_DMA_SIZE            264U

/* The complete product has one fixed USB/RF selection window. */
#define APP_BOOT_USB_WINDOW_MS 2000U
#define RF_USB_QUEUE_SIZE      512U

/* Receive capacity can be adjusted here independently of the TX ring. */
#define RF_USB_RX_QUEUE_SIZE RF_USB_QUEUE_SIZE

#if (RF_USB_RX_QUEUE_SIZE < RF_USB_QUEUE_SIZE || RF_USB_RX_QUEUE_SIZE > 32768U ||                  \
     RF_USB_RX_QUEUE_SIZE & (RF_USB_RX_QUEUE_SIZE - 1U))
#error "RF RX queue must be a power of two in512..32768 bytes"
#endif

/* Factory RF Basic TX power encoding; use only supported SDK values. */
#define RF_TX_POWER_CODE 0x12U /* LL_TX_PWR_0_DBM */

/* Both RF role banks negotiate channels; LOCAL has no radio traffic. */

/* Current receive credit bounds each TARGET reply turn to two frames. */
#define RF_REPLY_BURST   2U
#define RF_USB_DATA_SIZE 229U

#if (RF_USB_DATA_SIZE < 1 || RF_USB_DATA_SIZE > 229U)
#error "RF_USB_DATA_SIZE must fit the v6 251-byte frame minus its 22-byte header"
#endif

#define RF_USB_IDLE_MS      1U
#define RF_TX_PREFILL       APP_COMBINED_HOST
#define RF_PREFILL_EXTEND   APP_COMBINED_HOST
#define RF_HOST_FRAME_LIMIT RF_USB_DATA_SIZE

#if (RF_HOST_FRAME_LIMIT < 32 || RF_HOST_FRAME_LIMIT > RF_USB_DATA_SIZE)
#error "HOST transmit limit must fit the unchanged RF receive payload capacity"
#endif

#define RF_SPACE_RAM      APP_COMBINED_HOST
#define RF_RETRY_DELAY_US (APP_COMBINED_HOST ? 0U : 1000U)

#if RF_RETRY_DELAY_US > 1000
#error "RF retry delay must be in 0..1000 us"
#endif

#if (RF_TX_PREFILL && (!APP_COMBINED_HOST || RF_DMA_SHARED_TX))
#error "RF prefill requires a fixed-policy HOST with its own TX DMA bank"
#endif

/* RF_Basic uses 40 us. RF_Uart initiates with 60 us and its Dongle replies
 * with 20 us. First TX/channel changes still get >=80 us in the bridge. */
#define RF_USB_TX_WAIT_US (APP_COMBINED_HOST ? 60U : APP_COMBINED_TARGET ? 20U : 40U)

#if (RF_USB_TX_WAIT_US < 20 || RF_USB_TX_WAIT_US > 127)
#error "RF_USB_TX_WAIT_US must be in 20..127 us"
#endif

#define RF_USB_TIMEOUT_MS 3U
#endif
