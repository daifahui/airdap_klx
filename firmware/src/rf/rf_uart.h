#ifndef RF_UART_H
#define RF_UART_H

#include "uart/uart_io.h"

#define RF_UART_DATA_SIZE 64U

#if APP_COMBINED_HOST
extern uint8_t RF_uartRemaining;
#endif

#define RF_UART_STATUS_MAGIC         0x46524155U /* "UARF" */
#define RF_UART_FAULT_COMMIT         1
#define RF_UART_FAULT_ENCODE         2
#define RF_UART_FAULT_USB_RETIRE     3
#define RF_UART_FAULT_RESET_ROLE     4
#define RF_UART_FAULT_INIT_ROLE      5
#define RF_UART_FAULT_BAD_ROLE       6
#define RF_UART_FAULT_HEADER         7
#define RF_UART_FAULT_TARGET_FLAGS   8
#define RF_UART_FAULT_TARGET_ORDER   9
#define RF_UART_FAULT_TARGET_CONTROL 10
#define RF_UART_FAULT_TARGET_TYPE    11
#define RF_UART_FAULT_HOST_ORDER     12
#define RF_UART_FAULT_LINE_RESULT    13
#define RF_UART_FAULT_CONTROL_RESULT 14
#define RF_UART_FAULT_STATUS_KIND    15
#define RF_UART_FAULT_HOST_TYPE      16
#define RF_UART_FAULT_CONFIG_TIMEOUT 17

/* Physical drain/reliable confirmation may include min-baud queued bytes.
 * Once TARGET reaches the byte/shift fence, hardware apply gets only 5 s. */
#define RF_UART_CONFIG_TIMEOUT_MS 60000U
#define RF_UART_APPLY_TIMEOUT_MS  5000U

/* Reverse DATA: full 64-byte packets immediately, partial packets within
 * CDC_RX_FLUSH_MS of the first byte. Statistics rotate one group per 20 ms. */

/* Typed, ordered UART service over rf_bridge. The USB/RF owner must cancel
 * old USB IN ownership before reset and reset both RF queues in the same
 * session transition. Host reset keeps confirmed CDC settings and DTR;
 * target reset discards the old physical UART session. Host never owns pins. */

void RF_UartInit(uint8_t role);

void RF_UartReset(void);

void RF_UartPoll(void);

/* EP0 owns a class request from SETUP/data acceptance through status ACK.
 * Hold prevents automatic sync from taking that request's metadata slot. */

void RF_UartHoldControls(bool hold);

/* A request accepted by USB but no longer commit-able is observable and
 * fails closed until the next session, rather than resetting the MCU. */

void RF_UartCommitFailed(void);

bool RF_UartHostReady(void);

bool RF_UartSubmit(const uint8_t *data, unsigned length);

/* Read peeks the RF RX queue. Retire only the bytes acknowledged by USB IN. */

unsigned RF_UartHostRead(uint8_t *data, unsigned limit);

void RF_UartHostRetire(unsigned length);

bool RF_UartLineValid(const uart_line_t *line);

/* Pure preflight for the USB status ACK. One LINE and one CONTROL may be
 * accepted together; metadata queues preserve acceptance order without a
 * second payload queue. A second outstanding request of the same kind waits. */

bool RF_UartLineReady(const uart_line_t *line);

bool RF_UartLineSubmit(const uart_line_t *line);

bool RF_UartControlReady(void);

bool RF_UartControlSubmit(uint8_t dtr, uint16_t breakMs);

/* GET_LINE_CODING reports the last target-confirmed, realizable baud/frame. */

const uart_line_t *RF_UartHostLine(void);

/* HOST: number of distinct received DATA messages carrying overrun, modulo
 * 2^32, and sticky presence for this session. Queries never acknowledge or
 * clear an event. USB notification ownership keeps its own generation
 * snapshot; session/role reset clears both values and must cancel old IN. */

uint32_t RF_UartOverrunGeneration(void);
#endif
