#ifndef RF_MESSAGE_H
#define RF_MESSAGE_H

#include "uart/uart_io.h"
#include "usb/usb_protocol.h"
#include <stddef.h>

/* Optional application framing over the reliable RF byte stream. No second
 * payload buffer or software CRC is required. Activate only at both ends. */

/* Byte offsets in a typed record; the body follows HEADER_SIZE. */
#define RF_MESSAGE_TYPE_OFFSET        0U
#define RF_MESSAGE_CHANNEL_OFFSET     1U
#define RF_MESSAGE_FLAGS_OFFSET       2U
#define RF_MESSAGE_VERSION_OFFSET     3U
#define RF_MESSAGE_ID_OFFSET          4U
#define RF_MESSAGE_LENGTH_OFFSET      8U
#define RF_MESSAGE_STATUS_OFFSET      10U
#define RF_MESSAGE_CHANNEL_UART       0U
#define RF_MESSAGE_CHANNEL_DAP        1U
#define RF_MESSAGE_CHANNEL_STATUS     2U
#define RF_MESSAGE_UART_MAX_DATA      192U
#define RF_MESSAGE_CONTROL_SIZE       4U
#define RF_MESSAGE_STATUS_SIZE        12U
#define RF_MESSAGE_LINE_FENCE_OFFSET  0U
#define RF_MESSAGE_LINE_CODING_OFFSET 4U
#define RF_MESSAGE_LINE_SIZE          (RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_LINE_CODING_SIZE)
#define RF_MESSAGE_HEADER_SIZE        12U
#define RF_MESSAGE_VERSION            2U
#define RF_MESSAGE_FLAG_OVERRUN       0x01U
#define RF_MESSAGE_UART_DATA          1
#define RF_MESSAGE_UART_LINE          2
#define RF_MESSAGE_UART_LINE_RESULT   3
#define RF_MESSAGE_UART_CONTROL       4
#define RF_MESSAGE_DAP_COMMAND        5
#define RF_MESSAGE_DAP_RESPONSE       6
#define RF_MESSAGE_DAP_ABORT          7
#define RF_MESSAGE_STATUS             8

enum rf_message_status {
    RF_MESSAGE_OK,
    RF_MESSAGE_INVALID,
    RF_MESSAGE_BUSY,
    RF_MESSAGE_REPLAY
};

typedef struct {
    uint32_t id;
    uint16_t length, status;
    uint8_t  type, channel, flags;
} rf_message_t;

typedef struct {
    uint8_t  header[RF_MESSAGE_HEADER_SIZE];
    uint16_t remaining;
    uint8_t  used, fault;
} rf_message_parser_t;

typedef struct {
    bool (*begin)(void *context, const rf_message_t *message);

    void (*data)(void *context, const uint8_t *data, unsigned length);
    void (*end)(void *context);
    void *context;
} rf_message_sink_t;

bool RF_MessageHeader(uint8_t out[RF_MESSAGE_HEADER_SIZE], uint8_t type, uint32_t id,
                      unsigned length, unsigned status);

bool RF_MessageHeaderFlags(uint8_t out[RF_MESSAGE_HEADER_SIZE], uint8_t type, uint32_t id,
                           unsigned length, unsigned status, unsigned flags);

/* Internal cache state, not a wire status: a timed-out apply must replay BUSY
 * for the same id/payload, rather than claim the old line was newly applied. */
#define RF_UART_CACHE_BUSY 2U

/* One outstanding line-setting request per UART. Offsets refer to UART bytes
 * already accepted in this session, not USB/RF packet positions. */
typedef struct {
    uart_line_t current, proposed;
    uint32_t    id, fence, actual;
    uint8_t     pending, cached; /* cached: 0 none, 1 applied, 2 rejected BUSY. */
} rf_uart_barrier_t;

void RF_UartBarrierInit(rf_uart_barrier_t *barrier, const uart_line_t *line);

/* Payload: little-endian fence offset (4) followed by CDC line coding (7). */

enum rf_message_status RF_UartBarrierRequest(rf_uart_barrier_t *barrier, uint32_t id,
                                             const uint8_t payload[RF_MESSAGE_LINE_SIZE],
                                             uint32_t      accepted);

/* Apply only after earlier bytes AND the physical UART shift register drain.
 * If hardware refuses, preserve the old line and let the caller retry/report. */

bool RF_UartBarrierPoll(rf_uart_barrier_t *barrier, uint32_t transmitted, bool shiftEmpty,
                        bool (*apply)(void *context, const uart_line_t *line), void *context);
#endif
