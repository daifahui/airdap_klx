#include "dap/dap_config.h"

/* Bounded typed messages for a future wireless UART/DAP application. */
#include "rf/rf_message.h"
#include "app_config.h"
#include <string.h>

static uint32_t RF_Read32(const uint8_t *p) {
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint8_t RF_ChannelFor(unsigned type) {
    return type <= RF_MESSAGE_UART_CONTROL ? RF_MESSAGE_CHANNEL_UART
           : type <= RF_MESSAGE_DAP_ABORT  ? RF_MESSAGE_CHANNEL_DAP
                                           : RF_MESSAGE_CHANNEL_STATUS;
}

static bool RF_Valid(unsigned type, unsigned length, unsigned status) {
    if (status > RF_MESSAGE_REPLAY ||
        (status && type != RF_MESSAGE_UART_LINE_RESULT && type != RF_MESSAGE_STATUS)) {
        return false;
    }
    switch (type) {
        case RF_MESSAGE_UART_DATA:
            return length >= 1 && length <= RF_MESSAGE_UART_MAX_DATA;
        case RF_MESSAGE_UART_LINE:
        case RF_MESSAGE_UART_LINE_RESULT:
            return length == RF_MESSAGE_LINE_SIZE;
        case RF_MESSAGE_UART_CONTROL:
            return length == RF_MESSAGE_CONTROL_SIZE;
        case RF_MESSAGE_DAP_COMMAND:
        case RF_MESSAGE_DAP_RESPONSE:
            return length >= 1 && length <= DAP_PACKET_SIZE;
        case RF_MESSAGE_DAP_ABORT:
            return length == 0;
        case RF_MESSAGE_STATUS:
            return length == RF_MESSAGE_STATUS_SIZE;
        default:
            return false;
    }
}

bool RF_MessageHeader(uint8_t out[RF_MESSAGE_HEADER_SIZE], uint8_t type, uint32_t id,
                      unsigned length, unsigned status) {
    return RF_MessageHeaderFlags(out, type, id, length, status, 0);
}

bool RF_MessageHeaderFlags(uint8_t out[RF_MESSAGE_HEADER_SIZE], uint8_t type, uint32_t id,
                           unsigned length, unsigned status, unsigned flags) {
    if (!RF_Valid(type, length, status) || (flags & ~RF_MESSAGE_FLAG_OVERRUN) ||
        (flags && type != RF_MESSAGE_UART_DATA)) {
        return false;
    }
    out[RF_MESSAGE_TYPE_OFFSET]    = type;
    out[RF_MESSAGE_CHANNEL_OFFSET] = RF_ChannelFor(type);
    out[RF_MESSAGE_FLAGS_OFFSET]   = flags;
    out[RF_MESSAGE_VERSION_OFFSET] = RF_MESSAGE_VERSION;
    for (unsigned i = 0; i < 4; i++) {
        out[RF_MESSAGE_ID_OFFSET + i] = id >> (8 * i);
    }
    out[RF_MESSAGE_LENGTH_OFFSET]     = length;
    out[RF_MESSAGE_LENGTH_OFFSET + 1] = length >> 8;
    out[RF_MESSAGE_STATUS_OFFSET]     = status;
    out[RF_MESSAGE_STATUS_OFFSET + 1] = status >> 8;
    return true;
}

void RF_UartBarrierInit(rf_uart_barrier_t *b, const uart_line_t *line) {
    memset(b, 0, sizeof(*b));
    b->current = *line;
}

enum rf_message_status RF_UartBarrierRequest(rf_uart_barrier_t *b, uint32_t id,
                                             const uint8_t payload[RF_MESSAGE_LINE_SIZE],
                                             uint32_t      accepted) {
    uart_line_t line  = {RF_Read32(payload + RF_MESSAGE_LINE_CODING_OFFSET),
                         payload[RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_STOP_OFFSET],
                         payload[RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_PARITY_OFFSET],
                         payload[RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_DATA_BITS_OFFSET]};
    uint32_t    fence = RF_Read32(payload + RF_MESSAGE_LINE_FENCE_OFFSET);

    if (!id || !UART_LineValid(&line, NULL, NULL)) {
        return RF_MESSAGE_INVALID;
    }
    if ((b->pending || b->cached) && id == b->id) {
        bool same = fence == b->fence && line.baud == b->proposed.baud &&
                    line.bits == b->proposed.bits && line.stop == b->proposed.stop &&
                    line.parity == b->proposed.parity;
        return !same                                             ? RF_MESSAGE_INVALID
               : (b->pending || b->cached == RF_UART_CACHE_BUSY) ? RF_MESSAGE_BUSY
                                                                 : RF_MESSAGE_REPLAY;
    }
    if (b->pending) {
        return RF_MESSAGE_BUSY;
    }
    if (fence != accepted || (b->cached && (int32_t)(id - b->id) <= 0)) {
        return RF_MESSAGE_INVALID;
    }
    b->proposed = line;
    b->fence    = fence;
    b->id       = id;
    b->pending  = 1;
    b->cached   = 0;
    return RF_MESSAGE_OK;
}

bool RF_UartBarrierPoll(rf_uart_barrier_t *b, uint32_t transmitted, bool shiftEmpty,
                        bool (*apply)(void *context, const uart_line_t *line), void *context) {
    /* accepted/transmitted belong to the same bounded queue window. */
    if (!b->pending || (int32_t)(transmitted - b->fence) < 0 || !shiftEmpty ||
        !apply(context, &b->proposed)) {
        return false;
    }
    b->current = b->proposed;
    UART_LineValid(&b->current, NULL, &b->actual);
    b->pending = 0;
    b->cached  = 1;
    return true;
}
