/* Dedicated wireless UART application; payload storage belongs to RF/UART. */
#include "rf/rf_uart.h"
#include "rf/rf_bridge.h"
#include "rf/rf_message.h"
#include "bsp/board.h"
#include "usb/usb_device.h"
#include "ch572/ch572.h"

#if APP_COMBINED_TARGET
#include "rf/rf_dap.h"
#endif

#include <string.h>

#define STATUS_CONTROL     1
#define STATUS_STATS_FIRST 2
#define STATUS_STATS_LAST  4

typedef struct {
    rf_uart_barrier_t line;
    uart_line_t       requested;
    uart_line_t       actual_line;
    uint32_t          stats[6];
    uint32_t          tx_id, last_rx_id, tx_base, tx_bytes, rx_bytes, pressure, errors;
    uint32_t          result_id, result_fence;
    uint32_t          control_id, control_word, confirmed_control, control_fence, break_until;

    /* TARGET: telemetry while idle, config phase start while pending/sync.
     * HOST: config watchdog start; stats_group is its armed bit (no telemetry). */
    uint32_t stats_at;
    uint32_t loss_observed, overrun_generation;
    uint16_t result_status, last_status;
    uint8_t  initialized, active, fault, sync;
    uint8_t  result_pending, control_pending, control_cached, stats_group;
    uint8_t  controls_hold, overrun_flags; /* target pending / host sticky */
} rf_uart_state_t;

static rf_uart_state_t RF_state;

#if APP_COMBINED_HOST
uint8_t RF_uartRemaining;
#else
static uint8_t RF_uartRemaining;
#endif

#if RF_FIXED_ROLE
static const uint8_t RF_role = RF_FIXED_ROLE;
#else
static uint8_t RF_role;
#endif

_Static_assert(sizeof(RF_state) <= 200, "RF UART metadata budget");

static uint32_t RF_Read32(const uint8_t *p) {
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void RF_Write32(uint8_t *p, uint32_t value) {
    for (unsigned i = 0; i < 4; i++) {
        p[i] = value >> (i * 8);
    }
}

static void RF_LineEncode(uint8_t p[RF_MESSAGE_LINE_SIZE], uint32_t fence,
                          const uart_line_t *line) {
    RF_Write32(p, fence);
    RF_Write32(p + RF_MESSAGE_LINE_CODING_OFFSET, line->baud);
    p[RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_STOP_OFFSET]      = line->stop;
    p[RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_PARITY_OFFSET]    = line->parity;
    p[RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_DATA_BITS_OFFSET] = line->bits;
}

static bool RF_SameLine(const uart_line_t *a, const uart_line_t *b) {
    return a->baud == b->baud && a->stop == b->stop && a->parity == b->parity && a->bits == b->bits;
}

static void RF_Fault(unsigned reason, uint32_t context) {
    (void)reason;
    (void)context;
    ++RF_state.errors;
    RF_state.fault       = 1;
    RF_state.last_status = RF_MESSAGE_INVALID;
}

/* Do not reset RF queues, host_remaining or USB DMA ownership here. A sent
 * request without a result has unknown remote state and needs the owner's
 * existing session barrier before any new data/configuration is accepted. */
static void RF_ConfigTimeout(void) {
    ++RF_state.errors;
    RF_state.fault        = 1;
    RF_state.last_status  = RF_MESSAGE_BUSY;
    RF_state.line.pending = RF_state.control_pending = RF_state.result_pending = RF_state.sync = 0;
    if (RF_role == RF_ROLE_TARGET) {
        UART_ConfigureCancel();
    } else {
        RF_state.stats_group = 0;
    }
}

static void RF_HostWatchStart(void) {
    if (!RF_state.stats_group) {
        RF_state.stats_at    = BSP_ms;
        RF_state.stats_group = 1;
    }
}

static bool RF_ConfigExpired(uint32_t duration) {
    return (uint32_t)(BSP_ms - RF_state.stats_at) >= duration;
}

void RF_UartHoldControls(bool hold) {
    uint32_t irq = BSP_IrqSave();

    RF_state.controls_hold = hold;
    BSP_IrqRestore(irq);
}

void RF_UartCommitFailed(void) {
    uint32_t irq = BSP_IrqSave();

    RF_Fault(RF_UART_FAULT_COMMIT, 0);
    RF_state.last_status = RF_MESSAGE_BUSY;
    BSP_IrqRestore(irq);
}

static uint32_t RF_NextId(void) {
    uint32_t id = RF_state.tx_id + 1;

    return id ? id : 1;
}

/* The HOST USB owner and foreground producer are serialized. TARGET keeps
 * UART/RF IRQs live; only foreground consumes the reliable stream. Reserve
 * the whole typed record before publishing either span. */
static bool RF_SendFlags(unsigned type, uint32_t id, const uint8_t *payload, unsigned length,
                         unsigned status, unsigned flags) {
    uint8_t header[RF_MESSAGE_HEADER_SIZE];

    if (RF_BridgeSpace() < sizeof(header) + length) {
        ++RF_state.pressure;
        return false;
    }
    if (!RF_MessageHeaderFlags(header, type, id, length, status, flags)) {
        RF_Fault(RF_UART_FAULT_ENCODE, type);
        return false;
    }

    /* The shared publisher reserves the complete record before either copy.
     * A stop request can reject it without leaving a half-header in the ring. */
    if (!RF_BridgeSubmitPair(header, sizeof(header), payload, length)) {
        ++RF_state.pressure;
        return false;
    }
    return true;
}

static bool RF_Send(unsigned type, uint32_t id, const uint8_t *payload, unsigned length,
                    unsigned status) {
    return RF_SendFlags(type, id, payload, length, status, 0);
}

static bool RF_Breaking(void) {
    unsigned duration = RF_state.confirmed_control >> 16;

    if (!duration) {
        return false;
    }
    if (duration != UINT16_MAX && (int32_t)(BSP_ms - RF_state.break_until) >= 0) {
        return false;
    }
    return true;
}

static bool RF_HostAvailable(void) {
    return RF_role == RF_ROLE_HOST && RF_BridgeActive() && !RF_state.fault &&
           !RF_state.line.pending && !RF_state.control_pending;
}

bool RF_UartHostReady(void) {
    if (RF_role != RF_ROLE_HOST) {
        return false;
    }
    uint32_t irq   = BSP_IrqSave();
    bool     ready = RF_HostAvailable() && !RF_state.sync && (RF_state.confirmed_control & 1) &&
                 !RF_Breaking() && RF_BridgeSpace() >= RF_MESSAGE_HEADER_SIZE + RF_UART_DATA_SIZE;
    BSP_IrqRestore(irq);
    return ready;
}

bool RF_UartLineValid(const uart_line_t *line) {
    return line && UART_LineValid(line, NULL, NULL);
}

bool RF_UartLineReady(const uart_line_t *line) {
    uint32_t irq   = BSP_IrqSave();
    bool     ready = RF_UartLineValid(line) && RF_role == RF_ROLE_HOST && RF_BridgeActive() &&
                 !RF_state.fault && !RF_state.line.pending && !RF_Breaking() &&
                 !(RF_state.control_pending && (RF_state.control_word >> 16));
    BSP_IrqRestore(irq);
    return ready;
}

static bool RF_LineSubmit(const uart_line_t *line) {
    uint8_t  payload[RF_MESSAGE_LINE_SIZE];
    bool     queued = RF_state.line.pending == 2;
    uint32_t id     = queued ? RF_state.line.id : RF_NextId();

    RF_LineEncode(payload, RF_state.tx_bytes, line);
    if (!RF_Send(RF_MESSAGE_UART_LINE, id, payload, sizeof(payload), RF_MESSAGE_OK)) {
        return false;
    }
    if (!queued) {
        RF_state.tx_id = id;
    }
    RF_state.line.id       = id;
    RF_state.line.proposed = *line;
    RF_state.line.fence    = RF_state.tx_bytes;
    RF_state.line.pending  = 1;
    return true;
}

bool RF_UartLineSubmit(const uart_line_t *line) {
    uint32_t irq  = BSP_IrqSave();
    bool     sent = RF_UartLineReady(line);

    if (sent) {
        RF_HostWatchStart();
        RF_state.tx_id = RF_state.line.id = RF_NextId();
        RF_state.line.proposed            = *line;
        RF_state.line.fence               = RF_state.tx_bytes;
        RF_state.line.pending             = 2;
    }
    BSP_IrqRestore(irq);
    return sent;
}

bool RF_UartControlReady(void) {
    uint32_t irq   = BSP_IrqSave();
    bool     ready = RF_role == RF_ROLE_HOST && RF_BridgeActive() && !RF_state.fault &&
                 !RF_state.control_pending;
    BSP_IrqRestore(irq);
    return ready;
}

static bool RF_ControlSubmit(uint8_t dtr, uint16_t duration) {
    uint8_t  payload[4] = {dtr, 0, duration, duration >> 8};
    bool     queued     = RF_state.control_pending == 3;
    uint32_t id         = queued ? RF_state.control_id : RF_NextId();

    if (!RF_Send(RF_MESSAGE_UART_CONTROL, id, payload, sizeof(payload), RF_MESSAGE_OK)) {
        return false;
    }
    if (!queued) {
        RF_state.tx_id = id;
    }
    RF_state.control_id      = id;
    RF_state.control_word    = RF_Read32(payload);
    RF_state.control_fence   = RF_state.tx_bytes;
    RF_state.control_pending = 1;
    return true;
}

bool RF_UartControlSubmit(uint8_t dtr, uint16_t duration) {
    uint32_t irq  = BSP_IrqSave();
    bool     sent = dtr <= 1 && RF_UartControlReady();

    if (sent) {
        RF_HostWatchStart();
        RF_state.tx_id = RF_state.control_id = RF_NextId();
        RF_state.control_word                = dtr | ((uint32_t)duration << 16);
        RF_state.control_fence               = RF_state.tx_bytes;
        RF_state.control_pending             = 3;
    }
    BSP_IrqRestore(irq);
    return sent;
}

bool RF_UartSubmit(const uint8_t *data, unsigned length) {
    uint32_t irq  = BSP_IrqSave();
    bool     sent = false;

    if (data && length && length <= RF_UART_DATA_SIZE && RF_HostAvailable() && !RF_state.sync &&
        (RF_state.confirmed_control & 1) && !RF_Breaking()) {
        uint32_t id = RF_NextId();

        sent = RF_Send(RF_MESSAGE_UART_DATA, id, data, length, RF_MESSAGE_OK);
        if (sent) {
            RF_state.tx_id     = id;
            RF_state.tx_bytes += length;
        }
    }
    BSP_IrqRestore(irq);
    return sent;
}

unsigned RF_UartHostRead(uint8_t *data, unsigned limit) {
    uint32_t irq = BSP_IrqSave();
    unsigned n   = 0;

    if (RF_role == RF_ROLE_HOST && RF_BridgeActive() && !RF_state.fault && data) {
        if (limit > RF_uartRemaining) {
            limit = RF_uartRemaining;
        }
        n = RF_BridgeRead(data, limit);
    }
    BSP_IrqRestore(irq);
    return n;
}

void RF_UartHostRetire(unsigned length) {
    uint32_t irq = BSP_IrqSave();

    if (RF_role != RF_ROLE_HOST || length > RF_uartRemaining) {
        RF_Fault(RF_UART_FAULT_USB_RETIRE, length);
    } else {
        RF_BridgeRetire(length);
        RF_uartRemaining  -= length;
        RF_state.rx_bytes += length;
    }
    BSP_IrqRestore(irq);
}

const uart_line_t *RF_UartHostLine(void) {
    return &RF_state.actual_line;
}

uint32_t RF_UartOverrunGeneration(void) {
    return RF_role == RF_ROLE_HOST ? RF_state.overrun_generation : 0;
}

void RF_UartReset(void) {
    uint32_t    irq = BSP_IrqSave();
    uart_line_t req = RF_state.initialized ? RF_state.requested : (uart_line_t){115200, 0, 0, 8};
    uart_line_t confirmed = RF_state.initialized ? RF_state.line.current : req;
    uint8_t     role = RF_role, dtr = RF_state.confirmed_control & 1;
    uint32_t    actual;

    UART_LineValid(&confirmed, NULL, &actual);
    memset(&RF_state, 0, sizeof(RF_state));
    RF_uartRemaining = 0;

#if !RF_FIXED_ROLE
    RF_role = role;
#else
    if (role != RF_role) {
        RF_Fault(RF_UART_FAULT_RESET_ROLE, role);
        BSP_IrqRestore(irq);
        return;
    }
#endif

    RF_state.initialized = 1;
    RF_state.requested   = req;
    RF_UartBarrierInit(&RF_state.line, &confirmed);
    RF_state.line.actual       = actual;
    RF_state.actual_line       = confirmed;
    RF_state.actual_line.baud  = actual;
    RF_state.control_word      = dtr;
    RF_state.confirmed_control = dtr;
    if (role == RF_ROLE_TARGET) {
        /* This reset is also called after a USB STATUS ACK in the ISR.
         * Stop/clear the old owner here; restore the confirmed physical plan
         * only in RF_UartPoll, after radio reset events have been serviced. */
        UART_Stop();
        RF_state.sync = 1;
    } else if (role == RF_ROLE_HOST) {
        RF_state.sync = 1;
    }
    BSP_IrqRestore(irq);
}

void RF_UartInit(uint8_t role) {
    uint32_t irq = BSP_IrqSave();

#if RF_FIXED_ROLE
    if (role == 0 || role == APP_WORK_NONE) {
        memset(&RF_state, 0, sizeof(RF_state));
        RF_uartRemaining = 0;
        BSP_IrqRestore(irq);
        return;
    }
#endif

    if (RF_role == RF_ROLE_TARGET && role != RF_ROLE_TARGET) {
        UART_Stop();
    }
    memset(&RF_state, 0, sizeof(RF_state));
    RF_uartRemaining = 0;

#if !RF_FIXED_ROLE
    RF_role = role;
#else
    if (role != RF_role) {
        RF_Fault(RF_UART_FAULT_INIT_ROLE, role);
        BSP_IrqRestore(irq);
        return;
    }
#endif

    RF_UartReset();
    if (role && role != RF_ROLE_HOST && role != RF_ROLE_TARGET) {
        RF_Fault(RF_UART_FAULT_BAD_ROLE, role);
    }
    BSP_IrqRestore(irq);
}

static bool RF_ApplyLine(void *context, const uart_line_t *line) {
    (void)context;
    return UART_ConfigureLine(line);
}

static void RF_TargetBarriers(void) {
    if (RF_state.fault) {
        return;
    }
    if (RF_state.line.pending) {
        if (RF_ConfigExpired(RF_state.line.pending == 2 ? RF_UART_APPLY_TIMEOUT_MS
                                                        : RF_UART_CONFIG_TIMEOUT_MS)) {
            /* No successful apply occurred. Preserve confirmed line and all
             * owned bytes; negative result makes a new request id retryable. */
            UART_ConfigureCancel();
            RF_state.line.pending   = 0;
            RF_state.line.cached    = RF_UART_CACHE_BUSY;
            RF_state.result_id      = RF_state.line.id;
            RF_state.result_fence   = RF_state.line.fence;
            RF_state.result_status  = RF_MESSAGE_BUSY;
            RF_state.result_pending = 1;
            RF_state.last_status    = RF_MESSAGE_BUSY;
            RF_state.stats_at       = BSP_ms;
            ++RF_state.errors;
        } else {
            uint32_t transmitted = UART_TxTransmitted() - RF_state.tx_base;
            bool     idle        = UART_TxIdle();

            if (RF_state.line.pending == 1 && (int32_t)(transmitted - RF_state.line.fence) >= 0 &&
                idle) {
                RF_state.line.pending = 2; /* First hardware-apply attempt starts 5 s. */
                RF_state.stats_at     = BSP_ms;
            }
            if (RF_UartBarrierPoll(&RF_state.line, transmitted, idle, RF_ApplyLine, NULL)) {
                RF_state.requested        = RF_state.line.proposed;
                RF_state.actual_line      = RF_state.line.current;
                RF_state.actual_line.baud = RF_state.line.actual;
                RF_state.result_id        = RF_state.line.id;
                RF_state.result_fence     = RF_state.line.fence;
                RF_state.result_status    = RF_MESSAGE_OK;
                RF_state.result_pending   = 1;
                RF_state.stats_at         = BSP_ms;
            }
        }
    }
    if ((RF_state.result_pending || RF_state.control_pending) &&
        RF_ConfigExpired(RF_UART_CONFIG_TIMEOUT_MS)) {
        RF_ConfigTimeout();
        return;
    }
    if (RF_state.result_pending) {
        uint8_t payload[RF_MESSAGE_LINE_SIZE];

        /* Preserve the original request: realizable 300 baud is 299, which
         * must never be revalidated as a fresh out-of-range request. */
        RF_LineEncode(payload, RF_state.result_fence, &RF_state.line.current);
        if (RF_Send(RF_MESSAGE_UART_LINE_RESULT, RF_state.result_id, payload, sizeof(payload),
                    RF_state.result_status)) {
            RF_state.last_status    = RF_state.result_status;
            RF_state.result_pending = 0;
        }
    }
    if (RF_state.control_pending == 1) {
        /* BREAK is a control level, not an earlier data byte. A subsequent
         * control may replace/clear it once the preceding byte fence drained. */
        if (RF_Breaking() && UART_TxTransmitted() - RF_state.tx_base == RF_state.control_fence &&
            UART_TxDrained()) {
            UART_Break(0);
            RF_state.confirmed_control &= 0xFFFFU;
            RF_state.break_until        = BSP_ms;
        }
        if (UART_TxTransmitted() - RF_state.tx_base == RF_state.control_fence && UART_TxIdle()) {
            UART_Open(RF_state.control_word & 1);
            UART_Break(RF_state.control_word >> 16);
            RF_state.confirmed_control = RF_state.control_word;
            RF_state.break_until       = BSP_ms + (RF_state.control_word >> 16);
            RF_state.control_pending   = 2;
            RF_state.stats_at          = BSP_ms;
        }
    }
    if (RF_state.control_pending == 2) {
        uint8_t payload[RF_MESSAGE_STATUS_SIZE];

        RF_Write32(payload, STATUS_CONTROL);
        RF_Write32(payload + 4, RF_state.control_word);
        RF_Write32(payload + 8, RF_state.control_fence);
        if (RF_Send(RF_MESSAGE_STATUS, RF_state.control_id, payload, sizeof(payload),
                    RF_MESSAGE_OK)) {
            RF_state.last_status     = RF_MESSAGE_OK;
            RF_state.control_pending = 0;
            RF_state.control_cached  = 1;
        }
    }
}

static bool RF_Ordered(uint32_t id) {
    return id && (!RF_state.last_rx_id || (int32_t)(id - RF_state.last_rx_id) > 0);
}

static bool RF_Receive(void) {
#if APP_COMBINED_TARGET
    if (DAP_queue.rf.inputLength) {
        return false;
    }
#endif

#if (APP_COMBINED_TARGET || APP_COMBINED_HOST)
    uint8_t kind;

    if (RF_BridgeRead(&kind, 1) != 1 || kind == RF_MESSAGE_DAP_COMMAND ||
        kind == RF_MESSAGE_DAP_RESPONSE || kind == RF_MESSAGE_DAP_ABORT) {
        return false;
    }
#endif

    uint8_t  packet[RF_MESSAGE_HEADER_SIZE + RF_UART_DATA_SIZE], canonical[RF_MESSAGE_HEADER_SIZE];
    unsigned count = RF_BridgeRead(packet, sizeof(packet));

    if (count < RF_MESSAGE_HEADER_SIZE) {
        return false;
    }
    unsigned type   = packet[RF_MESSAGE_TYPE_OFFSET],
             length = packet[RF_MESSAGE_LENGTH_OFFSET] |
                      ((unsigned)packet[RF_MESSAGE_LENGTH_OFFSET + 1] << 8);
    unsigned status =
        packet[RF_MESSAGE_STATUS_OFFSET] | ((unsigned)packet[RF_MESSAGE_STATUS_OFFSET + 1] << 8);
    uint32_t id = RF_Read32(packet + RF_MESSAGE_ID_OFFSET);
    bool     valid;

    if (type == RF_MESSAGE_UART_DATA) {
        /* The decoded id/length bytes already reproduce themselves. Check
         * only the remaining wire fields instead of rebuilding a header. */
        valid = length && length <= RF_UART_DATA_SIZE && !status &&
                packet[RF_MESSAGE_CHANNEL_OFFSET] == RF_MESSAGE_CHANNEL_UART &&
                packet[RF_MESSAGE_VERSION_OFFSET] == RF_MESSAGE_VERSION &&
                !(packet[RF_MESSAGE_FLAGS_OFFSET] & ~RF_MESSAGE_FLAG_OVERRUN);
    } else {
        valid = length <= RF_UART_DATA_SIZE &&
                RF_MessageHeaderFlags(canonical, type, id, length, status,
                                      packet[RF_MESSAGE_FLAGS_OFFSET]) &&
                !memcmp(canonical, packet, sizeof(canonical));
    }
    if (!valid) {
        RF_Fault(RF_UART_FAULT_HEADER, count);
        return false;
    }
    if (count < RF_MESSAGE_HEADER_SIZE + length) {
        return false;
    }
    const uint8_t *payload = packet + RF_MESSAGE_HEADER_SIZE;

    if (RF_role == RF_ROLE_TARGET) {
        /* Overrun metadata is only meaningful for physical target RX.
         * A host may not claim an overrun in the lossless forward direction. */
        if (packet[RF_MESSAGE_FLAGS_OFFSET]) {
            RF_Fault(RF_UART_FAULT_TARGET_FLAGS, packet[RF_MESSAGE_FLAGS_OFFSET]);
            return false;
        }
        if (RF_state.line.pending || RF_state.result_pending || RF_state.control_pending) {
            return false;
        }
        if (type == RF_MESSAGE_UART_DATA) {
            if (!RF_Ordered(id)) {
                RF_Fault(RF_UART_FAULT_TARGET_ORDER, id);
                return false;
            }
            if (RF_Breaking() || UART_TxSpace() < length || !UART_TxSubmit(payload, length)) {
                ++RF_state.pressure;
                return false;
            }
            RF_state.rx_bytes += length;
        } else if (type == RF_MESSAGE_UART_LINE) {
            enum rf_message_status result = RF_MESSAGE_INVALID;
            if (RF_Ordered(id) || (RF_state.line.cached && id == RF_state.line.id)) {
                /* Global ordered IDs can wrap after many DATA messages
                 * without an intervening LINE. Its one-request cache must
                 * not mistake the fresh ordered request for an old replay. */
                if (RF_Ordered(id) && RF_state.line.cached &&
                    (int32_t)(id - RF_state.line.id) <= 0) {
                    RF_state.line.cached = 0;
                }
                result = RF_UartBarrierRequest(&RF_state.line, id, payload, RF_state.rx_bytes);
            }
            RF_state.stats_at = BSP_ms;
            if (result != RF_MESSAGE_OK) {
                RF_state.result_id      = id;
                RF_state.result_fence   = RF_Read32(payload);
                RF_state.result_status  = result;
                RF_state.result_pending = 1;
                if (result != RF_MESSAGE_REPLAY &&
                    !(result == RF_MESSAGE_BUSY && RF_state.line.cached == RF_UART_CACHE_BUSY &&
                      id == RF_state.line.id)) {
                    ++RF_state.errors;
                }
            }
        } else if (type == RF_MESSAGE_UART_CONTROL) {
            uint32_t word = RF_Read32(payload);
            bool replay = !RF_Ordered(id) && RF_state.control_cached && id == RF_state.control_id &&
                          word == RF_state.control_word;
            if (payload[0] > 1 || payload[1] || (!RF_Ordered(id) && !replay)) {
                RF_Fault(RF_UART_FAULT_TARGET_CONTROL, id);
                return false;
            }
            RF_state.control_id      = id;
            RF_state.control_word    = word;
            RF_state.control_fence   = RF_state.rx_bytes;
            RF_state.control_pending = replay ? 2 : 1;
            RF_state.stats_at        = BSP_ms;
        } else {
            RF_Fault(RF_UART_FAULT_TARGET_TYPE, type);
            return false;
        }
        if (RF_Ordered(id)) {
            RF_state.last_rx_id = id;
        }
    } else if (type == RF_MESSAGE_UART_DATA) {
        if (!RF_Ordered(id)) {
            RF_Fault(RF_UART_FAULT_HOST_ORDER, id);
            return false;
        }
        RF_state.last_rx_id = id;
        if (packet[RF_MESSAGE_FLAGS_OFFSET] & RF_MESSAGE_FLAG_OVERRUN) {
            ++RF_state.overrun_generation;
            RF_state.overrun_flags = 1;
        }
        RF_BridgeRetire(RF_MESSAGE_HEADER_SIZE);
        RF_uartRemaining = length;
        return false;
    } else if (type == RF_MESSAGE_UART_LINE_RESULT) {
        uart_line_t req = {RF_Read32(payload + RF_MESSAGE_LINE_CODING_OFFSET),
                           payload[RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_STOP_OFFSET],
                           payload[RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_PARITY_OFFSET],
                           payload[RF_MESSAGE_LINE_CODING_OFFSET + USB_CDC_DATA_BITS_OFFSET]};
        if (RF_state.line.pending != 1 || id != RF_state.line.id ||
            RF_Read32(payload) != RF_state.line.fence || !RF_UartLineValid(&req) ||
            ((status == RF_MESSAGE_OK || status == RF_MESSAGE_REPLAY) &&
             !RF_SameLine(&req, &RF_state.line.proposed))) {
            RF_Fault(RF_UART_FAULT_LINE_RESULT, id);
            return false;
        }
        RF_state.last_status = status;
        if (status == RF_MESSAGE_OK || status == RF_MESSAGE_REPLAY) {
            RF_state.line.current = req;
            UART_LineValid(&req, NULL, &RF_state.line.actual);
            RF_state.actual_line      = req;
            RF_state.actual_line.baud = RF_state.line.actual;
            RF_state.requested        = RF_state.line.proposed;
        } else {
            ++RF_state.errors;
        }
        RF_state.line.pending = 0;
        RF_state.stats_group  = 0;
        if (RF_state.sync == 1) {
            if (status != RF_MESSAGE_OK && status != RF_MESSAGE_REPLAY) {
                RF_state.fault = 1;
                RF_state.sync = RF_state.control_pending = 0;
                return false;
            }
            RF_state.sync = 2;
        }
    } else if (type == RF_MESSAGE_STATUS) {
        unsigned kind = RF_Read32(payload);

        if (kind == STATUS_CONTROL) {
            if (status || RF_state.control_pending != 1 || id != RF_state.control_id ||
                RF_Read32(payload + 4) != RF_state.control_word ||
                RF_Read32(payload + 8) != RF_state.control_fence) {
                RF_Fault(RF_UART_FAULT_CONTROL_RESULT, id);
                return false;
            }
            RF_state.control_pending   = 0;
            RF_state.stats_group       = 0;
            RF_state.confirmed_control = RF_state.control_word;
            RF_state.last_status       = RF_MESSAGE_OK;
            RF_state.break_until       = BSP_ms + (RF_state.control_word >> 16);
            if (RF_state.sync == 2) {
                RF_state.sync = 0;
            }
        } else if (!id && !status && kind >= STATUS_STATS_FIRST && kind <= STATUS_STATS_LAST) {
            unsigned index = (kind - STATUS_STATS_FIRST) * 2;

            RF_state.stats[index]     = RF_Read32(payload + 4);
            RF_state.stats[index + 1] = RF_Read32(payload + 8);
        } else {
            RF_Fault(RF_UART_FAULT_STATUS_KIND, kind);
            return false;
        }
    } else {
        RF_Fault(RF_UART_FAULT_HOST_TYPE, type);
        return false;
    }
    RF_BridgeRetire(RF_MESSAGE_HEADER_SIZE + length);
    return true;
}

static void RF_TargetSend(void) {
    /* Observe even without data/credit. The sticky bit survives counter
     * wrap during a long stall; no queue acceptance means no consumption. */
    uint32_t loss = UART_RxLossGeneration();

    if (loss != RF_state.loss_observed) {
        RF_state.overrun_flags = 1;
        RF_state.loss_observed = loss;
    }

    /* Do not let short reverse DATA packets consume the capacity that a
     * queued confirmation needs; UART RX retains those bytes meanwhile. */
    if (RF_state.result_pending || RF_state.control_pending == 2) {
        return;
    }
    unsigned length = UART_RxAvailable();

    if (length > RF_UART_DATA_SIZE) {
        length = RF_UART_DATA_SIZE;
    }
    if (length &&
        (length == RF_UART_DATA_SIZE || (uint32_t)(BSP_ms - UART_RxStarted()) >= CDC_RX_FLUSH_MS)) {
        if (RF_BridgeSpace() < RF_MESSAGE_HEADER_SIZE + length) {
            ++RF_state.pressure;
            return;
        }
        uint8_t payload[RF_UART_DATA_SIZE];

        /* UART IRQs remain enabled. Committing this captured generation,
         * rather than clearing a boolean, preserves a newer ISR loss while
         * reading/publishing the payload. RF owns the immutable flag through
         * all link retransmissions once the complete message is queued. */
        length         = UART_RxRead(payload, length);
        uint32_t id    = RF_NextId();
        unsigned flags = RF_state.overrun_flags ? RF_MESSAGE_FLAG_OVERRUN : 0;

        if (length &&
            RF_SendFlags(RF_MESSAGE_UART_DATA, id, payload, length, RF_MESSAGE_OK, flags)) {
            RF_state.tx_id     = id;
            RF_state.tx_bytes += length;

            /* A loss after the snapshot remains unreported. An IRQ after
             * this comparison is caught by loss_observed on the next poll. */
            RF_state.overrun_flags = UART_RxLossGeneration() != loss;
        }
    }

    /* Payload and application confirmations get RF capacity first. At most
     * one 24-byte statistics message every 20 ms, rotating three groups;
     * unchanged groups still advance the timer rather than busy-polling. */
    if (RF_state.line.pending || RF_state.control_pending || RF_state.result_pending ||
        (RF_state.active && (uint32_t)(BSP_ms - RF_state.stats_at) < 20U)) {
        return;
    }
    RF_state.stats_at = BSP_ms;
    uint32_t stats[6] = {UART_stats.rx_bytes,
                         UART_stats.tx_bytes,
                         UART_stats.rx_overflow,
                         UART_stats.line_errors,
                         UART_stats.max_rx_high_water,
                         (UART_stats.last_line_status & 0xFFU) |
                             ((uint32_t)UART_LineDivisor() << 16)};
    unsigned group = RF_state.stats_group, index = group * 2;

    if (!RF_state.active || RF_state.stats[index] != stats[index] ||
        RF_state.stats[index + 1] != stats[index + 1]) {
        uint8_t payload[RF_MESSAGE_STATUS_SIZE];

        RF_Write32(payload, STATUS_STATS_FIRST + group);
        RF_Write32(payload + 4, stats[index]);
        RF_Write32(payload + 8, stats[index + 1]);
        if (!RF_Send(RF_MESSAGE_STATUS, 0, payload, sizeof(payload), RF_MESSAGE_OK)) {
            return;
        }
        RF_state.stats[index]     = RF_Read32(payload + 4);
        RF_state.stats[index + 1] = RF_Read32(payload + 8);
    }
    RF_state.stats_group = (group + 1) % 3;
}

void RF_UartPoll(void) {
    /* Keep old USB completions alive while quiescing, but do not apply a new
     * remote UART configuration after the USB owner has requested a stop. */
    if (!RF_role || BSP_stopRequested || !BSP_workReady) {
        return;
    }
    bool target  = RF_role == RF_ROLE_TARGET;
    bool usbOnly = target;

    usbOnly      = true;
    uint32_t irq = 0, usbEnabled = 0;

    if (usbOnly) {
        usbEnabled = PFIC_GetStatusIRQ(USB_IRQn);
        PFIC_DisableIRQ(USB_IRQn);
    } else {
        irq = BSP_IrqSave();
    }

    /* USB may have requested a stop between the fast check and exclusion. */
    if (BSP_stopRequested || !BSP_workReady) {
        if (usbOnly) {
            if (usbEnabled) {
                PFIC_EnableIRQ(USB_IRQn);
            }
        } else {
            BSP_IrqRestore(irq);
        }
        return;
    }
    if (target) {
        UART_Poll();
    }
    if (target && RF_state.sync && !RF_state.fault) {
        if (RF_state.sync == 1) {
            RF_state.stats_at = BSP_ms;
            UART_Init();
            RF_state.loss_observed = UART_RxLossGeneration();
            RF_state.sync          = 2;
        }
        if (RF_ConfigExpired(RF_UART_APPLY_TIMEOUT_MS)) {
            RF_ConfigTimeout();
        }
        if (RF_state.fault || !UART_ConfigureLine(&RF_state.line.current)) {
            if (usbOnly) {
                if (usbEnabled) {
                    PFIC_EnableIRQ(USB_IRQn);
                }
            } else {
                BSP_IrqRestore(irq);
            }
            return;
        }
        RF_state.actual_line      = RF_state.line.current;
        RF_state.actual_line.baud = RF_state.line.actual;
        UART_Open(false);
        RF_state.tx_base = UART_TxTransmitted();
        RF_state.sync    = 0;
    }
    if (!RF_BridgeActive()) {
        if (RF_state.active) {
            RF_UartReset();
        }
        if (usbOnly) {
            if (usbEnabled) {
                PFIC_EnableIRQ(USB_IRQn);
            }
        } else {
            BSP_IrqRestore(irq);
        }
        return;
    }
    if (!target && !RF_state.fault &&
        (RF_state.line.pending || RF_state.control_pending ||
         (RF_state.sync && !RF_state.controls_hold))) {
        RF_HostWatchStart();
        if (RF_ConfigExpired(RF_UART_CONFIG_TIMEOUT_MS)) {
            RF_ConfigTimeout();
        }
    }
    if (!RF_state.fault) {
        if (RF_role == RF_ROLE_TARGET) {
            RF_TargetBarriers();
        }
        if (!RF_uartRemaining) {
            for (unsigned i = 0;
                 i < 8 && !RF_state.fault && !RF_BridgeEventPending() && RF_Receive(); i++) {
            }
        }
        if (RF_role == RF_ROLE_HOST && !RF_state.fault) {
            if (RF_state.line.pending || RF_state.control_pending ||
                (RF_state.sync && !RF_state.controls_hold)) {
                RF_HostWatchStart();
            }
            bool lineQueued    = RF_state.line.pending == 2;
            bool controlQueued = RF_state.control_pending == 3;

            if (RF_state.line.pending != 1 && RF_state.control_pending != 1 && lineQueued &&
                (!controlQueued || (int32_t)(RF_state.line.id - RF_state.control_id) < 0)) {
                RF_LineSubmit(&RF_state.line.proposed);
            } else if (RF_state.line.pending != 1 && RF_state.control_pending != 1 &&
                       controlQueued) {
                RF_ControlSubmit(RF_state.control_word & 1, RF_state.control_word >> 16);
            } else if (!RF_state.controls_hold && RF_HostAvailable() && RF_state.sync == 1) {
                RF_LineSubmit(&RF_state.requested);
            } else if (!RF_state.controls_hold && RF_HostAvailable() && RF_state.sync == 2) {
                RF_ControlSubmit(RF_state.confirmed_control & 1, 0);
            }
        } else if (target) {
            RF_TargetBarriers();
            if (!RF_state.fault) {
                RF_TargetSend();
            }
        }
    }
    RF_state.active = 1;
    if (usbOnly) {
        if (usbEnabled) {
            PFIC_EnableIRQ(USB_IRQn);
        }
    } else {
        BSP_IrqRestore(irq);
    }
}
