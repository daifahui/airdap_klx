/* USB DMA owns the normal DAP pool; RF carries ordered complete commands.
 * Admission/retirement and RF TX publication belong to the USB owner. The
 * foreground only consumes complete remote replies into the existing spare.
 * No local SWJ backend, physical UART FIFO or second DAP payload pool exists. */
#include "ch572/ch572.h"
#include "bsp/board.h"
#include "dap/dap_queue.h"
#include "dap/dap.h"
#include "rf/rf_dap.h"
#include "rf/rf_bridge.h"
#include "rf/rf_message.h"
#include "rf/rf_uart.h"
#include "runtime.h"
#include <string.h>

static uint32_t         RF_dapSent, RF_dapIdentity;
static volatile uint8_t RF_abortSource;
volatile uint8_t        RF_dapReceiving;

void RF_DapReset(void) {
    DAP_QueueInit(&DAP_queue);
    memset(&DAP_queue.rf, 0, sizeof(DAP_queue.rf));
    RF_dapSent = RF_dapIdentity = RF_abortSource = 0;
}

void RF_DapAbort(uint8_t source) {
    RF_abortSource = source;
}

bool RF_DapIdle(void) {
    return !RF_dapReceiving && !RF_abortSource && DAP_queue.executed == DAP_queue.produced;
}

void RF_DapTransmit(void) {
    if (!RF_BridgeActive() || BSP_stopRequested || !BSP_workReady || DAP_queue.rf.fault) {
        return;
    }
    uint8_t header[RF_MESSAGE_HEADER_SIZE];

    if (RF_abortSource) {
        if (DAP_queue.executed != RF_dapSent) {
            dap_slot_meta_t *meta    = &DAP_queue.meta[DAP_queue.executed & (DAP_PACKET_COUNT - 1)];
            uint8_t          command = DAP_queue.data[meta->buffer_index][0];

            if (meta->source == RF_abortSource &&
                (command == ID_DAP_Transfer || command == ID_DAP_TransferBlock

#if DAP_ATOMIC_COMMANDS
                 || command == ID_DAP_QueueCommands || command == ID_DAP_ExecuteCommands
#endif

                 )) {
                RF_MessageHeader(header, RF_MESSAGE_DAP_ABORT, meta->request_id, 0, 0);
                if (!RF_BridgeSubmitPair((const uint8_t *)header, sizeof(header), NULL, 0)) {
                    return;
                }
            }
        }
        RF_abortSource = 0;
    }
    while (RF_dapSent != DAP_queue.produced) {
        dap_slot_meta_t *meta     = &DAP_queue.meta[RF_dapSent & (DAP_PACKET_COUNT - 1)];
        uint32_t         identity = RF_dapIdentity + 1;

        if (!identity) {
            identity = 1;
        }
        RF_MessageHeader(header, RF_MESSAGE_DAP_COMMAND, identity, meta->length, 0);
        if (!RF_BridgeSubmitPair((const uint8_t *)header, sizeof(header),
                                 DAP_queue.data[meta->buffer_index], meta->length)) {
            break;
        }
        meta->request_id = RF_dapIdentity = identity;
        ++RF_dapSent;
    }
}

void RF_DapPoll(bool beforeReply) {
    (void)beforeReply;
    if (!RF_BridgeActive() || BSP_stopRequested || !BSP_workReady || DAP_queue.rf.fault) {
        return;
    }
    if ((RF_dapSent != DAP_queue.produced) || RF_abortSource) {
        PFIC_SetPendingIRQ(USB_IRQn);
    }
    if (RF_uartRemaining) {
        return;
    }
    RF_dapReceiving = 1;
    uint8_t packet[RF_MESSAGE_HEADER_SIZE];

    while (RF_BridgeRead(packet, sizeof(packet)) == RF_MESSAGE_HEADER_SIZE) {
        if (packet[RF_MESSAGE_TYPE_OFFSET] != RF_MESSAGE_DAP_RESPONSE) {
            break; /* UART owns its record through USB IN completion. */
        }
        unsigned length = packet[RF_MESSAGE_LENGTH_OFFSET] |
                          ((unsigned)packet[RF_MESSAGE_LENGTH_OFFSET + 1] << 8);
        uint32_t identity = packet[RF_MESSAGE_ID_OFFSET] |
                            ((uint32_t)packet[RF_MESSAGE_ID_OFFSET + 1] << 8) |
                            ((uint32_t)packet[RF_MESSAGE_ID_OFFSET + 2] << 16) |
                            ((uint32_t)packet[RF_MESSAGE_ID_OFFSET + 3] << 24);

        /* Type was checked above; id/length are the decoded wire fields. */
        if (!identity || DAP_queue.executed == RF_dapSent || !length || length > DAP_PACKET_SIZE ||
            packet[RF_MESSAGE_CHANNEL_OFFSET] != RF_MESSAGE_CHANNEL_DAP ||
            packet[RF_MESSAGE_FLAGS_OFFSET] ||
            packet[RF_MESSAGE_VERSION_OFFSET] != RF_MESSAGE_VERSION ||
            packet[RF_MESSAGE_STATUS_OFFSET] || packet[RF_MESSAGE_STATUS_OFFSET + 1]) {
            DAP_queue.rf.fault = 1;
            goto done;
        }
        uint8_t *resp = DAP_queue.data[DAP_queue.spare];

        if (!RF_BridgeReadAfter(resp, RF_MESSAGE_HEADER_SIZE, length)) {
            break;
        }
        dap_slot_meta_t *meta    = &DAP_queue.meta[DAP_queue.executed & (DAP_PACKET_COUNT - 1)];
        uint8_t          command = DAP_queue.data[meta->buffer_index][0];

#if DAP_ATOMIC_COMMANDS
        if (command == ID_DAP_QueueCommands) {
            command = ID_DAP_ExecuteCommands;
        }
#endif

        if (meta->request_id != identity || resp[0] != command) {
            DAP_queue.rf.fault = 1;
            goto done;
        }
        if (BSP_stopRequested || !RF_BridgeActive()) {
            goto done;
        }

        uint8_t n;

        DAP_QueueBegin(&DAP_queue, &n, &resp);
        DAP_QueueFinish(&DAP_queue, length);
        RF_BridgeRetire(RF_MESSAGE_HEADER_SIZE + length);
        PFIC_SetPendingIRQ(USB_IRQn);
    }
done:
    RF_dapReceiving = 0;
}
