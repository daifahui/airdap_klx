/* CMSIS-DAP over the reliable RF stream. USB and RF allocate this same pool
 * type in mutually exclusive builds; neither can own the other's RX buffer. */
#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "rf/rf_dap.h"
#include "rf/rf_bridge.h"
#include "rf/rf_message.h"
#include "bsp/board.h"
#include "dap/dap_queue.h"
#include "dap/dap_config.h"
#include "dap/dap.h"
#include <string.h>

#if APP_COMBINED_TARGET
dap_queue_t DAP_queue
#else
static dap_queue_t DAP_queue
#endif

    __ALIGNED(DAP_QUEUE_ALIGNMENT) __SECTION(MODE_LOCAL_SECTION("dap_queue"));

void RF_DapReset(void) {
    /* Foreground transport generation boundary, never concurrent with SWD.
     * RF link reset also discards all copies in its reliable TX/RX rings. */
    DAP_QueueInit(&DAP_queue);
    memset(&DAP_queue.rf, 0, sizeof(DAP_queue.rf));
}

void RF_DapPoll(bool beforeReply) {
    (void)beforeReply;
    if (!RF_BridgeActive() || BSP_stopRequested || !BSP_workReady || DAP_queue.rf.fault) {
        return;
    }

    /* Parse only complete headers. Keep an incomplete payload in the reliable
     * RX ring until it can be published directly from the shared RX buffer. */
    while (DAP_queue.produced - DAP_queue.retired < DAP_PACKET_COUNT) {
        if (!DAP_queue.rf.inputLength) {
            uint8_t header[RF_MESSAGE_HEADER_SIZE];

            if (RF_BridgeRead(header, sizeof(header)) != sizeof(header)) {
                break;
            }

#if APP_COMBINED_TARGET
            if (header[RF_MESSAGE_TYPE_OFFSET] == RF_MESSAGE_DAP_ABORT) {
                uint8_t  canonical[RF_MESSAGE_HEADER_SIZE];
                uint32_t id = header[RF_MESSAGE_ID_OFFSET] |
                              ((uint32_t)header[RF_MESSAGE_ID_OFFSET + 1] << 8) |
                              ((uint32_t)header[RF_MESSAGE_ID_OFFSET + 2] << 16) |
                              ((uint32_t)header[RF_MESSAGE_ID_OFFSET + 3] << 24);
                RF_MessageHeader(canonical, RF_MESSAGE_DAP_ABORT, id, 0, 0);
                if (!id || memcmp(canonical, header, sizeof(header))) {
                    DAP_queue.rf.fault = 1;
                    return;
                }
                for (uint32_t i = DAP_queue.executed; i != DAP_queue.produced; ++i) {
                    dap_slot_meta_t *meta = &DAP_queue.meta[i & (DAP_PACKET_COUNT - 1)];

                    if (meta->request_id == id) {
                        meta->reserved[0] = 1;
                    }
                }
                RF_BridgeRetire(sizeof(header));
                continue;
            }

            /* UART owns its entire typed record, including a partial payload.
             * DAP may still execute/publish earlier commands below. */
            if (header[RF_MESSAGE_TYPE_OFFSET] != RF_MESSAGE_DAP_COMMAND) {
                break;
            }
#endif

            unsigned length = header[RF_MESSAGE_LENGTH_OFFSET] |
                              ((unsigned)header[RF_MESSAGE_LENGTH_OFFSET + 1] << 8);
            uint32_t id = header[RF_MESSAGE_ID_OFFSET] |
                          ((uint32_t)header[RF_MESSAGE_ID_OFFSET + 1] << 8) |
                          ((uint32_t)header[RF_MESSAGE_ID_OFFSET + 2] << 16) |
                          ((uint32_t)header[RF_MESSAGE_ID_OFFSET + 3] << 24);

            /* id/length already reproduce their input bytes. Validate the
             * fixed DAP channel/version/status without rebuilding a header. */
            if (!id || !length || length > DAP_PACKET_SIZE ||
                header[RF_MESSAGE_TYPE_OFFSET] != RF_MESSAGE_DAP_COMMAND ||
                header[RF_MESSAGE_CHANNEL_OFFSET] != RF_MESSAGE_CHANNEL_DAP ||
                header[RF_MESSAGE_FLAGS_OFFSET] ||
                header[RF_MESSAGE_VERSION_OFFSET] != RF_MESSAGE_VERSION ||
                header[RF_MESSAGE_STATUS_OFFSET] || header[RF_MESSAGE_STATUS_OFFSET + 1]) {
                DAP_queue.rf.fault = 1;
                return;
            }
            DAP_queue.meta[DAP_queue.produced & (DAP_PACKET_COUNT - 1)].request_id = id;
            DAP_queue.rf.inputLength                                               = length;
            RF_BridgeRetire(sizeof(header));
        }
        unsigned n = DAP_queue.rf.inputLength;

        if (RF_BridgeRead(DAP_queue.data[DAP_queue.bulk_rx], n) != n) {
            break;
        }
        unsigned slot = DAP_queue.produced & (DAP_PACKET_COUNT - 1);
        uint32_t id   = DAP_queue.meta[slot].request_id;

        DAP_QueueAcceptSource(&DAP_queue, n, SysTick->CNTL, DAP_SOURCE_RF);
        DAP_queue.meta[slot].request_id = id;
        RF_BridgeRetire(n);
        DAP_queue.rf.inputLength = 0;
    }

    /* Serialize whole commands into the existing response slots. Start no
     * further command after 1 ms; a single slow command is never split. */

#if RF_DAP_COMMAND_BATCH > 1
    uint32_t batchStarted = SysTick->CNTL;
#endif

    for (unsigned batch = 0; batch < RF_DAP_COMMAND_BATCH; ++batch) {
        if (DAP_queue.executed == DAP_queue.produced) {
            break;
        }

        /* Yield only between whole commands. The RF service remains the
         * event owner; completed responses are still published below. */
        if (!beforeReply && RF_BridgeEventPending()) {
            break;
        }
        if (RF_BridgeActive() && BSP_DapEnter()) {
            uint8_t        length, *resp;
            const uint8_t *req = DAP_QueueBegin(&DAP_queue, &length, &resp);

#if DAP_ATOMIC_COMMANDS
            if (!req) {
                BSP_DapLeave();
                break; /* Queued atomic group is waiting for its release packet. */
            }
#endif

            DAP_transferAbort     = 0;
            dap_slot_meta_t *meta = &DAP_queue.meta[DAP_queue.executed & (DAP_PACKET_COUNT - 1)];
            unsigned         n;

#if DAP_ATOMIC_COMMANDS
            if (meta->reserved[0] && req[0] == ID_DAP_ExecuteCommands) {
                resp[0] = ID_DAP_ExecuteCommands;
                resp[1] = 0;
                n       = 2;
            } else
#endif

                if (meta->reserved[0] && (req[0] == 5 || req[0] == 6)) {
                resp[0] = req[0];
                resp[1] = resp[2] = 0;
                n                 = req[0] == 5 ? 3 : 4;
                resp[n - 1]       = DAP_TRANSFER_ERROR;
            } else {
                n = (uint16_t)DAP_ProcessCommand(req, resp);
            }
            BSP_DapLeave();
            if (BSP_stopRequested || !n || n > DAP_PACKET_SIZE) {
                DAP_queue.rf.fault = 1;
                return;
            }
            if (!RF_BridgeActive()) {
                return; /* Old generation will be reset before another execution. */
            }
            DAP_QueueFinish(&DAP_queue, n);
        } else {
            break;
        }

#if RF_DAP_COMMAND_BATCH > 1
        if ((uint32_t)(SysTick->CNTL - batchStarted) >= BSP_clockHz / 1000U) {
            break;
        }
#endif
    }
    while (DAP_queue.retired != DAP_queue.executed) {
        uint8_t        length;
        uint8_t        header[RF_MESSAGE_HEADER_SIZE];
        const uint8_t *resp = DAP_QueueResponseSource(&DAP_queue, &length, DAP_SOURCE_RF);

        if (RF_BridgeSpace() < sizeof(header) + length) {
            DAP_QueueRetrySource(&DAP_queue, DAP_SOURCE_RF);
            break;
        }
        uint32_t id = DAP_queue.meta[DAP_queue.retired & (DAP_PACKET_COUNT - 1)].request_id;

        RF_MessageHeader(header, RF_MESSAGE_DAP_RESPONSE, id, length, 0);

        /* Sole foreground producer; preflight reserves the complete message.
         * The RF ring owns both copies until ACK, including every retransmit. */
        if (!RF_BridgeSubmitPair((const uint8_t *)header, sizeof(header), resp, length)) {
            DAP_QueueRetrySource(&DAP_queue, DAP_SOURCE_RF);
            break;
        }
        DAP_QueueRetireSource(&DAP_queue, DAP_SOURCE_RF);
    }
}
