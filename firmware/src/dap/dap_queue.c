/* SPDX-License-Identifier: Apache-2.0 */
/* Shared USB/RF pool. Rotate the spare without copying responses. The
 * transport retains a slot until DMA completion or reliable RF queue takeover. */

#include "dap/dap_queue.h"
#include "ch572/attributes.h"
#include "dap/dap.h"
#include <string.h>

#if (APP_HAS_DAP && !APP_HAS_RF)
#define __DAP_QUEUE_CODE __HIGH_CODE
#else
#define __DAP_QUEUE_CODE
#endif

#if APP_COMBINED_HOST
dap_queue_t DAP_queue __ALIGNED(DAP_QUEUE_ALIGNMENT);
#endif

void DAP_QueueInit(dap_queue_t *q) {
    /* Payloads are bounded by their lengths, not zero-padded. No need to clear
     * the packet pool on a reset; only reset ownership and transaction state. */
    memset(q->meta, 0, sizeof(q->meta));
    for (unsigned i = 0; i < DAP_PACKET_COUNT; i++) {
        q->meta[i].buffer_index = i;
    }
    q->produced = q->executed = q->retired = 0;
    q->spare                               = DAP_PACKET_COUNT;
    q->bulk_rx                             = DAP_PACKET_COUNT + 1U;
}

__DAP_QUEUE_CODE bool DAP_QueueAcceptSource(dap_queue_t *q, uint8_t n, uint32_t time,
                                            uint8_t source) {
    if (!n || n > DAP_PACKET_SIZE || ((q)->produced - (q)->retired == DAP_PACKET_COUNT) ||
        (source != DAP_SOURCE_BULK && source != DAP_SOURCE_RF)) {
        return false;
    }
    unsigned i      = q->produced & (DAP_PACKET_COUNT - 1);
    uint8_t  nextRx = q->meta[i].buffer_index;

    q->meta[i] = (dap_slot_meta_t){.request_id   = q->produced,
                                   .received_at  = time,
                                   .source       = source,
                                   .state        = SLOT_REQUEST,
                                   .length       = n,
                                   .buffer_index = q->bulk_rx};
    q->bulk_rx = nextRx;
    __asm__ volatile("" ::: "memory");
    ++q->produced;

    return true;
}

bool DAP_QueuePushSource(dap_queue_t *q, const uint8_t *p, uint8_t n, uint32_t time,
                         uint8_t source) {
    if (!n || n > DAP_PACKET_SIZE || ((q)->produced - (q)->retired == DAP_PACKET_COUNT) ||
        (source != DAP_SOURCE_BULK && source != DAP_SOURCE_HID)) {
        return false;
    }
    unsigned i      = q->produced & (DAP_PACKET_COUNT - 1);
    uint8_t  buffer = q->meta[i].buffer_index;

    memcpy(q->data[buffer], p, n);
    q->meta[i] = (dap_slot_meta_t){.request_id   = q->produced,
                                   .received_at  = time,
                                   .source       = source,
                                   .state        = SLOT_REQUEST,
                                   .length       = n,
                                   .buffer_index = buffer};
    __asm__ volatile("" ::: "memory");
    ++q->produced;

    return true;
}

#if (DAP_ATOMIC_COMMANDS && !APP_COMBINED_HOST)
static __NO_INLINE bool DAP_QueueReleaseCommands(dap_queue_t *q) {
    uint32_t end    = q->produced;
    uint32_t stop   = end;
    uint8_t  source = q->meta[q->executed & (DAP_PACKET_COUNT - 1)].source;

    for (uint32_t id = q->executed; id != end; ++id) {
        const dap_slot_meta_t *meta = &q->meta[id & (DAP_PACKET_COUNT - 1)];

        if (meta->source == source && q->data[meta->buffer_index][0] != ID_DAP_QueueCommands) {
            stop = id;
            break;
        }
    }

    /* Match the CMSIS-DAP reference's full-queue wakeup: otherwise all slots
     * could hold QueueCommands and the releasing packet could never enter.
     * A different USB source alone is not a releasing packet. */
    if (stop == end && end - q->retired != DAP_PACKET_COUNT) {
        return false;
    }
    for (uint32_t id = q->executed; id != stop; ++id) {
        const dap_slot_meta_t *meta = &q->meta[id & (DAP_PACKET_COUNT - 1)];
        uint8_t               *req  = q->data[meta->buffer_index];

        if (meta->source == source && req[0] == ID_DAP_QueueCommands) {
            req[0] = ID_DAP_ExecuteCommands;
        }
    }
    return true;
}
#endif

__DAP_QUEUE_CODE const uint8_t *DAP_QueueBegin(dap_queue_t *q, uint8_t *n, uint8_t **resp) {
    if (q->executed == q->produced) {
        return 0;
    }
    __asm__ volatile("" ::: "memory");
    unsigned i = q->executed & (DAP_PACKET_COUNT - 1);

    if (q->meta[i].state != SLOT_REQUEST) {
        return 0;
    }

#if (DAP_ATOMIC_COMMANDS && !APP_COMBINED_HOST)
    if (q->data[q->meta[i].buffer_index][0] == ID_DAP_QueueCommands &&
        !DAP_QueueReleaseCommands(q)) {
        return 0;
    }
#endif

    q->meta[i].state = SLOT_EXECUTING;
    *n               = q->meta[i].length;
    *resp            = q->data[q->spare];
    return q->data[q->meta[i].buffer_index];
}

__DAP_QUEUE_CODE void DAP_QueueFinish(dap_queue_t *q, uint8_t n) {
    unsigned i = q->executed & (DAP_PACKET_COUNT - 1);

    if (q->meta[i].state != SLOT_EXECUTING || n > DAP_PACKET_SIZE) {
        return;
    }
    uint8_t buffer = q->meta[i].buffer_index;

    q->meta[i].buffer_index    = q->spare;
    q->spare                   = buffer;
    q->meta[i].response_length = n;
    q->meta[i].state           = SLOT_RESPONSE;
    __asm__ volatile("" ::: "memory");
    ++q->executed;
}

const uint8_t *DAP_QueueResponseSource(dap_queue_t *q, uint8_t *n, uint8_t source) {
    __asm__ volatile("" ::: "memory");
    for (uint32_t id = q->retired; id != q->executed; ++id) {
        unsigned i = id & (DAP_PACKET_COUNT - 1);

        if (q->meta[i].source != source || q->meta[i].state == SLOT_FREE) {
            continue;
        }
        if (q->meta[i].state != SLOT_RESPONSE) {
            return 0; /* At most one DMA owner per transport, in source order. */
        }
        *n               = q->meta[i].response_length;
        q->meta[i].state = SLOT_TRANSPORT_IN;
        return q->data[q->meta[i].buffer_index];
    }
    return 0;
}

__DAP_QUEUE_CODE void DAP_QueueRetireSource(dap_queue_t *q, uint8_t source) {
    for (uint32_t id = q->retired; id != q->executed; ++id) {
        unsigned i = id & (DAP_PACKET_COUNT - 1);

        if (q->meta[i].source == source && q->meta[i].state == SLOT_TRANSPORT_IN) {
            q->meta[i].state = SLOT_FREE;
            __asm__ volatile("" ::: "memory");
            while (q->retired != q->executed &&
                   q->meta[q->retired & (DAP_PACKET_COUNT - 1)].state == SLOT_FREE) {
                ++q->retired;
            }

            return;
        }
    }
}

void DAP_QueueRetrySource(dap_queue_t *q, uint8_t source) {
    for (uint32_t id = q->retired; id != q->executed; ++id) {
        unsigned i = id & (DAP_PACKET_COUNT - 1);

        if (q->meta[i].source == source && q->meta[i].state == SLOT_TRANSPORT_IN) {
            q->meta[i].state = SLOT_RESPONSE;
            return;
        }
    }
}
