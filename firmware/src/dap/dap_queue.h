/* SPDX-License-Identifier: Apache-2.0 */

#ifndef DAP_QUEUE_H
#define DAP_QUEUE_H

#include "dap/dap_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* DMA addresses are selected by row index, never by masking the pool's base.
 * Align each packet, rather than adding a whole-window alignment gap. */
#define DAP_QUEUE_ALIGNMENT DAP_PACKET_SIZE
#define DAP_SOURCE_BULK     1
#define DAP_SOURCE_RF       2
#define DAP_SOURCE_HID      6
#define SLOT_FREE           0
#define SLOT_REQUEST        1
#define SLOT_EXECUTING      2
#define SLOT_RESPONSE       3
#define SLOT_TRANSPORT_IN   4

typedef struct {
    uint32_t request_id, received_at;
    uint8_t  source, reserved[3];
    uint8_t  state, length, response_length, buffer_index;
} dap_slot_meta_t;

typedef struct {
    /* Allocation aligns the whole object to DAP_QUEUE_ALIGNMENT; each row
     * remains packet-aligned. Metadata needs only word alignment, so avoid
     * rounding the object tail up to another unused DMA packet. */
    uint8_t           data[DAP_POOL_BUFFERS][DAP_PACKET_SIZE];
    dap_slot_meta_t   meta[DAP_PACKET_COUNT];
    volatile uint32_t produced, executed, retired;
    uint8_t           spare, bulk_rx;

#if APP_HAS_RF_DAP
    /* RF reassembly state; no profiling counters in the queue. */
    struct {
        uint8_t inputLength, fault;
    } rf;
#endif
} dap_queue_t;

_Static_assert(sizeof(dap_slot_meta_t) == 16, "slot metadata budget");

#if (RF_DAP_TARGET && DAP_PACKET_COUNT == 8)
_Static_assert(sizeof(dap_queue_t) <= 832, "RF pool state budget");
#endif

_Static_assert(offsetof(dap_queue_t, data) == 0, "DMA pool must lead the aligned allocation");
_Static_assert(DAP_QUEUE_ALIGNMENT % DAP_PACKET_SIZE == 0, "DMA row alignment");

void DAP_QueueInit(dap_queue_t *q);

bool DAP_QueuePushSource(dap_queue_t *q, const uint8_t *data, uint8_t n, uint32_t time,
                         uint8_t source);

/* USB OUT DMA or the RF reassembler owns bulk_rx. Accept publishes it without
 * copying and rotates the independent RX buffer. */

bool DAP_QueueAcceptSource(dap_queue_t *q, uint8_t n, uint32_t time, uint8_t source);

/* Input and output never overlap. Finish publishes output by swapping buffer
 * ownership; the consumed request becomes the next command's spare buffer. */

const uint8_t *DAP_QueueBegin(dap_queue_t *q, uint8_t *n, uint8_t **resp);

void DAP_QueueFinish(dap_queue_t *q, uint8_t n);

const uint8_t *DAP_QueueResponseSource(dap_queue_t *q, uint8_t *n, uint8_t source);

void DAP_QueueRetireSource(dap_queue_t *q, uint8_t source);

void DAP_QueueRetrySource(dap_queue_t *q, uint8_t source);
#endif
