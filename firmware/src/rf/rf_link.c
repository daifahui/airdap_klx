/* Stop-and-wait byte stream. A radio retry never retires unacknowledged data. */
#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "rf/rf_link.h"
#include "runtime.h"
#include <string.h>

#if !APP_COMBINED_HOST
#define __RF_LINK_REPLY_CODE __HIGH_CODE
#else
#define __RF_LINK_REPLY_CODE
#endif

__HIGH_CODE static void RF_CopyOut(const byte_ring_t *ring, uint8_t *data, unsigned n,
                                   uint32_t position) {
    unsigned offset = position & ring->mask;
    unsigned first  = ring->mask + 1 - offset;

    if (first > n) {
        first = n;
    }
    __asm__ volatile("" ::: "memory");
    RUNTIME_CopyRam(data, ring->data + offset, first);
    RUNTIME_CopyRam(data + first, ring->data, n - first);
}

/* Publish accepted payload only after both ring spans have been copied. */
__HIGH_CODE static void RF_CopyIn(byte_ring_t *ring, const uint8_t *data, unsigned n) {
    unsigned offset = ring->write & ring->mask;
    unsigned first  = ring->mask + 1 - offset;

    if (first > n) {
        first = n;
    }
    RUNTIME_CopyRam(ring->data + offset, data, first);
    RUNTIME_CopyRam(ring->data, data + first, n - first);
    __asm__ volatile("" ::: "memory");
    ring->write += n;
}

void RF_LinkInit(rf_link_t *link, uint8_t *tx, uint8_t *rx, uint8_t role, uint64_t session) {
    RING_Init(&link->tx, tx, RF_USB_QUEUE_SIZE);
    RING_Init(&link->rx, rx, RF_USB_RX_QUEUE_SIZE);
    link->session    = session;
    link->received   = 0;
    link->pending    = 0;
    link->role       = role;
    link->burst_sent = link->peer_credit = 0;
}

bool RF_LinkSubmit(rf_link_t *link, const uint8_t *data, unsigned length) {
    if (!link->session || length > (link->tx.mask + 1 - (link->tx.write - link->tx.read))) {
        return false;
    }
    RF_CopyIn(&link->tx, data, length);
    return true;
}

unsigned RF_LinkRead(rf_link_t *link, uint8_t *data, unsigned length, bool commit) {
    unsigned available = (link->rx.write - link->rx.read);

    if (length > available) {
        length = available;
    }
    if (!length) {
        return 0;
    }
    RF_CopyOut(&link->rx, data, length, link->rx.read);
    if (commit) {
        __asm__ volatile("" ::: "memory");
        link->rx.read += length;
    }
    return length;
}

/* Serialize the live link without cloning its rings or state. The retained
 * TX prefix owns the payload until ACK; position selects one reply slice. */
__HIGH_CODE static unsigned RF_LinkEncode(const rf_link_t *link, uint8_t *frame, unsigned n,
                                          uint32_t position) {
    /* Both PHY DMA banks are word aligned; their two-byte PHY prefix
     * makes the body halfword aligned. The RF builder owns that contract. */
    typedef uint16_t rf_halfword_t __MAY_ALIAS;
    rf_halfword_t                 *out = (rf_halfword_t *)frame;

    out[0]           = (uint16_t)(RF_LINK_MAGIC_0 | (RF_LINK_MAGIC_1 << 8));
    uint16_t control = (RF_LINK_VERSION << RF_LINK_CONTROL_VERSION_SHIFT) | link->role;

    out[1]       = control | (n << 8);
    uint32_t low = link->session, high = link->session >> 32;

    out[RF_LINK_SESSION_OFFSET / 2]      = low;
    out[RF_LINK_SESSION_OFFSET / 2 + 1]  = low >> 16;
    out[RF_LINK_SESSION_OFFSET / 2 + 2]  = high;
    out[RF_LINK_SESSION_OFFSET / 2 + 3]  = high >> 16;
    out[RF_LINK_SEQUENCE_OFFSET / 2]     = position;
    out[RF_LINK_SEQUENCE_OFFSET / 2 + 1] = position >> 16;
    out[RF_LINK_ACK_OFFSET / 2]          = link->received;
    out[RF_LINK_ACK_OFFSET / 2 + 1]      = link->received >> 16;
    unsigned credit                      = link->rx.mask + 1 - (link->rx.write - link->rx.read);

    if (credit > RF_REPLY_BURST * RF_USB_DATA_SIZE) {
        credit = RF_REPLY_BURST * RF_USB_DATA_SIZE;
    }
    out[RF_LINK_CREDIT_OFFSET / 2] = credit;
    RF_CopyOut(&link->tx, frame + RF_LINK_HEADER_SIZE, n, position);
    return RF_LINK_HEADER_SIZE + n;
}

__HIGH_CODE unsigned RF_LinkBuild(rf_link_t *link, uint8_t *frame) {
    unsigned n = (link->tx.write - link->tx.read);

    if (!link->pending) {
#if APP_COMBINED_HOST
        link->pending = n > RF_HOST_FRAME_LIMIT ? RF_HOST_FRAME_LIMIT : n;
#else
        link->pending = n > RF_USB_DATA_SIZE ? RF_USB_DATA_SIZE : n;
#endif
    }
    n = link->pending;
    return RF_LinkEncode(link, frame, n, link->tx.read);
}

/* Metadata-only view: two RX DMA banks already provide the physical frame
 * storage. Unconfirmed payload stays in the single existing reliable ring. */

__RF_LINK_REPLY_CODE unsigned RF_LinkBuildReply(rf_link_t *link, uint8_t *frame) {
    unsigned limit = link->peer_credit;

    if (limit > RF_REPLY_BURST * RF_USB_DATA_SIZE) {
        limit = RF_REPLY_BURST * RF_USB_DATA_SIZE;
    }
    if (!link->pending) {
        unsigned queued = link->tx.write - link->tx.read;

        link->pending = queued < limit ? queued : limit;
    }
    if (limit > link->pending) {
        limit = link->pending;
    }
    unsigned n = limit - link->burst_sent;

    if (n > RF_USB_DATA_SIZE) {
        n = RF_USB_DATA_SIZE;
    }
    unsigned length = RF_LinkEncode(link, frame, n, link->tx.read + link->burst_sent);

    if (link->burst_sent + n < limit) {
        frame[RF_LINK_CONTROL_OFFSET] |= RF_LINK_MORE;
    }
    return length;
}

__HIGH_CODE enum rf_link_result RF_LinkReceive(rf_link_t *link, const uint8_t *frame,
                                               unsigned length) {
    /* The radio driver must reject hardware CRC errors before calling us. */
    if (length < RF_LINK_HEADER_SIZE || length > RF_LINK_FRAME_SIZE ||
        frame[RF_LINK_LENGTH_OFFSET] > RF_USB_DATA_SIZE ||
        length != RF_LINK_HEADER_SIZE + frame[RF_LINK_LENGTH_OFFSET] ||
        frame[0] != RF_LINK_MAGIC_0 || frame[1] != RF_LINK_MAGIC_1) {
        return RF_LINK_BAD_FRAME;
    }
    uint8_t role = frame[RF_LINK_CONTROL_OFFSET] & RF_LINK_CONTROL_ROLE_MASK;

    role &= RF_LINK_BURST_ROLE_MASK;
    if ((frame[RF_LINK_CONTROL_OFFSET] & RF_LINK_INPUT_MORE) ||
        ((frame[RF_LINK_CONTROL_OFFSET] & RF_LINK_MORE) && role != RF_ROLE_TARGET)) {
        return RF_LINK_BAD_FRAME;
    }
    unsigned credit =
        frame[RF_LINK_CREDIT_OFFSET] | ((unsigned)frame[RF_LINK_CREDIT_HIGH_OFFSET] << 8);
    if (credit > RF_USB_QUEUE_SIZE) {
        return RF_LINK_BAD_FRAME;
    }
    if ((frame[RF_LINK_CONTROL_OFFSET] >> RF_LINK_CONTROL_VERSION_SHIFT) != RF_LINK_VERSION ||
        (role != RF_ROLE_HOST && role != RF_ROLE_TARGET)) {
        return RF_LINK_BAD_FRAME;
    }

    /* The PHY prefix is two bytes in word-aligned DMA banks. Read only
     * aligned halfwords; a 32-bit frame load would be misaligned. */
    typedef uint16_t rf_halfword_t __MAY_ALIAS;
    const rf_halfword_t           *in      = (const rf_halfword_t *)frame;
    uint64_t                       session = in[RF_LINK_SESSION_OFFSET / 2] |
                       ((uint64_t)in[RF_LINK_SESSION_OFFSET / 2 + 1] << 16) |
                       ((uint64_t)in[RF_LINK_SESSION_OFFSET / 2 + 2] << 32) |
                       ((uint64_t)in[RF_LINK_SESSION_OFFSET / 2 + 3] << 48);
    if (!link->session || session != link->session || role == link->role) {
        return RF_LINK_STALE;
    }
    uint32_t ack = in[RF_LINK_ACK_OFFSET / 2] | ((uint32_t)in[RF_LINK_ACK_OFFSET / 2 + 1] << 16);
    unsigned n   = frame[RF_LINK_LENGTH_OFFSET];
    uint32_t sequence =
        in[RF_LINK_SEQUENCE_OFFSET / 2] | ((uint32_t)in[RF_LINK_SEQUENCE_OFFSET / 2 + 1] << 16);
    bool duplicate = n && sequence + n == link->received;

    /* A lost cumulative ACK may repeat any complete slice of the retained
     * turn, not just its last frame. Partial overlaps remain invalid. */
    duplicate = n && (uint32_t)(link->received - sequence) <= RF_USB_QUEUE_SIZE &&
                (uint32_t)(link->received - sequence) >= n;
    if (!duplicate && sequence != link->received) {
        return RF_LINK_STALE;
    }
    unsigned acknowledged = ack - link->tx.read;

    if (acknowledged > link->pending) {
        return RF_LINK_STALE;
    }
    link->tx.read      = ack;
    link->pending     -= acknowledged;
    link->peer_credit  = credit;
    if (duplicate) {
        return RF_LINK_DUPLICATE;
    }
    if (n > (link->rx.mask + 1 - (link->rx.write - link->rx.read))) {
        return RF_LINK_FULL;
    }
    RF_CopyIn(&link->rx, frame + RF_LINK_HEADER_SIZE, n);
    link->received += n;
    return RF_LINK_OK;
}
