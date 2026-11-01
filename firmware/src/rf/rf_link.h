#ifndef RF_LINK_H
#define RF_LINK_H

#include "app_config.h"
#include "ring.h"

/* Reliable v6 stream frame: compact header plus receive credit. */
#define RF_LINK_MAGIC_0               'R'
#define RF_LINK_MAGIC_1               'U'
#define RF_LINK_CONTROL_OFFSET        2U
#define RF_LINK_SESSION_OFFSET        4U
#define RF_LINK_SEQUENCE_OFFSET       12U
#define RF_LINK_ACK_OFFSET            16U
#define RF_LINK_CREDIT_OFFSET         20U
#define RF_LINK_CREDIT_HIGH_OFFSET    21U
#define RF_LINK_CONTROL_VERSION_SHIFT 4U
#define RF_LINK_CONTROL_ROLE_MASK     0x0FU
#define RF_LINK_BURST_ROLE_MASK       0x03U
#define RF_LINK_HEADER_SIZE           22U
#define RF_LINK_VERSION               6U
#define RF_LINK_LENGTH_OFFSET         3U
#define RF_LINK_FRAME_SIZE            (RF_LINK_HEADER_SIZE + RF_USB_DATA_SIZE)
#define RF_LINK_PROPOSE               1U
#define RF_LINK_READY                 2U
#define RF_LINK_COMMIT                4U
#define RF_LINK_MORE                  4U
#define RF_LINK_INPUT_MORE            8U

/* Byte offsets and acknowledgements are independent in the two directions. */
typedef struct {
    byte_ring_t tx, rx;
    uint64_t    session;
    uint32_t    received;
    uint16_t    pending, burst_sent;
    uint16_t    peer_credit;
    uint8_t     role;
} rf_link_t;

enum rf_link_result {
    RF_LINK_OK,
    RF_LINK_DUPLICATE,
    RF_LINK_FULL,
    RF_LINK_BAD_FRAME,
    RF_LINK_STALE
};

void RF_LinkInit(rf_link_t *link, uint8_t *tx, uint8_t *rx, uint8_t role, uint64_t session);

bool RF_LinkSubmit(rf_link_t *link, const uint8_t *data, unsigned length);

unsigned RF_LinkRead(rf_link_t *link, uint8_t *data, unsigned length, bool commit);

/* PHY frame bodies passed to build/receive must be at least halfword
 * aligned: word-aligned DMA bank + the two-byte hardware prefix. */

unsigned RF_LinkBuild(rf_link_t *link, uint8_t *frame);

/* Build one slice without retiring the original ring. burst_sent advances
 * only on successful PHY TX_FINISH, and resets at each new HOST request. */

unsigned RF_LinkBuildReply(rf_link_t *link, uint8_t *frame);

enum rf_link_result RF_LinkReceive(rf_link_t *link, const uint8_t *frame, unsigned length);
#endif
