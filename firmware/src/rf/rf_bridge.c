/* USB bulk <-> reliable raw RF. The coordinator polls; responses carry reverse data. */
#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "ch572/ch572rf.h"
#include "rf/rf_bridge.h"
#include "rf/rf_control_protocol.h"
#include "rf/core/rf_basic_2m.h"
#include "bsp/board.h"
#include "mode/mode_image.h"

#if APP_COMBINED_HOST
#include "mode/app_mode.h"
#endif

#include "runtime.h"

#if APP_HAS_RF_DAP
#include "rf/rf_dap.h"
#endif

#if APP_COMBINED_TARGET
#include "rf/rf_uart.h"
#endif

#include "rf/rf_policy.h"
#include <string.h>

#if RF_SPACE_RAM
#define __RF_BRIDGE_SPACE_CODE __HIGH_CODE
#else
#define __RF_BRIDGE_SPACE_CODE
#endif

#if RF_POLL_RAM
#define __RF_BRIDGE_POLL_CODE __HIGH_CODE
#else
#define __RF_BRIDGE_POLL_CODE
#endif

#if RF_UART_RADIO_TX_RAM
#define __RF_TRANSMIT_CODE __HIGH_CODE
#else
#define __RF_TRANSMIT_CODE __UNUSED
#endif

#define RF_binding         MODE_mailbox.binding
#define DISCOVERY_OFF      0
#define DISCOVERY_SCAN     1
#define DISCOVERY_SELECTED 2
#define DISCOVERY_CONFIRM  3
#define DISCOVERY_SWITCH   4
#define DISCOVERY_PAIRED   5
#define DISCOVERY_PROBE    1U
#define DISCOVERY_OFFER    2U
#define DISCOVERY_COMMIT   3U
#define DISCOVERY_ACK      4U
#define DISCOVERY_ADVERT   5U
#define DISCOVERY_READY    6U
#define RF_CHANNEL_CONTROL 0x80U
#define RF_SLEEP_REQUEST   0x81U
#define RF_SLEEP_ACK       0x82U

/* Queue bookkeeping runs in Flash; the RF IRQ/PHY path remains in SRAM. */
static uint8_t RF_txData[RF_USB_QUEUE_SIZE] __ALIGNED(RF_USB_QUEUE_SIZE)
    __SECTION(MODE_RF_SECTION("queue"));
static uint8_t RF_rxData[RF_USB_RX_QUEUE_SIZE] __ALIGNED(RF_USB_QUEUE_SIZE)
    __SECTION(MODE_RF_SECTION("queue"));
static uint8_t RF_rxDma[2][RF_DMA_SIZE] __ALIGNED(4);

#if RF_DMA_SHARED_TX
static uint8_t *RF_txDma = RF_rxDma[0];
#else
static uint8_t RF_txDma[RF_DMA_SIZE] __ALIGNED(4);
#endif

static rf_link_t         RF_link;
static rfipRx_t          RF_rxParameters;
static rfipTx_t          RF_txParameters;
static volatile uint32_t RF_radioEvents;
static volatile uint8_t  RF_rxReady, RF_rxActive, RF_resetRequested;

#if !APP_HAS_RF_UART
#error "RF bridge requires a HOST or TARGET role bank"
#endif

static volatile uint8_t RF_phyBusy, RF_rxRearmed;

#if APP_COMBINED_HOST
extern volatile uint32_t *gptrLLEReg;
#endif

static uint64_t      RF_requestedSession;
static volatile bool RF_stopped = true;
static bool          RF_initialized;

/* SCAN/CONFIRM carry discovery only; candidate sessions cannot enable business queues. */
static volatile uint8_t RF_discoveryPhase;
static uint32_t         RF_discoveryContact, RF_discoveryPairedAt;
static bool             RF_discoveryDiscarded;

#if APP_COMBINED_HOST
static bool RF_suspendWaiting, RF_suspendAck;
#else
static bool RF_sleepPower;
#endif

enum RF_radioState {
    RADIO_LISTEN,
    RADIO_TX,
    RADIO_WAIT,
    RADIO_IDLE
};
static volatile enum RF_radioState RF_radioState;
static volatile uint8_t            CLOCK_paused;
static uint32_t                    RF_deadline, RF_nextPoll;
static uint8_t                     RF_replyMore, RF_replySize, RF_replyFrame;

#if RF_DMA_SHARED_TX
static uint8_t RF_replyPrepared;
#endif

#if (APP_COMBINED_HOST && RF_RETRY_DELAY_US != 1000)
static uint32_t RF_retryAt, RF_retryStarted;
static uint8_t  RF_retryWaiting;
#endif

#if RF_TX_PREFILL
static uint32_t RF_prefillSequence, RF_prefillLength;
#endif

#if (APP_COMBINED_HOST && RF_DAP_POLL_US)
static uint32_t RF_dapPollAt;
static uint8_t  RF_dapPollWait;
#endif

static uint32_t RF_txPackets, RF_rxPackets, RF_retries, RF_crcErrors, RF_frameErrors, RF_duplicates;
static uint32_t RF_apiErrors, RF_dmaOverflows, USB_txBytes, USB_rxBytes, RF_backpressure;
static uint32_t RF_hardwareTimeouts, RF_lastTimeoutId, RF_txTimeouts;
static rf_policy_t RF_policy;
static uint8_t     RF_channelSettle;

/* Optional owner hook; the USB implementation is strong and uses no RF RAM. */
bool RF_BridgeTransportReset(void) __WEAK;

#if (APP_COMBINED_HOST)
/* Complete a PHY transaction without interleaving register sequences. An RX
 * callback interrupted inside the API publishes its bank; the outer release
 * freezes that bank before foreground can read it. IRQs stay enabled. */
static __HIGH_CODE void RF_ReleasePhy(uint8_t busy) {
    RF_phyBusy = busy;
    if (!busy && RF_rxReady == RF_rxActive + 1U) {
        RF_phyBusy = 1;
        RFRole_Stop();
        RF_phyBusy = 0;
    }
}
#endif

/* The other existing RX DMA bank is idle throughout a continued TX turn.
 * TX_FINISH cannot rearm RX while MORE is set; foreground alone starts TX. */
void RF_BridgeReplyPrefill(void) {
#if RF_DMA_SHARED_TX
    if (RF_link.role != RF_ROLE_TARGET || RF_radioState != RADIO_TX || !RF_replyMore ||
        RF_replyPrepared || RF_radioEvents || RF_rxReady || RF_stopped || CLOCK_paused ||
        RF_resetRequested || BSP_stopRequested) {
        return;
    }
    unsigned  bank = RF_txDma == RF_rxDma[0] ? 1U : 0U;
    rf_link_t next = RF_link;

    next.burst_sent   += RF_replySize;
    RF_rxDma[bank][0]  = 0x55;
    RF_rxDma[bank][1]  = RF_LinkBuildReply(&next, RF_rxDma[bank] + 2);
    RF_replyPrepared   = bank + 1;
#endif
}

/* The callback leaves the next TX to foreground. It only avoids arming RX
 * while this TARGET still owns an explicitly granted response turn. */
static __NO_INLINE unsigned RF_BridgeReplyDone(void) {
    if (!RF_replyFrame) {
        return false;
    }
    RF_link.burst_sent += RF_replySize;

    if (!RF_replyMore) {
    }
    return RF_replyMore;
}

static __NO_INLINE bool RF_BridgeReplyContinue(const uint8_t *frame) {
    if (frame[4] & RF_LINK_MORE) {
        RF_radioState = RADIO_WAIT;
        RF_deadline   = BSP_ms + RF_USB_TIMEOUT_MS;
        return true;
    }

    return false;
}

#if (RF_TX_PREFILL) && (RF_PREFILL_EXTEND)
/* Foreground/USB producers only append while these bytes are retained. Read
 * write once, append the missing suffix, then freeze this transmission's size.
 * The radio has not acquired TX DMA at this point. */
static __NO_INLINE unsigned RF_BridgeExtendFrame(unsigned length) {
    unsigned available = RF_link.tx.write - RF_link.tx.read;

    if (available > RF_HOST_FRAME_LIMIT) {
        available = RF_HOST_FRAME_LIMIT;
    }
    if (!RF_link.pending && available > length) {
        unsigned offset = (RF_link.tx.read + length) & RF_link.tx.mask;
        unsigned extra  = available - length;
        unsigned first  = RF_link.tx.mask + 1 - offset;

        if (first > extra) {
            first = extra;
        }
        RUNTIME_CopyRam(RF_txDma + RF_PHY_DATA_OFFSET + RF_LINK_HEADER_SIZE + length,
                        RF_link.tx.data + offset, first);
        if (extra != first) {
            RUNTIME_CopyRam(RF_txDma + RF_PHY_DATA_OFFSET + RF_LINK_HEADER_SIZE + length + first,
                            RF_link.tx.data, extra - first);
        }

        RF_prefillLength                    = available;
        RF_txDma[2 + RF_LINK_LENGTH_OFFSET] = available;
        return available;
    }
    return length;
}
#endif

#if (RF_TX_PREFILL)
/* TX DMA is idle after TX_FINISH; unacknowledged bytes remain in the reliable
 * ring. Build a speculative next frame from metadata only, without retiring
 * bytes or making the real link consider them transmitted. */
void RF_BridgePrefill(void) {
    if (RF_stopped || CLOCK_paused || RF_resetRequested || BSP_stopRequested ||
        RF_radioState != RADIO_WAIT || RF_rxReady || RF_discoveryPhase != DISCOVERY_PAIRED) {
        return;
    }
    uint32_t sequence = RF_link.tx.read + RF_link.pending;
    unsigned length   = RF_link.tx.write - sequence;

    if (length > RF_HOST_FRAME_LIMIT) {
        length = RF_HOST_FRAME_LIMIT;
    }
    if (!length || (RF_prefillSequence == sequence && RF_prefillLength == length)) {
        return;
    }
    rf_link_t next = RF_link;

    next.tx.read = sequence;
    next.pending = 0;
    RF_LinkBuild(&next, RF_txDma + RF_PHY_DATA_OFFSET);
    RF_prefillSequence = sequence;
    RF_prefillLength   = length;
}
#endif

#if (RF_TX_PREFILL)
/* A cache miss reconstructs a frame in Flash. Retried bytes are still owned
 * by link.tx; discovery/control invalidate this cache before using TX DMA. */
static __NO_INLINE unsigned RF_BridgeFrameBuild(void) {
    unsigned length = RF_LinkBuild(&RF_link, RF_txDma + RF_PHY_DATA_OFFSET);

    RF_prefillSequence = RF_link.tx.read;
    RF_prefillLength   = RF_link.pending;
    return length;
}
#endif

#if (RF_TX_PREFILL)
static __NO_INLINE unsigned RF_BridgeFrame(void) {
    unsigned length = RF_prefillLength;

    /* A prepared partial frame remains valid when the producer appends more
     * bytes. Its frozen length becomes pending only at submission; no queued
     * bytes are retired until the peer acknowledges them. */
    if (length && RF_prefillSequence == RF_link.tx.read) {
#if RF_PREFILL_EXTEND
        length = RF_BridgeExtendFrame(length);
#endif

        RF_link.pending = length;
        uint32_t ack    = RF_link.received;

        /* DMA starts on a 4-byte boundary; the ACK at offset 18 is halfword
         * aligned. may_alias permits these stores into byte-array storage. */
        typedef uint16_t RF_ackHalfword __MAY_ALIAS;
        RF_ackHalfword                 *ackWords = (RF_ackHalfword *)(RF_txDma + 18);
        ackWords[0]                              = ack;
        ackWords[1]                              = ack >> 16;
        unsigned credit = RF_link.rx.mask + 1 - (RF_link.rx.write - RF_link.rx.read);

        if (credit > RF_REPLY_BURST * RF_USB_DATA_SIZE) {
            credit = RF_REPLY_BURST * RF_USB_DATA_SIZE;
        }
        ackWords[2] = credit;

        return RF_LINK_HEADER_SIZE + length;
    }
    return RF_BridgeFrameBuild();
}
#endif

static __NO_INLINE bool RF_ApplyPolicyChannel(void) {
    uint8_t channel = RF_PolicyChannel(&RF_policy);

    if (RF_rxParameters.frequency == channel) {
        return false;
    }
    RF_phyBusy = 1;
    RFRole_Stop();
    uint32_t irq = BSP_IrqSave();

    RF_radioEvents = 0;
    RF_rxReady     = 0;
    RF_rxRearmed   = 0;
    RF_phyBusy     = 0;
    BSP_IrqRestore(irq);
    RF_rxParameters.frequency = RF_txParameters.frequency = channel;
    RF_channelSettle                                      = 1;
    RF_radioState = RF_link.role == RF_ROLE_HOST ? RADIO_IDLE : RADIO_LISTEN;
    RF_nextPoll   = BSP_ms;
    return true;
}

__HIGH_CODE bool RF_BridgeActive(void) {
    return !RF_stopped && !RF_resetRequested && RF_link.session != 0 &&
           (RF_discoveryPhase == DISCOVERY_OFF || RF_discoveryPhase == DISCOVERY_PAIRED);
}

bool RF_BridgeEventPending(void) {
    return RF_radioEvents || RF_rxReady;
}

__RF_BRIDGE_SPACE_CODE unsigned RF_BridgeSpace(void) {
    return RF_BridgeActive() ? (RF_link.tx.mask + 1 - (RF_link.tx.write - RF_link.tx.read)) : 0;
}

/* Sole foreground producer. Reset may be requested in IRQ, but only the
 * foreground reset branch changes the queue generation; publish both spans
 * before returning ownership. No temporary packet buffer or IRQ mask. */
bool RF_BridgeSubmitPair(const uint8_t *first, unsigned firstLength, const uint8_t *second,
                         unsigned secondLength) {
    unsigned total = firstLength + secondLength;

    if (!RF_BridgeActive() || total > RF_link.tx.mask + 1 - (RF_link.tx.write - RF_link.tx.read)) {
        return false;
    }
    RF_LinkSubmit(&RF_link, first, firstLength);
    if (secondLength) {
        RF_LinkSubmit(&RF_link, second, secondLength);
    }
    USB_txBytes += total;

#if APP_COMBINED_HOST
    /* Fresh work need not wait for an earlier idle-poll deadline. A live
     * request/reply exchange retains its radio ownership and timeout. */
    if (RF_link.role == RF_ROLE_HOST && RF_radioState == RADIO_IDLE) {
        RF_nextPoll = BSP_ms;

#if RF_DAP_POLL_US
        RF_dapPollWait = 0;
#endif
    }
#endif

    return true;
}

unsigned RF_BridgeRead(uint8_t *data, unsigned n) {
    return RF_LinkRead(&RF_link, data, n, false);
}

bool RF_BridgeReadAfter(uint8_t *data, unsigned prefix, unsigned length) {
    if (RF_link.rx.write - RF_link.rx.read < prefix + length) {
        return false;
    }
    unsigned offset = (RF_link.rx.read + prefix) & RF_link.rx.mask;
    unsigned first  = RF_link.rx.mask + 1 - offset;

    if (first > length) {
        first = length;
    }
    RUNTIME_CopyRam(data, RF_link.rx.data + offset, first);
    if (first != length) {
        RUNTIME_CopyRam(data + first, RF_link.rx.data, length - first);
    }
    return true;
}

void RF_BridgeRetire(unsigned n) {
    /* USB IN owns these bytes until the host acknowledges the DMA packet. */
    __asm__ volatile("" ::: "memory");
    RF_link.rx.read += n;
    USB_rxBytes     += n;
}

void RF_BridgeFlush(void) {
    RF_discoveryPhase   = DISCOVERY_OFF;
    RF_requestedSession = 0;
    RF_resetRequested   = 1;
}

static __HIGH_CODE bool RF_ReceiveDma(void) {
    uint8_t busy = RF_phyBusy;

    RF_phyBusy             = 1;
    RF_rxActive           ^= 1;
    RF_rxParameters.rxDMA  = (uintptr_t)RF_rxDma[RF_rxActive];
    bool armed             = RFIP_SetRx2m(&RF_rxParameters) == 0;

    if (!armed) {
        ++RF_apiErrors;
    }

#if APP_COMBINED_HOST
    RF_ReleasePhy(busy);
#else
    RF_phyBusy = busy;
#endif

    return armed;
}

__HIGH_CODE static void RF_RadioCallback(rfRole_States_t events, uint8_t id) {
    if (RF_stopped || CLOCK_paused || RF_resetRequested ||
        (!RF_link.session && RF_discoveryPhase == DISCOVERY_OFF)) {
        return;
    }
    if ((events & RF_STATE_TX_FINISH) && !RF_phyBusy && RF_radioState != RADIO_TX) {
        events &= ~RF_STATE_TX_FINISH;
    }

    /* Never publish a failed hardware CRC as a frame, even if events combine. */
    if (events & RF_STATE_RX_CRCERR) {
        ++RF_crcErrors;
        events &= ~RF_STATE_RX;
    }
    if (events & RF_STATE_RX) {
#if APP_COMBINED_HOST
        /* Basic RX can receive another frame into the same hardware DMA.
         * Freeze the completed bank before publishing it to foreground.
         * A foreground PHY call must finish first; its exit / the IRQ tail
         * applies the same hold before foreground can consume the frame. */
        if (!RF_phyBusy)
            RFRole_Stop();
#endif

        if (RF_rxReady) {
            ++RF_dmaOverflows;
        } else {
            RF_rxReady = RF_rxActive + 1;
        }
    }
    if (events & RF_STATE_TIMEOUT) {
        ++RF_hardwareTimeouts;
        RF_lastTimeoutId = id;
    }
    RF_radioEvents |= events;

    /* The TX callback can re-arm RF receive DMA. Only arm DMA here;
     * typed parsing, ACKs and queue ownership remain in foreground. A PHY
     * call interrupted in foreground must finish before another may start. */
    if ((events & RF_STATE_TX_FINISH) && RF_radioState == RADIO_TX && !RF_phyBusy &&
        !RF_rxRearmed && !RF_replyMore && !BSP_stopRequested && BSP_workReady && !RF_rxReady &&
        (RF_discoveryPhase == DISCOVERY_OFF || RF_discoveryPhase == DISCOVERY_PAIRED)) {
        RF_rxRearmed = RF_ReceiveDma();
    }
}

__INTERRUPT __HIGH_CODE void BB_IRQHandler(void) {
    BB_LibIRQHandler();
}

__INTERRUPT __HIGH_CODE void LLE_IRQHandler(void) {
#if APP_COMBINED_HOST
    /* RF Basic emits TX and RX as separate callbacks from one IRQ snapshot.
     * Do not change RX DMA inside the first callback while that same batch
     * still owns an unread RX completion/CRC result. Pure TX keeps early RX. */
    uint8_t phyBusy = RF_phyBusy;

    if (gptrLLEReg[2] & gptrLLEReg[3] & 1U) {
        RF_phyBusy = 1;
    }
#endif

    LLE_LibIRQHandler();

#if APP_COMBINED_HOST
    RF_ReleasePhy(phyBusy);
#endif
}

static __UNUSED void RF_Receive(void) {
    if (!RF_ReceiveDma()) {
        RF_radioState = RADIO_IDLE;
        RF_nextPoll   = BSP_ms + 1;
    }
}

/* UID is six factory bytes plus the existing little-endian checksum.
 * FNV-1a maps it to a stable 32-bit Basic-2M AA; full UID remains on wire,
 * since a 48->32 mapping cannot guarantee global collision freedom. */
static void RF_DiscoveryUid(uint8_t uid[8]) {
    for (unsigned i = 0; i < 8; ++i) {
        uint8_t hi = BSP_serial[2 * i], lo = BSP_serial[2 * i + 1];

        hi     = hi <= '9' ? hi - (unsigned)'0' : (hi & ~32U) - 'A' + 10;
        lo     = lo <= '9' ? lo - (unsigned)'0' : (lo & ~32U) - 'A' + 10;
        uid[i] = (hi << 4) | lo;
    }
}

static uint32_t RF_DiscoveryAddress(const uint8_t uid[8]) {
    uint32_t h   = 2166136261U;
    unsigned sum = 0, any = 0;

    for (unsigned i = 0; i < 6; ++i) {
        h    = (h ^ uid[i]) * 16777619U;
        any |= uid[i];
        sum += (unsigned)uid[i] << (8 * (i & 1));
    }
    if (!any || (uint16_t)sum != (uid[6] | (uid[7] << 8)))
        return 0;
    h ^= 0x52464141U; /* RF AA domain, independent of the session nonce. */
    if (!h || h == UINT32_MAX || h == RF_PAIR_ACCESS_ADDRESS)
        h ^= 0x9E3779B9U;
    return h;
}

static uint32_t RF_DiscoveryPrivate(void) {
    return RF_link.role == RF_ROLE_TARGET ? RF_binding.own_aa : RF_binding.peer_aa;
}

/* Caller owns a stopped PHY or a completed control TX. */
static void RF_DiscoveryTune(bool private) {
    RF_rxParameters.accessAddress = RF_txParameters.accessAddress =
        private ? RF_DiscoveryPrivate() : RF_PAIR_ACCESS_ADDRESS;
}

/* Rare control encoding stays in Flash; keep the qualified data TX in SRAM. */
static __NO_INLINE void RF_ChannelBuild(uint8_t *p) {
    memset(p, 0, RF_CHANNEL_SIZE);
    p[0]                         = RF_CONTROL_MAGIC_0;
    p[1]                         = RF_CHANNEL_MAGIC_1;
    p[RF_CONTROL_VERSION_OFFSET] = RF_CONTROL_VERSION;
    p[RF_CONTROL_ROLE_OFFSET]    = RF_link.role;
    for (unsigned i = 0; i < RF_CONTROL_SESSION_SIZE; ++i) {
        p[RF_CONTROL_SESSION_OFFSET + i] = RF_link.session >> (8 * i);
    }
    p[RF_CHANNEL_PHASE_OFFSET]  = RF_policy.phase;
    p[RF_CHANNEL_NUMBER_OFFSET] = RF_PolicyTarget(&RF_policy);
    p[RF_CHANNEL_EPOCH_OFFSET]  = RF_policy.epoch;
}

static __NO_INLINE void RF_ChannelTxSample(void) {
    RF_PolicyTx(&RF_policy);
}

static __NO_INLINE void RF_ChannelDataSample(void) {
    RF_PolicyReceive(&RF_policy, 0, RF_rxParameters.frequency, RF_policy.epoch, BSP_ms);
}

static __NO_INLINE bool RF_ChannelPoll(void) {
    RF_PolicyPoll(&RF_policy, BSP_ms);
    return RF_ApplyPolicyChannel();
}

static __NO_INLINE bool RF_ChannelTxDone(void) {
    RF_PolicyTxDone(&RF_policy);
    return RF_ApplyPolicyChannel();
}

/* Discovery and channel handshakes are cold control paths. Their packet
 * serialization does not need to occupy the normal SRAM launch function. */
static __NO_INLINE bool RF_BridgeControlFrame(uint8_t discovery) {
#if APP_COMBINED_HOST
    if ((discovery == DISCOVERY_PROBE || discovery == DISCOVERY_COMMIT ||
         discovery == DISCOVERY_READY) &&
        !MODE_AppModeRfCommitAllowed()) {
        RF_Receive(); /* Candidate only: no COMMIT or business before USB ACK. */
        return false;
    }
#endif

#if RF_TX_PREFILL
    RF_prefillLength = 0;
#endif

    RF_replyMore = RF_replyFrame = 0;

#if RF_DMA_SHARED_TX
    RF_replyPrepared = 0;
#endif

    if (discovery == RF_SLEEP_REQUEST || discovery == RF_SLEEP_ACK) {
        uint8_t *p = RF_txDma + RF_PHY_DATA_OFFSET;

        memset(p, 0, RF_SLEEP_SIZE);
        p[0]                         = RF_CONTROL_MAGIC_0;
        p[1]                         = RF_SLEEP_MAGIC_1;
        p[RF_CONTROL_VERSION_OFFSET] = RF_CONTROL_VERSION;
        p[RF_CONTROL_ROLE_OFFSET]    = RF_link.role;
        for (unsigned i = 0; i < RF_CONTROL_SESSION_SIZE; ++i)
            p[RF_CONTROL_SESSION_OFFSET + i] = RF_link.session >> (8 * i);
        p[RF_SLEEP_KIND_OFFSET] =
            discovery == RF_SLEEP_REQUEST ? RF_SLEEP_KIND_REQUEST : RF_SLEEP_KIND_ACK;
        RF_txDma[RF_PHY_LENGTH_OFFSET] = RF_SLEEP_SIZE;
        return true;
    }
    if (discovery == RF_CHANNEL_CONTROL) {
        RF_ChannelBuild(RF_txDma + RF_PHY_DATA_OFFSET);
        RF_txDma[RF_PHY_LENGTH_OFFSET] = RF_CHANNEL_SIZE;
        return true;
    }
    uint8_t *p = RF_txDma + RF_PHY_DATA_OFFSET;

    memset(p, 0, RF_DISCOVERY_SIZE);
    p[0]                         = RF_CONTROL_MAGIC_0;
    p[1]                         = RF_DISCOVERY_MAGIC_1;
    p[RF_CONTROL_VERSION_OFFSET] = RF_DISCOVERY_VERSION;
    p[RF_CONTROL_ROLE_OFFSET]    = RF_link.role;
    for (unsigned i = 0; i < RF_CONTROL_SESSION_SIZE; ++i) {
        p[RF_CONTROL_SESSION_OFFSET + i] = RF_link.session >> (8 * i);
        p[RF_DISCOVERY_NONCE_OFFSET + i] = RF_requestedSession >> (8 * i);
    }
    p[RF_DISCOVERY_KIND_OFFSET] = discovery;
    if (RF_link.role == RF_ROLE_TARGET)
        RF_DiscoveryUid(p + RF_DISCOVERY_UID_OFFSET);
    else
        memcpy(p + RF_DISCOVERY_UID_OFFSET, RF_binding.peer_uid, 8);
    uint32_t aa = RF_DiscoveryPrivate();

    for (unsigned i = 0; i < 4; ++i)
        p[RF_DISCOVERY_AA_OFFSET + i] = aa >> (8 * i);
    RF_DiscoveryTune(discovery == DISCOVERY_READY);
    RF_txDma[RF_PHY_LENGTH_OFFSET] = RF_DISCOVERY_SIZE;
    return true;
}

static __NO_INLINE unsigned RF_BridgeReplyPrepare(void) {
    if (RF_link.role == RF_ROLE_TARGET) {
        RF_replyFrame = 1;
        unsigned length;

#if RF_DMA_SHARED_TX
        if (RF_replyPrepared) {
            RF_txDma         = RF_rxDma[RF_replyPrepared - 1U];
            RF_replyPrepared = 0;

            length = RF_txDma[RF_PHY_LENGTH_OFFSET];
        } else
#endif

        {
            length = RF_LinkBuildReply(&RF_link, RF_txDma + RF_PHY_DATA_OFFSET);
        }
        RF_replySize = RF_txDma[2 + RF_LINK_LENGTH_OFFSET];
        RF_replyMore = (RF_txDma[4] & RF_LINK_MORE) != 0;
        return length;
    }
    RF_replyFrame = RF_replyMore = 0;

#if RF_TX_PREFILL
    return RF_BridgeFrame();
#else
    return RF_LinkBuild(&RF_link, RF_txDma + RF_PHY_DATA_OFFSET);
#endif
}

static __RF_TRANSMIT_CODE void RF_Transmit(uint8_t discovery) {
    RF_phyBusy = 1;
    RFRole_Stop();

#if RF_DMA_SHARED_TX
    /* Stop has cancelled the active RX lease. Preserve any completed frame
     * still waiting for foreground; otherwise use the other RX bank. RX may
     * resume in this storage only after TX_FINISH (or an explicit stop).
     * rx_ready can only publish zero -> active bank during this handoff;
     * foreground is its sole consumer, so no extra IRQ mask is necessary. */
    uint8_t reserved = RF_rxReady;

    RF_txDma = RF_rxDma[(reserved ? reserved - 1U : RF_rxActive) ^ 1U];
#endif

    RF_txDma[RF_PHY_TAG_OFFSET] = RF_PHY_TX_TAG;
    if (!discovery && RF_discoveryPhase && RF_discoveryPhase != DISCOVERY_PAIRED) {
        discovery = RF_discoveryPhase == DISCOVERY_SELECTED ? DISCOVERY_PROBE
                    : RF_discoveryPhase == DISCOVERY_SWITCH ? DISCOVERY_READY
                                                            : DISCOVERY_COMMIT;
    }
    if (!discovery && RF_policy.phase && RF_BridgeActive()) {
        discovery = RF_CHANNEL_CONTROL;
    }
    if (discovery) {
        if (!RF_BridgeControlFrame(discovery)) {
#if APP_COMBINED_HOST
            RF_ReleasePhy(0);
#else
            RF_phyBusy = 0;
#endif

            return;
        }
    } else {
        RF_txDma[RF_PHY_LENGTH_OFFSET] = RF_BridgeReplyPrepare();
    }
    RF_txParameters.txDMA = (uintptr_t)RF_txDma;

    /* Allow the first transmission a full PLL settling interval. Later
     * transfers use the configured role settling interval. */
    RF_txParameters.waitTime =
        (RF_txPackets || RF_USB_TX_WAIT_US >= 80U ? RF_USB_TX_WAIT_US : 80U) * 2U;
    if (RF_channelSettle) {
        RF_txParameters.waitTime = 160;
    }
    if (RFIP_StartTx2m(&RF_txParameters)) {
        ++RF_apiErrors;
        RF_radioState = RADIO_IDLE;
        RF_nextPoll   = BSP_ms + 1;
        RF_Receive();
    } else {
        RF_channelSettle = 0;
        if (!discovery && RF_policy.sample < RF_POLICY_CHANNELS) {
            RF_ChannelTxSample();
        }
        ++RF_txPackets;
        RF_radioState = RADIO_TX;
        RF_deadline   = BSP_ms + RF_USB_TIMEOUT_MS;
    }

#if APP_COMBINED_HOST
    RF_ReleasePhy(0);
#else
    RF_phyBusy = 0;
#endif
}

/* Control frames use no stream space and cannot acknowledge/consume bytes.
 * Both peers retain their v2 offsets, pending fragment and application queues. */
static __NO_INLINE bool RF_ChannelReceive(const uint8_t *p, unsigned n) {
    if (n != RF_CHANNEL_SIZE || p[RF_CONTROL_VERSION_OFFSET] != RF_CONTROL_VERSION ||
        p[RF_CONTROL_ROLE_OFFSET] !=
            (RF_link.role == RF_ROLE_HOST ? RF_ROLE_TARGET : RF_ROLE_HOST) ||
        p[RF_CHANNEL_NUMBER_OFFSET] < RF_CHANNEL_FIRST ||
        p[RF_CHANNEL_NUMBER_OFFSET] > RF_CHANNEL_LAST) {
        return false;
    }
    for (unsigned i = 0; i < RF_CONTROL_SESSION_SIZE; ++i) {
        if (p[RF_CONTROL_SESSION_OFFSET + i] != (uint8_t)(RF_link.session >> (8 * i)) ||
            p[RF_CHANNEL_RESERVED_TAIL_OFFSET + i]) {
            return false;
        }
    }
    if (p[RF_CHANNEL_RESERVED_OFFSET] ||
        (RF_link.role == RF_ROLE_HOST
             ? (p[RF_CHANNEL_PHASE_OFFSET] != 0 && p[RF_CHANNEL_PHASE_OFFSET] != RF_LINK_READY) ||
                   p[RF_CHANNEL_EPOCH_OFFSET] != RF_policy.epoch
             : (p[RF_CHANNEL_PHASE_OFFSET] != RF_LINK_PROPOSE &&
                p[RF_CHANNEL_PHASE_OFFSET] != RF_LINK_COMMIT))) {
        return false;
    }
    if (RF_link.role == RF_ROLE_TARGET && p[RF_CHANNEL_PHASE_OFFSET] == RF_LINK_COMMIT &&
        (p[RF_CHANNEL_EPOCH_OFFSET] != RF_policy.epoch ||
         p[RF_CHANNEL_NUMBER_OFFSET] != RF_rxParameters.frequency)) {
        return false;
    }
    if (RF_link.role == RF_ROLE_HOST &&
        (p[RF_CHANNEL_PHASE_OFFSET] == RF_LINK_READY
             ? RF_policy.phase != RF_LINK_PROPOSE ||
                   p[RF_CHANNEL_NUMBER_OFFSET] != RF_PolicyTarget(&RF_policy)
             : p[RF_CHANNEL_NUMBER_OFFSET] != RF_rxParameters.frequency ||
                   (RF_policy.phase != 0 && RF_policy.phase != RF_LINK_COMMIT))) {
        return false;
    }
    if (RF_link.role == RF_ROLE_TARGET && p[RF_CHANNEL_PHASE_OFFSET] == RF_LINK_PROPOSE &&
        (int8_t)(p[RF_CHANNEL_EPOCH_OFFSET] - RF_policy.epoch) <= 0 &&
        !(p[RF_CHANNEL_EPOCH_OFFSET] == RF_policy.epoch && RF_policy.phase == RF_LINK_READY &&
          p[RF_CHANNEL_NUMBER_OFFSET] == RF_PolicyTarget(&RF_policy))) {
        return false;
    }
    RF_PolicyReceive(&RF_policy, p[RF_CHANNEL_PHASE_OFFSET], p[RF_CHANNEL_NUMBER_OFFSET],
                     p[RF_CHANNEL_EPOCH_OFFSET], BSP_ms);
    RF_discoveryContact = BSP_ms;
    if (RF_link.role == RF_ROLE_TARGET) {
        RF_Transmit(RF_CHANNEL_CONTROL);
    } else {
        RF_ApplyPolicyChannel();
        RF_radioState = RADIO_IDLE;
        RF_nextPoll   = BSP_ms;
        RF_Receive();
    }
    return true;
}

/* TARGET first advertises its attempt nonce on the public PHY address.
 * HOST selects that nonce; its fresh nonce becomes the shared session.
 * These nonces are neither fixed UID nor authenticated identity. */
bool RF_BridgeDiscover(uint64_t nonce) {
    if (RF_stopped) {
        return false;
    }
    uint32_t irq = BSP_IrqSave();

    if (!nonce) {
        if (RF_discoveryPhase && RF_discoveryPhase != DISCOVERY_PAIRED) {
            if (RF_BridgeTransportReset && !RF_BridgeTransportReset()) {
                BSP_IrqRestore(irq);
                return false;
            }
            RF_BridgeFlush();
        } else {
            RF_discoveryPhase = DISCOVERY_OFF;
        }
        BSP_IrqRestore(irq);
        return true;
    }
    if (RF_BridgeTransportReset && !RF_BridgeTransportReset()) {
        BSP_IrqRestore(irq);
        return false;
    }
    if (RF_discoveryPhase >= DISCOVERY_SELECTED &&
        (uint32_t)(BSP_ms - RF_discoveryContact) >= RF_DISCOVERY_LEASE_MS &&
        ((RF_link.tx.write - RF_link.tx.read) || (RF_link.rx.write - RF_link.rx.read))) {
        RF_discoveryDiscarded = true;
    }
    RFRole_Stop();

#if APP_COMBINED_HOST
    MODE_AppModeForgetCandidate(); /* a new RF nonce invalidates an old OFFER */
#endif

    RF_LinkInit(&RF_link, RF_txData, RF_rxData, RF_link.role,
                RF_link.role == RF_ROLE_HOST ? nonce : 0);
    RF_requestedSession = RF_link.role == RF_ROLE_TARGET ? nonce : 0;
    RF_discoveryPhase   = DISCOVERY_SCAN;
    RF_DiscoveryTune(false);
    RF_discoveryPairedAt = 0;
    RF_discoveryContact  = BSP_ms;
    RF_resetRequested = RF_radioEvents = RF_rxReady = RF_rxActive = 0;
    RF_phyBusy = RF_rxRearmed = 0;
    RF_deadline               = 0;
    RF_nextPoll               = BSP_ms;
    RF_radioState             = RADIO_IDLE;
    RF_PolicyInit(&RF_policy, RF_link.role, BSP_ms);
    RF_channelSettle          = 1;
    RF_rxParameters.frequency = RF_txParameters.frequency = RF_DEFAULT_CHANNEL;
    PFIC_EnableIRQ(BLEB_IRQn);
    PFIC_EnableIRQ(BLEL_IRQn);
    BSP_IrqRestore(irq);
    RF_Receive();
    return true;
}

bool RF_BridgeDiscovering(void) {
    return RF_discoveryPhase != DISCOVERY_OFF;
}

uint32_t RF_BridgePairedAt(void) {
    return RF_discoveryPairedAt;
}

bool RF_BridgePaired(void) {
    return RF_discoveryPhase == DISCOVERY_PAIRED && RF_BridgeActive();
}

static void RF_DiscoveryComplete(void) {
#if APP_COMBINED_HOST
    bool connected = RF_discoveryPhase != DISCOVERY_PAIRED;
#endif

#if APP_COMBINED_TARGET
    if (MODE_peerSleeping && RF_discoveryPhase != DISCOVERY_PAIRED) {
        MODE_peerSleeping = false;
        BSP_TargetPower(RF_sleepPower);
    }
#endif

    if (RF_discoveryPhase != DISCOVERY_PAIRED) {
        RF_discoveryContact  = BSP_ms;
        RF_discoveryPairedAt = BSP_ms;
    }
    if (RF_discoveryPhase != DISCOVERY_PAIRED) {
        /* A target may have listened for minutes; begin policy timeouts now. */
        RF_PolicyInit(&RF_policy, RF_link.role, BSP_ms);
    }
    RF_discoveryPhase = DISCOVERY_PAIRED;
    RF_binding.bound  = 1;

#if APP_COMBINED_HOST
    /* OUT was NAKed while unpaired. NAK tokens do not enter the USB ISR;
     * publish readiness to its endpoint owner instead of waiting for EP0. */
    if (connected)
        PFIC_SetPendingIRQ(USB_IRQn);
#endif
}

static __NO_INLINE bool RF_DiscoveryReceive(const uint8_t *p, unsigned n) {
    if (!RF_discoveryPhase || n != RF_DISCOVERY_SIZE || p[0] != RF_CONTROL_MAGIC_0 ||
        p[1] != RF_DISCOVERY_MAGIC_1 || p[RF_CONTROL_VERSION_OFFSET] != RF_DISCOVERY_VERSION ||
        p[RF_CONTROL_ROLE_OFFSET] !=
            (RF_link.role == RF_ROLE_HOST ? RF_ROLE_TARGET : RF_ROLE_HOST) ||
        p[RF_DISCOVERY_RESERVED_OFFSET] || p[RF_DISCOVERY_RESERVED_OFFSET + 1] ||
        p[RF_DISCOVERY_RESERVED_OFFSET + 2]) {
        return false;
    }
    uint64_t session = 0, challenge = 0;

    for (unsigned i = 0; i < RF_CONTROL_SESSION_SIZE; i++) {
        session   |= (uint64_t)p[RF_CONTROL_SESSION_OFFSET + i] << (8 * i);
        challenge |= (uint64_t)p[RF_DISCOVERY_NONCE_OFFSET + i] << (8 * i);
    }
    uint32_t aa = RF_DiscoveryAddress(p + RF_DISCOVERY_UID_OFFSET), wireAa = 0;

    for (unsigned i = 0; i < 4; ++i)
        wireAa |= (uint32_t)p[RF_DISCOVERY_AA_OFFSET + i] << (8 * i);
    if (!aa || wireAa != aa)
        return false;
    if (RF_link.role == RF_ROLE_TARGET) {
        uint8_t uid[8];

        RF_DiscoveryUid(uid);
        if (memcmp(uid, p + RF_DISCOVERY_UID_OFFSET, RF_DISCOVERY_UID_SIZE) ||
            aa != RF_binding.own_aa)
            return false;
    } else if ((RF_discoveryPhase != DISCOVERY_SCAN || RF_binding.bound) &&
               memcmp(RF_binding.peer_uid, p + RF_DISCOVERY_UID_OFFSET, RF_DISCOVERY_UID_SIZE))
        return false;
    if (RF_link.role == RF_ROLE_HOST) {
        if (p[RF_DISCOVERY_KIND_OFFSET] == DISCOVERY_ADVERT && !session && challenge) {
            if (RF_discoveryPhase == DISCOVERY_SCAN) {
#if APP_COMBINED_HOST
                app_mode_status_t mode;

                MODE_AppModeSnapshot(&mode);
                if (mode.phase == APP_MODE_PAIRING && !MODE_AppModeNotePeerMatched(BSP_ms))
                    return true;
#endif

                memcpy(RF_binding.peer_uid, p + RF_DISCOVERY_UID_OFFSET, RF_DISCOVERY_UID_SIZE);
                RF_binding.peer_aa  = aa;
                RF_requestedSession = challenge;
                RF_discoveryPhase   = DISCOVERY_SELECTED;
                RF_discoveryContact = BSP_ms;
            }
            if (RF_discoveryPhase == DISCOVERY_SELECTED && challenge == RF_requestedSession)
                RF_Transmit(DISCOVERY_PROBE);
            else
                RF_Receive();
            return true;
        }
        if (session != RF_link.session || !challenge) {
            return false;
        }
        if (p[RF_DISCOVERY_KIND_OFFSET] == DISCOVERY_OFFER &&
            RF_discoveryPhase == DISCOVERY_SELECTED && challenge == RF_requestedSession) {
            RF_discoveryPhase   = DISCOVERY_CONFIRM;
            RF_discoveryContact = BSP_ms;
            RF_Transmit(DISCOVERY_COMMIT);
            return true;
        }
        if (p[RF_DISCOVERY_KIND_OFFSET] == DISCOVERY_ACK &&
            RF_discoveryPhase == DISCOVERY_CONFIRM && challenge == RF_requestedSession) {
#if APP_COMBINED_HOST
            if (!MODE_AppModeRfCommitAllowed())
                return true;
#endif

            RF_discoveryPhase = DISCOVERY_SWITCH;
            RF_Transmit(DISCOVERY_READY);
            return true;
        }
        if (p[RF_DISCOVERY_KIND_OFFSET] == DISCOVERY_READY &&
            RF_discoveryPhase == DISCOVERY_SWITCH && challenge == RF_requestedSession) {
#if APP_COMBINED_HOST
            if (!MODE_AppModeRfCommitAllowed())
                return true;
#endif

            RF_DiscoveryComplete();
            RF_radioState = RADIO_IDLE;
            RF_nextPoll   = BSP_ms;
            RF_Receive();
            return true;
        }
    } else {
        if (!session)
            return false;
        if (p[RF_DISCOVERY_KIND_OFFSET] == DISCOVERY_PROBE && challenge == RF_requestedSession &&
            (RF_discoveryPhase == DISCOVERY_SCAN ||
             (RF_discoveryPhase == DISCOVERY_CONFIRM && session == RF_link.session))) {
            if (RF_discoveryPhase == DISCOVERY_SCAN)
                RF_discoveryContact = BSP_ms;
            RF_link.session   = session;
            RF_discoveryPhase = DISCOVERY_CONFIRM;
            RF_Transmit(DISCOVERY_OFFER);
            return true;
        }
        if (p[RF_DISCOVERY_KIND_OFFSET] == DISCOVERY_COMMIT &&
            (RF_discoveryPhase == DISCOVERY_CONFIRM || RF_discoveryPhase == DISCOVERY_SWITCH) &&
            session == RF_link.session && challenge == RF_requestedSession) {
            RF_discoveryPhase = DISCOVERY_SWITCH;
            RF_Transmit(DISCOVERY_ACK); /* Last public-AA TX. */
            return true;
        }
        if (p[RF_DISCOVERY_KIND_OFFSET] == DISCOVERY_READY &&
            RF_discoveryPhase >= DISCOVERY_SWITCH && session == RF_link.session &&
            challenge == RF_requestedSession) {
            RF_DiscoveryComplete(); /* Duplicate READY does not renew the lease. */
            RF_Transmit(DISCOVERY_READY);
            return true;
        }
    }
    return false;
}

bool RF_BridgeStart(uint8_t role) {
    /* Library initialization/calibration retains the qualified 100 MHz
     * baseline. Business UART clocks are committed only after this start. */
    if (BSP_clockHz != CPU_CLOCK) {
        if (!BSP_ClockAcquire()) {
            return false;
        }
        bool restored = BSP_ClockApply(CPU_CLOCK);

        BSP_ClockRelease();
        if (!restored) {
            return false;
        }
    }
    uint32_t irq = BSP_IrqSave();

    RF_BridgeStop();

    /* Reuse the existing buffers. No prior session, ACK or queued command may
     * survive a role change, including requests published while stopped. */
    RF_LinkInit(&RF_link, RF_txData, RF_rxData, 0, 0);
    RF_requestedSession = 0;
    RF_discoveryPhase   = DISCOVERY_OFF;
    RF_discoveryContact = RF_discoveryPairedAt = 0;
    RF_discoveryDiscarded                      = false;
    RF_resetRequested = RF_rxActive = 0;
    RF_deadline = RF_nextPoll = 0;

#if (APP_COMBINED_HOST && RF_DAP_POLL_US)
    RF_dapPollWait = 0;
#endif

    RF_radioState = RADIO_IDLE;
    memset(&RF_rxParameters, 0, sizeof(RF_rxParameters));
    memset(&RF_txParameters, 0, sizeof(RF_txParameters));
    memset(RF_rxDma, 0, sizeof(RF_rxDma));

#if RF_DMA_SHARED_TX
    RF_txDma = RF_rxDma[0];
#else
    memset(RF_txDma, 0, sizeof(RF_txDma));
#endif

    RF_txPackets = RF_rxPackets = RF_retries = RF_crcErrors = RF_frameErrors = RF_duplicates = 0;
    RF_apiErrors = RF_dmaOverflows = USB_txBytes = USB_rxBytes = RF_backpressure = 0;
    RF_hardwareTimeouts = RF_lastTimeoutId = RF_txTimeouts = 0;
    RF_replyMore = RF_replySize = RF_replyFrame = 0;

#if RF_DMA_SHARED_TX
    RF_replyPrepared = 0;
#endif

#if (APP_COMBINED_HOST && RF_RETRY_DELAY_US != 1000)
    RF_retryWaiting = 0;
#endif

#if RF_TX_PREFILL
    RF_prefillLength = RF_prefillSequence = 0;
#endif

    memset(&RF_policy, 0, sizeof(RF_policy));
    RF_channelSettle = 1;
    if (role != RF_ROLE_HOST && role != RF_ROLE_TARGET) {
        BSP_IrqRestore(irq);
        return false;
    }
    BSP_ReleaseTarget();
    RF_LinkInit(&RF_link, RF_txData, RF_rxData, role, 0);
    RF_PolicyInit(&RF_policy, RF_link.role, BSP_ms);
    CH572_SAFE_ACCESS_ENABLE();
    R32_MISC_CTRL = (R32_MISC_CTRL & ~(0x3FU << 24)) | (0xEU << 24);
    CH572_SAFE_ACCESS_DISABLE();
    rfRoleConfig_t configuration = {RF_RadioCallback, RF_STATE_RX | RF_STATE_RX_CRCERR |
                                                          RF_STATE_TX_FINISH | RF_STATE_TIMEOUT};
    RF_initialized               = true;
    if (RFRole_BasicInit(&configuration)) {
        RF_BridgeStop();
        RF_link.role = 0;
        ++RF_apiErrors;
        BSP_IrqRestore(irq);
        return false;
    }
    uint8_t ownUid[8];

    RF_DiscoveryUid(ownUid);
    RF_binding.own_aa             = RF_DiscoveryAddress(ownUid);
    RF_rxParameters.accessAddress = RF_txParameters.accessAddress = RF_PAIR_ACCESS_ADDRESS;
    RF_rxParameters.crcInit = RF_txParameters.crcInit = RF_BASIC2M_CRC_INIT;
    RF_rxParameters.crcPoly                           = RF_txParameters.crcPoly =
        RF_BASIC2M_CRC_POLY; /* WCH RF Basic PHY_2M encoding. */
    RF_rxParameters.frequency = RF_txParameters.frequency = RF_DEFAULT_CHANNEL;
    RF_rxParameters.properties = RF_txParameters.properties = PHY_MODE_PHY_2M;

    /* Use the driver's full receive limit; the parser enforces our smaller
     * frame cap. This also sizes the driver's address-match receive timer. */
    RF_rxParameters.rxMaxLen   = RF_MAX_DATA_LEN;
    RF_txParameters.txLen      = RF_LINK_FRAME_SIZE;
    RF_txParameters.txPowerVal = RF_TX_POWER_CODE;
    RF_txParameters.waitTime   = RF_USB_TX_WAIT_US * 2U;
    PFIC_ClearPendingIRQ(BLEB_IRQn);
    PFIC_ClearPendingIRQ(BLEL_IRQn);
    RF_stopped = false;
    PFIC_EnableIRQ(BLEB_IRQn);
    PFIC_EnableIRQ(BLEL_IRQn);
    RF_radioState = RADIO_IDLE;
    BSP_IrqRestore(irq);
    return true;
}

void RF_BridgeStop(void) {
    uint32_t irq = BSP_IrqSave();

#if APP_COMBINED_TARGET
    if (MODE_peerSleeping) {
        MODE_peerSleeping = false;
        BSP_TargetPower(RF_sleepPower);
    }
#endif

    RF_stopped   = true;
    CLOCK_paused = 0;
    RF_phyBusy = RF_rxRearmed = 0;
    PFIC_DisableIRQ(BLEB_IRQn);
    PFIC_DisableIRQ(BLEL_IRQn);
    if (RF_initialized) {
        RFRole_Stop();
    }
    PFIC_ClearPendingIRQ(BLEB_IRQn);
    PFIC_ClearPendingIRQ(BLEL_IRQn);
    RF_radioEvents = 0;
    RF_rxReady     = 0;
    BSP_IrqRestore(irq);
}

#if APP_COMBINED_HOST
bool RF_BridgePrepareSuspend(void) {
    if (!RF_BridgePaired())
        return false;
    RF_suspendWaiting = true;
    RF_suspendAck     = false;
    uint32_t start    = BSP_TimeUs();

    do {
        RF_BridgePoll();
    } while (!RF_suspendAck && (uint32_t)(BSP_TimeUs() - start) < 5000U);
    RF_suspendWaiting = false;
    return RF_suspendAck;
}
#endif

/* This control cannot ACK stream bytes or execute DAP. Only the current
 * private session may suspend a peer; duplicates only repeat the ACK. */
static __NO_INLINE bool RF_SleepReceive(const uint8_t *p, unsigned n) {
    if (n != RF_SLEEP_SIZE || p[RF_CONTROL_VERSION_OFFSET] != RF_CONTROL_VERSION ||
        p[RF_CONTROL_ROLE_OFFSET] !=
            (RF_link.role == RF_ROLE_HOST ? RF_ROLE_TARGET : RF_ROLE_HOST) ||
        p[RF_SLEEP_RESERVED_OFFSET] || p[RF_SLEEP_RESERVED_OFFSET + 1] ||
        p[RF_SLEEP_RESERVED_OFFSET + 2])
        return false;
    for (unsigned i = 0; i < RF_CONTROL_SESSION_SIZE; ++i)
        if (p[RF_CONTROL_SESSION_OFFSET + i] != (uint8_t)(RF_link.session >> (8 * i)))
            return false;

#if APP_COMBINED_HOST
    if (p[RF_SLEEP_KIND_OFFSET] != RF_SLEEP_KIND_ACK || !RF_suspendWaiting)
        return false;
    RF_suspendAck = true;
    RF_radioState = RADIO_IDLE;
    RF_Receive();
#else
    if (p[RF_SLEEP_KIND_OFFSET] != RF_SLEEP_KIND_REQUEST)
        return false;
    if (!MODE_peerSleeping) {
        RF_sleepPower     = BSP_TargetPowerEnabled();
        MODE_peerSleeping = true;
        UART_Stop();
        BSP_ReleaseTarget();
        RF_DapReset();
        BSP_TargetPower(false);
    }
    RF_Transmit(RF_SLEEP_ACK);
#endif

    return true;
}

bool RF_BridgeClockPause(void) {
    uint32_t irq = BSP_IrqSave();

    if (CLOCK_paused || RF_stopped || !RF_initialized) {
        BSP_IrqRestore(irq);
        return true;
    }
    if (RF_radioState == RADIO_TX || RF_radioEvents || RF_rxReady || RF_resetRequested ||
        RF_phyBusy) {
        BSP_IrqRestore(irq);
        return false;
    }
    CLOCK_paused = 1;
    RF_phyBusy   = 1;
    PFIC_DisableIRQ(BLEB_IRQn);
    PFIC_DisableIRQ(BLEL_IRQn);
    BSP_IrqRestore(irq);

    /* An in-flight, unconfirmed RX frame may be retried by the peer. No
     * reliable queue pointers, sequence numbers or ACKs are modified. */
    RFRole_Stop();
    irq          = BSP_IrqSave();
    RF_rxActive  = 0;
    RF_rxRearmed = 0;
    PFIC_ClearPendingIRQ(BLEB_IRQn);
    PFIC_ClearPendingIRQ(BLEL_IRQn);
    BSP_IrqRestore(irq);
    return true;
}

void RF_BridgeClockResume(void) {
    uint32_t irq = BSP_IrqSave();

    if (!CLOCK_paused) {
        BSP_IrqRestore(irq);
        return;
    }
    CLOCK_paused = 0;
    RF_phyBusy   = 0;
    if (!RF_stopped && !BSP_stopRequested) {
        PFIC_EnableIRQ(BLEB_IRQn);
        PFIC_EnableIRQ(BLEL_IRQn);
        if (RF_link.session || RF_discoveryPhase != DISCOVERY_OFF) {
            RF_Receive();
        }
    }
    BSP_IrqRestore(irq);
}

/* Session replacement is cold. The normal radio pass only tests its
 * deadline/flag; keep cleanup and initialization out of the SRAM hot loop. */
static __NO_INLINE bool RF_BridgeMaintain(void) {
    if (!RF_resetRequested && RF_discoveryPhase == DISCOVERY_SCAN && !RF_radioEvents &&
        !RF_rxReady && RF_radioState != RADIO_TX) {
        if (RF_link.role == RF_ROLE_TARGET && (int32_t)(BSP_ms - RF_nextPoll) >= 0) {
            RF_Transmit(DISCOVERY_ADVERT);
            RF_nextPoll = BSP_ms + 20U + ((uint32_t)RF_requestedSession ^ BSP_ms) % 13U;
            return false;
        }
        if (RF_link.role == RF_ROLE_HOST) {
            if (RF_radioState == RADIO_IDLE) {
                RF_radioState = RADIO_LISTEN;
                RF_Receive();
            }
            return false; /* silent scan; never send a blind PROBE */
        }
    }

    /* A lost final ACK must not reserve the target forever. Duplicate RD
     * confirmation never renews this lease; only validated RU traffic does.
     * The host also abandons an unacknowledged COMMIT, so a retry can complete
     * inside the controller's two-second enumeration window. */
    uint32_t leaseIrq = BSP_IrqSave();

    /* A completed DMA frame must be parsed before a delayed foreground
     * declares its lease expired (notably after a long DAP command). */
    if (!RF_resetRequested && !RF_rxReady && RF_discoveryPhase >= DISCOVERY_SELECTED &&
        (uint32_t)(BSP_ms - RF_discoveryContact) >= RF_DISCOVERY_LEASE_MS

#if APP_COMBINED_HOST
        && (RF_discoveryPhase == DISCOVERY_PAIRED || MODE_AppModeRfCommitAllowed())
#endif

    ) {
        uint64_t nonce = (RF_link.role == RF_ROLE_HOST ? RF_link.session : RF_requestedSession) + 1;

        RF_BridgeDiscover(nonce ? nonce : 1);
        BSP_IrqRestore(leaseIrq);
        return false;
    }
    BSP_IrqRestore(leaseIrq);
    if (RF_resetRequested) {
        RFRole_Stop();
        uint32_t irq = BSP_IrqSave();

#if APP_COMBINED_HOST
        if (!RF_BridgeTransportReset()) {
            BSP_IrqRestore(irq);
            return false;
        }
#endif

#if APP_COMBINED_TARGET
        RF_UartReset();
#endif

        RF_LinkInit(&RF_link, RF_txData, RF_rxData, RF_link.role, RF_requestedSession);

#if (APP_COMBINED_HOST && RF_RETRY_DELAY_US != 1000)
        RF_retryWaiting = 0;
#endif

#if RF_TX_PREFILL
        RF_prefillLength = 0;
#endif

#if APP_HAS_RF_DAP
        /* This is the actual foreground queue-generation boundary, including
         * explicit USB sessions. An ISR may request reset during DAP execution;
         * never clear or reuse its response buffer inside that ISR. */
        RF_DapReset();
#endif

        RF_PolicyInit(&RF_policy, RF_link.role, BSP_ms);
        RF_rxParameters.frequency = RF_txParameters.frequency = RF_PolicyChannel(&RF_policy);
        RF_channelSettle                                      = 1;
        RF_radioEvents                                        = 0;
        RF_rxReady = RF_resetRequested = 0;
        RF_phyBusy = RF_rxRearmed = 0;
        RF_radioState             = RADIO_IDLE;
        RF_nextPoll               = BSP_ms;
        BSP_IrqRestore(irq);
        if (RF_link.session) {
            RF_Receive();
        }
    }
    return true;
}

static __NO_INLINE void RF_DiscoveryRetry(void) {
    if (RF_link.role != RF_ROLE_HOST)
        return;
    if (RF_discoveryPhase == DISCOVERY_CONFIRM)
        RF_discoveryPhase = DISCOVERY_SWITCH;
    else if (RF_discoveryPhase == DISCOVERY_SWITCH)
        RF_discoveryPhase = DISCOVERY_CONFIRM;
}

#if APP_COMBINED_HOST
/* Rare recovery stays in Flash; the successful SRAM exchange path only
 * checks its deadline. The reliable ring still owns unacknowledged bytes. */
static __NO_INLINE void RF_BridgeRetry(void) {
    RF_txTimeouts += RF_radioState == RADIO_TX;
    RF_phyBusy     = 1;
    RFRole_Stop();

    /* Missing COMMIT leaves TARGET public; missing ACK leaves it private.
     * Alternate both rendezvous addresses, never guess from a data frame. */
    RF_DiscoveryRetry();
    ++RF_retries;
    if (RF_BridgeActive()) {
        RF_PolicyTimeout(&RF_policy, BSP_ms);
    }
    RF_radioState = RADIO_IDLE;
    RF_nextPoll   = BSP_ms + (RF_RETRY_DELAY_US == 1000 ? 1U : 0U);
    RF_Receive();

#if RF_RETRY_DELAY_US != 1000
    RF_retryStarted = SysTick->CNTL;
    RF_retryAt      = RF_retryStarted + RF_RETRY_DELAY_US * (CPU_CLOCK / 1000000U);
    RF_retryWaiting = 1;
#endif

    RF_ReleasePhy(0);
}
#endif

__RF_BRIDGE_POLL_CODE void RF_BridgePoll(void) {
    if (RF_stopped || CLOCK_paused) {
        return;
    }
    if ((RF_resetRequested || RF_discoveryPhase == DISCOVERY_SCAN ||
         (RF_discoveryPhase >= DISCOVERY_SELECTED &&
          (uint32_t)(BSP_ms - RF_discoveryContact) >= RF_DISCOVERY_LEASE_MS)) &&
        !RF_BridgeMaintain()) {
        return;
    }
    if (!RF_link.session && RF_discoveryPhase == DISCOVERY_OFF) {
        return;
    }
    uint32_t irq     = BSP_IrqSave();
    uint32_t events  = RF_radioEvents;
    uint8_t  ready   = RF_rxReady;
    uint8_t  rearmed = RF_rxRearmed;

    RF_rxRearmed = 0;

    /* Leave TX in the same handoff that consumes its completion/marker, so
     * a late duplicate callback cannot arm a second DMA bank in this gap. */
    if (events & RF_STATE_TX_FINISH) {
        RF_radioState = RF_link.role == RF_ROLE_HOST ? RADIO_WAIT : RADIO_LISTEN;
    }
    RF_radioEvents = 0;
    RF_rxReady     = 0;
    BSP_IrqRestore(irq);
    if (events & RF_STATE_TX_FINISH) {
#if APP_COMBINED_TARGET
        /* The sleep ACK has left DMA. Listen/advertise on the public address
         * for the waking HOST's new session, retaining the TARGET/power hold.
         * No old private-session traffic is needed to start recovery. */
        if (MODE_peerSleeping && RF_discoveryPhase == DISCOVERY_PAIRED) {
            RF_BridgeDiscover(BSP_DiscoveryNonce());
            return;
        }
#endif

        if (RF_link.role == RF_ROLE_TARGET && RF_discoveryPhase == DISCOVERY_SWITCH) {
            /* ACK DMA has completed; now listen at the advertised AA. */
            RF_DiscoveryTune(true);
            rearmed = 0;
        }
        unsigned replyAction = RF_BridgeReplyDone();

        if (replyAction) {
            if (replyAction == 1) {
                RF_Transmit(0);
            }
            return;
        }
        if (RF_policy.phase == RF_LINK_READY && RF_ChannelTxDone()) {
            rearmed = 0;
        }
        RF_radioState = RF_link.role == RF_ROLE_HOST ? RADIO_WAIT : RADIO_LISTEN;
        RF_deadline   = BSP_ms + RF_USB_TIMEOUT_MS;
        if (!rearmed) {
            RF_Receive();
        }
    }
    bool accepted = false;

    if (ready) {
        uint8_t *p = RF_rxDma[ready - 1];

        if (p[RF_PHY_LENGTH_OFFSET] >= RF_PHY_DATA_OFFSET &&
            p[RF_PHY_DATA_OFFSET] == RF_CONTROL_MAGIC_0 &&
            p[RF_PHY_DATA_OFFSET + 1] == RF_SLEEP_MAGIC_1 && RF_BridgeActive()) {
            ++RF_rxPackets;
            if (!RF_SleepReceive(p + RF_PHY_DATA_OFFSET, p[RF_PHY_LENGTH_OFFSET])) {
                ++RF_frameErrors;
                RF_Receive();
            }
            return;
        }
        if (p[RF_PHY_LENGTH_OFFSET] >= RF_PHY_DATA_OFFSET &&
            p[RF_PHY_DATA_OFFSET] == RF_CONTROL_MAGIC_0 &&
            p[RF_PHY_DATA_OFFSET + 1] == RF_CHANNEL_MAGIC_1 && RF_BridgeActive()) {
            ++RF_rxPackets;
            if (!RF_ChannelReceive(p + RF_PHY_DATA_OFFSET, p[RF_PHY_LENGTH_OFFSET])) {
                ++RF_frameErrors;
                RF_Receive();
            }
            return;
        }
        if ((p[RF_PHY_LENGTH_OFFSET] >= RF_PHY_DATA_OFFSET &&
             p[RF_PHY_DATA_OFFSET] == RF_CONTROL_MAGIC_0 &&
             p[RF_PHY_DATA_OFFSET + 1] == RF_DISCOVERY_MAGIC_1) ||
            !RF_BridgeActive()) {
            ++RF_rxPackets;
            uint32_t savedIrq = BSP_IrqSave();

            if (RF_resetRequested ||
                !RF_DiscoveryReceive(p + RF_PHY_DATA_OFFSET, p[RF_PHY_LENGTH_OFFSET])) {
                ++RF_frameErrors;
                if (!RF_resetRequested) {
                    RF_Receive();
                }
            }
            BSP_IrqRestore(savedIrq);
            return;
        }
        ++RF_rxPackets;
        enum rf_link_result result =
            RF_LinkReceive(&RF_link, p + RF_PHY_DATA_OFFSET, p[RF_PHY_LENGTH_OFFSET]);
        accepted = result == RF_LINK_OK || result == RF_LINK_DUPLICATE || result == RF_LINK_FULL;
        if (result == RF_LINK_DUPLICATE) {
            ++RF_duplicates;
        } else if (result == RF_LINK_FULL) {
            ++RF_backpressure;
        } else if (result != RF_LINK_OK) {
            ++RF_frameErrors;
        }
        if (accepted) {
            if (RF_discoveryPhase == DISCOVERY_PAIRED) {
                RF_discoveryContact = BSP_ms;
            }
            if (!RF_policy.phase) {
                RF_policy.last_rx  = BSP_ms;
                RF_policy.failures = 0;
                if (RF_policy.sample < RF_POLICY_CHANNELS) {
                    RF_ChannelDataSample();
                }
            }
            if (RF_link.role == RF_ROLE_TARGET) {
                RF_link.burst_sent = 0;

#if RF_DMA_SHARED_TX
                RF_replyPrepared = 0;
#endif

                RF_Transmit(0);
            } else {
                if (RF_BridgeReplyContinue(p)) {
                    RF_Receive();
                    return;
                }
                RF_radioState = RADIO_IDLE;

#if (APP_COMBINED_HOST && RF_RETRY_DELAY_US != 1000)
                RF_retryWaiting = 0;
#endif

#if (APP_COMBINED_HOST && RF_DAP_POLL_US)
                RF_dapPollWait = 0;
#endif

#if (APP_COMBINED_HOST && RF_DAP_POLL_US)
                unsigned idle = !(RF_link.pending || p[2 + RF_LINK_LENGTH_OFFSET] ||
                                  RF_link.tx.write != RF_link.tx.read);
                RF_nextPoll   = BSP_ms + idle * RF_USB_IDLE_MS;
#else
                RF_nextPoll = BSP_ms + (RF_link.pending || p[2 + RF_LINK_LENGTH_OFFSET] ||
                                                (RF_link.tx.write - RF_link.tx.read)
                                            ? 0
                                            : RF_USB_IDLE_MS);
#endif

#if (APP_COMBINED_HOST && RF_DAP_POLL_US)
                if (idle && DAP_queue.produced != DAP_queue.executed) {
                    RF_dapPollAt   = SysTick->CNTL + RF_DAP_POLL_US * (CPU_CLOCK / 1000000U);
                    RF_dapPollWait = 1;
                    RF_nextPoll    = BSP_ms;
                }
#endif

                RF_Receive();
            }
        } else {
            RF_Receive();
        }
    } else if (events & (RF_STATE_RX_CRCERR | RF_STATE_TIMEOUT)) {
        RF_Receive();
    }
    if ((RF_radioState == RADIO_TX || RF_radioState == RADIO_WAIT) &&
        (int32_t)(BSP_ms - RF_deadline) >= 0) {
#if APP_COMBINED_HOST
        RF_BridgeRetry();
#else
        RF_txTimeouts += RF_radioState == RADIO_TX;
        RF_phyBusy     = 1;
        RFRole_Stop();
        RF_DiscoveryRetry();
        ++RF_retries;
        if (RF_BridgeActive()) {
            RF_PolicyTimeout(&RF_policy, BSP_ms);
        }
        RF_radioState = RADIO_IDLE;
        RF_nextPoll   = BSP_ms + 1;
        RF_Receive();
        RF_phyBusy = 0;
#endif
    }
    if (RF_radioState != RADIO_TX &&
        (RF_policy.sample != UINT8_MAX - 1U || RF_policy.phase ||
         (uint32_t)(BSP_ms - RF_policy.last_rx) >= RF_POLICY_RECOVER_MS) &&
        RF_BridgeActive() && RF_ChannelPoll()) {
        RF_Receive();
    }

#if APP_COMBINED_HOST
    if (RF_suspendWaiting) {
        if (RF_radioState == RADIO_IDLE && !RF_phyBusy)
            RF_Transmit(RF_SLEEP_REQUEST);
        return;
    }
#endif

    if (RF_radioState == RADIO_IDLE && (int32_t)(BSP_ms - RF_nextPoll) >= 0) {
#if (APP_COMBINED_HOST && RF_RETRY_DELAY_US != 1000)
        if (RF_retryWaiting && (int32_t)(SysTick->CNTL - RF_retryAt) < 0) {
            return;
        }
        RF_retryWaiting = 0;
#endif

#if (APP_COMBINED_HOST && RF_DAP_POLL_US)
        if (RF_dapPollWait && (int32_t)(SysTick->CNTL - RF_dapPollAt) < 0) {
            return;
        }
        RF_dapPollWait = 0;
#endif

        if (RF_link.role == RF_ROLE_HOST && RF_discoveryPhase != DISCOVERY_SCAN) {
            RF_Transmit(0);
        } else {
            RF_radioState = RADIO_LISTEN;
            RFRole_Stop();
            RF_Receive();
        }
    }
}
