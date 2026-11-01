/* UART IRQ rings, deferred close and CDC line-state support. */

#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "uart/uart_io.h"
#include "usb/usb_protocol.h"
#include "bsp/board.h"
#include "ring.h"
#include "runtime.h"

#if APP_HAS_RF_UART
#include "rf/rf_bridge.h"
#endif

#include <string.h>

/* LOCAL queue service runs in SRAM; TARGET bookkeeping runs in Flash. */

#if APP_HAS_RF_UART
#define __UART_FOREGROUND_CODE
#else
#define __UART_FOREGROUND_CODE __HIGH_CODE
#endif

static uint8_t UART_rxData[UART_RX_SIZE] __ALIGNED(UART_RX_SIZE)
    __SECTION(MODE_LOCAL_SECTION("rx_data"));

#if (APP_COMBINED_TARGET && UART_RX_SIZE > UART_TX_SIZE)
/* The smaller TX ring fills the gap before the larger aligned RX ring. */
static uint8_t UART_txData[UART_TX_SIZE] __ALIGNED(UART_TX_SIZE)
    __SECTION(MODE_LOCAL_SECTION("target_tx_data"));
#else
static uint8_t UART_txData[UART_TX_SIZE] __ALIGNED(UART_TX_SIZE)
    __SECTION(MODE_LOCAL_SECTION("tx_data"));
#endif

#if UART_RX_PACKET_RING
/* LOCAL RX packets share the UART payload storage; there is no second RX queue. */
static struct {
    volatile uint32_t write, read; /* accepted and ACK-retired payload bytes */
    volatile uint32_t prod, cons;  /* slot sequences; prod slot is open */
} UART_rxRing __SECTION(MODE_LOCAL_SECTION("rx_ring"));

static uint8_t          UART_rxPacketLengths[UART_RX_PACKET_COUNT];
static volatile uint8_t UART_rxPacketFill;

#else
static byte_ring_t UART_rxRing __SECTION(MODE_LOCAL_SECTION("rx_ring")) = {0};
#endif

static byte_ring_t       UART_txRing;
static volatile uint16_t UART_serialErrors;

#if APP_HAS_RF_UART
/* Independent event generation; diagnostic reads and ring resets retain it. */
static volatile uint32_t UART_rxLossGeneration;
#endif

static uint32_t          UART_ledRxSeen, UART_ledTxSeen, UART_ledAt;
static volatile bool     UART_enabled;
static volatile bool     UART_closeRequested;
static volatile uint32_t UART_breakUntil;
static volatile bool     UART_breakInfinite;
static volatile uint32_t UART_rxStarted;
uart_stats_t             UART_stats;

#if APP_HAS_RF_UART
/* The RF target initializes these before its physical UART becomes ready;
 * the host's confirmed CDC line is held separately in rf_uart_state. */
uart_line_t        UART_line;
static uart_line_t UART_requestedLine;
#else
uart_line_t        UART_line          = {115200, 0, 0, 8};
static uart_line_t UART_requestedLine = {115200, 0, 0, 8};
#endif

static uint32_t                      UART_quietAt, UART_quietRxBytes;
static bool                          UART_quietPending;
static uint32_t UART_closedDiscarded __SECTION(MODE_LOCAL_SECTION("closed_discarded")) = 0;

#if (UART_RX_PACKET_RING)
static void UART_ResetRxPackets(void) {
    UART_rxRing.write = UART_rxRing.read = UART_rxRing.prod = UART_rxRing.cons = 0;
    UART_rxPacketFill                                                          = 0;
}
#endif

static void UART_InterruptUpdate(void) {
    uint8_t ier =
        UART_enabled && !BSP_stopRequested ? RB_IER_TXD_EN | RB_IER_RECV_RDY | RB_IER_LINE_STAT : 0;
    if (UART_enabled && !BSP_stopRequested && (UART_txRing.write - UART_txRing.read)) {
        ier |= RB_IER_THR_EMPTY;
    }
    R8_UART_IER = ier;
}

static __ALWAYS_INLINE void UART_RxStatus(uint8_t status) {
    if (!(status & (RB_LSR_OVER_ERR | RB_LSR_PAR_ERR | RB_LSR_FRAME_ERR | RB_LSR_BREAK_ERR))) {
        return;
    }
    ++UART_stats.line_errors;
    UART_stats.last_line_status = status;
    if (status & RB_LSR_OVER_ERR) {
        UART_serialErrors |= USB_CDC_STATE_OVERRUN;
    }
    if (status & RB_LSR_PAR_ERR) {
        UART_serialErrors |= USB_CDC_STATE_PARITY;
    }
    if (status & RB_LSR_FRAME_ERR) {
        UART_serialErrors |= USB_CDC_STATE_FRAMING;
    }
    if (status & RB_LSR_BREAK_ERR) {
        UART_serialErrors |= USB_CDC_STATE_BREAK;
    }
}

static __HIGH_CODE void UART_TxFill(void) {
    uint32_t read  = UART_txRing.read;
    uint32_t write = UART_txRing.write;
    unsigned count = write - read;
    unsigned space = UART_FIFO_SIZE - R8_UART_TFC;

    if (count > space) {
        count = space;
    }
    if (count) {
        __asm__ volatile("" ::: "memory");
        for (unsigned i = 0; i < count; i++) {
            R8_UART_THR = UART_txData[read++ & (UART_TX_SIZE - 1)];
        }
        __asm__ volatile("" ::: "memory");
        UART_txRing.read     = read;
        UART_stats.tx_bytes += count;
    }
    if (read == write) {
        R8_UART_IER &= ~RB_IER_THR_EMPTY;
    } else {
        R8_UART_IER |= RB_IER_THR_EMPTY;
    }
}

bool UART_TxDrained(void) {
    return !(UART_txRing.write - UART_txRing.read) && !R8_UART_TFC &&
           (R8_UART_LSR & RB_LSR_TX_ALL_EMP);
}

bool UART_TxIdle(void) {
    return UART_TxDrained() && !(R8_UART_LCR & RB_LCR_BREAK_EN);
}

uint32_t UART_TxTransmitted(void) {
    return UART_stats.tx_bytes;
}

uint16_t UART_LineDivisor(void) {
    return R16_UART_DL;
}

/* Caller serializes hardware programming with USB producers and the UART ISR. */
static bool UART_LineConfigure(const uart_line_t *l, uint16_t div) {
    /* Never alter frame format or baud while queued bytes/FIFO are in flight. */
    if (!UART_TxIdle()) {
        return false;
    }
    R8_UART_IER = 0;
    R8_UART_DIV = 1;
    R16_UART_DL = div;
    uint8_t lcr = l->bits - 5;

    if (l->stop) {
        lcr |= RB_LCR_STOP_BIT;
    }
    if (l->parity) {
        lcr |= RB_LCR_PAR_EN | ((l->parity - 1) << 4);
    }
    R8_UART_LCR = lcr;
    R8_UART_FCR = UART_RX_TRIGGER_BITS | RB_FCR_FIFO_EN;
    return true;
}

static bool UART_LineQuiet(void) {
    if (!UART_TxIdle()) {
        UART_quietPending = false;
        return false;
    }
    if (!UART_enabled) {
        return true;
    }
    uint32_t now = BSP_TimeUs();

    if (!UART_quietPending || UART_quietRxBytes != UART_stats.rx_bytes || R8_UART_RFC ||
        !(R32_PA_PIN & PIN_RX)) {
        UART_quietPending = true;
        UART_quietAt      = now;
        UART_quietRxBytes = UART_stats.rx_bytes;
        return false;
    }

    /* RX FIFO empty is not RX shifter idle. The external peer must stop
     * before requesting new settings, and remain quiet until completion.
     * Conservatively allow start + 8 data + parity + 2 stop bits. */
    uint32_t guard = (12000000U + UART_line.baud - 1U) / UART_line.baud;

    return (uint32_t)(now - UART_quietAt) >= guard;
}

void UART_ConfigureCancel(void) {
    UART_quietPending = false;
    BSP_ClockQueue(false);
}

bool UART_ConfigureLine(const uart_line_t *l) {
    uart_line_plan_t plan;

    if (!UART_LinePlan(l, &plan)) {
        return false;
    }
    if (R16_UART_DL == plan.divisor && R8_UART_DIV == plan.uart_div &&
        BSP_clockHz == plan.clock_hz &&
        (UART_requestedLine.baud == l->baud && UART_requestedLine.stop == l->stop &&
         UART_requestedLine.parity == l->parity && UART_requestedLine.bits == l->bits)) {
        UART_ConfigureCancel();
        return true;
    }
    BSP_ClockQueue(true);
    if (!UART_LineQuiet() || !BSP_ClockAcquire()) {
        return false;
    }
    uint32_t oldClock = BSP_clockHz;

#if APP_HAS_RF_UART
    if (!RF_BridgeClockPause()) {
        BSP_ClockRelease();
        BSP_ClockQueue(true);
        return false;
    }
#endif

    uint32_t irq = BSP_IrqSave();

    if (!UART_enabled) {
        /* Closed UART has no RX owner. Drain its stale FIFO without
         * changing queued owned bytes or mislabelling this as ring-full. */
        for (unsigned i = 0; i < UART_FIFO_SIZE && R8_UART_RFC; ++i) {
            (void)R8_UART_RBR;
            ++UART_closedDiscarded;
        }
    }
    bool ready =
        UART_TxIdle() && !R8_UART_RFC &&
        (!UART_enabled || (UART_quietRxBytes == UART_stats.rx_bytes && (R32_PA_PIN & PIN_RX))) &&
        !BSP_stopRequested;
    uint8_t ier = R8_UART_IER;

    if (ready) {
        R8_UART_IER = 0;
    }
    BSP_IrqRestore(irq);
    bool result = ready && BSP_ClockApply(plan.clock_hz);

    irq = BSP_IrqSave();
    if (result) {
        result = !BSP_stopRequested &&
                 (!UART_enabled ||
                  (UART_quietRxBytes == UART_stats.rx_bytes && (R32_PA_PIN & PIN_RX))) &&
                 !R8_UART_RFC && UART_LineConfigure(l, plan.divisor);
        if (result) {
            UART_requestedLine = *l;
            UART_line          = *l;
            UART_line.baud     = plan.actual;
        }
    }
    if (result) {
        UART_InterruptUpdate();
        UART_quietPending = false;
    } else {
        R8_UART_IER = 0;
    }
    BSP_IrqRestore(irq);
    if (!result && BSP_clockHz != oldClock) {
        if (!BSP_ClockRestore(oldClock)) {
            BSP_Fault();
        }
    }
    irq = BSP_IrqSave();
    if (!result && !BSP_stopRequested) {
        R8_UART_IER = ier;
    }
    BSP_IrqRestore(irq);
    BSP_ClockRelease();

#if APP_HAS_RF_UART
    RF_BridgeClockResume();
#endif

    if (!result && !BSP_stopRequested) {
        BSP_ClockQueue(true);
    }
    return result;
}

void UART_Init(void) {
#if APP_HAS_RF_UART
    if (!UART_requestedLine.baud) {
        UART_requestedLine = (uart_line_t){115200, 0, 0, 8};
        UART_line          = UART_requestedLine;
    }
#endif

#if UART_RX_PACKET_RING
    UART_ResetRxPackets();
#else
    RING_Init(&UART_rxRing, UART_rxData, sizeof(UART_rxData));
#endif

    RING_Init(&UART_txRing, UART_txData, sizeof(UART_txData));
    R16_PIN_ALTERNATE_H = (R16_PIN_ALTERNATE_H & ~(RB_UART_TXD | RB_UART_RXD)) |
                          (BSP_UART_TX_REMAP << 3) | BSP_UART_RX_REMAP;
    R8_UART_MCR = RB_MCR_INT_OE;

    /* Initial programming is foreground work with no live UART owner. */
    UART_enabled        = false;
    UART_closeRequested = false;
    UART_quietPending   = false;
    UART_ConfigureLine(&UART_requestedLine);
    PFIC_EnableIRQ(UART_IRQn);
}

void UART_Open(bool on) {
    uint32_t irq = BSP_IrqSave();

    if (on && BSP_LocalSelected() && !BSP_stopRequested) {
        if (UART_enabled) {
            UART_closeRequested = false;
            BSP_IrqRestore(irq);
            return;
        }
        R32_PA_PD_DRV &= ~(PIN_TX | PIN_RX);
        R32_PA_PU     |= PIN_RX;
        R32_PA_SET     = PIN_TX;
        R32_PA_DIR    |= PIN_TX;
        R32_PA_DIR    &= ~PIN_RX;

        /* The unowned RX pin can accumulate noise while the UART is closed. */
        R8_UART_FCR = 0;
        R8_UART_FCR = UART_RX_TRIGGER_BITS | RB_FCR_FIFO_EN;
        for (unsigned i = 0; i < UART_FIFO_SIZE && R8_UART_RFC; i++) {
            (void)R8_UART_RBR;
        }
        (void)R8_UART_LSR;
        UART_enabled        = true;
        UART_closeRequested = false;
        UART_InterruptUpdate();
    } else {
        UART_closeRequested = true;
        if (UART_enabled && ((UART_txRing.write - UART_txRing.read) || R8_UART_TFC ||
                             !(R8_UART_LSR & RB_LSR_TX_ALL_EMP))) {
            BSP_IrqRestore(irq);
            return;
        }
        UART_enabled = false;
        UART_InterruptUpdate();
        R32_PA_DIR &= ~PIN_TX;

        /* DTR close releases TX but retains the UART's idle RX bias. Removing
         * both leaves a two-wire loopback floating between sessions. Full
         * stop/reset/route cancellation still releases RX in uart_stop. */
    }
    BSP_IrqRestore(irq);
}

void UART_Stop(void) {
    UART_ConfigureCancel();
    UART_enabled         = false;
    UART_closeRequested  = false;
    R8_UART_IER          = 0;
    R32_PA_DIR          &= ~PIN_TX;
    R32_PA_PU           &= ~PIN_RX;
    uint32_t irq         = BSP_IrqSave();

#if UART_RX_PACKET_RING
    UART_ResetRxPackets();
#else
    RING_Init(&UART_rxRing, UART_rxData, sizeof(UART_rxData));
#endif

    RING_Init(&UART_txRing, UART_txData, sizeof(UART_txData));
    R8_UART_LCR        &= ~RB_LCR_BREAK_EN;
    R8_UART_FCR         = 0;
    R8_UART_FCR         = UART_RX_TRIGGER_BITS | RB_FCR_FIFO_EN;
    UART_breakUntil     = 0;
    UART_breakInfinite  = false;
    UART_serialErrors   = 0;
    BSP_IrqRestore(irq);
}

__UART_FOREGROUND_CODE unsigned UART_TxSpace(void) {
    return (UART_txRing.mask + 1 - (UART_txRing.write - UART_txRing.read));
}

__UART_FOREGROUND_CODE bool UART_TxSubmit(const uint8_t *p, unsigned n) {
    void *(*copy)(void *, const void *, size_t) = APP_HAS_RF_UART ? memcpy : RUNTIME_CopyRam;
    if (n > (UART_txRing.mask + 1 - (UART_txRing.write - UART_txRing.read)) ||
        !BSP_LocalSelected() || BSP_stopRequested) {
        return false;
    }
    if (!UART_enabled || UART_closeRequested) {
        UART_Open(true);
    }
    uint32_t write  = UART_txRing.write;
    unsigned offset = write & (UART_TX_SIZE - 1);
    unsigned first  = UART_TX_SIZE - offset;

    if (first > n) {
        first = n;
    }

    /* Publish a complete USB packet once; MMIO FIFO access remains byte-wise. */
    copy(UART_txData + offset, p, first);
    if (first < n) {
        copy(UART_txData, p + first, n - first);
    }
    __asm__ volatile("" ::: "memory");

    /* USB is the sole producer. The UART consumer can run during the copy;
     * it sees the old cursor until the complete packet is published. */
    uint32_t irq = BSP_IrqSave();

    UART_txRing.write = write + n;
    UART_TxFill();
    BSP_IrqRestore(irq);
    return true;
}

#if !UART_RX_PACKET_RING
__UART_FOREGROUND_CODE unsigned UART_RxRead(uint8_t *p, unsigned limit) {
    void *(*copy)(void *, const void *, size_t) = APP_HAS_RF_UART ? memcpy : RUNTIME_CopyRam;
    uint32_t read                               = UART_rxRing.read;
    unsigned n                                  = UART_rxRing.write - read;

    if (n > limit) {
        n = limit;
    }
    if (!n) {
        return 0;
    }
    unsigned offset = read & (UART_RX_SIZE - 1);
    unsigned first  = UART_RX_SIZE - offset;

    if (first > n) {
        first = n;
    }
    __asm__ volatile("" ::: "memory");
    copy(p, UART_rxData + offset, first);
    if (first < n) {
        copy(p + first, UART_rxData, n - first);
    }
    __asm__ volatile("" ::: "memory");
    UART_rxRing.read = read + n;
    return n;
}
#endif

__UART_FOREGROUND_CODE unsigned UART_RxAvailable(void) {
    return UART_rxRing.write - UART_rxRing.read;
}

#if UART_RX_PACKET_RING
__UART_FOREGROUND_CODE unsigned UART_RxPacketPressure(void) {
    return (UART_rxRing.prod - UART_rxRing.cons) * 64U + UART_rxPacketFill;
}

__UART_FOREGROUND_CODE void UART_RxPacketPollFifo(void) {
    /* Caller throttles this assist to once per board_ms. The ISR alone reads
     * RBR, including short FIFO-only tails below the hardware ready level. */
    if (UART_enabled && !UART_closeRequested && !BSP_stopRequested && R8_UART_RFC) {
        PFIC_SetPendingIRQ(UART_IRQn);
    }
}

__UART_FOREGROUND_CODE unsigned UART_RxPacketPeek(const uint8_t **data) {
    uint32_t cons = UART_rxRing.cons;

    if (cons == UART_rxRing.prod) {
        if (!UART_rxPacketFill || (uint32_t)(BSP_ms - UART_rxStarted) < 1U) {
            return 0;
        }
        uint32_t wasEnabled = PFIC_GetStatusIRQ(UART_IRQn);

        PFIC_DisableIRQ(UART_IRQn);

        /* UART could fill the old slot before exclusion. Recheck the current
         * slot/time; never seal a newly opened slot using the old deadline. */
        if (cons == UART_rxRing.prod && UART_rxPacketFill &&
            (uint32_t)(BSP_ms - UART_rxStarted) >= 1U) {
            UART_rxPacketLengths[cons & (UART_RX_PACKET_COUNT - 1U)] = UART_rxPacketFill;
            UART_rxPacketFill                                        = 0;
            __asm__ volatile("" ::: "memory");
            UART_rxRing.prod = cons + 1U;
        }
        if (wasEnabled) {
            PFIC_EnableIRQ(UART_IRQn);
        }
        if (cons == UART_rxRing.prod) {
            return 0;
        }
    }
    __asm__ volatile("" ::: "memory");
    *data = UART_rxData + (cons & (UART_RX_PACKET_COUNT - 1U)) * 64U;
    return UART_rxPacketLengths[cons & (UART_RX_PACKET_COUNT - 1U)];
}

__UART_FOREGROUND_CODE void UART_RxPacketRetire(unsigned length) {
    if (!length) {
        return;
    } /* ZLP owns no slot. USB validates its busy owner. */

    /* Retire payload first, then free the slot. A UART preemption in between
     * sees conservative slot room and smaller valid occupancy, never reuse
     * before retirement or a transient high-water above the true capacity. */
    UART_rxRing.read += length;
    __asm__ volatile("" ::: "memory");
    ++UART_rxRing.cons;
}
#endif

__UART_FOREGROUND_CODE uint32_t UART_RxStarted(void) {
    return UART_rxStarted;
}

void UART_Break(uint16_t duration) {
    if (!duration) {
        R8_UART_LCR        &= ~RB_LCR_BREAK_EN;
        UART_breakUntil     = 0;
        UART_breakInfinite  = false;
        return;
    }
    if (!UART_enabled) {
        UART_Open(true);
    }
    if (!UART_enabled) {
        return;
    }
    R8_UART_LCR        |= RB_LCR_BREAK_EN;
    UART_breakInfinite  = duration == UINT16_MAX;
    UART_breakUntil     = BSP_ms + duration;
}

void UART_Poll(void) {
    /* Indicators need millisecond resolution, not one scan per RF poll.
     * An IRQ after either monotonic sample is observed on the next tick;
     * there is no read-clear flag, FIFO access or IRQ exclusion. */
    uint32_t now = BSP_ms;

    if (now != UART_ledAt) {
        UART_ledAt  = now;
        uint32_t rx = UART_stats.rx_bytes, tx = UART_stats.tx_bytes;

        if (rx != UART_ledRxSeen) {
            UART_ledRxSeen = rx;
            BSP_LedActivity(1);
        }
        if (tx != UART_ledTxSeen) {
            UART_ledTxSeen = tx;
            BSP_LedActivity(0);
        }
    }
    if (UART_closeRequested) {
        UART_Open(false);
    }
    if ((R8_UART_LCR & RB_LCR_BREAK_EN) && !UART_breakInfinite &&
        (int32_t)(BSP_ms - UART_breakUntil) >= 0) {
        UART_Break(0);
    }
}

__UART_FOREGROUND_CODE uint16_t UART_TakeSerialErrors(void) {
    uint32_t irq = BSP_IrqSave();
    uint16_t e   = UART_serialErrors;

    UART_serialErrors = 0;
    BSP_IrqRestore(irq);
    return e;
}

uint32_t UART_RxLossGeneration(void) {
#if APP_HAS_RF_UART
    return UART_rxLossGeneration;
#else
    return 0;
#endif
}

__INTERRUPT __HIGH_CODE void UART_IRQHandler(void) {
#if UART_RX_PACKET_RING
    unsigned fifo   = R8_UART_RFC;
    uint8_t  status = R8_UART_LSR;

    /* Preserve LSR-before-RBR error sampling while freeing one FIFO entry
     * before status accounting and packet-space/slot calculations. */
    uint8_t first = fifo ? R8_UART_RBR : 0;

    if (UART_enabled) {
        UART_RxStatus(status);
    }
    if (fifo) {
        uint32_t prod     = UART_rxRing.prod;
        unsigned fill     = UART_rxPacketFill;
        unsigned room     = (UART_RX_PACKET_COUNT - (prod - UART_rxRing.cons)) * 64U - fill;
        unsigned accepted = UART_enabled ? (fifo < room ? fifo : room) : 0;
        unsigned dropped  = UART_enabled ? fifo - accepted : 0;
        unsigned left     = accepted;

        while (left) { /* RFC<=8: at most two contiguous accepted spans. */
            if (!fill) {
                UART_rxStarted = BSP_ms;
            }
            unsigned n = 64U - fill;

            if (n > left) {
                n = left;
            }
            uint8_t *out = UART_rxData + (prod & (UART_RX_PACKET_COUNT - 1U)) * 64U + fill;

            fill           += n;
            bool firstSpan  = left == accepted;

            left -= n;
            if (firstSpan) {
                *out++ = first;
                --n;
            }
            if (n) {
                do {
                    *out++ = R8_UART_RBR;
                } while (--n);
            }
            if (fill == 64U) {
                UART_rxPacketLengths[prod & (UART_RX_PACKET_COUNT - 1U)] = 64;
                ++prod;
                fill = 0;
            }
        }
        unsigned discard = fifo - accepted;

        if (!accepted) {
            --discard;
        } /* The prefetched byte was dropped. */
        while (discard--) {
            (void)R8_UART_RBR;
        }
        if (accepted) {
            UART_rxPacketFill  = fill;
            UART_rxRing.write += accepted;
            __asm__ volatile("" ::: "memory");

            bool published = prod != UART_rxRing.prod;

            UART_rxRing.prod = prod;

            /* USB cannot preempt this producer under either policy. Publish the whole
             * batch first, then wake its sole endpoint owner for sealed data. */
            if (published) {
                PFIC_SetPendingIRQ(USB_IRQn);
            }

            unsigned used = UART_rxRing.write - UART_rxRing.read;

            if (used > UART_stats.max_rx_high_water) {
                UART_stats.max_rx_high_water = used;
            }
        }
        UART_stats.rx_bytes += fifo;
        if (dropped) {
            UART_stats.rx_overflow += dropped;
            UART_serialErrors      |= USB_CDC_STATE_OVERRUN;
        }
    }
#else
    unsigned   received = 0, dropped = 0;
    const bool capture = true;
    uint32_t   write   = UART_rxRing.write;
    unsigned   space   = UART_RX_SIZE - (write - UART_rxRing.read);

    if (capture && space == UART_RX_SIZE && R8_UART_RFC) {
        /* Keep the oldest byte's timestamp until this burst is consumed. */
        UART_rxStarted = BSP_ms;
    }
    uint8_t status = R8_UART_LSR;

    if (capture) {
        UART_RxStatus(status);
    }
    while (R8_UART_RFC) {
        uint8_t v = R8_UART_RBR;

        ++received;
        if (capture && space) {
            UART_rxData[write++ & (UART_RX_SIZE - 1)] = v;
            --space;
        } else if (capture) {
            ++dropped;
        }
    }
    __asm__ volatile("" ::: "memory");
    UART_rxRing.write    = write;
    UART_stats.rx_bytes += received;

#if APP_HAS_RF_UART
    /* One loss event per serviced FIFO batch; byte and line counters stay
     * separate. A hardware overrun's unknown lost-byte count is not guessed. */
    if (dropped || (capture && (status & RB_LSR_OVER_ERR))) {
        ++UART_rxLossGeneration;
    }
#endif

    if (dropped) {
        UART_stats.rx_overflow += dropped;
        UART_serialErrors      |= USB_CDC_STATE_OVERRUN;
    }
    unsigned used = write - UART_rxRing.read;

    if (used > UART_stats.max_rx_high_water) {
        UART_stats.max_rx_high_water = used;
    }
#endif

    (void)R8_UART_IIR;
    if (UART_enabled) {
#if UART_RX_PACKET_RING
        /* The preceding nonempty fill disables THR_EMPTY as it retires the
         * last queued byte. No FIFO owner work remains for an empty ring. */
        if (UART_txRing.read != UART_txRing.write) {
            UART_TxFill();
        }
#else
        UART_TxFill();
#endif
    }
}
