/* Pure UART frame validation and four-step system-clock selection. */
#include "uart/uart_line.h"
#include "usb/usb_protocol.h"
#include "app_config.h"

/* The same ordered table serves both exact and approximate selection. */
static const uint32_t UART_clockCandidatesHz[] = {100000000u, 60000000u, 40000000u};

typedef struct {
    uint32_t baud, clock_hz;
    uint16_t uart_dl;
} uart_baud_exception_t;

static const uart_baud_exception_t UART_baudExceptions[] = {{921600u, 75000000u, 10u}};

static bool UART_PlanCandidate(uint32_t baud, uint32_t hz, uint32_t dl, uint8_t step,
                               uart_line_plan_t *plan) {
    if (!dl || dl > 65535u) {
        return false;
    }
    uint64_t expected  = (uint64_t)baud * 8u * dl;
    int64_t  error     = (int64_t)hz - (int64_t)expected;
    uint64_t magnitude = error < 0 ? (uint64_t)-error : (uint64_t)error;

    if (magnitude * 100u > expected * 2u) {
        return false;
    }
    if (plan) {
        *plan = (uart_line_plan_t){.requested         = baud,
                                   .actual            = hz / (8u * dl),
                                   .clock_hz          = hz,
                                   .divisor           = (uint16_t)dl,
                                   .uart_div          = 1,
                                   .policy_step       = step,
                                   .error_numerator   = error,
                                   .error_denominator = expected};
    }
    return true;
}

bool UART_LinePlan(const uart_line_t *line, uart_line_plan_t *plan) {
    /* Step 1: validate the request before arithmetic or clock selection. */
    if (!line || line->baud < 300u || line->baud > UART_MAX_BAUD || line->bits < 5 ||
        line->bits > 8 || line->parity > USB_CDC_PARITY_SPACE || line->stop > USB_CDC_STOP_2 ||
        (line->stop == USB_CDC_STOP_1_5 && line->bits != 5) ||
        (line->stop == USB_CDC_STOP_2 && line->bits == 5)) {
        return false;
    }
    uint32_t baud = line->baud;

    /* Step 2: all exact candidates have priority over every approximation. */
    for (unsigned i = 0; i < sizeof UART_clockCandidatesHz / sizeof UART_clockCandidatesHz[0];
         ++i) {
        uint32_t hz = UART_clockCandidatesHz[i];

        if ((hz / 8u) % baud == 0 && UART_PlanCandidate(baud, hz, (hz / 8u) / baud, 2, plan)) {
            return true;
        }
    }

    /* Step 3: only the fixed 921600 request receives its special clock. */
    for (unsigned i = 0; i < sizeof UART_baudExceptions / sizeof UART_baudExceptions[0]; ++i) {
        const uart_baud_exception_t *entry = &UART_baudExceptions[i];

        if (baud == entry->baud) {
            return UART_PlanCandidate(baud, entry->clock_hz, entry->uart_dl, 3, plan);
        }
    }

    /* Step 4: nearest integer DL, first clock within the exact rational bound.
     * All operands are bounded by step 1; 64-bit error products avoid overflow. */
    for (unsigned i = 0; i < sizeof UART_clockCandidatesHz / sizeof UART_clockCandidatesHz[0];
         ++i) {
        uint32_t hz = UART_clockCandidatesHz[i];
        uint32_t dl = (hz / 8u + baud / 2u) / baud;

        if (UART_PlanCandidate(baud, hz, dl, 4, plan)) {
            return true;
        }
    }
    return false;
}

bool UART_LineValid(const uart_line_t *line, uint16_t *divisor, uint32_t *actual) {
    uart_line_plan_t plan;

    if (!UART_LinePlan(line, &plan)) {
        return false;
    }
    if (divisor) {
        *divisor = plan.divisor;
    }
    if (actual) {
        *actual = plan.actual;
    }
    return true;
}
