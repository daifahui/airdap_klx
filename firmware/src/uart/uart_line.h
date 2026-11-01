#ifndef UART_LINE_H
#define UART_LINE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t baud;
    uint8_t  stop, parity, bits;
} uart_line_t;

typedef struct {
    uint32_t requested; /* Requested baud, never replaced by actual. */
    uint32_t actual;    /* Floor(clock_hz / (8 * divisor)); reporting only. */
    uint32_t clock_hz;
    uint16_t divisor;     /* UART_DL, 1..65535. */
    uint8_t  uart_div;    /* UART_DIV, always 1. */
    uint8_t  policy_step; /* 2: exact, 3: fixed exception, 4: approximate. */

    /* Signed relative nominal error, without truncating the baud rate:
     * (clock_hz - requested * 8 * divisor) / (requested * 8 * divisor).
     * The fraction need not be reduced. Multiply by 100 for percent. */
    int64_t  error_numerator;
    uint64_t error_denominator;
} uart_line_plan_t;

/* Pure selection: no MMIO or active state changes. NULL plan validates only.
 * On rejection, a non-NULL plan is left unchanged. Frame format is validated
 * from line; the caller must retain that same line for eventual application. */

bool UART_LinePlan(const uart_line_t *line, uart_line_plan_t *plan);

/* Compatibility wrapper. Divisor is meaningful with the selected clock, not
 * necessarily CPU_CLOCK; hardware callers should apply the complete plan. */

bool UART_LineValid(const uart_line_t *line, uint16_t *divisor, uint32_t *actual);
#endif
