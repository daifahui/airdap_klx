#ifndef UART_IO_H
#define UART_IO_H

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"
#include "uart/uart_line.h"

#if (UART_RX_PACKET_RING != 0 && UART_RX_PACKET_RING != 1)
#error "UART_RX_PACKET_RING must be 0 or 1"
#endif

#if UART_RX_PACKET_RING
#if APP_MODE != APP_MODE_COMPOSITE
#error "UART packet ring requires the LOCAL bank"
#endif

#if UART_RX_SIZE < 256
#error "UART packet ring requires at least four 64-byte RX slots"
#endif

#define UART_RX_PACKET_COUNT (UART_RX_SIZE / 64U)

/* Serialized USB owner: peek may seal a due partial under UART-only exclusion.
 * A returned bank remains owned until ACK. Reset only after the USB SIE barrier. */

unsigned UART_RxPacketPeek(const uint8_t **data);

void UART_RxPacketRetire(unsigned length);

unsigned UART_RxPacketPressure(void);

void UART_RxPacketPollFifo(void);
#endif

#if UART_CDC_IN_PACKET_LIMIT != 64
#error "UART_CDC_IN_PACKET_LIMIT must equal the 64-byte USB FS packet size"
#endif

typedef struct {
    volatile uint32_t rx_bytes, tx_bytes, rx_overflow, line_errors, max_rx_high_water;
    volatile uint32_t last_line_status;
} uart_stats_t;

extern uart_stats_t UART_stats;
extern uart_line_t  UART_line;

void UART_Init(void);

void UART_Stop(void);

bool UART_ConfigureLine(const uart_line_t *line);

/* Foreground lifecycle cancellation; does not reset diagnostic counters. */

void UART_ConfigureCancel(void);

bool UART_TxIdle(void);

bool UART_TxDrained(void);

uint32_t UART_TxTransmitted(void);

uint16_t UART_LineDivisor(void);

void UART_Open(bool enabled);

bool UART_TxSubmit(const uint8_t *p, unsigned n);

unsigned UART_TxSpace(void);

unsigned UART_RxRead(uint8_t *p, unsigned limit);

unsigned UART_RxAvailable(void);

uint32_t UART_RxStarted(void);

void UART_Break(uint16_t durationMs);

void UART_Poll(void);

uint16_t UART_TakeSerialErrors(void);

/* Modulo-2^32 event count: drop-new batches or hardware overrun observations.
 * Hardware lost-byte counts remain unknown; reads do not clear this value. */

uint32_t UART_RxLossGeneration(void);
#endif
