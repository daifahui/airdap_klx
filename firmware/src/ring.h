#ifndef RING_H
#define RING_H

#include <stdint.h>
#include <stdbool.h>

/* Single producer and single consumer; aligned 32-bit counters are atomic on RV32.
 * CH572 publication needs compiler ordering only; empty asm emits no instruction.
 * IRQ/main reset and multiple producers require the caller's critical section. */
typedef struct {
    volatile uint32_t write, read;
    uint8_t          *data;
    uint32_t          mask;
} byte_ring_t;

static inline void RING_Init(byte_ring_t *r, uint8_t *data, uint32_t size) {
    r->read = r->write = 0;
    r->data            = data;
    r->mask            = size - 1;
}
#endif
