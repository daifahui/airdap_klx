/* SPDX-License-Identifier: Apache-2.0 */
#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "dap/dap_config.h"
#include "bsp/board.h"
#include "dap/swj.h"
#include "dap/dap.h"

#if (DAP_TIMESTAMP_ENABLE && APP_COMBINED_TARGET)
/* Optional Timestamp grows the fast Transfer. Move only raw Sequence's
 * partial-byte/stop tail out of the shared overlay; DP/AP FIFO transfers keep
 * their SRAM path and the same buffers and clock phases. */
static __COLD_FLASH __NO_INLINE void SWD_SpiTail(bool input, unsigned tail, const uint8_t *tx,
                                                 uint8_t *rx) {
    if (tail && !BSP_stopRequested) {
        uint32_t value =
            GPIO_Shift(input ? 0 : *tx, tail, input ? 0 : PIN_DIO, input ? PIN_DIO : 0);
        if (input) {
            *rx = value;
        }
    }
    if (BSP_stopRequested) {
        R8_SPI_CTRL_MOD  = RB_SPI_MST_SCK_MOD | RB_SPI_2WIRE_MOD;
        R32_PA_DIR      &= ~(PIN_CLK | PIN_DIO);
    }
}
#endif

/* Raw SWD sequences fit in the eight-byte FIFO. Use one batch without packing
 * words into temporary buffers; only a final partial byte needs GPIO clocks. */

/* The fast DP/AP Transfer remains in SRAM. Only this raw Sequence's setup,
 * polling and GPIO-tail dispatch execute from Flash; bit engines stay in RAM.
 * Leave room for Timestamp and the watchdog admission code in the full image. */
__COLD_FLASH void SWD_SpiSequence(uint32_t info, const uint8_t *tx, uint8_t *rx) {
    unsigned bits = info & 63U;

    if (!bits) {
        bits = 64;
    }
    unsigned bytes = bits / 8U, tail = bits % 8U;
    bool     input = (info & SWD_SEQUENCE_DIN) != 0;

    if (BSP_stopRequested) {
        return;
    }
    if (bytes) {
        R32_PA_SET      = PIN_CLK;
        R8_SPI_CTRL_MOD = RB_SPI_ALL_CLEAR | RB_SPI_MST_SCK_MOD;
        uint8_t mode    = RB_SPI_MST_SCK_MOD | RB_SPI_2WIRE_MOD | (input ? RB_SPI_FIFO_DIR : 0);

        R8_SPI_CTRL_MOD = mode;
        if (!input) {
            for (unsigned i = 0; i < bytes; ++i) {
                R8_SPI_FIFO = tx[i];
            }
        }
        R8_SPI_CTRL_MOD   = mode | RB_SPI_SCK_OE | (input ? 0 : RB_SPI_MOSI_OE);
        R16_SPI_TOTAL_CNT = bytes;
        uint32_t started  = SysTick->CNTL;
        uint32_t cycles   = bytes * 8U * SWJ_spiDiv;

        for (;;) {
            uint32_t elapsed = SysTick->CNTL - started;

            if (BSP_stopRequested) {
                break;
            }
            if (elapsed >= SWJ_spiBudgetCycles) {
                ++SWJ_spiTimeouts;
                BSP_stopRequested = 1;
                break;
            }
            if ((input ? R8_SPI_FIFO_COUNT >= bytes : elapsed >= cycles && !R8_SPI_FIFO_COUNT) &&
                !R16_SPI_TOTAL_CNT && (R8_SPI_INT_FLAG & RB_SPI_FREE)) {
                break;
            }
        }
        R8_SPI_CTRL_MOD = mode;
        if (input && !BSP_stopRequested) {
            for (unsigned i = 0; i < bytes; ++i) {
                rx[i] = R8_SPI_FIFO;
            }
        }
    }

#if (DAP_TIMESTAMP_ENABLE && APP_COMBINED_TARGET)
    SWD_SpiTail(input, tail, input ? NULL : tx + bytes, input ? rx + bytes : NULL);
#else
    if (tail && !BSP_stopRequested) {
        uint32_t value =
            GPIO_Shift(input ? 0 : tx[bytes], tail, input ? 0 : PIN_DIO, input ? PIN_DIO : 0);
        if (input) {
            rx[bytes] = value;
        }
    }
    if (BSP_stopRequested) {
        R8_SPI_CTRL_MOD  = RB_SPI_MST_SCK_MOD | RB_SPI_2WIRE_MOD;
        R32_PA_DIR      &= ~(PIN_CLK | PIN_DIO);
    }
#endif
}
