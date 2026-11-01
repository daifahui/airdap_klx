/* SPDX-License-Identifier: Apache-2.0 */
/* GPIO SWD framing stays inside its SRAM overlay. */
#include "dap/dap_config.h"
#include "bsp/board.h"
#include "dap/swj.h"
#include "dap/dap.h"

static void SWD_Clocks(unsigned n) {
    while (n && !BSP_stopRequested) {
        unsigned k = n > 32 ? 32 : n;

        GPIO_Shift(0, k, 0, 0);
        n -= k;
    }
}

void SWD_GpioSequence(uint32_t info, const uint8_t *tx, uint8_t *rx) {
    unsigned bits  = info & SWD_SEQUENCE_CLK;
    bool     input = (info & SWD_SEQUENCE_DIN) != 0;

    if (!bits) {
        bits = 64;
    }
    while (bits && !BSP_stopRequested) {
        unsigned k = bits > 32 ? 32 : bits, bytes = (k + 7) / 8;
        uint32_t value = 0;

        if (!(info & SWD_SEQUENCE_DIN)) {
            for (unsigned i = 0; i < bytes; i++) {
                value |= (uint32_t)tx[i] << (8 * i);
            }
            tx += bytes;
        }
        uint32_t v = GPIO_Shift(value, k, input ? 0 : PIN_DIO, input ? PIN_DIO : 0);

        if (info & SWD_SEQUENCE_DIN) {
            for (unsigned i = 0; i < bytes; i++) {
                rx[i] = v >> (8 * i);
            }
            rx += bytes;
        }
        bits -= k;
    }
}

uint8_t SWD_GpioTransfer(uint32_t req, const uint8_t *tx, uint8_t *rx) {
    uint32_t oldTimeouts = SWJ_spiTimeouts;
    bool     read        = (req & DAP_TRANSFER_RnW) != 0;
    uint32_t value       = 0;
    uint32_t parity      = 0;

    if (!read) {
        value = (uint32_t)tx[0] | ((uint32_t)tx[1] << 8) | ((uint32_t)tx[2] << 16) |
                ((uint32_t)tx[3] << 24);
        uint32_t folded = value ^ (value >> 16);

        folded ^= folded >> 8;
        folded ^= folded >> 4;
        parity  = (0x6996U >> (folded & 15U)) & 1U;
    }

    /* Finish TX parity before any bus clocks. The empty compiler barrier keeps
     * this pure calculation from being moved into the data/parity boundary. */
    __asm__ volatile("" : "+r"(parity)::"memory");
    SWJ_DioEnable(true);
    uint32_t r = req & SWD_REQUEST_MASK;
    uint8_t  header =
        SWD_REQUEST_START_PARK | (r << 1) | (((0x6996U >> r) & 1U) << SWD_REQUEST_PARITY_SHIFT);
    GPIO_Shift(header, SWD_REQUEST_BITS, PIN_DIO, 0);
    SWJ_DioEnable(false);
    SWD_Clocks(DAP_data.swd_conf.turnaround);
    uint32_t ack = GPIO_Shift(0, SWD_ACK_BITS, 0, PIN_DIO);

    if (ack == DAP_TRANSFER_OK) {
        if (read) {
            value  = GPIO_Shift(0, SWD_DATA_BITS, 0, PIN_DIO);
            parity = GPIO_Shift(0, 1, 0, PIN_DIO);

            /* Check once after all input bits, before turnaround. These empty
             * barriers keep the calculation between capture and bus cleanup. */
            __asm__ volatile("" : "+r"(value), "+r"(parity)::"memory");
            uint32_t folded = value ^ (value >> 16);

            folded ^= folded >> 8;
            folded ^= folded >> 4;
            if (parity != ((0x6996U >> (folded & 15U)) & 1U)) {
                ack = DAP_TRANSFER_ERROR;
            }
            __asm__ volatile("" : "+r"(ack)::"memory");
            SWD_Clocks(DAP_data.swd_conf.turnaround);
            SWJ_DioEnable(true);
        } else {
            SWD_Clocks(DAP_data.swd_conf.turnaround);
            SWJ_DioEnable(true);
            GPIO_Shift(value, SWD_DATA_BITS, PIN_DIO, 0);
            GPIO_Shift(parity, 1, PIN_DIO, 0);
        }

#if DAP_TIMESTAMP_ENABLE
        if (req & DAP_TRANSFER_TIMESTAMP) {
            DAP_data.timestamp = TIMESTAMP_GET();
        }
#endif

        if (DAP_data.transfer.idle_cycles) {
            SWJ_GpioOutput(PIN_DIO, 0);
            SWD_Clocks(DAP_data.transfer.idle_cycles);
        }
    } else {
        ack = SWD_Recover(req, ack);
    }
    SWJ_GpioOutput(PIN_DIO, 1);
    if (BSP_stopRequested || oldTimeouts != SWJ_spiTimeouts) {
        ack = DAP_TRANSFER_ERROR;
    }

    /* Commit only a validated read. The Arm command core checks this status
     * before counting the transfer or appending data to the DAP response. */
    if (ack == DAP_TRANSFER_OK && read && rx) {
        rx[0] = (uint8_t)value;
        rx[1] = (uint8_t)(value >> 8);
        rx[2] = (uint8_t)(value >> 16);
        rx[3] = (uint8_t)(value >> 24);
    }
    return ack;
}
