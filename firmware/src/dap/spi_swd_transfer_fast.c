/* SPDX-License-Identifier: Apache-2.0 */
#include "ch572/ch572.h"
#include "ch572/watchdog.h"
#include "dap/dap_config.h"
#include "bsp/board.h"
#include "dap/swj.h"
#include "dap/dap.h"

_Static_assert(PIN_CLK == 32 && PIN_DIO == 128, "SWD GPIO mapping");

/* The linker places this object in the SPI SRAM overlay. FIFO batches and
 * framing share one function; only the GPIO half-periods need fixed assembly.
 * TX/RX point directly into the command/response and need no word alignment. */
uint8_t SWD_SpiTransfer(uint32_t req, const uint8_t *tx, uint8_t *rx) {
    uint32_t read   = req & DAP_TRANSFER_RnW;
    uint32_t parity = 0;
    uint8_t  scratch[4];
    uint8_t *data         = read ? (rx ? rx : scratch) : (uint8_t *)tx;
    uint32_t request      = req & SWD_REQUEST_MASK;
    uint32_t headerParity = request ^ (request >> 2);

    headerParity ^= headerParity >> 1;
    uint8_t header =
        SWD_REQUEST_START_PARK | (request << 1) | ((headerParity & 1U) << SWD_REQUEST_PARITY_SHIFT);

    uint32_t bitDelay   = SWJ_delayLoops[GPIO_TIMING_SHIFT] * 2U;
    uint32_t clockDelay = SWJ_delayLoops[GPIO_TIMING_CLOCK] * 2U;
    uint32_t budget     = SWJ_spiBudgetCycles;
    uint32_t divider    = SWJ_spiDiv;
    uint32_t ack        = 0;
    unsigned bytes      = 1;
    unsigned clocks;

    enum {
        ACK,
        WRITE_DATA,
        READ_IDLE,
        RECOVER,
        FINISH
    } next;

    R32_PA_DIR |= PIN_DIO;

spiBatch:
    if (BSP_stopRequested) {
        goto stopped;
    }

    /* Preserve Mode 3 idle HIGH across GPIO/SPI ownership changes. */
    R32_PA_SET      = PIN_CLK;
    R8_SPI_CTRL_MOD = RB_SPI_ALL_CLEAR | RB_SPI_MST_SCK_MOD;
    uint8_t mode    = RB_SPI_MST_SCK_MOD | RB_SPI_2WIRE_MOD;
    uint8_t outputs = RB_SPI_SCK_OE | RB_SPI_MOSI_OE;

    if (bytes != 1 && read) {
        mode    |= RB_SPI_FIFO_DIR;
        outputs  = RB_SPI_SCK_OE;
    }
    R8_SPI_CTRL_MOD = mode;
    if (bytes == 1) {
        R8_SPI_FIFO = header;
    } else if (!read) {
        uint32_t byte = data[0];

        R8_SPI_FIFO  = byte;
        parity       = byte;
        byte         = data[1];
        R8_SPI_FIFO  = byte;
        parity      ^= byte;
        byte         = data[2];
        R8_SPI_FIFO  = byte;
        parity      ^= byte;
        byte         = data[3];
        R8_SPI_FIFO  = byte;
        parity      ^= byte;
        parity      ^= parity >> 4;
        parity      ^= parity >> 2;
        parity      ^= parity >> 1;
        parity      &= 1U;
    }
    R8_SPI_CTRL_MOD   = mode | outputs;
    R16_SPI_TOTAL_CNT = bytes;
    uint32_t started  = SysTick->CNTL;
    uint32_t cycles   = bytes * 8U * divider;

    for (;;) {
        if (BSP_stopRequested) {
            goto stopped;
        }
        uint32_t elapsed = SysTick->CNTL - started;

        if (elapsed >= budget) {
            goto timeout;
        }

        /* FREE indicates the shifter is idle. CNT_END is sticky and cannot
         * establish completion of this batch by itself. */
        if ((mode & RB_SPI_FIFO_DIR ? R8_SPI_FIFO_COUNT >= bytes
                                    : elapsed >= cycles && !R8_SPI_FIFO_COUNT) &&
            !R16_SPI_TOTAL_CNT && (R8_SPI_INT_FLAG & RB_SPI_FREE)) {
            break;
        }
    }
    if (mode & RB_SPI_FIFO_DIR) {
        uint32_t byte = R8_SPI_FIFO;

        data[0]  = byte;
        parity   = byte;
        byte     = R8_SPI_FIFO;
        data[1]  = byte;
        parity  ^= byte;
        byte     = R8_SPI_FIFO;
        data[2]  = byte;
        parity  ^= byte;
        byte     = R8_SPI_FIFO;
        data[3]  = byte;
        parity  ^= byte;
    }
    R8_SPI_CTRL_MOD = mode;
    if (bytes == 1) {
        R32_PA_DIR &= ~PIN_DIO;
        clocks      = DAP_data.swd_conf.turnaround;
        next        = ACK;
        goto clockBits;
    }
    if (read) {
        uint32_t captured, delay;

        /* One sampled GPIO parity bit, using the same balanced halves as ACK. */
        __asm__ volatile("sw %[clk],0(%[clr])\n"
                         "nop\n"
                         "beqz %[limit],2f\n"
                         "mv %[delay],%[limit]\n"
                         "1: addi %[delay],%[delay],-1\n"
                         "bnez %[delay],1b\n"
                         "2: lw %[value],-8(%[clr])\n"
                         "srli %[value],%[value],7\n"
                         "andi %[value],%[value],1\n"
                         "sw %[clk],12(%[clr])\n"
                         "nop\n"
                         "nop\n nop\n nop\n nop\n"
                         "beqz %[limit],4f\n"
                         "mv %[delay],%[limit]\n"
                         "3: addi %[delay],%[delay],-1\n"
                         "bnez %[delay],3b\n"
                         "4:\n"
                         : [value] "=&r"(captured), [delay] "=&r"(delay)
                         : [clr] "r"(&R32_PA_CLR), [clk] "r"(PIN_CLK), [limit] "r"(bitDelay)
                         : "memory");
        parity ^= parity >> 4;
        parity ^= parity >> 2;
        parity ^= parity >> 1;
        parity &= 1U;
        if (parity != captured) {
            ack = DAP_TRANSFER_ERROR;
        }
        __asm__ volatile("" : "+r"(ack)::"memory");
        clocks = DAP_data.swd_conf.turnaround;
        next   = READ_IDLE;
        goto clockBits;
    }
    {
        uint32_t delay;

        /* Write parity: equal delay loops on both sides of the rising edge. */
        __asm__ volatile("sw %[low],0(%[clr])\n"
                         "sw %[value],12(%[clr])\n"
                         "nop\n nop\n nop\n"
                         "beqz %[limit],2f\n"
                         "mv %[delay],%[limit]\n"
                         "1: addi %[delay],%[delay],-1\n"
                         "bnez %[delay],1b\n"
                         "2: sw %[clk],12(%[clr])\n"
                         "nop\n nop\n nop\n nop\n"
                         "beqz %[limit],4f\n"
                         "mv %[delay],%[limit]\n"
                         "3: addi %[delay],%[delay],-1\n"
                         "bnez %[delay],3b\n"
                         "4:\n"
                         : [delay] "=&r"(delay)
                         : [clr] "r"(&R32_PA_CLR), [clk] "r"(PIN_CLK), [low] "r"(PIN_CLK | PIN_DIO),
                           [value] "r"(parity << 7), [limit] "r"(bitDelay)
                         : "memory");
    }
idle:

#if DAP_TIMESTAMP_ENABLE
    if (req & DAP_TRANSFER_TIMESTAMP) {
        DAP_data.timestamp = TIMESTAMP_GET();
    }
#endif

    clocks = DAP_data.transfer.idle_cycles;
    if (clocks) {
        R32_PA_CLR = PIN_DIO;
        next       = FINISH;
        goto clockBits;
    }
    goto finish;

clockBits:
    if (BSP_stopRequested) {
        goto stopped;
    }
    if (clocks) {
        uint32_t delay;

        /* Keep loop bookkeeping in the HIGH half, balanced by three LOW NOPs.
         * Delay limits were read once and remain in registers for the batch. */
        __asm__ volatile("1: sw %[clk],0(%[clr])\n"
                         "nop\n nop\n nop\n"
                         "beqz %[limit],3f\n"
                         "mv %[delay],%[limit]\n"
                         "2: addi %[delay],%[delay],-1\n"
                         "bnez %[delay],2b\n"
                         "3: sw %[clk],12(%[clr])\n"
                         "beqz %[limit],5f\n"
                         "mv %[delay],%[limit]\n"
                         "4: addi %[delay],%[delay],-1\n"
                         "bnez %[delay],4b\n"
                         "5: addi %[count],%[count],-1\n"
                         "bnez %[count],1b\n"
                         : [count] "+&r"(clocks), [delay] "=&r"(delay)
                         : [clr] "r"(&R32_PA_CLR), [clk] "r"(PIN_CLK), [limit] "r"(clockDelay)
                         : "memory");
    }
    if (next == ACK) {
        /* The HIGH padding now carries loop bookkeeping: OR + NOP +
         * decrement + taken branch has the same budget as OR + four NOPs.
         * Keep the whole three-bit sample loop here so C adds no edge gaps. */
        uint32_t count = 3, delay, sample;

        __asm__ volatile(
            "1: sw %[clk],0(%[clr])\n"
            "srli %[value],%[value],1\n"
            "beqz %[limit],3f\n"
            "mv %[delay],%[limit]\n"
            "2: addi %[delay],%[delay],-1\n"
            "bnez %[delay],2b\n"
            "3: lw %[sample],-8(%[clr])\n"
            "andi %[sample],%[sample],128\n"
            "slli %[sample],%[sample],24\n"
            "sw %[clk],12(%[clr])\n"
            "or %[value],%[value],%[sample]\n"
            "nop\n"
            "beqz %[limit],5f\n"
            "mv %[delay],%[limit]\n"
            "4: addi %[delay],%[delay],-1\n"
            "bnez %[delay],4b\n"
            "5: addi %[count],%[count],-1\n"
            "bnez %[count],1b\n"
            : [value] "+&r"(ack), [count] "+&r"(count), [delay] "=&r"(delay), [sample] "=&r"(sample)
            : [clr] "r"(&R32_PA_CLR), [clk] "r"(PIN_CLK), [limit] "r"(bitDelay)
            : "memory");
        ack >>= 29;
        if (ack == DAP_TRANSFER_OK) {
            if (read) {
                bytes = 4;
                goto spiBatch;
            }
            clocks = DAP_data.swd_conf.turnaround;
            next   = WRITE_DATA;
        } else if (ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT) {
            /* A completed ACK is progress in Arm's bounded retry loop.
             * Never feed inside the hardware-completion polling loop. */
            WDOG_Feed();
            clocks = DAP_data.swd_conf.turnaround;
            if (read && DAP_data.swd_conf.data_phase) {
                clocks += 33;
            }
            next = !read && DAP_data.swd_conf.data_phase ? RECOVER : FINISH;
        } else {
            ack    = DAP_TRANSFER_ERROR;
            clocks = DAP_data.swd_conf.turnaround + 33U;
            next   = FINISH;
        }
        goto clockBits;
    }
    R32_PA_DIR |= PIN_DIO;
    if (next == WRITE_DATA) {
        bytes = 4;
        goto spiBatch;
    }
    if (next == READ_IDLE) {
        goto idle;
    }
    if (next == RECOVER) {
        R32_PA_CLR = PIN_DIO;
        clocks     = 33;
        next       = FINISH;
        goto clockBits;
    }
finish:
    R32_PA_SET = PIN_DIO;
    return ack;

timeout:
    ++SWJ_spiTimeouts;
    BSP_stopRequested = 1;
stopped:
    R8_SPI_CTRL_MOD  = RB_SPI_MST_SCK_MOD | RB_SPI_2WIRE_MOD;
    R32_PA_DIR      &= ~(PIN_CLK | PIN_DIO);
    return DAP_TRANSFER_ERROR;
}
