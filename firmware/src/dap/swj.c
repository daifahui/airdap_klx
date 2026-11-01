/* SPDX-License-Identifier: Apache-2.0 */
/* Load one SRAM backend at command boundaries and dispatch debug operations. */

#include "ch572/ch572.h"
#include "dap/dap_config.h"
#include "bsp/board.h"
#include "dap/swj.h"
#include "dap/dap.h"
#include "dap/debug_pins.h"

#if !APP_COMBINED_TARGET
#define __SWJ_JTAG_CODE __HIGH_CODE
#else
#define __SWJ_JTAG_CODE
#endif

extern uint8_t    __ov_gpio_load[], __ov_gpio_start[], __ov_gpio_end[];
extern uint8_t    __ov_spi_load[], __ov_spi_start[], __ov_spi_end[];
extern uint8_t    __ov_jtag_load[], __ov_jtag_start[], __ov_jtag_end[];
volatile uint32_t SWJ_delayLoops[GPIO_TIMING_COUNT];
uint32_t          SWJ_requestedHz = DAP_DEFAULT_SWJ_CLOCK, SWJ_actualHz;
enum swj_backend  SWJ_backend     = SWJ_NONE;
uint8_t           SWJ_spiDiv;
uint32_t          SWJ_spiBudgetCycles;
uint32_t          SWJ_spiTimeouts;
static uint8_t    SWJ_currentPort;

static void SWJ_LoadBackend(enum swj_backend id) {
    if (SWJ_backend == id) {
        return;
    }

    /* Called exclusively at a command boundary, never from an ISR or an overlay. */
    const uint8_t *load;
    uint8_t       *dst, *end;

    if (id == SWJ_GPIO_SWD) {
        load = __ov_gpio_load;
        dst  = __ov_gpio_start;
        end  = __ov_gpio_end;
    } else if (id == SWJ_SPI_SWD) {
        load = __ov_spi_load;
        dst  = __ov_spi_start;
        end  = __ov_spi_end;
    } else {
        load = __ov_jtag_load;
        dst  = __ov_jtag_start;
        end  = __ov_jtag_end;
    }
    R8_SPI_CTRL_MOD = 0;
    memcpy(dst, load, (unsigned)(end - dst));
    __asm__ volatile("fence.i" ::: "memory");
    SWJ_backend = id;
}

bool SWJ_Clock(uint32_t hz) {
    if (!hz) {
        return false;
    }
    SWJ_requestedHz       = hz;
    SWJ_spiBudgetCycles   = BSP_clockHz / 1000U;
    enum swj_backend next = SWJ_GPIO_SWD;
    if (SWJ_currentPort == 2) {
        next = SWJ_GPIO_JTAG;
    } else if (hz >= SWD_SPI_THRESHOLD) {
        next = SWJ_SPI_SWD;
    }
    if (next == SWJ_SPI_SWD) {
        uint32_t target = hz > SWJ_MAX_SPI_HZ ? SWJ_MAX_SPI_HZ : hz;
        uint32_t div    = (BSP_clockHz + target - 1) / target;

        if (div < 2) {
            div = 2;
        }
        if (div & 1) {
            ++div;
        }
        if (div > 254) {
            next = SWJ_GPIO_SWD;
        } else {
            SWJ_spiDiv   = div;
            SWJ_actualHz = BSP_clockHz / div;
        }
    }
    {
        uint32_t target = hz > SWJ_MAX_GPIO_HZ ? SWJ_MAX_GPIO_HZ : hz;
        uint32_t half   = (BSP_clockHz / 2 + target - 1) / target;

        SWJ_delayLoops[GPIO_TIMING_SHIFT] =
            GPIO_DelayLoops(half, GPIO_SHIFT_HALF_BASE, GPIO_SHIFT_DELAY_BASE);
        SWJ_delayLoops[GPIO_TIMING_JTAG] =
            GPIO_DelayLoops(half, GPIO_JTAG_HALF_BASE, GPIO_JTAG_DELAY_BASE);
        SWJ_delayLoops[GPIO_TIMING_CLOCK] =
            GPIO_DelayLoops(half, GPIO_CLOCK_HALF_BASE, GPIO_CLOCK_DELAY_BASE);
        SWJ_delayLoops[GPIO_TIMING_ACK] =
            GPIO_DelayLoops(half, GPIO_ACK_HALF_BASE, GPIO_ACK_DELAY_BASE);
        if (next != SWJ_SPI_SWD) {
            unsigned kind = next == SWJ_GPIO_JTAG ? GPIO_TIMING_JTAG : GPIO_TIMING_SHIFT;
            unsigned base = kind == GPIO_TIMING_JTAG ? GPIO_JTAG_HALF_BASE : GPIO_SHIFT_HALF_BASE;
            unsigned delay =
                kind == GPIO_TIMING_JTAG ? GPIO_JTAG_DELAY_BASE : GPIO_SHIFT_DELAY_BASE;
            SWJ_actualHz = BSP_clockHz / GPIO_BitCycles(base, delay, SWJ_delayLoops[kind]);
        }
    }
    SWJ_LoadBackend(next);
    if (next == SWJ_SPI_SWD) {
        /* Frequency is stable throughout a command. ALL_CLEAR resets the
         * FIFO/count/flags, retaining these configuration registers. */
        R8_SPI_CTRL_CFG =
            RB_SPI_BIT_ORDER | RB_SPI_AUTO_IF | (SWJ_spiDiv == 2 ? RB_SPI_MST_DLY_EN : 0);
        R8_SPI_CLOCK_DIV = SWJ_spiDiv;
    }
    return true;
}

void SWJ_Init(void) {
    SWJ_Off();
    SWJ_Clock(DAP_DEFAULT_SWJ_CLOCK);
}

void SWJ_Port(uint8_t port) {
    if (!BSP_LocalSelected() || BSP_stopRequested) {
        DAP_data.debug_port = 0;
        return;
    }
    SWJ_currentPort = port;
    SWJ_Clock(SWJ_requestedHz);
    if (BSP_stopRequested) {
        DAP_data.debug_port = 0;
        return;
    }
    SWJ_DebugPinsSetup(port);
    if (BSP_stopRequested) {
        SWJ_Off();
    }
}

void SWJ_Off(void) {
    SWJ_currentPort = 0;
    {
        R32_PA_DIR    &= ~(PIN_DEBUG);
        R32_PA_PU     &= ~(PIN_DEBUG);
        R32_PA_PD_DRV &= ~(PIN_DEBUG);
    }
    R8_SPI_CTRL_MOD = 0;
    R32_PA_CLR      = PIN_RESET;
}

__HIGH_CODE uint32_t SWJ_GpioInput(uint32_t mask) {
    return !!(R32_PA_PIN & mask);
}

__HIGH_CODE void SWJ_GpioOutput(uint32_t mask, uint32_t bit) {
    if (bit & 1) {
        R32_PA_SET = mask;
    } else {
        R32_PA_CLR = mask;
    }
}

__HIGH_CODE void SWJ_DioEnable(bool on) {
    /* SPI releases peripheral ownership at every backend_shift boundary.
     * Turnaround needs only DIR switching; data is set by the next bit cycle. */
    if (on && SWJ_currentPort && !BSP_stopRequested) {
        R32_PA_DIR |= PIN_DIO;
    } else {
        R32_PA_DIR &= ~PIN_DIO;
    };
    if (BSP_stopRequested) {
        SWJ_Off();
    }
}

void SWJ_ResetOutput(uint32_t bit) {
    R32_PA_CLR = PIN_RESET;
    if (!(bit & 1) && BSP_LocalSelected() && !BSP_stopRequested) {
        R32_PA_DIR |= PIN_RESET;
    } else {
        R32_PA_DIR &= ~PIN_RESET;
    }
    if (BSP_stopRequested) {
        SWJ_Off();
    }
}

void SWJ_Sequence(uint32_t count, const uint8_t *data) {
    while (count && !BSP_stopRequested) {
        uint32_t n = count > 32 ? 32 : count, bytes = (n + 7) / 8;
        uint32_t value = 0;

        for (unsigned i = 0; i < bytes; i++) {
            value |= (uint32_t)data[i] << (8 * i);
        }
        GPIO_Shift(value, n, PIN_DIO, 0);
        data  += bytes;
        count -= n;
    }
}

void SWD_Sequence(uint32_t info, const uint8_t *tx, uint8_t *rx) {
    if (BSP_stopRequested || SWJ_currentPort != 1) {
        return;
    }
    if (SWJ_backend == SWJ_SPI_SWD) {
        SWD_SpiSequence(info, tx, rx);
    } else {
        SWD_GpioSequence(info, tx, rx);
    }
}

__HIGH_CODE uint8_t SWD_Transfer(uint32_t req, const uint8_t *tx, uint8_t *rx) {
    if (BSP_stopRequested || SWJ_currentPort != 1) {
        return DAP_TRANSFER_ERROR;
    }
    uint8_t result;

    if (SWJ_backend == SWJ_SPI_SWD) {
        result = SWD_SpiTransfer(req, tx, rx);

    } else {
        result = SWD_GpioTransfer(req, tx, rx);
    }
    return result;
}

/* Very low requested clocks can make even one 32-bit GPIO batch exceed
 * WTR's 336 ms at 100 MHz. Keep such half-periods bounded and fed without
 * inserting watchdog work into the balanced, ordinary bit loops. */
uint32_t GPIO_ShiftSlow(uint32_t tx, uint32_t bits, uint32_t drive, uint32_t sample) {
    uint32_t half  = (500000U + SWJ_requestedHz - 1U) / SWJ_requestedHz;
    uint32_t value = 0;

    for (uint32_t bit = 0; bit < bits && !BSP_stopRequested; ++bit) {
        R32_PA_CLR = PIN_CLK | drive;
        if (tx & 1U)
            R32_PA_SET = drive;
        tx >>= 1;
        BSP_DelayUs(half);
        if (sample && (R32_PA_PIN & sample))
            value |= 1U << bit;
        R32_PA_SET = PIN_CLK;
        BSP_DelayUs(half);
    }
    return value;
}

void JTAG_Sequence(uint32_t info, const uint8_t *tx, uint8_t *rx) {
    if (!BSP_stopRequested && SWJ_currentPort == 2) {
        JTAG_GpioSequence(info, tx, rx);
    }
}

void JTAG_Ir(uint32_t ir) {
    if (!BSP_stopRequested && SWJ_currentPort == 2) {
        JTAG_GpioIr(ir);
    }
}

uint32_t JTAG_ReadIdCode(void) {
    return BSP_stopRequested || SWJ_currentPort != 2 ? 0 : JTAG_GpioIdCode();
}

void JTAG_WriteAbort(uint32_t d) {
    if (!BSP_stopRequested && SWJ_currentPort == 2) {
        JTAG_GpioAbort(d);
    }
}

__SWJ_JTAG_CODE uint8_t JTAG_Transfer(uint32_t r, uint32_t *d) {
    return BSP_stopRequested || SWJ_currentPort != 2 ? DAP_TRANSFER_ERROR : JTAG_GpioTransfer(r, d);
}
