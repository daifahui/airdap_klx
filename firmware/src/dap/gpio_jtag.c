/* SPDX-License-Identifier: Apache-2.0 */
/* ADIv5 JTAG scan semantics follow Arm CMSIS-DAP JTAG_DP.c. Batch shift data
 * through the shared SRAM loop; TAP transition/last-bit boundaries may stretch. */
#include "dap/dap_config.h"
#include "bsp/board.h"
#include "dap/swj.h"
#include "dap/dap.h"

static uint32_t JTAG_Step(unsigned tms, unsigned tdi, bool capture) {
    SWJ_GpioOutput(PIN_DIO, tms);
    return GPIO_Shift(tdi, 1, PIN_TDI, capture ? PIN_TDO : 0);
}

static void JTAG_Bypass(unsigned n, bool exit) {
    if (exit && n) {
        --n;
    }
    while (n) {
        unsigned k = n > 32 ? 32 : n;

        GPIO_Shift(0xFFFFFFFF, k, PIN_TDI, 0);
        n -= k;
    }
    if (exit) {
        JTAG_Step(1, 1, false);
    }
}

static void JTAG_Enter(bool ir) {
    JTAG_Step(1, 1, false);
    if (ir) {
        JTAG_Step(1, 1, false);
    }
    JTAG_Step(0, 1, false);
    JTAG_Step(0, 1, false);
}

static void JTAG_Finish(void) {
    JTAG_Step(1, 1, false);
    JTAG_Step(0, 1, false);
    SWJ_GpioOutput(PIN_TDI, 1);
}

static uint32_t JTAG_Data32(uint32_t value, bool capture) {
    unsigned after = DAP_data.jtag_dev.count - DAP_data.jtag_dev.index - 1;

    if (after) {
        uint32_t out = GPIO_Shift(value, 32, PIN_TDI, capture ? PIN_TDO : 0);

        JTAG_Bypass(after, true);
        return out;
    }
    uint32_t out = GPIO_Shift(value, 31, PIN_TDI, capture ? PIN_TDO : 0);

    return out | (JTAG_Step(1, value >> 31, capture) << 31);
}

void JTAG_GpioSequence(uint32_t info, const uint8_t *tx, uint8_t *rx) {
    unsigned n = info & JTAG_SEQUENCE_TCK;

    if (!n) {
        n = 64;
    }
    SWJ_GpioOutput(PIN_DIO, !!(info & JTAG_SEQUENCE_TMS));
    while (n && !BSP_stopRequested) {
        unsigned k = n > 32 ? 32 : n, bytes = (k + 7) / 8;
        uint32_t v = 0;

        for (unsigned i = 0; i < bytes; i++) {
            v |= (uint32_t)tx[i] << (8 * i);
        }
        uint32_t out = GPIO_Shift(v, k, PIN_TDI, ((info & JTAG_SEQUENCE_TDO) != 0) ? PIN_TDO : 0);

        if (info & JTAG_SEQUENCE_TDO) {
            for (unsigned i = 0; i < bytes; i++) {
                rx[i] = out >> (8 * i);
            }
            rx += bytes;
        }
        tx += bytes;
        n  -= k;
    }
}

void JTAG_GpioIr(uint32_t ir) {
    unsigned index = DAP_data.jtag_dev.index;

    JTAG_Enter(true);
    JTAG_Bypass(DAP_data.jtag_dev.ir_before[index], false);
    unsigned n     = DAP_data.jtag_dev.ir_length[index];
    unsigned after = DAP_data.jtag_dev.ir_after[index];

    if (after) {
        GPIO_Shift(ir, n, PIN_TDI, 0);
        JTAG_Bypass(after, true);
    } else {
        if (n > 1) {
            GPIO_Shift(ir, n - 1, PIN_TDI, 0);
        }
        JTAG_Step(1, ir >> (n - 1), false);
    }
    JTAG_Finish();
}

uint8_t JTAG_GpioTransfer(uint32_t req, uint32_t *data) {
    JTAG_Enter(false);
    JTAG_Bypass(DAP_data.jtag_dev.index, false);
    uint32_t raw = GPIO_Shift(req >> 1, 3, PIN_TDI, PIN_TDO);
    uint8_t  ack = ((raw & 1) << 1) | ((raw & 2) >> 1) | (raw & 4);

    if (ack == DAP_TRANSFER_OK) {
        uint32_t value = (req & DAP_TRANSFER_RnW) ? 0 : (data ? *data : 0);
        uint32_t read  = JTAG_Data32(value, (req & DAP_TRANSFER_RnW) != 0);

        if ((req & DAP_TRANSFER_RnW) && data) {
            *data = read;
        }
    } else {
        JTAG_Step(1, 1, false);
    }
    JTAG_Finish();

#if DAP_TIMESTAMP_ENABLE
    if (req & DAP_TRANSFER_TIMESTAMP) {
        DAP_data.timestamp = TIMESTAMP_GET();
    }
#endif

    JTAG_Bypass(DAP_data.transfer.idle_cycles, false);
    return BSP_stopRequested ? DAP_TRANSFER_ERROR : ack;
}

uint32_t JTAG_GpioIdCode(void) {
    JTAG_Enter(false);
    JTAG_Bypass(DAP_data.jtag_dev.index, false);
    uint32_t out = JTAG_Data32(0, true);

    JTAG_Finish();
    return out;
}

void JTAG_GpioAbort(uint32_t data) {
    JTAG_Enter(false);
    JTAG_Bypass(DAP_data.jtag_dev.index, false);
    GPIO_Shift(0, 3, PIN_TDI, 0);
    JTAG_Data32(data, false);
    JTAG_Finish();
}
