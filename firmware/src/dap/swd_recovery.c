/* SPDX-License-Identifier: Apache-2.0 */
#include "dap/dap_config.h"
#include "bsp/board.h"
#include "dap/swj.h"
#include "dap/dap.h"

/* No overlay dependency: GPIO_Shift is resident highcode. Both SWD backends
 * may recover while their own code occupies the shared overlay VMA. */
uint8_t SWD_Recover(uint32_t req, uint32_t ack) {
    bool     read       = (req & DAP_TRANSFER_RnW) != 0;
    bool     valid      = ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT;
    bool     writePhase = valid && !read && DAP_data.swd_conf.data_phase;
    unsigned clocks     = DAP_data.swd_conf.turnaround;

    if (!valid || (read && DAP_data.swd_conf.data_phase)) {
        clocks += 33;
    }
    for (;;) {
        while (clocks && !BSP_stopRequested) {
            unsigned count = clocks > 32 ? 32 : clocks;

            GPIO_Shift(0, count, 0, 0);
            clocks -= count;
        }
        SWJ_DioEnable(true);
        if (!writePhase || BSP_stopRequested) {
            break;
        }
        SWJ_GpioOutput(PIN_DIO, 0);
        clocks     = 33;
        writePhase = false;
    }
    return valid ? ack : DAP_TRANSFER_ERROR;
}
