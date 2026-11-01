/* The USB-free TARGET owns both typed consumers in one foreground thread. */
#include "ch572/ch572.h"
#include "bsp/board.h"
#include "rf/rf_bridge.h"
#include "rf/rf_dap.h"
#include "rf/rf_uart.h"
#include "uart/uart_io.h"
#include "dap/dap_queue.h"
#include "dap/dap_config.h"
#include <string.h>

bool RF_BridgeTransportReset(void) {
    /* Called before replacing the reliable stream generation. No USB DMA
     * owner exists in this image; SWD execution cannot nest this foreground. */
    RF_UartReset();
    RF_DapReset();
    return true;
}
