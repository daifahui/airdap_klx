#ifndef RF_DAP_H
#define RF_DAP_H

#include <stdint.h>
#include "ch572/attributes.h"
#include <stdbool.h>
#include "app_config.h"

#if (APP_COMBINED_TARGET || APP_COMBINED_HOST)
#include "dap/dap_queue.h"

extern dap_queue_t DAP_queue __ALIGNED(DAP_QUEUE_ALIGNMENT);
#endif

#if APP_COMBINED_HOST
extern volatile uint8_t RF_dapReceiving;

void RF_DapAbort(uint8_t source);

/* Called only by the USB owner; UART's foreground producer excludes USB. */

void RF_DapTransmit(void);
#endif

void RF_DapPoll(bool beforeReply);

void RF_DapReset(void);
#endif
