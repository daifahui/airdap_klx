/* Modified for AirDAP; see NOTICE. */
/*
 * Copyright (c) 2013-2022 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 *
 * Project:      CMSIS-DAP Source
 * Title:        DAP.c CMSIS-DAP Commands
 *
 *---------------------------------------------------------------------------*/

#include <string.h>
#include "ch572/attributes.h"
#include "dap/dap_config.h"
#include "bsp/board.h"
#include "dap/swj.h"
#include "dap/dap.h"

#if DAP_PACKET_COUNT < 1U
#error "Minimum Packet Count is 1!"
#endif

#if DAP_PACKET_COUNT > 255U
#error "Maximum Packet Count is 255!"
#endif

#define __DAP_TRANSFER_BLOCK_CODE __HIGH_CODE_SECTION("dap_block") __NO_INLINE

#if APP_COMBINED_TARGET
#define __DAP_JTAG_BLOCK_CODE __NO_INLINE
#else
#define __DAP_JTAG_BLOCK_CODE __DAP_TRANSFER_BLOCK_CODE
#endif

#if !APP_HAS_RF
#define __DAP_PROCESS_CODE __DAP_TRANSFER_BLOCK_CODE
#else
#define __DAP_PROCESS_CODE
#endif

// Clock Macros
#define MAX_SWJ_CLOCK(delay_cycles) ((CPU_CLOCK / 2U) / (IO_PORT_WRITE_CYCLES + delay_cycles))

DAP_Data_t       DAP_data;          // DAP Data
volatile uint8_t DAP_transferAbort; // Transfer Abort Flag

static const char DAP_fwVer[] = DAP_FW_VER;

// Common clock delay calculation routine
//   clock:    requested SWJ frequency in Hertz
static void DAP_SetClockDelay(uint32_t clock) {
    DAP_data.fast_clock  = 0;
    DAP_data.clock_delay = 1;
    SWJ_Clock(clock);
}

// Get DAP Information
//   id:      info identifier
//   info:    pointer to info data
//   return:  number of bytes in info data
static uint8_t DAP_Info(uint8_t id, uint8_t *info) {
    uint8_t length = 0U;

    switch (id) {
        case DAP_ID_VENDOR:
            length = (uint8_t)(strlen("WCH probe project") + 1);
            memcpy(info, "WCH probe project", length);
            break;
        case DAP_ID_PRODUCT:
            length = (uint8_t)(strlen("AirDAP") + 1);
            memcpy(info, "AirDAP", length);
            break;
        case DAP_ID_SER_NUM:
            length = (uint8_t)(strlen(BSP_serial) + 1);
            memcpy(info, BSP_serial, length);
            break;
        case DAP_ID_DAP_FW_VER:
            length = (uint8_t)sizeof(DAP_fwVer);
            memcpy(info, DAP_fwVer, length);
            break;
        case DAP_ID_DEVICE_VENDOR:
            length = 0U;
            break;
        case DAP_ID_DEVICE_NAME:
            length = 0U;
            break;
        case DAP_ID_BOARD_VENDOR:
            length = 0U;
            break;
        case DAP_ID_BOARD_NAME:
            length = 0U;
            break;
        case DAP_ID_PRODUCT_FW_VER:
            length = (uint8_t)(strlen("0.1.0-dev") + 1);
            memcpy(info, "0.1.0-dev", length);
            break;
        case DAP_ID_CAPABILITIES:
            info[0] = ((DAP_SWD != 0) ? (1U << 0) : 0U) | ((DAP_JTAG != 0) ? (1U << 1) : 0U) |
                      ((DAP_ATOMIC_COMMANDS != 0) ? (1U << 4) : 0U) |
                      ((TIMESTAMP_CLOCK != 0U) ? (1U << 5) : 0U) |
                      ((DAP_UART != 0U) ? (1U << 7) : 0U);

            info[1] = ((DAP_UART_USB_COM_PORT != 0) ? (1U << 0) : 0U);
            length  = 2U;
            break;
        case DAP_ID_TIMESTAMP_CLOCK:

#if TIMESTAMP_CLOCK != 0U
            info[0] = (uint8_t)(TIMESTAMP_CLOCK >> 0);
            info[1] = (uint8_t)(TIMESTAMP_CLOCK >> 8);
            info[2] = (uint8_t)(TIMESTAMP_CLOCK >> 16);
            info[3] = (uint8_t)(TIMESTAMP_CLOCK >> 24);
            length  = 4U;
#endif

            break;
        case DAP_ID_UART_RX_BUFFER_SIZE:

#if DAP_UART != 0
            info[0] = (uint8_t)(DAP_UART_RX_BUFFER_SIZE >> 0);
            info[1] = (uint8_t)(DAP_UART_RX_BUFFER_SIZE >> 8);
            info[2] = (uint8_t)(DAP_UART_RX_BUFFER_SIZE >> 16);
            info[3] = (uint8_t)(DAP_UART_RX_BUFFER_SIZE >> 24);
            length  = 4U;
#endif

            break;
        case DAP_ID_UART_TX_BUFFER_SIZE:

#if DAP_UART != 0
            info[0] = (uint8_t)(DAP_UART_TX_BUFFER_SIZE >> 0);
            info[1] = (uint8_t)(DAP_UART_TX_BUFFER_SIZE >> 8);
            info[2] = (uint8_t)(DAP_UART_TX_BUFFER_SIZE >> 16);
            info[3] = (uint8_t)(DAP_UART_TX_BUFFER_SIZE >> 24);
            length  = 4U;
#endif

            break;
        case DAP_ID_PACKET_SIZE:
            info[0] = (uint8_t)(DAP_PACKET_SIZE >> 0);
            info[1] = (uint8_t)(DAP_PACKET_SIZE >> 8);
            length  = 2U;
            break;
        case DAP_ID_PACKET_COUNT:
            info[0] = DAP_PACKET_COUNT;
            length  = 1U;
            break;
        default:
            break;
    }

    return (length);
}

// Process Delay command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_Delay(const uint8_t *req, uint8_t *resp) {
    uint32_t delay;

    delay = (uint32_t)(*(req + 0)) | (uint32_t)(*(req + 1) << 8);
    BSP_DelayUs(delay);

    *resp = DAP_OK;
    return ((2U << 16) | 1U);
}

// Process Host Status command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_HostStatus(const uint8_t *req, uint8_t *resp) {
    switch (*req) {
        case DAP_DEBUGGER_CONNECTED:
            BSP_LedStatus(0, (*(req + 1) & 1U));
            break;
        case DAP_TARGET_RUNNING:
            BSP_LedStatus(1, (*(req + 1) & 1U));
            break;
        default:
            *resp = DAP_ERROR;
            return ((2U << 16) | 1U);
    }

    *resp = DAP_OK;
    return ((2U << 16) | 1U);
}

// Process Connect command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_Connect(const uint8_t *req, uint8_t *resp) {
    uint32_t port;

    if (*req == DAP_PORT_AUTODETECT) {
        port = DAP_DEFAULT_PORT;
    } else {
        port = *req;
    }

    switch (port) {
#if DAP_SWD != 0
        case DAP_PORT_SWD:
            DAP_data.debug_port = DAP_PORT_SWD;
            SWJ_Port(1);
            break;
#endif

#if DAP_JTAG != 0
        case DAP_PORT_JTAG:
            DAP_data.debug_port = DAP_PORT_JTAG;
            SWJ_Port(2);
            break;
#endif

        default:
            port = DAP_PORT_DISABLED;
            break;
    }

    *resp = DAP_data.debug_port == 0 ? 0 : (uint8_t)port;
    return ((1U << 16) | 1U);
}

// Process Disconnect command and prepare response
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_Disconnect(uint8_t *resp) {
    DAP_data.debug_port = DAP_PORT_DISABLED;
    SWJ_Off();

    *resp = DAP_OK;
    return (1U);
}

// Process Reset Target command and prepare response
//   response: pointer to response data
//   return:   number of bytes in response
static uint32_t DAP_ResetTarget(uint8_t *resp) {
    SWJ_ResetOutput(0);
    BSP_DelayUs(1000);
    SWJ_ResetOutput(1);
    *(resp + 1) = 1;
    *(resp + 0) = DAP_OK;
    return (2U);
}

// Process SWJ Pins command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SwjPins(const uint8_t *req, uint8_t *resp) {
#if (DAP_SWD != 0 || DAP_JTAG != 0)
    uint32_t value;
    uint32_t select;
    uint32_t wait;

    value  = (uint32_t)*(req + 0);
    select = (uint32_t)*(req + 1);
    wait   = (uint32_t)(*(req + 2) << 0) | (uint32_t)(*(req + 3) << 8) |
           (uint32_t)(*(req + 4) << 16) | ((uint32_t)*(req + 5) << 24);

    if ((select & (1U << DAP_SWJ_SWCLK_TCK)) != 0U) {
        if ((value & (1U << DAP_SWJ_SWCLK_TCK)) != 0U) {
            SWJ_GpioOutput(PIN_CLK, 1);
        } else {
            SWJ_GpioOutput(PIN_CLK, 0);
        }
    }
    if ((select & (1U << DAP_SWJ_SWDIO_TMS)) != 0U) {
        if ((value & (1U << DAP_SWJ_SWDIO_TMS)) != 0U) {
            SWJ_GpioOutput(PIN_DIO, 1);
        } else {
            SWJ_GpioOutput(PIN_DIO, 0);
        }
    }
    if ((select & (1U << DAP_SWJ_TDI)) != 0U) {
        SWJ_GpioOutput(PIN_TDI, (value >> DAP_SWJ_TDI) & 1U);
    }
    if ((select & (1U << DAP_SWJ_nTRST)) != 0U) {
        (void)(value >> DAP_SWJ_nTRST);
    }
    if ((select & (1U << DAP_SWJ_nRESET)) != 0U) {
        SWJ_ResetOutput(value >> DAP_SWJ_nRESET);
    }

    if (wait != 0U) {
        if (wait > 3000000U) {
            wait = 3000000U;
        }
        uint32_t startMs = BSP_ms;

#if TIMESTAMP_CLOCK != 0U
        if (wait > 3000000U) {
            wait = 3000000U;
        }

#if TIMESTAMP_CLOCK >= 1000000U
        wait *= TIMESTAMP_CLOCK / 1000000U;
#else
        wait /= 1000000U / TIMESTAMP_CLOCK;
#endif

#else
        wait = (wait + 999U) / 1000U;
#endif

        do {
            if ((select & (1U << DAP_SWJ_SWCLK_TCK)) != 0U) {
                if (((value >> DAP_SWJ_SWCLK_TCK) & 1U) ^ SWJ_GpioInput(PIN_CLK)) {
                    continue;
                }
            }
            if ((select & (1U << DAP_SWJ_SWDIO_TMS)) != 0U) {
                if (((value >> DAP_SWJ_SWDIO_TMS) & 1U) ^ SWJ_GpioInput(PIN_DIO)) {
                    continue;
                }
            }
            if ((select & (1U << DAP_SWJ_TDI)) != 0U) {
                if (((value >> DAP_SWJ_TDI) & 1U) ^ SWJ_GpioInput(PIN_TDI)) {
                    continue;
                }
            }
            if ((select & (1U << DAP_SWJ_nTRST)) != 0U) {
                if (((value >> DAP_SWJ_nTRST) & 1U) ^ 1U) {
                    continue;
                }
            }
            if ((select & (1U << DAP_SWJ_nRESET)) != 0U) {
                if (((value >> DAP_SWJ_nRESET) & 1U) ^ SWJ_GpioInput(PIN_RESET)) {
                    continue;
                }
            }
            break;
        } while (!BSP_stopRequested && (BSP_ms - startMs) < wait);
    }

    value = (SWJ_GpioInput(PIN_CLK) << DAP_SWJ_SWCLK_TCK) |
            (SWJ_GpioInput(PIN_DIO) << DAP_SWJ_SWDIO_TMS) |
            (SWJ_GpioInput(PIN_TDI) << DAP_SWJ_TDI) | (SWJ_GpioInput(PIN_TDO) << DAP_SWJ_TDO) |
            (1U << DAP_SWJ_nTRST) | (SWJ_GpioInput(PIN_RESET) << DAP_SWJ_nRESET);

    *resp = (uint8_t)value;
#else
    *resp = 0U;
#endif

    return ((6U << 16) | 1U);
}

// Process SWJ Clock command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SwjClock(const uint8_t *req, uint8_t *resp) {
#if (DAP_SWD != 0 || DAP_JTAG != 0)
    uint32_t clock;

    clock = (uint32_t)(*(req + 0) << 0) | (uint32_t)(*(req + 1) << 8) |
            (uint32_t)(*(req + 2) << 16) | ((uint32_t)*(req + 3) << 24);

    if (clock == 0U) {
        *resp = DAP_ERROR;
        return ((4U << 16) | 1U);
    }

    DAP_SetClockDelay(clock);

    *resp = DAP_OK;
#else
    *resp = DAP_ERROR;
#endif

    return ((4U << 16) | 1U);
}

// Process SWJ Sequence command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SwjSequence(const uint8_t *req, uint8_t *resp) {
    uint32_t count;

    count = *req++;
    if (count == 0U) {
        count = 256U;
    }

#if (DAP_SWD != 0 || DAP_JTAG != 0)
    SWJ_Sequence(count, req);
    *resp = DAP_OK;
#else
    *resp = DAP_ERROR;
#endif

    count = (count + 7U) >> 3;

    return (((count + 1U) << 16) | 1U);
}

// Process SWD Configure command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SwdConfigure(const uint8_t *req, uint8_t *resp) {
#if DAP_SWD != 0
    uint8_t value;

    value                        = *req;
    DAP_data.swd_conf.turnaround = (value & 0x03U) + 1U;
    DAP_data.swd_conf.data_phase = (value & 0x04U) ? 1U : 0U;

    *resp = DAP_OK;
#else
    *resp = DAP_ERROR;
#endif

    return ((1U << 16) | 1U);
}

// Process SWD Sequence command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_SwdSequence(const uint8_t *req, uint8_t *resp) {
    uint32_t sequenceInfo;
    uint32_t sequenceCount;
    uint32_t requestCount;
    uint32_t responseCount;
    uint32_t count;

#if DAP_SWD != 0
    *resp++ = DAP_OK;
#else
    *resp++ = DAP_ERROR;
#endif

    requestCount  = 1U;
    responseCount = 1U;

    sequenceCount = *req++;
    while (sequenceCount--) {
        sequenceInfo = *req++;
        count        = sequenceInfo & SWD_SEQUENCE_CLK;
        if (count == 0U) {
            count = 64U;
        }
        count = (count + 7U) / 8U;

#if DAP_SWD != 0
        if ((sequenceInfo & SWD_SEQUENCE_DIN) != 0U) {
            SWJ_DioEnable(false);
        } else {
            SWJ_DioEnable(true);
        }
        SWD_Sequence(sequenceInfo, req, resp);
        if (sequenceCount == 0U) {
            SWJ_DioEnable(true);
        }
#endif

        if ((sequenceInfo & SWD_SEQUENCE_DIN) != 0U) {
            requestCount++;

#if DAP_SWD != 0
            resp          += count;
            responseCount += count;
#endif
        } else {
            req          += count;
            requestCount += count + 1U;
        }
    }

    return ((requestCount << 16) | responseCount);
}

// Process JTAG Sequence command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_JtagSequence(const uint8_t *req, uint8_t *resp) {
    uint32_t sequenceInfo;
    uint32_t sequenceCount;
    uint32_t requestCount;
    uint32_t responseCount;
    uint32_t count;

#if DAP_JTAG != 0
    *resp++ = DAP_OK;
#else
    *resp++ = DAP_ERROR;
#endif

    requestCount  = 1U;
    responseCount = 1U;

    sequenceCount = *req++;
    while (sequenceCount--) {
        sequenceInfo = *req++;
        count        = sequenceInfo & JTAG_SEQUENCE_TCK;
        if (count == 0U) {
            count = 64U;
        }
        count = (count + 7U) / 8U;

#if DAP_JTAG != 0
        JTAG_Sequence(sequenceInfo, req, resp);
#endif

        req          += count;
        requestCount += count + 1U;

#if DAP_JTAG != 0
        if ((sequenceInfo & JTAG_SEQUENCE_TDO) != 0U) {
            resp          += count;
            responseCount += count;
        }
#endif
    }

    return ((requestCount << 16) | responseCount);
}

// Process JTAG Configure command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_JtagConfigure(const uint8_t *req, uint8_t *resp) {
    uint32_t count;

#if DAP_JTAG != 0
    uint32_t length;
    uint32_t bits;
    uint32_t n;

    count                   = *req++;
    DAP_data.jtag_dev.count = (uint8_t)count;

    bits = 0U;
    for (n = 0U; n < count; n++) {
        length                          = *req++;
        DAP_data.jtag_dev.ir_length[n]  = (uint8_t)length;
        DAP_data.jtag_dev.ir_before[n]  = (uint16_t)bits;
        bits                           += length;
    }
    for (n = 0U; n < count; n++) {
        bits                          -= DAP_data.jtag_dev.ir_length[n];
        DAP_data.jtag_dev.ir_after[n]  = (uint16_t)bits;
    }

    *resp = DAP_OK;
#else
    count = *req;
    *resp = DAP_ERROR;
#endif

    return (((count + 1U) << 16) | 1U);
}

// Process JTAG IDCODE command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_JtagIdCode(const uint8_t *req, uint8_t *resp) {
#if DAP_JTAG != 0
    uint32_t data;

    if (DAP_data.debug_port != DAP_PORT_JTAG) {
        goto id_error;
    }

    // Device index (JTAP TAP)
    DAP_data.jtag_dev.index = *req;
    if (DAP_data.jtag_dev.index >= DAP_data.jtag_dev.count) {
        goto id_error;
    }

    // Select JTAG chain
    JTAG_Ir(JTAG_IDCODE);

    // Read IDCODE register
    data = JTAG_ReadIdCode();

    // Store Data
    *(resp + 0) = DAP_OK;
    *(resp + 1) = (uint8_t)(data >> 0);
    *(resp + 2) = (uint8_t)(data >> 8);
    *(resp + 3) = (uint8_t)(data >> 16);
    *(resp + 4) = (uint8_t)(data >> 24);

    return ((1U << 16) | 5U);

id_error:
#endif

    *resp = DAP_ERROR;
    return ((1U << 16) | 1U);
}

// Process Transfer Configure command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_TransferConfigure(const uint8_t *req, uint8_t *resp) {
    DAP_data.transfer.idle_cycles = *(req + 0);
    DAP_data.transfer.retry_count = (uint16_t)*(req + 1) | (uint16_t)(*(req + 2) << 8);
    DAP_data.transfer.match_retry = (uint16_t)*(req + 3) | (uint16_t)(*(req + 4) << 8);

    *resp = DAP_OK;
    return ((5U << 16) | 1U);
}

// Process SWD Transfer command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)

#if DAP_SWD != 0
static uint32_t DAP_SwdTransfer(const uint8_t *req, uint8_t *resp) {
    const uint8_t *requestHead;
    uint32_t       requestCount;
    uint32_t       requestValue;
    uint8_t       *responseHead;
    uint32_t       responseCount;
    uint32_t       responseValue;
    uint32_t       postRead;
    uint32_t       checkWrite;
    uint32_t       matchValue;
    uint32_t       matchRetry;
    uint32_t       retry;
    uint32_t       data;

#if TIMESTAMP_CLOCK != 0U
    uint32_t timestamp;
#endif

    requestHead = req;

    responseCount  = 0U;
    responseValue  = 0U;
    responseHead   = resp;
    resp          += 2;

    // TransferAbort is cleared atomically by the USB command dispatcher.

    postRead   = 0U;
    checkWrite = 0U;

    req++; // Ignore DAP index

    requestCount = *req++;

    while (requestCount != 0) {
        requestCount--;
        requestValue = *req++;
        if ((requestValue & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if (postRead) {
                // Read was posted before
                retry = DAP_data.transfer.retry_count;
                if ((requestValue & (DAP_TRANSFER_APnDP | DAP_TRANSFER_MATCH_VALUE)) ==
                    DAP_TRANSFER_APnDP) {
                    // Read previous AP data and post next AP read
                    do {
                        responseValue = SWD_Transfer(requestValue, NULL, resp);
                    } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                } else {
                    // Read previous AP data
                    do {
                        responseValue = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL, resp);
                    } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                    postRead = 0U;
                }
                if (responseValue != DAP_TRANSFER_OK) {
                    break;
                }
                // Data already resides in the response buffer.
                resp += 4;

#if TIMESTAMP_CLOCK != 0U
                if (postRead) {
                    // Store Timestamp of next AP read
                    if ((requestValue & DAP_TRANSFER_TIMESTAMP) != 0U) {
                        timestamp = DAP_data.timestamp;
                        *resp++   = (uint8_t)timestamp;
                        *resp++   = (uint8_t)(timestamp >> 8);
                        *resp++   = (uint8_t)(timestamp >> 16);
                        *resp++   = (uint8_t)(timestamp >> 24);
                    }
                }
#endif
            }
            if ((requestValue & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                matchValue = (uint32_t)(*(req + 0) << 0) | (uint32_t)(*(req + 1) << 8) |
                             (uint32_t)(*(req + 2) << 16) | ((uint32_t)*(req + 3) << 24);
                req        += 4;
                matchRetry  = DAP_data.transfer.match_retry;
                if ((requestValue & DAP_TRANSFER_APnDP) != 0U) {
                    // Post AP read
                    retry = DAP_data.transfer.retry_count;
                    do {
                        responseValue = SWD_Transfer(requestValue, NULL, NULL);
                    } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                    if (responseValue != DAP_TRANSFER_OK) {
                        break;
                    }
                }
                do {
                    // Read register until its value matches or retry counter expires
                    retry = DAP_data.transfer.retry_count;
                    do {
                        responseValue = SWD_Transfer(requestValue, NULL, (uint8_t *)&data);
                    } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                    if (responseValue != DAP_TRANSFER_OK) {
                        break;
                    }
                } while (((data & DAP_data.transfer.match_mask) != matchValue) && matchRetry-- &&
                         !DAP_transferAbort);
                if ((data & DAP_data.transfer.match_mask) != matchValue) {
                    responseValue |= DAP_TRANSFER_MISMATCH;
                }
                if (responseValue != DAP_TRANSFER_OK) {
                    break;
                }
            } else {
                // Normal read
                retry = DAP_data.transfer.retry_count;
                if ((requestValue & DAP_TRANSFER_APnDP) != 0U) {
                    // Read AP register
                    if (postRead == 0U) {
                        // Post AP read
                        do {
                            responseValue = SWD_Transfer(requestValue, NULL, NULL);
                        } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- &&
                                 !DAP_transferAbort);
                        if (responseValue != DAP_TRANSFER_OK) {
                            break;
                        }

#if TIMESTAMP_CLOCK != 0U
                        // Store Timestamp
                        if ((requestValue & DAP_TRANSFER_TIMESTAMP) != 0U) {
                            timestamp = DAP_data.timestamp;
                            *resp++   = (uint8_t)timestamp;
                            *resp++   = (uint8_t)(timestamp >> 8);
                            *resp++   = (uint8_t)(timestamp >> 16);
                            *resp++   = (uint8_t)(timestamp >> 24);
                        }
#endif

                        postRead = 1U;
                    }
                } else {
                    // Read DP data directly after its optional timestamp.
                    uint8_t *readResponse = resp;

#if TIMESTAMP_CLOCK != 0U
                    if ((requestValue & DAP_TRANSFER_TIMESTAMP) != 0U) {
                        readResponse += 4;
                    }
#endif

                    do {
                        responseValue = SWD_Transfer(requestValue, NULL, readResponse);

                    } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                    if (responseValue != DAP_TRANSFER_OK) {
                        break;
                    }

#if TIMESTAMP_CLOCK != 0U
                    // Store Timestamp
                    if ((requestValue & DAP_TRANSFER_TIMESTAMP) != 0U) {
                        timestamp = DAP_data.timestamp;
                        *resp++   = (uint8_t)timestamp;
                        *resp++   = (uint8_t)(timestamp >> 8);
                        *resp++   = (uint8_t)(timestamp >> 16);
                        *resp++   = (uint8_t)(timestamp >> 24);
                    }
#endif

                    // Data already resides in the response buffer.
                    resp += 4;
                }
            }
            checkWrite = 0U;
        } else {
            // Write register
            if (postRead) {
                // Read previous data
                retry = DAP_data.transfer.retry_count;
                do {
                    responseValue = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL, resp);
                } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                if (responseValue != DAP_TRANSFER_OK) {
                    break;
                }
                // Data already resides in the response buffer.
                resp     += 4;
                postRead  = 0U;
            }
            req += 4;
            if ((requestValue & DAP_TRANSFER_MATCH_MASK) != 0U) {
                // Write match mask
                DAP_data.transfer.match_mask = (uint32_t)req[-4] | ((uint32_t)req[-3] << 8) |
                                               ((uint32_t)req[-2] << 16) |
                                               ((uint32_t)req[-1] << 24);
                responseValue = DAP_TRANSFER_OK;
            } else {
                // Write DP/AP register
                retry = DAP_data.transfer.retry_count;
                do {
                    responseValue = SWD_Transfer(requestValue, req - 4, NULL);
                } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                if (responseValue != DAP_TRANSFER_OK) {
                    break;
                }

#if TIMESTAMP_CLOCK != 0U
                // Store Timestamp
                if ((requestValue & DAP_TRANSFER_TIMESTAMP) != 0U) {
                    timestamp = DAP_data.timestamp;
                    *resp++   = (uint8_t)timestamp;
                    *resp++   = (uint8_t)(timestamp >> 8);
                    *resp++   = (uint8_t)(timestamp >> 16);
                    *resp++   = (uint8_t)(timestamp >> 24);
                }
#endif

                checkWrite = 1U;
            }
        }
        responseCount++;
        if (DAP_transferAbort) {
            break;
        }
    }

    while (requestCount != 0) {
        // Process canceled requests
        requestCount--;
        requestValue = *req++;
        if ((requestValue & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if ((requestValue & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                req += 4;
            }
        } else {
            // Write register
            req += 4;
        }
    }

    if (responseValue == DAP_TRANSFER_OK) {
        if (postRead) {
            // Read previous data
            retry = DAP_data.transfer.retry_count;
            do {
                responseValue = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL, resp);
            } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
            if (responseValue != DAP_TRANSFER_OK) {
                goto end;
            }
            // Data already resides in the response buffer.
            resp += 4;
        } else if (checkWrite) {
            // Check last write
            retry = DAP_data.transfer.retry_count;
            do {
                responseValue = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL, NULL);
            } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
        }
    }

end:
    *(responseHead + 0) = (uint8_t)responseCount;
    *(responseHead + 1) = (uint8_t)responseValue;

    return (((uint32_t)(req - requestHead) << 16) | (uint32_t)(resp - responseHead));
}
#endif

// Process JTAG Transfer command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)

#if DAP_JTAG != 0
static uint32_t DAP_JtagTransfer(const uint8_t *req, uint8_t *resp) {
    const uint8_t *requestHead;
    uint32_t       requestCount;
    uint32_t       requestValue;
    uint32_t       requestIr;
    uint8_t       *responseHead;
    uint32_t       responseCount;
    uint32_t       responseValue;
    uint32_t       postRead;
    uint32_t       matchValue;
    uint32_t       matchRetry;
    uint32_t       retry;
    uint32_t       data;
    uint32_t       ir;

#if TIMESTAMP_CLOCK != 0U
    uint32_t timestamp;
#endif

    requestHead = req;

    responseCount  = 0U;
    responseValue  = 0U;
    responseHead   = resp;
    resp          += 2;

    // TransferAbort is cleared atomically by the USB command dispatcher.

    ir       = 0U;
    postRead = 0U;

    // Device index (JTAP TAP)
    DAP_data.jtag_dev.index = *req++;
    if (DAP_data.jtag_dev.index >= DAP_data.jtag_dev.count) {
        goto end;
    }

    requestCount = *req++;

    while (requestCount != 0) {
        requestCount--;
        requestValue = *req++;
        requestIr    = (requestValue & DAP_TRANSFER_APnDP) ? JTAG_APACC : JTAG_DPACC;
        if ((requestValue & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if (postRead) {
                // Read was posted before
                retry = DAP_data.transfer.retry_count;
                if ((ir == requestIr) && ((requestValue & DAP_TRANSFER_MATCH_VALUE) == 0U)) {
                    // Read previous data and post next read
                    do {
                        responseValue = JTAG_Transfer(requestValue, &data);
                    } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                } else {
                    // Select JTAG chain
                    if (ir != JTAG_DPACC) {
                        ir = JTAG_DPACC;
                        JTAG_Ir(ir);
                    }
                    // Read previous data
                    do {
                        responseValue = JTAG_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
                    } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                    postRead = 0U;
                }
                if (responseValue != DAP_TRANSFER_OK) {
                    break;
                }
                // Store previous data
                *resp++ = (uint8_t)data;
                *resp++ = (uint8_t)(data >> 8);
                *resp++ = (uint8_t)(data >> 16);
                *resp++ = (uint8_t)(data >> 24);

#if TIMESTAMP_CLOCK != 0U
                if (postRead) {
                    // Store Timestamp of next AP read
                    if ((requestValue & DAP_TRANSFER_TIMESTAMP) != 0U) {
                        timestamp = DAP_data.timestamp;
                        *resp++   = (uint8_t)timestamp;
                        *resp++   = (uint8_t)(timestamp >> 8);
                        *resp++   = (uint8_t)(timestamp >> 16);
                        *resp++   = (uint8_t)(timestamp >> 24);
                    }
                }
#endif
            }
            if ((requestValue & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                matchValue = (uint32_t)(*(req + 0) << 0) | (uint32_t)(*(req + 1) << 8) |
                             (uint32_t)(*(req + 2) << 16) | ((uint32_t)*(req + 3) << 24);
                req        += 4;
                matchRetry  = DAP_data.transfer.match_retry;
                // Select JTAG chain
                if (ir != requestIr) {
                    ir = requestIr;
                    JTAG_Ir(ir);
                }
                // Post DP/AP read
                retry = DAP_data.transfer.retry_count;
                do {
                    responseValue = JTAG_Transfer(requestValue, NULL);
                } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                if (responseValue != DAP_TRANSFER_OK) {
                    break;
                }
                do {
                    // Read register until its value matches or retry counter expires
                    retry = DAP_data.transfer.retry_count;
                    do {
                        responseValue = JTAG_Transfer(requestValue, &data);
                    } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                    if (responseValue != DAP_TRANSFER_OK) {
                        break;
                    }
                } while (((data & DAP_data.transfer.match_mask) != matchValue) && matchRetry-- &&
                         !DAP_transferAbort);
                if ((data & DAP_data.transfer.match_mask) != matchValue) {
                    responseValue |= DAP_TRANSFER_MISMATCH;
                }
                if (responseValue != DAP_TRANSFER_OK) {
                    break;
                }
            } else {
                // Normal read
                if (postRead == 0U) {
                    // Select JTAG chain
                    if (ir != requestIr) {
                        ir = requestIr;
                        JTAG_Ir(ir);
                    }
                    // Post DP/AP read
                    retry = DAP_data.transfer.retry_count;
                    do {
                        responseValue = JTAG_Transfer(requestValue, NULL);
                    } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                    if (responseValue != DAP_TRANSFER_OK) {
                        break;
                    }

#if TIMESTAMP_CLOCK != 0U
                    // Store Timestamp
                    if ((requestValue & DAP_TRANSFER_TIMESTAMP) != 0U) {
                        timestamp = DAP_data.timestamp;
                        *resp++   = (uint8_t)timestamp;
                        *resp++   = (uint8_t)(timestamp >> 8);
                        *resp++   = (uint8_t)(timestamp >> 16);
                        *resp++   = (uint8_t)(timestamp >> 24);
                    }
#endif

                    postRead = 1U;
                }
            }
        } else {
            // Write register
            if (postRead) {
                // Select JTAG chain
                if (ir != JTAG_DPACC) {
                    ir = JTAG_DPACC;
                    JTAG_Ir(ir);
                }
                // Read previous data
                retry = DAP_data.transfer.retry_count;
                do {
                    responseValue = JTAG_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
                } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                if (responseValue != DAP_TRANSFER_OK) {
                    break;
                }
                // Store previous data
                *resp++  = (uint8_t)data;
                *resp++  = (uint8_t)(data >> 8);
                *resp++  = (uint8_t)(data >> 16);
                *resp++  = (uint8_t)(data >> 24);
                postRead = 0U;
            }
            // Load data
            data = (uint32_t)(*(req + 0) << 0) | (uint32_t)(*(req + 1) << 8) |
                   (uint32_t)(*(req + 2) << 16) | ((uint32_t)*(req + 3) << 24);
            req += 4;
            if ((requestValue & DAP_TRANSFER_MATCH_MASK) != 0U) {
                // Write match mask
                DAP_data.transfer.match_mask = data;
                responseValue                = DAP_TRANSFER_OK;
            } else {
                // Select JTAG chain
                if (ir != requestIr) {
                    ir = requestIr;
                    JTAG_Ir(ir);
                }
                // Write DP/AP register
                retry = DAP_data.transfer.retry_count;
                do {
                    responseValue = JTAG_Transfer(requestValue, &data);
                } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
                if (responseValue != DAP_TRANSFER_OK) {
                    break;
                }

#if TIMESTAMP_CLOCK != 0U
                // Store Timestamp
                if ((requestValue & DAP_TRANSFER_TIMESTAMP) != 0U) {
                    timestamp = DAP_data.timestamp;
                    *resp++   = (uint8_t)timestamp;
                    *resp++   = (uint8_t)(timestamp >> 8);
                    *resp++   = (uint8_t)(timestamp >> 16);
                    *resp++   = (uint8_t)(timestamp >> 24);
                }
#endif
            }
        }
        responseCount++;
        if (DAP_transferAbort) {
            break;
        }
    }

    while (requestCount != 0) {
        // Process canceled requests
        requestCount--;
        requestValue = *req++;
        if ((requestValue & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if ((requestValue & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                req += 4;
            }
        } else {
            // Write register
            req += 4;
        }
    }

    if (responseValue == DAP_TRANSFER_OK) {
        // Select JTAG chain
        if (ir != JTAG_DPACC) {
            ir = JTAG_DPACC;
            JTAG_Ir(ir);
        }
        if (postRead) {
            // Read previous data
            retry = DAP_data.transfer.retry_count;
            do {
                responseValue = JTAG_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
            } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
            if (responseValue != DAP_TRANSFER_OK) {
                goto end;
            }
            // Store previous data
            *resp++ = (uint8_t)data;
            *resp++ = (uint8_t)(data >> 8);
            *resp++ = (uint8_t)(data >> 16);
            *resp++ = (uint8_t)(data >> 24);
        } else {
            // Check last write
            retry = DAP_data.transfer.retry_count;
            do {
                responseValue = JTAG_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
            } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
        }
    }

end:
    *(responseHead + 0) = (uint8_t)responseCount;
    *(responseHead + 1) = (uint8_t)responseValue;

    return (((uint32_t)(req - requestHead) << 16) | (uint32_t)(resp - responseHead));
}
#endif

// Process Dummy Transfer command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_DummyTransfer(const uint8_t *req, uint8_t *resp) {
    const uint8_t *requestHead;
    uint32_t       requestCount;
    uint32_t       requestValue;

    requestHead = req;

    req++; // Ignore DAP index

    requestCount = *req++;

    for (; requestCount != 0U; requestCount--) {
        // Process dummy requests
        requestValue = *req++;
        if ((requestValue & DAP_TRANSFER_RnW) != 0U) {
            // Read register
            if ((requestValue & DAP_TRANSFER_MATCH_VALUE) != 0U) {
                // Read with value match
                req += 4;
            }
        } else {
            // Write register
            req += 4;
        }
    }

    *(resp + 0) = 0U; // Response count
    *(resp + 1) = 0U; // Response value

    return (((uint32_t)(req - requestHead) << 16) | 2U);
}

// Process Transfer command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_Transfer(const uint8_t *req, uint8_t *resp) {
    uint32_t num;

    switch (DAP_data.debug_port) {
#if DAP_SWD != 0
        case DAP_PORT_SWD:
            num = DAP_SwdTransfer(req, resp);
            break;
#endif

#if DAP_JTAG != 0
        case DAP_PORT_JTAG:
            num = DAP_JtagTransfer(req, resp);
            break;
#endif

        default:
            num = DAP_DummyTransfer(req, resp);
            break;
    }

    return (num);
}

// Process SWD Transfer Block command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response

#if DAP_SWD != 0
static __DAP_TRANSFER_BLOCK_CODE uint32_t DAP_SwdTransferBlock(const uint8_t *req, uint8_t *resp) {
    uint32_t requestCount;
    uint32_t requestValue;
    uint32_t responseCount;
    uint32_t responseValue;
    uint8_t *responseHead;
    uint32_t retry;

    responseCount  = 0U;
    responseValue  = 0U;
    responseHead   = resp;
    resp          += 3;

    // TransferAbort is cleared atomically by the USB command dispatcher.

    req++; // Ignore DAP index

    requestCount  = (uint32_t)(*(req + 0) << 0) | (uint32_t)(*(req + 1) << 8);
    req          += 2;
    if (requestCount == 0U) {
        goto end;
    }

    requestValue = *req++;
    if ((requestValue & DAP_TRANSFER_RnW) != 0U) {
        // Read register block
        if ((requestValue & DAP_TRANSFER_APnDP) != 0U) {
            // Post AP read
            retry = DAP_data.transfer.retry_count;
            do {
                responseValue = SWD_Transfer(requestValue, NULL, NULL);
            } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
            if (responseValue != DAP_TRANSFER_OK) {
                goto end;
            }
        }
        while (requestCount--) {
            // Read DP/AP register
            if ((requestCount == 0U) && ((requestValue & DAP_TRANSFER_APnDP) != 0U)) {
                // Last AP read
                requestValue = DP_RDBUFF | DAP_TRANSFER_RnW;
            }
            retry = DAP_data.transfer.retry_count;
            do {
                responseValue = SWD_Transfer(requestValue, NULL, resp);
            } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
            if (responseValue != DAP_TRANSFER_OK) {
                goto end;
            }
            // Data already resides in the response buffer.
            resp += 4;
            responseCount++;
        }
    } else {
        // Write register block
        while (requestCount--) {
            req += 4;
            // Write DP/AP register
            retry = DAP_data.transfer.retry_count;
            do {
                responseValue = SWD_Transfer(requestValue, req - 4, NULL);
            } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
            if (responseValue != DAP_TRANSFER_OK) {
                goto end;
            }
            responseCount++;
        }
        // Check last write
        retry = DAP_data.transfer.retry_count;
        do {
            responseValue = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL, NULL);
        } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
    }

end:
    *(responseHead + 0) = (uint8_t)(responseCount >> 0);
    *(responseHead + 1) = (uint8_t)(responseCount >> 8);
    *(responseHead + 2) = (uint8_t)responseValue;

    return ((uint32_t)(resp - responseHead));
}
#endif

// Process JTAG Transfer Block command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response

#if DAP_JTAG != 0
/* Keep the less frequent JTAG loop in Flash in the RF TARGET working set. */
static __DAP_JTAG_BLOCK_CODE uint32_t DAP_JtagTransferBlock(const uint8_t *req, uint8_t *resp) {
    uint32_t requestCount;
    uint32_t requestValue;
    uint32_t responseCount;
    uint32_t responseValue;
    uint8_t *responseHead;
    uint32_t retry;
    uint32_t data;
    uint32_t ir;

    responseCount  = 0U;
    responseValue  = 0U;
    responseHead   = resp;
    resp          += 3;

    // TransferAbort is cleared atomically by the USB command dispatcher.

    // Device index (JTAP TAP)
    DAP_data.jtag_dev.index = *req++;
    if (DAP_data.jtag_dev.index >= DAP_data.jtag_dev.count) {
        goto end;
    }

    requestCount  = (uint32_t)(*(req + 0) << 0) | (uint32_t)(*(req + 1) << 8);
    req          += 2;
    if (requestCount == 0U) {
        goto end;
    }

    requestValue = *req++;

    // Select JTAG chain
    ir = (requestValue & DAP_TRANSFER_APnDP) ? JTAG_APACC : JTAG_DPACC;
    JTAG_Ir(ir);

    if ((requestValue & DAP_TRANSFER_RnW) != 0U) {
        // Post read
        retry = DAP_data.transfer.retry_count;
        do {
            responseValue = JTAG_Transfer(requestValue, NULL);
        } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
        if (responseValue != DAP_TRANSFER_OK) {
            goto end;
        }
        // Read register block
        while (requestCount--) {
            // Read DP/AP register
            if (requestCount == 0U) {
                // Last read
                if (ir != JTAG_DPACC) {
                    JTAG_Ir(JTAG_DPACC);
                }
                requestValue = DP_RDBUFF | DAP_TRANSFER_RnW;
            }
            retry = DAP_data.transfer.retry_count;
            do {
                responseValue = JTAG_Transfer(requestValue, &data);
            } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
            if (responseValue != DAP_TRANSFER_OK) {
                goto end;
            }
            // Store data
            *resp++ = (uint8_t)data;
            *resp++ = (uint8_t)(data >> 8);
            *resp++ = (uint8_t)(data >> 16);
            *resp++ = (uint8_t)(data >> 24);
            responseCount++;
        }
    } else {
        // Write register block
        while (requestCount--) {
            // Load data
            data = (uint32_t)(*(req + 0) << 0) | (uint32_t)(*(req + 1) << 8) |
                   (uint32_t)(*(req + 2) << 16) | ((uint32_t)*(req + 3) << 24);
            req += 4;
            // Write DP/AP register
            retry = DAP_data.transfer.retry_count;
            do {
                responseValue = JTAG_Transfer(requestValue, &data);
            } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
            if (responseValue != DAP_TRANSFER_OK) {
                goto end;
            }
            responseCount++;
        }
        // Check last write
        if (ir != JTAG_DPACC) {
            JTAG_Ir(JTAG_DPACC);
        }
        retry = DAP_data.transfer.retry_count;
        do {
            responseValue = JTAG_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
        } while ((responseValue == DAP_TRANSFER_WAIT) && retry-- && !DAP_transferAbort);
    }

end:
    *(responseHead + 0) = (uint8_t)(responseCount >> 0);
    *(responseHead + 1) = (uint8_t)(responseCount >> 8);
    *(responseHead + 2) = (uint8_t)responseValue;

    return ((uint32_t)(resp - responseHead));
}
#endif

// Process Transfer Block command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)

#if APP_HAS_RF
static uint32_t DAP_TransferBlock(const uint8_t *req, uint8_t *resp) {
    uint32_t num;

    switch (DAP_data.debug_port) {
#if DAP_SWD != 0
        case DAP_PORT_SWD:
            num = DAP_SwdTransferBlock(req, resp);
            break;
#endif

#if DAP_JTAG != 0
        case DAP_PORT_JTAG:
            num = DAP_JtagTransferBlock(req, resp);
            break;
#endif

        default:
            *(resp + 0) = 0U; // Response count [7:0]
            *(resp + 1) = 0U; // Response count[15:8]
            *(resp + 2) = 0U; // Response value
            num         = 3U;
            break;
    }

    if ((*(req + 3) & DAP_TRANSFER_RnW) != 0U) {
        // Read register block
        num |= 4U << 16;
    } else {
        // Write register block
        num |= (4U + (((uint32_t)(*(req + 1)) | (uint32_t)(*(req + 2) << 8)) * 4)) << 16;
    }

    return (num);
}
#endif

// Process SWD Write ABORT command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response

#if DAP_SWD != 0
static uint32_t DAP_SwdWriteAbort(const uint8_t *req, uint8_t *resp) {
    // Write Abort register (Ignore DAP index).
    SWD_Transfer(DP_ABORT, req + 1, NULL);

    *resp = DAP_OK;
    return (1U);
}
#endif

// Process JTAG Write ABORT command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response

#if DAP_JTAG != 0
static uint32_t DAP_JtagWriteAbort(const uint8_t *req, uint8_t *resp) {
    uint32_t data;

    // Device index (JTAP TAP)
    DAP_data.jtag_dev.index = *req;
    if (DAP_data.jtag_dev.index >= DAP_data.jtag_dev.count) {
        *resp = DAP_ERROR;
        return (1U);
    }

    // Select JTAG chain
    JTAG_Ir(JTAG_ABORT);

    // Load data
    data = (uint32_t)(*(req + 1) << 0) | (uint32_t)(*(req + 2) << 8) |
           (uint32_t)(*(req + 3) << 16) | ((uint32_t)*(req + 4) << 24);

    // Write Abort register
    JTAG_WriteAbort(data);

    *resp = DAP_OK;
    return (1U);
}
#endif

// Process Write ABORT command and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static uint32_t DAP_WriteAbort(const uint8_t *req, uint8_t *resp) {
    uint32_t num;

    switch (DAP_data.debug_port) {
#if DAP_SWD != 0
        case DAP_PORT_SWD:
            num = DAP_SwdWriteAbort(req, resp);
            break;
#endif

#if DAP_JTAG != 0
        case DAP_PORT_JTAG:
            num = DAP_JtagWriteAbort(req, resp);
            break;
#endif

        default:
            *resp = DAP_ERROR;
            num   = 1U;
            break;
    }
    return ((5U << 16) | num);
}

// Process DAP Vendor command request and prepare response
// Default function (can be overridden)
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
__WEAK uint32_t DAP_ProcessVendorCommand(const uint8_t *req, uint8_t *resp) {
    (void)req;
    *resp = ID_DAP_Invalid;
    return ((1U << 16) | 1U);
}

// Process DAP command request and prepare response
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
static __NO_INLINE uint32_t DAP_ProcessStandardCommand(const uint8_t *req, uint8_t *resp) {
    uint32_t num;

    if ((*req >= ID_DAP_Vendor0) && (*req <= ID_DAP_Vendor31)) {
        return DAP_ProcessVendorCommand(req, resp);
    }

    *resp++ = *req;

    switch (*req++) {
        case ID_DAP_Info:
            num   = DAP_Info(*req, resp + 1);
            *resp = (uint8_t)num;
            return ((2U << 16) + 2U + num);

        case ID_DAP_HostStatus:
            num = DAP_HostStatus(req, resp);
            break;

        case ID_DAP_Connect:
            num = DAP_Connect(req, resp);
            break;
        case ID_DAP_Disconnect:
            num = DAP_Disconnect(resp);
            break;

        case ID_DAP_Delay:
            num = DAP_Delay(req, resp);
            break;

        case ID_DAP_ResetTarget:
            num = DAP_ResetTarget(resp);
            break;

        case ID_DAP_SWJ_Pins:
            num = DAP_SwjPins(req, resp);
            break;
        case ID_DAP_SWJ_Clock:
            num = DAP_SwjClock(req, resp);
            break;
        case ID_DAP_SWJ_Sequence:
            num = DAP_SwjSequence(req, resp);
            break;

        case ID_DAP_SWD_Configure:
            num = DAP_SwdConfigure(req, resp);
            break;
        case ID_DAP_SWD_Sequence:
            num = DAP_SwdSequence(req, resp);
            break;

        case ID_DAP_JTAG_Sequence:
            num = DAP_JtagSequence(req, resp);
            break;
        case ID_DAP_JTAG_Configure:
            num = DAP_JtagConfigure(req, resp);
            break;
        case ID_DAP_JTAG_IDCODE:
            num = DAP_JtagIdCode(req, resp);
            break;

        case ID_DAP_TransferConfigure:
            num = DAP_TransferConfigure(req, resp);
            break;
        case ID_DAP_Transfer:
            num = DAP_Transfer(req, resp);
            break;

#if APP_HAS_RF
        case ID_DAP_TransferBlock:
            num = DAP_TransferBlock(req, resp);
            break;
#endif

        case ID_DAP_WriteABORT:
            num = DAP_WriteAbort(req, resp);
            break;

#if DAP_UART != 0
        case ID_DAP_UART_Transport:
            num = UART_Transport(req, resp);
            break;
        case ID_DAP_UART_Configure:
            num = UART_Configure(req, resp);
            break;
        case ID_DAP_UART_Control:
            num = UART_Control(req, resp);
            break;
        case ID_DAP_UART_Status:
            num = UART_Status(resp);
            break;
        case ID_DAP_UART_Transfer:
            num = UART_Transfer(req, resp);
            break;
#endif

        default:
            *(resp - 1) = ID_DAP_Invalid;
            return ((1U << 16) | 1U);
    }

    return ((1U << 16) + 1U + num);
}

/* Block transfers stay in SRAM from dispatch through response accounting;
 * setup and management commands retain their Flash implementation. */
__DAP_PROCESS_CODE uint32_t DAP_ProcessCommand(const uint8_t *req, uint8_t *resp) {
#if DAP_ATOMIC_COMMANDS
    if (*req == ID_DAP_ExecuteCommands) {
        return DAP_ExecuteCommand(req, resp);
    }
#endif

#if !APP_HAS_RF
    if (*req != ID_DAP_TransferBlock) {
        return DAP_ProcessStandardCommand(req, resp);
    }
    *resp++ = *req++;
    uint32_t num;

    switch (DAP_data.debug_port) {
#if DAP_SWD != 0
        case DAP_PORT_SWD:
            num = DAP_SwdTransferBlock(req, resp);
            break;
#endif

#if DAP_JTAG != 0
        case DAP_PORT_JTAG:
            num = DAP_JtagTransferBlock(req, resp);
            break;
#endif

        default:
            *(resp + 0) = 0U; // Response count [7:0]
            *(resp + 1) = 0U; // Response count[15:8]
            *(resp + 2) = 0U; // Response value
            num         = 3U;
            break;
    }

    if ((*(req + 3) & DAP_TRANSFER_RnW) != 0U) {
        // Read register block
        num |= 4U << 16;
    } else {
        // Write register block
        num |= (4U + (((uint32_t)(*(req + 1)) | (uint32_t)(*(req + 2) << 8)) * 4)) << 16;
    }

    return ((1U << 16) + 1U + num);
#else
    return DAP_ProcessStandardCommand(req, resp);
#endif
}

// Execute DAP command (process request and prepare response)
//   request:  pointer to request data
//   response: pointer to response data
//   return:   number of bytes in response (lower 16 bits)
//             number of bytes in request (upper 16 bits)
uint32_t DAP_ExecuteCommand(const uint8_t *req, uint8_t *resp) {
#if DAP_ATOMIC_COMMANDS
    uint32_t cnt, num, n;

    if (*req == ID_DAP_ExecuteCommands) {
        *resp++            = *req++;
        cnt                = *req++;
        uint8_t *completed = resp++;
        *completed         = 0;
        num                = (2U << 16) | 2U;
        while (cnt-- && !BSP_stopRequested && !DAP_transferAbort) {
            /* Atomic containers do not recursively contain other containers. */
            n     = DAP_ProcessStandardCommand(req, resp);
            num  += n;
            req  += (uint16_t)(n >> 16);
            resp += (uint16_t)n;
            ++*completed;
        }
        return (num);
    }
#endif

    return DAP_ProcessCommand(req, resp);
}

// Setup DAP
void DAP_Setup(void) {
    // Default settings
    DAP_data.debug_port           = 0U;
    DAP_data.transfer.idle_cycles = 0U;
    DAP_data.transfer.retry_count = 100U;
    DAP_data.transfer.match_retry = 0U;
    DAP_data.transfer.match_mask  = 0x00000000U;

#if DAP_SWD != 0
    DAP_data.swd_conf.turnaround = 1U;
    DAP_data.swd_conf.data_phase = 0U;
#endif

#if DAP_JTAG != 0
    DAP_data.jtag_dev.count = 0U;
#endif

    // Sets DAP_Data.fast_clock and DAP_Data.clock_delay.
    DAP_SetClockDelay(DAP_DEFAULT_SWJ_CLOCK);
}
