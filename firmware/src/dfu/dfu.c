#include "usb/usb_requests.h"
#include "ch572/attributes.h"
#include "dfu/dfu.h"

#define DFU_PENDING_NONE      0U
#define DFU_PENDING_DNLOAD    1U
#define DFU_PENDING_DETACH    2U
#define DFU_PENDING_GETSTATUS 3U

uint8_t         DFU_buffer[DFU_TRANSFER] __ALIGNED(4);
uint8_t         DFU_response[USB_DFU_STATUS_SIZE];
uint8_t         DFU_state, DFU_error;
bool            DFU_reboot;
bool            DFU_reconnect;
bool            DFU_detachAccepted; /* one successful DETACH per DFU entry */
uint8_t         DFU_protocol;
static uint16_t DFU_block, DFU_received, DFU_staged;
static uint8_t  DFU_requestPending;
static bool     DFU_running, DFU_complete, DFU_ready, DFU_downloadStarted;
static uint32_t DFU_pointer;
static uint16_t DFU_nextBlock;

static int DFU_Fail(uint8_t error) {
    DFU_error = error;
    DFU_state = DFU_ERROR;
    return DFU_SETUP_STALL;
}

void DFU_CancelControl(void) {
    if (DFU_requestPending == DFU_PENDING_GETSTATUS) {
        if (DFU_state == DFU_BUSY)
            DFU_state = DFU_SYNC;
        if (DFU_state == DFU_MANIFEST)
            DFU_state = DFU_MANIFEST_SYNC;
    }
    DFU_requestPending = DFU_PENDING_NONE;
    DFU_staged         = 0;
}

void DFU_BusReset(void) {
    /* WillDetach=1: an uncommitted bus reset never requests MCU reset. */
    if (DFU_downloadStarted || DFU_running) {
        DFU_error = ERR_USBR;
        DFU_state = DFU_ERROR;
    } else {
        DFU_error = 0;
        DFU_state = DFU_IDLE;
    }
    DFU_running = DFU_ready = DFU_complete = false;
    DFU_pointer                            = 0;
    DFU_CancelControl();
}

void DFU_Configure(uint8_t protocol) {
    DFU_protocol = protocol;
    DFU_state    = DFU_IDLE;
    DFU_error    = 0;
    DFU_pointer  = 0;
    DFU_running = DFU_ready = DFU_complete = DFU_downloadStarted = false;
    DFU_nextBlock                                                = 0;
    DFU_CancelControl();
}

int DFU_Setup(uint8_t type, uint8_t req, uint16_t value, uint16_t index, uint16_t length) {
    DFU_CancelControl();
    if (index || ((type != USB_RT_OUT_CLASS_INTERFACE) && (type != USB_RT_IN_CLASS_INTERFACE)))
        return DFU_SETUP_STALL;
    if (type == USB_RT_OUT_CLASS_INTERFACE && req == USB_DFU_DETACH && !length &&
        DFU_state == DFU_IDLE) {
        /* WTR recovery is already in DFU. Accept one completed DETACH,
         * without leaving dfuIDLE; duplicates STALL without corrupting it. */
        if (DFU_detachAccepted)
            return DFU_SETUP_STALL;
        DFU_requestPending = DFU_PENDING_DETACH;
        return DFU_SETUP_STATUS_IN;
    }
    if (type == USB_RT_IN_CLASS_INTERFACE && req == USB_DFU_GETSTATUS && !value &&
        length == USB_DFU_STATUS_SIZE) {
        if (DFU_state == DFU_SYNC) {
            if (DFU_complete) {
                DFU_state    = DFU_DOWNLOAD;
                DFU_complete = false;
            } else {
                DFU_state          = DFU_BUSY;
                DFU_requestPending = DFU_PENDING_GETSTATUS;
            }
        } else if (DFU_state == DFU_MANIFEST_SYNC) {
            DFU_state          = DFU_MANIFEST;
            DFU_requestPending = DFU_PENDING_GETSTATUS;
        }
        DFU_response[USB_DFU_STATUS_OFFSET]           = DFU_error;
        uint16_t wait                                 = (DFU_state == DFU_BUSY)
                                                            ? ((DFU_protocol != DFU_PROTOCOL_DFUSE ||
                                (!DFU_block && DFU_buffer[0] == DFUSE_CMD_ERASE))
                                                                   ? DFU_POLL_ERASE_MS
                                                                   : DFU_POLL_PROGRAM_MS)
                                                            : DFU_POLL_IDLE_MS;
        DFU_response[USB_DFU_POLL_TIMEOUT_OFFSET]     = wait;
        DFU_response[USB_DFU_POLL_TIMEOUT_OFFSET + 1] = wait >> 8;
        DFU_response[USB_DFU_POLL_TIMEOUT_OFFSET + 2] = 0;
        DFU_response[USB_DFU_STATE_OFFSET]            = DFU_state;
        DFU_response[USB_DFU_STATUS_STRING_OFFSET]    = 0;
        return USB_DFU_STATUS_SIZE;
    }
    if (type == USB_RT_IN_CLASS_INTERFACE && req == USB_DFU_GETSTATE && !value &&
        length == USB_DFU_STATE_SIZE) {
        DFU_response[0] = DFU_state;
        return USB_DFU_STATE_SIZE;
    }
    if (type == USB_RT_OUT_CLASS_INTERFACE && !value && !length && req == USB_DFU_CLRSTATUS &&
        DFU_state == DFU_ERROR) {
        DFU_error           = 0;
        DFU_state           = DFU_IDLE;
        DFU_downloadStarted = false;
        return DFU_SETUP_STATUS_IN;
    }
    if (type == USB_RT_OUT_CLASS_INTERFACE && !value && !length && req == USB_DFU_ABORT &&
        (DFU_state == DFU_IDLE || DFU_state == DFU_DOWNLOAD || DFU_state == DFU_UPLOAD)) {
        DFU_state           = DFU_IDLE;
        DFU_downloadStarted = DFU_complete = false;
        return DFU_SETUP_STATUS_IN;
    }
    if (type == USB_RT_IN_CLASS_INTERFACE && req == USB_DFU_UPLOAD && length &&
        length <= DFU_TRANSFER && (DFU_state == DFU_IDLE || DFU_state == DFU_UPLOAD)) {
        if (DFU_protocol == DFU_PROTOCOL_DFUSE && !value) {
            DFU_buffer[0]  = DFUSE_CMD_GET_COMMANDS;
            DFU_buffer[1]  = DFUSE_CMD_SET_ADDRESS;
            DFU_buffer[2]  = DFUSE_CMD_ERASE;
            uint16_t count = length < DFUSE_COMMAND_COUNT ? length : DFUSE_COMMAND_COUNT;

            DFU_state = count < length ? DFU_IDLE : DFU_UPLOAD;
            return count;
        }
        if (DFU_protocol == DFU_PROTOCOL_DFUSE && value < DFUSE_FIRST_DATA_BLOCK)
            return DFU_Fail(ERR_ADDRESS);
        uint32_t address =
            DFU_protocol == DFU_PROTOCOL_DFUSE
                ? DFU_pointer + (uint32_t)(value - DFUSE_FIRST_DATA_BLOCK) * DFU_TRANSFER
                : (uint32_t)value * DFU_TRANSFER;
        if (address > DFU_FLASH_SIZE)
            return DFU_Fail(ERR_ADDRESS);
        uint16_t count = DFU_FLASH_SIZE - address < length ? DFU_FLASH_SIZE - address : length;
        uint8_t  error = FLASH_Read(address, DFU_buffer, count);

        if (error)
            return DFU_Fail(error);
        DFU_state = count < length ? DFU_IDLE : DFU_UPLOAD;
        return count;
    }
    if (type == USB_RT_OUT_CLASS_INTERFACE && req == USB_DFU_DNLOAD && length <= DFU_TRANSFER &&
        (DFU_state == DFU_IDLE || DFU_state == DFU_DOWNLOAD)) {
        if (!length) {
            if (DFU_protocol == DFU_PROTOCOL_DFUSE) {
                if (DFU_pointer)
                    return DFU_Fail(ERR_TARGET);
            } else if (DFU_state != DFU_DOWNLOAD)
                return DFU_Fail(ERR_NOTDONE);
            DFU_requestPending = DFU_PENDING_DNLOAD;
            DFU_staged         = 0;
            return DFU_SETUP_STATUS_IN;
        }
        if (DFU_protocol == DFU_PROTOCOL_DFUSE) {
            if (value == DFUSE_RESERVED_BLOCK)
                return DFU_Fail(ERR_ADDRESS);
            if (value >= DFUSE_FIRST_DATA_BLOCK &&
                (DFU_pointer + (uint32_t)(value - DFUSE_FIRST_DATA_BLOCK) * DFU_TRANSFER >=
                     DFU_FLASH_SIZE ||
                 (DFU_pointer & (DFU_TRANSFER - 1))))
                return DFU_Fail(ERR_ADDRESS);
        } else {
            if (value >= DFU_FLASH_SIZE / DFU_TRANSFER ||
                (DFU_state == DFU_IDLE ? value != 0
                                       : (value != DFU_nextBlock || DFU_received != DFU_TRANSFER)))
                return DFU_Fail(ERR_ADDRESS);
        }
        DFU_block          = value;
        DFU_staged         = length;
        DFU_requestPending = DFU_PENDING_DNLOAD;
        return DFU_SETUP_DATA_OUT;
    }
    return DFU_Fail(ERR_STALL);
}

void DFU_DataComplete(uint16_t length) {
    if (length != DFU_staged)
        (void)DFU_Fail(ERR_STALL);
}

void DFU_StatusComplete(void) {
    if (DFU_requestPending == DFU_PENDING_DNLOAD && DFU_state != DFU_ERROR) {
        DFU_received = DFU_staged;
        DFU_state    = DFU_staged ? DFU_SYNC : DFU_MANIFEST_SYNC;
        DFU_complete = false;
    } else if (DFU_requestPending == DFU_PENDING_DETACH) {
        DFU_detachAccepted = true;
        DFU_reconnect      = true; /* only after the DETACH status was ACKed */
    } else if (DFU_requestPending == DFU_PENDING_GETSTATUS)
        DFU_ready = true;
    DFU_requestPending = DFU_PENDING_NONE;
}

void DFU_Work(void) {
    if (!DFU_ready)
        return;
    DFU_ready   = false;
    DFU_running = true;
    if (DFU_state == DFU_MANIFEST) {
        /* AN3156 Leave executes only after the full GETSTATUS status stage.
         * No return into old SRAM, and no USB reset is required as a trigger. */

        /* Both functional descriptors advertise WillDetach=1. Standard
         * manifestation also detaches itself; never require WinUSB bus reset. */
        if (DFU_protocol == DFU_PROTOCOL_STANDARD)
            DFU_state = DFU_WAIT_RESET; /* Self-detach, Section 7. */
        DFU_reboot = true;
    } else if (DFU_state == DFU_BUSY) {
        uint8_t error = 0;

        if (DFU_protocol == DFU_PROTOCOL_DFUSE && !DFU_block) {
            if (DFU_received != DFUSE_COMMAND_SIZE ||
                (DFU_buffer[0] != DFUSE_CMD_SET_ADDRESS && DFU_buffer[0] != DFUSE_CMD_ERASE))
                error = ERR_STALL;
            else {
                uint32_t a = DFU_buffer[1] | (uint32_t)DFU_buffer[2] << 8 |
                             (uint32_t)DFU_buffer[3] << 16 | (uint32_t)DFU_buffer[4] << 24;
                if (a > DFU_FLASH_SIZE)
                    error = ERR_ADDRESS;
                else if (DFU_buffer[0] == DFUSE_CMD_SET_ADDRESS)
                    DFU_pointer = a;
                else
                    error = FLASH_Program(a, DFU_buffer, 0, true);
            }
        } else {
            uint16_t n = (DFU_received + 3) & ~3u;

            for (uint16_t i = DFU_received; i < n; i++)
                DFU_buffer[i] = 0xff;
            bool     standard = DFU_protocol != DFU_PROTOCOL_DFUSE;
            uint32_t address =
                standard
                    ? (uint32_t)DFU_block * DFU_TRANSFER
                    : DFU_pointer + (uint32_t)(DFU_block - DFUSE_FIRST_DATA_BLOCK) * DFU_TRANSFER;
            error = FLASH_Program(address, DFU_buffer, n, standard);
        }
        if (error)
            (void)DFU_Fail(error);
        else {
            DFU_nextBlock       = DFU_block + 1;
            DFU_state           = DFU_SYNC;
            DFU_complete        = true;
            DFU_downloadStarted = true;
        }
    }
    DFU_running = false;
}
