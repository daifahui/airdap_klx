#ifndef CH572_DFU_H
#define CH572_DFU_H

#include <stdint.h>
#include "ch572/attributes.h"
#include <stdbool.h>
#include "flash_layout.h"
#include "usb/usb_protocol.h"

#define DFU_SETUP_STALL     (-1)
#define DFU_SETUP_DATA_OUT  (-2)
#define DFU_SETUP_STATUS_IN (-3)

/* Standard DFU 1.1 and DfuSe use separate, explicit alternate settings. */
#define DFU_STANDARD_ALT          0
#define DFU_DFUSE_ALT             1
#define DFU_PROTOCOL_FOR_ALT(alt) (1 + (alt))
#define DFU_TRANSFER              USB_DFU_TRANSFER_SIZE
#define DFU_FLASH_SIZE            FLASH_CODE_END
#define DFU_IDLE                  2
#define DFU_SYNC                  3
#define DFU_BUSY                  4
#define DFU_DOWNLOAD              5
#define DFU_MANIFEST_SYNC         6
#define DFU_MANIFEST              7
#define DFU_WAIT_RESET            8
#define DFU_UPLOAD                9
#define DFU_ERROR                 10
#define OK                        0
#define ERR_TARGET                1
#define ERR_WRITE                 3
#define ERR_ERASE                 4
#define ERR_CHECK                 5
#define ERR_PROG                  6
#define ERR_VERIFY                7
#define ERR_ADDRESS               8
#define ERR_NOTDONE               9
#define ERR_FIRMWARE              10
#define ERR_VENDOR                11
#define ERR_USBR                  12
#define ERR_STALL                 15

/* DfuSe (ST AN3156) command block, address and data block numbering. */
#define DFU_PROTOCOL_STANDARD  1
#define DFU_PROTOCOL_DFUSE     2
#define DFUSE_CMD_GET_COMMANDS 0x00
#define DFUSE_CMD_SET_ADDRESS  0x21
#define DFUSE_CMD_ERASE        0x41
#define DFUSE_COMMAND_COUNT    3
#define DFUSE_COMMAND_SIZE     5
#define DFUSE_FIRST_DATA_BLOCK 2
#define DFUSE_RESERVED_BLOCK   1
#define DFU_POLL_ERASE_MS      200
#define DFU_POLL_PROGRAM_MS    20
#define DFU_POLL_IDLE_MS       1

/* AN3156 Leave: select address zero, DNLOAD(length=0), then complete
 * GETSTATUS. The MCU resets through the DFU boot entry. */
extern uint8_t DFU_buffer[DFU_TRANSFER];
extern uint8_t DFU_response[USB_DFU_STATUS_SIZE];
extern uint8_t DFU_state, DFU_error;
extern bool    DFU_reboot;
extern bool    DFU_reconnect;
extern uint8_t DFU_protocol; /* Internal:1=standard,2=DfuSe; alt policy above. */

extern uint8_t USB_ep0Dma[64], USB_ep0Control, USB_ep0Length, USB_address;

void USB_ControlReset(void);
void USB_ControlSetup(const uint8_t *);
void USB_ControlIn(void);
void USB_ControlOut(const uint8_t *, uint8_t, bool);

void DFU_Configure(uint8_t protocol);

/* Return a nonnegative IN byte count or a DFU_SETUP_* control-stage code. */

int DFU_Setup(uint8_t type, uint8_t req, uint16_t value, uint16_t index, uint16_t length);

void DFU_DataComplete(uint16_t length);

void DFU_StatusComplete(void);

void DFU_CancelControl(void);

void DFU_BusReset(void);

void DFU_Work(void);

uint8_t FLASH_Read(uint32_t address, uint8_t *data, uint16_t length);

uint8_t FLASH_Program(uint32_t address, const uint8_t *data, uint16_t length, bool erase);

uint8_t FLASH_Finish(void);

void DFU_Reboot(void) __NO_RETURN;

void DFU_Main(void) __NO_RETURN;
#endif
