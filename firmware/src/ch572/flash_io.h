#ifndef CH572_FLASH_IO_H
#define CH572_FLASH_IO_H

#include <stdint.h>

#define FLASH_CTRL_IDLE        0x00U
#define FLASH_CTRL_DIRECT      0x04U
#define FLASH_CTRL_COMMAND     0x05U
#define FLASH_CTRL_SHIFT_BYTE  0x15U
#define FLASH_CMD_WRITE_ENABLE 0x06U
#define FLASH_CMD_READ_STATUS  0x05U
#define FLASH_CMD_RELEASE      0xffU
#define FLASH_CMD_READ_FAST    0x0bU
#define FLASH_CMD_ERASE_SECTOR 0x20U
#define FLASH_CMD_PROGRAM_PAGE 0x02U
#define FLASH_CMD_RESET_ENABLE 0x66U
#define FLASH_CMD_RESET        0x99U
#define FLASH_STATUS_BUSY      0x01U
#define FLASH_ERASED_BYTE      0xffU
#define FLASH_SECTOR_SIZE      4096U
#define FLASH_PAGE_SIZE        256U
#define FLASH_WORD_SIZE        4U
#define FLASH_READ_DUMMY_BYTES 2U

/* MODE_MemoryBoot must load the stopped startup bank before either call.
 * The UID destination has six writable bytes and four-byte alignment. */
void FLASH_ReadUid(uint8_t *data);

void FLASH_Reset(void);
#endif
