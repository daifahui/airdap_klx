/* Flash controller access for factory UID and software reset. Command order,
 * data widths, dummy cycles and protected-register windows follow the chip's
 * controller protocol. These operations do not erase or program Flash. */
#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "ch572/flash_io.h"

#define __FLASH_IO_CODE __NO_INLINE __SECTION(".boot_flash_io")

static __FLASH_IO_CODE void FLASH_Wait(void) {
    while (R8_FLASH_CTRL & RB_FLASH_CTRL_BUSY) {
    }
}

static __FLASH_IO_CODE void FLASH_Begin(uint8_t command) {
    R8_FLASH_CTRL = FLASH_CTRL_IDLE;
    R8_FLASH_CTRL = FLASH_CTRL_COMMAND;
    __asm__ volatile("nop; nop");
    R8_FLASH_DATA = command;
}

static __FLASH_IO_CODE void FLASH_End(void) {
    FLASH_Wait();
    R8_FLASH_CTRL = FLASH_CTRL_IDLE;
}

static __FLASH_IO_CODE void FLASH_Write(uint8_t value) {
    FLASH_Wait();
    R8_FLASH_DATA = value;
}

static __FLASH_IO_CODE void FLASH_Start(void) {
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG1;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG2;
    R8_GLOB_ROM_CFG    |= (RB_ROM_CODE_WE | RB_ROM_CTRL_EN);
    R8_FLASH_CTRL       = FLASH_CTRL_DIRECT;
    FLASH_Begin(FLASH_CMD_RELEASE);
    FLASH_Write(FLASH_CMD_RELEASE);
    FLASH_Wait();
    FLASH_End();
}

static __FLASH_IO_CODE void FLASH_ReadUidIo(uint8_t *data) {
    FLASH_Start();
    FLASH_Begin(FLASH_CMD_READ_FAST);
    uint32_t address = CH572_ROM_UID_ADDR;

    for (unsigned i = 0; i < 3U + FLASH_READ_DUMMY_BYTES; i++) {
        FLASH_Write(address >> 16);
        address <<= 8;
    }
    for (unsigned i = 0; i < 8U; i++) {
        FLASH_Wait();
        uint8_t shifted = R8_FLASH_DATA;

        (void)shifted;
        if (i == 3U) {
            *(uint32_t *)data = R32_FLASH_DATA;
        }
    }
    *(uint16_t *)(data + 4) = (uint16_t)R32_FLASH_DATA;
    FLASH_End();
}

static __FLASH_IO_CODE void FLASH_ResetIo(void) {
    FLASH_Start();
    FLASH_Begin(FLASH_CMD_RESET_ENABLE);
    FLASH_End();
    FLASH_Begin(FLASH_CMD_RESET);
    FLASH_End();
}

static void FLASH_Close(void) {
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG1;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG2;
    R8_GLOB_ROM_CFG    &= RB_ROM_CODE_OFS;
}

void FLASH_ReadUid(uint8_t *data) {
    uint32_t enabled0 = PFIC->ISR[0];
    uint32_t enabled1 = PFIC->ISR[1];

    PFIC->IRER[0] = UINT32_MAX;
    PFIC->IRER[1] = UINT32_MAX;
    FLASH_ReadUidIo(data);
    FLASH_Close();
    PFIC->IENR[0] = enabled0;
    PFIC->IENR[1] = enabled1;
}

void FLASH_Reset(void) {
    uint32_t enabled0 = PFIC->ISR[0];
    uint32_t enabled1 = PFIC->ISR[1];

    PFIC->IRER[0] = UINT32_MAX;
    PFIC->IRER[1] = UINT32_MAX;
    FLASH_ResetIo();
    FLASH_Close();
    PFIC->IENR[0] = enabled0;
    PFIC->IENR[1] = enabled1;
}
