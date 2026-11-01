/* The controller sequence is cross-checked against ROM 2.30 20001622..1af2.
 * Deliberately no SDK dispatcher (its 3c000 boundary is not the controller).
 * Reserved-region hardware permissions remain real: never clear manufacturer
 * locks, never synthesize success, verify both erase and every program word. */
#include "dfu/dfu.h"
#include "ch572/ch572sfr.h"
#include "ch572/flash_io.h"

#define FLASH_POLL_LIMIT 0x80000U

volatile uint32_t DFU_readGuard, DFU_readFault;
static bool       FLASH_ioOk;

static bool FLASH_ControllerWait(void) {
    uint32_t timeout = FLASH_POLL_LIMIT;

    while (R8_FLASH_CTRL & RB_FLASH_CTRL_BUSY)
        if (!--timeout) {
            FLASH_ioOk = false;
            return false;
        }
    return true;
}

static void FLASH_End(void) {
    (void)FLASH_ControllerWait();
    R8_FLASH_CTRL = FLASH_CTRL_IDLE;
}

static void FLASH_WriteByte(uint8_t value) {
    if (FLASH_ControllerWait())
        R8_FLASH_DATA = value;
}

static uint8_t FLASH_ReadByte(void) {
    return FLASH_ControllerWait() ? R8_FLASH_DATA : 0;
}

static void FLASH_Begin(uint8_t cmd) {
    R8_FLASH_CTRL = FLASH_CTRL_IDLE;
    R8_FLASH_CTRL = FLASH_CTRL_COMMAND;
    __asm__ volatile("nop; nop");
    FLASH_WriteByte(cmd);
}

static void FLASH_Address(uint8_t cmd, uint32_t a) {
    FLASH_Begin(FLASH_CMD_WRITE_ENABLE);
    FLASH_End();
    FLASH_Begin(cmd);
    FLASH_WriteByte(a >> 16);
    FLASH_WriteByte(a >> 8);
    FLASH_WriteByte(a);
}

static bool FLASH_WaitFlash(void) {
    FLASH_End();
    uint32_t count = FLASH_POLL_LIMIT;

    do {
        FLASH_Begin(FLASH_CMD_READ_STATUS);
        (void)FLASH_ReadByte();
        uint8_t s = FLASH_ReadByte();

        FLASH_End();
        if (!FLASH_ioOk)
            return false;
        if (!(s & FLASH_STATUS_BUSY))
            return true;
    } while (--count);
    return false;
}

static bool FLASH_Open(void) {
    FLASH_ioOk = true;
    if (R32_SAFE_ACCESS_SIG2 & RB_FLASH_HALTED)
        return false;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG1;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG2;
    R8_GLOB_ROM_CFG    |= RB_ROM_CODE_WE | RB_ROM_CTRL_EN;
    R8_FLASH_CTRL       = FLASH_CTRL_DIRECT;
    FLASH_Begin(FLASH_CMD_RELEASE);
    FLASH_WriteByte(FLASH_CMD_RELEASE);
    FLASH_End();
    return FLASH_ioOk;
}

static void FLASH_Close(void) {
    FLASH_End();
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG1;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG2;
    R8_GLOB_ROM_CFG    &= RB_ROM_CODE_OFS;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG0;
    __asm__ volatile("fence.i" ::: "memory");
}

uint8_t FLASH_Read(uint32_t a, uint8_t *data, uint16_t n) {
    if (a > DFU_FLASH_SIZE || n > DFU_FLASH_SIZE - a)
        return ERR_ADDRESS;
    DFU_readFault = 0;
    DFU_readGuard = 1;
    for (uint16_t i = 0; i < n; i++)
        data[i] = *(const volatile uint8_t *)(uintptr_t)(a + i);
    DFU_readGuard = 0;
    return DFU_readFault ? ERR_TARGET : OK;
}

uint8_t FLASH_Finish(void) {
    bool good = FLASH_Open();

    if (good)
        good = FLASH_WaitFlash();
    FLASH_Close();
    return good && FLASH_ioOk ? OK : ERR_NOTDONE;
}

uint8_t FLASH_Program(uint32_t a, const uint8_t *data, uint16_t n, bool erase) {
    if (a >= DFU_FLASH_SIZE || n > DFU_FLASH_SIZE - a || (a & (FLASH_WORD_SIZE - 1U)) ||
        (n & (FLASH_WORD_SIZE - 1U)) ||
        (erase && ((a & (FLASH_SECTOR_SIZE - 1U)) || FLASH_SECTOR_SIZE > DFU_FLASH_SIZE - a)))
        return ERR_ADDRESS;
    if (!FLASH_Open()) {
        FLASH_Close();
        return ERR_TARGET;
    }
    bool good = true;

    if (erase) {
        FLASH_Address(FLASH_CMD_ERASE_SECTOR, a);
        good = FLASH_WaitFlash();
    }
    if (erase) {
        /* CodeFlash CPU/XIP reads are scrambled: erased words read as
         * f3f9bda9, not ffffffff on CH572. Verify raw SPI bytes, not XIP. */
        if (!good || !FLASH_ioOk) {
            FLASH_Close();
            return ERR_ERASE;
        }
        FLASH_Begin(FLASH_CMD_READ_FAST);
        FLASH_WriteByte(a >> 16);
        FLASH_WriteByte(a >> 8);
        FLASH_WriteByte(a);
        FLASH_WriteByte(0);
        FLASH_WriteByte(0);
        for (uint32_t i = 0; i < FLASH_SECTOR_SIZE; i++)
            if (FLASH_ReadByte() != FLASH_ERASED_BYTE) {
                good = false;
                break;
            }
    }
    FLASH_Close();
    if (!good || !FLASH_ioOk)
        return ERR_CHECK;
    if (!n)
        return OK;
    if (!FLASH_Open()) {
        FLASH_Close();
        return ERR_TARGET;
    }
    uint32_t endAddress = a + n;

    for (uint32_t p = a; p < endAddress && good;) {
        FLASH_Address(FLASH_CMD_PROGRAM_PAGE, p);
        do {
            R32_FLASH_DATA = *(const uint32_t *)(const void *)(data + (p - a));
            for (unsigned k = 0; k < FLASH_WORD_SIZE; k++) {
                if (!FLASH_ControllerWait()) {
                    good = false;
                    break;
                }
                R8_FLASH_CTRL = FLASH_CTRL_SHIFT_BYTE;
            }
            p += FLASH_WORD_SIZE;
        } while (p < endAddress && (p & (FLASH_PAGE_SIZE - 1U)) && good);
        good = good && FLASH_WaitFlash();
    }
    FLASH_Close();
    if (!good || !FLASH_ioOk)
        return ERR_PROG;
    DFU_readGuard = 1;
    DFU_readFault = 0;
    for (uint16_t i = 0; i < n; i++)
        if (*(const volatile uint8_t *)(uintptr_t)(a + i) != data[i]) {
            good = false;
            break;
        }
    DFU_readGuard = 0;
    return good && !DFU_readFault ? OK : ERR_VERIFY;
}
