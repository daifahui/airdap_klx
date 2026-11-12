/* HOST/DONGLE peer-binding persistence. Records alternate between two Flash
 * sectors so an interrupted erase/program always leaves the other record
 * valid. The erase/program closure executes from SRAM (XIP stops there). */
#include "ch572/ch572.h"
#include "ch572/watchdog.h"
#include "bsp/board.h"
#include "cfg/bind_store.h"
#include "flash_layout.h"
#include "mode/app_mode.h"
#include "mode/mode_image.h"

#if APP_COMBINED_HOST

#include "rf/rf_bridge.h"
#include "rf/rf_dap.h"

#define BIND_MAGIC         0x314E4442U /* "BDN1" little-endian. */
#define BIND_VERSION       1
#define BIND_FLAG_BOUND    0x0001
#define BIND_RECORD_SIZE   32U
#define BIND_RECORD_BODY   20U /* magic..seq, covered by crc. */
#define BIND_RECORD_MAGIC  0U
#define BIND_RECORD_VER    4U
#define BIND_RECORD_FLAGS  6U
#define BIND_RECORD_UID    8U
#define BIND_RECORD_SEQ    16U
#define BIND_RECORD_CRC    20U

#define BIND_REQ_NONE  0U
#define BIND_REQ_SAVE  1U
#define BIND_REQ_CLEAR 2U
#define BIND_REQ_SCAN  3U

static uint8_t  BIND_uid[BIND_UID_SIZE];
static uint32_t BIND_sequence;
static uint8_t  BIND_newest; /* 0 = none, 1 = A, 2 = B. */
static uint8_t  BIND_bound;
static uint8_t  BIND_pending;
static uint8_t  BIND_pendingUid[BIND_UID_SIZE];

static const volatile uint8_t *BIND_Sector(uint8_t sector) {
    uint32_t address = sector == 2 ? FLASH_BIND_B : FLASH_BIND_A;

    return (const volatile uint8_t *)(uintptr_t)address;
}

static uint32_t BIND_SectorAddress(uint8_t sector) {
    return sector == 2 ? FLASH_BIND_B : FLASH_BIND_A;
}

static uint32_t BIND_Read32(const volatile uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint16_t BIND_Read16(const volatile uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void BIND_Put32(uint8_t *p, uint32_t value) {
    p[0] = value;
    p[1] = value >> 8;
    p[2] = value >> 16;
    p[3] = value >> 24;
}

static void BIND_Put16(uint8_t *p, uint16_t value) {
    p[0] = value;
    p[1] = value >> 8;
}

/* Bitwise CRC-32; stays in Flash and runs only outside the flash window. */
static uint32_t BIND_Crc(const volatile uint8_t *p, unsigned n) {
    uint32_t crc = 0xFFFFFFFFU;

    for (unsigned i = 0; i < n; ++i) {
        crc ^= p[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

static bool BIND_RecordValid(const volatile uint8_t *p) {
    if (BIND_Read32(p + BIND_RECORD_MAGIC) != BIND_MAGIC ||
        BIND_Read16(p + BIND_RECORD_VER) != BIND_VERSION) {
        return false;
    }
    return BIND_Read32(p + BIND_RECORD_CRC) == BIND_Crc(p, BIND_RECORD_BODY);
}

static void BIND_CopyUid(uint8_t *dst, const volatile uint8_t *src) {
    for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
        dst[i] = src[i];
    }
}

static bool BIND_UidValid(const uint8_t uid[BIND_UID_SIZE]) {
    return RF_DiscoveryAddressForUid(uid) != 0;
}

void BIND_Load(void) {
    uint32_t irq  = BSP_IrqSave();
    bool     hasA = BIND_RecordValid(BIND_Sector(1));
    bool     hasB = BIND_RecordValid(BIND_Sector(2));
    uint8_t  sector;

    if (hasA && hasB) {
        sector = BIND_Read32(BIND_Sector(1) + BIND_RECORD_SEQ) >
                         BIND_Read32(BIND_Sector(2) + BIND_RECORD_SEQ)
                     ? 1U
                     : 2U;
    } else if (hasA) {
        sector = 1U;
    } else if (hasB) {
        sector = 2U;
    } else {
        BIND_newest  = 0;
        BIND_bound   = 0;
        BIND_sequence = 0;
        for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
            BIND_uid[i] = 0;
        }
        BSP_IrqRestore(irq);
        return;
    }
    const volatile uint8_t *p = BIND_Sector(sector);

    BIND_newest   = sector;
    BIND_sequence = BIND_Read32(p + BIND_RECORD_SEQ);
    BIND_CopyUid(BIND_uid, p + BIND_RECORD_UID);
    BIND_bound = (BIND_Read16(p + BIND_RECORD_FLAGS) & BIND_FLAG_BOUND) != 0 &&
                 BIND_Read16(p + BIND_RECORD_FLAGS) == BIND_FLAG_BOUND && BIND_UidValid(BIND_uid);
    BSP_IrqRestore(irq);
}

bool BIND_Get(uint8_t uid[BIND_UID_SIZE]) {
    uint32_t irq   = BSP_IrqSave();
    bool     bound = BIND_bound != 0;

    if (uid) {
        for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
            uid[i] = bound ? BIND_uid[i] : 0;
        }
    }
    BSP_IrqRestore(irq);
    return bound;
}

bool BIND_RequestSave(const uint8_t uid[BIND_UID_SIZE]) {
    if (!BIND_UidValid(uid)) {
        return false;
    }
    uint32_t irq = BSP_IrqSave();

    if (BIND_bound && BIND_newest && BIND_uid[0] == uid[0] &&
        memcmp((const void *)BIND_uid, uid, BIND_UID_SIZE) == 0 && !RF_BridgeScanOnlyActive()) {
        BSP_IrqRestore(irq);
        return true; /* Already stored: no flash wear. */
    }
    for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
        BIND_pendingUid[i] = uid[i];
    }
    BIND_pending = BIND_REQ_SAVE;
    BSP_IrqRestore(irq);
    return true;
}

bool BIND_RequestClear(void) {
    uint32_t irq = BSP_IrqSave();

    for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
        BIND_pendingUid[i] = 0;
    }
    BIND_pending = BIND_REQ_CLEAR;
    BSP_IrqRestore(irq);
    return true;
}

bool BIND_RequestScanOnly(void) {
    uint32_t irq = BSP_IrqSave();

    BIND_pending = BIND_REQ_SCAN;
    BSP_IrqRestore(irq);
    return true;
}

/* ---- SRAM-only flash window ------------------------------------------- */

#define __BIND_FLASH_CODE __HIGH_CODE __NO_INLINE

static __BIND_FLASH_CODE bool BIND_FlashWait(void) {
    uint32_t limit = 0x80000U;

    while (R8_FLASH_CTRL & RB_FLASH_CTRL_BUSY) {
        if (!--limit) {
            return false;
        }
    }
    return true;
}

static __BIND_FLASH_CODE void BIND_FlashBegin(uint8_t command) {
    R8_FLASH_CTRL = 0x00U;
    R8_FLASH_CTRL = 0x05U;
    __asm__ volatile("nop; nop");
    R8_FLASH_DATA = command;
}

static __BIND_FLASH_CODE bool BIND_FlashWrite8(uint8_t value) {
    if (!BIND_FlashWait()) {
        return false;
    }
    R8_FLASH_DATA = value;
    return true;
}

static __BIND_FLASH_CODE uint8_t BIND_FlashRead8(void) {
    return BIND_FlashWait() ? R8_FLASH_DATA : 0xFFU;
}

static __BIND_FLASH_CODE bool BIND_FlashEnd(void) {
    bool ok = BIND_FlashWait();

    R8_FLASH_CTRL = 0x00U;
    return ok;
}

static __BIND_FLASH_CODE bool BIND_FlashWaitFlash(void) {
    uint32_t limit = 0x80000U;

    if (!BIND_FlashEnd()) {
        return false;
    }
    do {
        BIND_FlashBegin(0x05U);
        (void)BIND_FlashRead8();
        uint8_t status = BIND_FlashRead8();

        if (!BIND_FlashEnd()) {
            return false;
        }
        if (!(status & 0x01U)) {
            return true;
        }
    } while (--limit);
    return false;
}

static __BIND_FLASH_CODE bool BIND_FlashAddress(uint8_t command, uint32_t address) {
    BIND_FlashBegin(0x06U); /* WRITE_ENABLE */
    if (!BIND_FlashEnd()) {
        return false;
    }
    BIND_FlashBegin(command);
    return BIND_FlashWrite8((uint8_t)(address >> 16)) &&
           BIND_FlashWrite8((uint8_t)(address >> 8)) && BIND_FlashWrite8((uint8_t)address);
}

static __BIND_FLASH_CODE void BIND_FlashOpen(void) {
    R8_SAFE_ACCESS_SIG  = 0x57U;
    R8_SAFE_ACCESS_SIG  = 0xA8U;
    R8_GLOB_ROM_CFG     = (uint8_t)(0xC0U | 0x20U);
    R8_FLASH_CTRL       = 0x04U; /* DIRECT */
    BIND_FlashBegin(0xFFU);      /* RELEASE */
    (void)BIND_FlashWrite8(0xFFU);
    (void)BIND_FlashEnd();
}

static __BIND_FLASH_CODE void BIND_FlashClose(void) {
    (void)BIND_FlashEnd();
    R8_SAFE_ACCESS_SIG = 0x57U;
    R8_SAFE_ACCESS_SIG = 0xA8U;
    R8_GLOB_ROM_CFG    = (uint8_t)(R8_GLOB_ROM_CFG & RB_ROM_CODE_OFS);
    R8_SAFE_ACCESS_SIG = 0x00U;
    __asm__ volatile("fence.i" ::: "memory");
}

static __BIND_FLASH_CODE bool BIND_FlashWrite(uint32_t address, const uint8_t *record) {
    if ((address != FLASH_BIND_A && address != FLASH_BIND_B) || !record) {
        return false;
    }
    if (R32_SAFE_ACCESS_SIG2 & RB_FLASH_HALTED) {
        return false;
    }

    WDOG_Feed();
    BIND_FlashOpen();
    bool ok = BIND_FlashAddress(0x20U, address) && BIND_FlashWaitFlash(); /* ERASE_SECTOR */
    BIND_FlashClose();
    if (!ok) {
        return false;
    }

    WDOG_Feed();
    BIND_FlashOpen();
    ok = BIND_FlashAddress(0x02U, address); /* PROGRAM_PAGE */
    for (unsigned i = 0; ok && i < BIND_RECORD_SIZE; i += 4U) {
        uint32_t word = (uint32_t)record[i] | ((uint32_t)record[i + 1] << 8) |
                        ((uint32_t)record[i + 2] << 16) | ((uint32_t)record[i + 3] << 24);

        if (!BIND_FlashWait()) {
            ok = false;
            break;
        }
        R32_FLASH_DATA = word;
        for (unsigned byte = 0; byte < 4U; ++byte) {
            if (!BIND_FlashWait()) {
                ok = false;
                break;
            }
            R8_FLASH_CTRL = 0x15U; /* SHIFT_BYTE */
        }
    }
    if (ok) {
        ok = BIND_FlashWaitFlash();
    }
    BIND_FlashClose();
    return ok;
}

/* ---- foreground commit ------------------------------------------------ */

static void BIND_BuildRecord(uint8_t record[BIND_RECORD_SIZE], const uint8_t uid[BIND_UID_SIZE],
                             bool bound, uint32_t sequence) {
    for (unsigned i = 0; i < BIND_RECORD_SIZE; ++i) {
        record[i] = 0;
    }
    BIND_Put32(record + BIND_RECORD_MAGIC, BIND_MAGIC);
    BIND_Put16(record + BIND_RECORD_VER, BIND_VERSION);
    BIND_Put16(record + BIND_RECORD_FLAGS, bound ? BIND_FLAG_BOUND : 0U);
    for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
        record[BIND_RECORD_UID + i] = uid[i];
    }
    BIND_Put32(record + BIND_RECORD_SEQ, sequence);
    BIND_Put32(record + BIND_RECORD_CRC, BIND_Crc(record, BIND_RECORD_BODY));
}

static bool BIND_Commit(bool bound, const uint8_t uid[BIND_UID_SIZE]) {
    uint8_t  record[BIND_RECORD_SIZE];
    uint32_t irq      = BSP_IrqSave();
    uint8_t  sector   = BIND_newest == 1 ? 2 : 1; /* Never erase the newest. */
    uint32_t sequence = BIND_sequence + 1U;

    BSP_IrqRestore(irq);
    BIND_BuildRecord(record, uid, bound, sequence);

    irq       = BSP_IrqSave();
    bool done = BIND_FlashWrite(BIND_SectorAddress(sector), record);

    BSP_IrqRestore(irq);
    if (!done) {
        return false;
    }

    const volatile uint8_t *p = BIND_Sector(sector);

    if (!BIND_RecordValid(p) || BIND_Read32(p + BIND_RECORD_SEQ) != sequence ||
        BIND_Read16(p + BIND_RECORD_FLAGS) != (bound ? BIND_FLAG_BOUND : 0)) {
        return false;
    }
    for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
        if (p[BIND_RECORD_UID + i] != uid[i]) {
            return false;
        }
    }

    irq           = BSP_IrqSave();
    BIND_newest   = sector;
    BIND_sequence = sequence;
    BIND_bound    = bound;
    for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
        BIND_uid[i] = uid[i];
    }
    BSP_IrqRestore(irq);
    return true;
}

static void BIND_ApplySave(const uint8_t uid[BIND_UID_SIZE]) {
    uint8_t previous[BIND_UID_SIZE];

    for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
        previous[i] = MODE_mailbox.binding.peer_uid[i];
    }
    for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
        MODE_mailbox.binding.peer_uid[i] = uid[i];
    }
    MODE_mailbox.binding.peer_aa = RF_DiscoveryAddressForUid(uid);
    MODE_mailbox.binding.bound   = 1;
    MODE_AppModeSetBound(true);

    bool changed = memcmp((const void *)previous, uid, BIND_UID_SIZE) != 0;

    RF_BridgeScanOnly(false);
    /* Any discovery in flight may belong to the previous record. */
    if (changed && RF_BridgeDiscovering())
        (void)RF_BridgeDiscover(BSP_DiscoveryNonce());
}

void BIND_Poll(void) {
    uint8_t uid[BIND_UID_SIZE];
    uint32_t irq = BSP_IrqSave();

    uint8_t command = BIND_pending;

    if (!command) {
        BSP_IrqRestore(irq);
        return;
    }
    for (unsigned i = 0; i < BIND_UID_SIZE; ++i) {
        uid[i] = BIND_pendingUid[i];
    }
    BSP_IrqRestore(irq);

    if (BSP_stopRequested || !RF_DapIdle()) {
        return;
    }

    if (command == BIND_REQ_SCAN) {
        /* No Flash access: only a foreground discovery restart is required. */
        irq            = BSP_IrqSave();
        BIND_pending   = BIND_REQ_NONE;
        BSP_IrqRestore(irq);
        MODE_AppModeSetBound(true);
        RF_BridgeScanOnly(true);
        return;
    }

    if (!RF_BridgeClockPause()) {
        return; /* Radio still owns an event; retry on the next pass. */
    }

    irq           = BSP_IrqSave();
    BIND_pending  = BIND_REQ_NONE;
    BSP_IrqRestore(irq);

    bool done = command == BIND_REQ_SAVE ? BIND_Commit(true, uid) : BIND_Commit(false, uid);

    RF_BridgeClockResume();
    if (!done) {
        return; /* The untouched sector still holds the previous record. */
    }
    if (command == BIND_REQ_SAVE) {
        BIND_ApplySave(uid);
    } else {
        MODE_AppModeSetBound(true);
        RF_BridgeForget();
    }
}

#endif
