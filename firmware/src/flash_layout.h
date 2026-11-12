#ifndef FLASH_LAYOUT_H
#define FLASH_LAYOUT_H

/* Shared by the reset entry, role handoffs and DFU address bounds.
 * Native linker scripts enforce the same boundaries independently. */
#define FLASH_DFU_BASE       0x00000000
#define FLASH_DFU_SIZE       0x00002000
#define FLASH_DFU_ENTRY_SLOT (FLASH_DFU_BASE + FLASH_DFU_SIZE - 4)
#define FLASH_APP_BASE       (FLASH_DFU_BASE + FLASH_DFU_SIZE)
#define FLASH_CODE_END       0x0003C000
#define FLASH_LOCAL_BASE     FLASH_APP_BASE
#define FLASH_HOST_BASE      0x00010000
#define FLASH_TARGET_BASE    0x00020000

/* Host peer-binding records: two alternating sectors at the end of the Host
 * bank. The Host image must never extend into them (see host.ld/CI). */
#define FLASH_BIND_A 0x0001E000
#define FLASH_BIND_B 0x0001F000
#endif
