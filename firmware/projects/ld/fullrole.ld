OUTPUT_ARCH(riscv)
OUTPUT_FORMAT("elf32-littleriscv")
TARGET(binary)
ENTRY(_start)
_start = 0;
SECTIONS {
    .dfu 0x0 : { KEEP(*DFU.bin(.data)) }
    ASSERT(SIZEOF(.dfu) == 8192, "DFU must fill the first 8 KiB")
    .local 0x2000 : { KEEP(*Local.bin(.data)) }
    ASSERT(SIZEOF(.local) > 0 && ADDR(.local) + SIZEOF(.local) <= 0x10000, "Local image missing or overlaps Host")
    .host 0x10000 : { KEEP(*Host.bin(.data)) }
    ASSERT(SIZEOF(.host) > 0 && ADDR(.host) + SIZEOF(.host) <= 0x20000, "Host image missing or overlaps Target")
    .target 0x20000 : { KEEP(*Target.bin(.data)) }
    ASSERT(SIZEOF(.target) > 0 && ADDR(.target) + SIZEOF(.target) <= 0x3c000, "Target image exceeds application Flash")
    /* MRS adds -lm; only the named binary banks belong in this image. */
    /DISCARD/ : { *libm.a(*) }
}
