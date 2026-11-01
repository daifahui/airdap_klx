OUTPUT_ARCH(riscv)
OUTPUT_FORMAT("elf32-littleriscv")
TARGET(binary)
ENTRY(_start)
_start = 0;
SECTIONS {
    .router 0x0 : { KEEP(*Router.bin(.data)) }
    ASSERT(SIZEOF(.router) > 0 && ADDR(.router) + SIZEOF(.router) <= 0x1000, "Router image missing or overlaps next bank")
    .local 0x1000 : { KEEP(*Local.bin(.data)) }
    ASSERT(SIZEOF(.local) > 0 && ADDR(.local) + SIZEOF(.local) <= 0x10000, "Local image missing or overlaps next bank")
    .host 0x10000 : { KEEP(*Host.bin(.data)) }
    ASSERT(SIZEOF(.host) > 0 && ADDR(.host) + SIZEOF(.host) <= 0x20000, "Host image missing or overlaps next bank")
    .target 0x20000 : { KEEP(*Target.bin(.data)) }
    ASSERT(SIZEOF(.target) > 0 && ADDR(.target) + SIZEOF(.target) <= 0x34000, "Target image missing or overlaps next bank")
    .dfu 0x34000 : { KEEP(*DFU.bin(.data)) }
    ASSERT(SIZEOF(.dfu) > 0 && ADDR(.dfu) + SIZEOF(.dfu) <= 0x36000, "DFU image missing or overlaps next bank")
    ASSERT(SIZEOF(.dfu) == 8192, "DFU must contain two sectors and its entry pointer")
    /* MRS adds -lm; only the named binary banks belong in this image. */
    /DISCARD/ : { *libm.a(*) }
}
