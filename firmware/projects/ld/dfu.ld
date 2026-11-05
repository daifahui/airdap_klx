OUTPUT_ARCH(riscv)
ENTRY(_start)
SECTIONS {
    .init 0 : { KEEP(*(.init)) }
    .dfu_entry ALIGN(4) : { KEEP(*(.dfu_entry)) }
    __ram_load = ALIGN(LOADADDR(.dfu_entry) + SIZEOF(.dfu_entry), 4);
    __dfu_pointer = 0x1ffc;
    .dfu_pointer __dfu_pointer : { LONG(dfu_enter); }
    .ram 0x20000000 : AT(__ram_load) {
        __ram_start = .;
        *(.text .text.*) *(.rodata .rodata.* .srodata .srodata.*)
        . = ALIGN(4);
        __rw_start = .;
        *(.data .data.* .sdata .sdata.*)
        . = ALIGN(4);
        __ram_initialized_end = .;
    }
    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss .bss.* .sbss .sbss.* COMMON)
        . = ALIGN(4);
        __bss_end = .;
    }
    __global_pointer$ = __rw_start + 0x800;
    __stack_bottom = 0x20003000 - 1024;
    __dfu_payload_end = __ram_load + SIZEOF(.ram);
    /DISCARD/ : { *(.comment .riscv.attributes .eh_frame*) }
    ASSERT(_start == 0, "DFU must own the hardware reset entry")
    ASSERT((dfu_enter & 3) == 0, "DFU entry must be 4-byte aligned")
    ASSERT(__dfu_payload_end <= __dfu_pointer, "DFU exceeds the first 8 KiB")
    ASSERT(__bss_end <= __stack_bottom, "DFU SRAM exhausted")
}
