ENTRY(_start)
SECTIONS {
    .init 0 : { KEEP(*(.init)) }
    /DISCARD/ : { *(.riscv.attributes .comment .note .eh_frame) }
    ASSERT(SIZEOF(.init) <= 0x1000, "Role router overlaps LOCAL")
}
