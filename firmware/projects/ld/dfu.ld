OUTPUT_ARCH(riscv)
ENTRY(dfu_enter)
SECTIONS {
 .dfu_entry 0x34000 : { KEEP(*(.dfu_gate)) KEEP(*(.dfu_entry)) }
 __ram_load = ALIGN(LOADADDR(.dfu_entry)+SIZEOF(.dfu_entry),4);
 __tail_pointer = 0x35ffc;
 .tail_pointer __tail_pointer : { LONG(dfu_enter); }
 .ram 0x20000000 : AT(__ram_load) {
   __ram_start = .; *(.text .text.*) *(.rodata .rodata.* .srodata .srodata.*)
   . = ALIGN(4); __rw_start = .; *(.data .data.* .sdata .sdata.*)
   . = ALIGN(4); __ram_initialized_end = .;
 }
 .bss (NOLOAD) : { __bss_start = .; *(.bss .bss.* .sbss .sbss.* COMMON)
   . = ALIGN(4); __bss_end = .; }
 __global_pointer$ = __rw_start+0x800;
 __stack_bottom = 0x20003000-1024;
 /DISCARD/ : { *(.comment .riscv.attributes .eh_frame*) }
 ASSERT((ADDR(.dfu_entry)&4095)==0,"tail start must be 4KiB aligned")
 ASSERT((dfu_enter&3)==0,"entry must be callable and 4B aligned")
 ASSERT(__ram_load+SIZEOF(.ram)<=0x35ffc,"tail payload overlaps last4B")
 ASSERT(__bss_end<=__stack_bottom,"SRAM exhausted")
}
