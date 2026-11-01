ENTRY(_start)
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00020000, LENGTH = 114688
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 12K
}
PROVIDE(__stack_size = 1536);
SECTIONS
{
    .init : { KEEP(*(.init)) } > FLASH
    /* Align the RAM vector, without padding its Flash image to 1024 bytes.
     * WCH reserves the vector's option word at application address 0x14. */
    .highcode ORIGIN(RAM) : AT(LOADADDR(.init) + SIZEOF(.init))
    {
        _highcode_vma_start = .;
        KEEP(*(.vector))
        MODE_mailbox = .;
        . += 64;
        __role_mailbox_end = .;
        *(.highcode .highcode.*)
        . = ALIGN(4);
        _highcode_vma_end = .;
    } > RAM
    _highcode_lma = LOADADDR(.highcode);

    .text (LOADADDR(.highcode) + SIZEOF(.highcode)) : ALIGN(4)
    {
        *(.handle_reset .vector_handler)
        *(.cold_flash)
        *(EXCLUDE_FILE (*gpio_swd.o *spi_swd.o *spi_swd_transfer_fast.o *gpio_jtag.o) .text)
        *(EXCLUDE_FILE (*gpio_swd.o *spi_swd.o *spi_swd_transfer_fast.o *gpio_jtag.o) .text.*)
        *(EXCLUDE_FILE (*gpio_swd.o *spi_swd.o *spi_swd_transfer_fast.o *gpio_jtag.o) .rodata)
        *(EXCLUDE_FILE (*gpio_swd.o *spi_swd.o *spi_swd_transfer_fast.o *gpio_jtag.o) .rodata.*)
        . = ALIGN(4);
    } > FLASH

    .data : ALIGN(4)
    {
        _data_vma = .;
        __global_pointer$ = . + 0x800;
        *(.data .data.* .sdata .sdata.* .srodata .srodata.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT> FLASH
    _data_lma = LOADADDR(.data);

    .bss ADDR(.data) + SIZEOF(.data) (NOLOAD) :
    {
        _sbss = .;
        /* Use the small gap after .data before the first capacity-aligned
         * buffer. These ordinary state objects are cleared by the same BSS
         * startup/role-loader range as the remaining variables. */
        *(.bss.BSP_leds)
        /* Fit the 64/128-byte USB banks into the gap before the first
         * capacity-aligned UART ring. Their DMA owners are unchanged. */
        *(.bss.ep5)
        *(.bss.USB_ep0)
        /* Preserve every DMA/ring object's own input-section alignment. */
        *(.bss.target_tx_data)
        *(.bss.rx_data)
        *(.bss.tx_data)
        /* A DAP-only 64-byte EP0 fits the packet window's alignment tail,
         * before the 128-byte HID bank. Keep each buffer's own alignment. */
        *(.bss.queue)
        *(.bss.dap_queue)
        /* Word-aligned state fills the pool tail before the next DMA bank.
         * An eight-slot 64-byte pool ends at +16 modulo 64. */
        *(.bss.SWJ_delayLoops)
        *(.bss.MODE_state)
        *(.bss.CLOCK_timebase)
        *(.bss.ep6)
        *(.bss.ep1)
        *(SORT_BY_ALIGNMENT(.bss*))
        *(SORT_BY_ALIGNMENT(.sbss*))
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        _end = .;
    } > RAM


    OVERLAY ALIGN(4) : NOCROSSREFS AT(ALIGN(LOADADDR(.data) + SIZEOF(.data), 4))
    {
        .ov_boot { *(.boot_flash_io) }
        .ov_gpio { *gpio_swd.o(.text .text.* .rodata .rodata.*) }
        .ov_spi  { *spi_swd_transfer_fast.o(.text .text.* .rodata .rodata.*) *spi_swd.o(.text .text.* .rodata .rodata.*) }
        .ov_jtag { *gpio_jtag.o(.text .text.* .rodata .rodata.*) }
        .ov_sleep { *(.usb_sleep_ram) }
    } > RAM
    __ov_boot_start = ADDR(.ov_boot);
    __ov_boot_end   = ADDR(.ov_boot) + SIZEOF(.ov_boot);
    __ov_boot_load  = LOADADDR(.ov_boot);
    __ov_gpio_start = ADDR(.ov_gpio);
    __ov_gpio_end   = ADDR(.ov_gpio) + SIZEOF(.ov_gpio);
    __ov_gpio_load  = LOADADDR(.ov_gpio);
    __ov_spi_start  = ADDR(.ov_spi);
    __ov_spi_end    = ADDR(.ov_spi) + SIZEOF(.ov_spi);
    __ov_spi_load   = LOADADDR(.ov_spi);
    __ov_jtag_start = ADDR(.ov_jtag);
    __ov_jtag_end   = ADDR(.ov_jtag) + SIZEOF(.ov_jtag);
    __ov_jtag_load  = LOADADDR(.ov_jtag);
    __usb_sleep_load = LOADADDR(.ov_sleep);
    __usb_sleep_start = ADDR(.ov_sleep);
    __usb_sleep_end = ADDR(.ov_sleep) + SIZEOF(.ov_sleep);
    ASSERT(SIZEOF(.ov_sleep) <= 1024, "USB sleep closure exceeds reusable bank")
    __flash_end = ALIGN(LOADADDR(.data) + SIZEOF(.data), 4) + SIZEOF(.ov_boot) + SIZEOF(.ov_gpio) + SIZEOF(.ov_spi) + SIZEOF(.ov_jtag) + SIZEOF(.ov_sleep);


    /* BSS precedes the overlays so code growth does not force a second
     * capacity-alignment gap. The overlay storage also counts against RAM. */
    __ram_end = ADDR(.ov_boot) + MAX(SIZEOF(.ov_sleep), MAX(SIZEOF(.ov_boot), MAX(SIZEOF(.ov_gpio), MAX(SIZEOF(.ov_spi), SIZEOF(.ov_jtag)))));
    __stack_bottom = ORIGIN(RAM) + LENGTH(RAM) - __stack_size;
    .stack __stack_bottom (NOLOAD) :
    {
        . += __stack_size;
        _eusrstack = .;
    } > RAM
    /DISCARD/ : { *(.comment .note .eh_frame .eh_frame.*) }

    ASSERT(SIZEOF(.ov_gpio) <= 1536, "GPIO SWD overlay exceeds 1536 bytes")
    ASSERT(SIZEOF(.ov_spi) <= 1536, "SPI SWD overlay exceeds 1536 bytes")
    ASSERT(SIZEOF(.ov_jtag) <= 1536, "GPIO JTAG overlay exceeds 1536 bytes")
    ASSERT(__stack_size >= 768 && (__stack_size & 255) == 0, "Stack reserve must be at least 768 bytes and 256-byte aligned")
    ASSERT((__stack_bottom & 255) == 0 && (_eusrstack & 15) == 0, "Stack bounds are misaligned")
    ASSERT(__ram_end <= __stack_bottom, "RAM data/code overlaps the reserved stack")
    ASSERT(__flash_end <= ORIGIN(FLASH) + LENGTH(FLASH), "application overlaps BootLoader")
    ASSERT(SIZEOF(.init) == 4, "WCH option word requires a 4-byte startup jump")
    ASSERT(LOADADDR(.highcode) == 131076, "WCH vector Flash image must start at address 4")
    ASSERT((_vector_base & 1023) == 0, "RAM interrupt vector must be 1024-byte aligned")
}

ASSERT(MODE_mailbox == 0x20000090 && __role_mailbox_end == 0x200000d0, "Role mailbox/vector ABI")
