ENTRY(__cold_start)
__cold_start = 0;
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x00010000, LENGTH = 180224
    ROUTER (rx) : ORIGIN = 0x00000000, LENGTH = 4K
    DFUSTORE (rx) : ORIGIN = 0x00034000, LENGTH = 8K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 12K
}
PROVIDE(__stack_size = 1536);
SECTIONS
{
    .router_image : { KEEP(*(.router_image)) } > ROUTER
    .dfu_image : { KEEP(*(.dfu_image)) } > DFUSTORE
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
        *(.cold_flash)
        *(.handle_reset .vector_handler)
        *(EXCLUDE_FILE (*gpio_swd.o *spi_swd.o *gpio_jtag.o) .text)
        *(EXCLUDE_FILE (*gpio_swd.o *spi_swd.o *gpio_jtag.o) .text.*)
        *(EXCLUDE_FILE (*gpio_swd.o *spi_swd.o *gpio_jtag.o) .rodata)
        *(EXCLUDE_FILE (*gpio_swd.o *spi_swd.o *gpio_jtag.o) .rodata.*)
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

    /* UID Flash-driver code runs only before RF initialization. The reliable
     * RF payload queues then reuse these addresses; metadata remains resident. */
    OVERLAY ALIGN(512) : NOCROSSREFS AT(ALIGN(LOADADDR(.data) + SIZEOF(.data), 4))
    {
        .rf_uart_boot { *(.boot_flash_io) }
        .rf_uart_queue { *(.rf_uart.queue) }
        .rf_sleep { *(.usb_sleep_ram) }
    } > RAM
    __rf_uart_boot_load = LOADADDR(.rf_uart_boot);
    __rf_uart_boot_start = ADDR(.rf_uart_boot);
    __rf_uart_boot_end = ADDR(.rf_uart_boot) + SIZEOF(.rf_uart_boot);
    __usb_sleep_load = LOADADDR(.rf_sleep);
    __usb_sleep_start = ADDR(.rf_sleep);
    __usb_sleep_end = ADDR(.rf_sleep) + SIZEOF(.rf_sleep);
    ASSERT(SIZEOF(.rf_sleep) <= SIZEOF(.rf_uart_queue), "USB sleep exceeds stopped RF queues")

    .bss (NOLOAD) : ALIGN(4)
    {
        _sbss = .;
        /* Place capacity-aligned buffers first to avoid repeated large gaps. */
        *(.bss.rx_data)
        *(.bss.tx_data)
        *(SORT_BY_ALIGNMENT(.bss*))
        *(SORT_BY_ALIGNMENT(.sbss*))
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        _end = .;
    } > RAM


    OVERLAY ALIGN(4) : NOCROSSREFS AT(ALIGN(LOADADDR(.rf_sleep) + SIZEOF(.rf_sleep), 4))
    {
        .ov_gpio { *gpio_swd.o(.text .text.* .rodata .rodata.*) }
        .ov_spi  { *spi_swd.o(.text .text.* .rodata .rodata.*) }
        .ov_jtag { *gpio_jtag.o(.text .text.* .rodata .rodata.*) }
    } > RAM
    __ov_gpio_start = ADDR(.ov_gpio);
    __ov_gpio_end   = ADDR(.ov_gpio) + SIZEOF(.ov_gpio);
    __ov_gpio_load  = LOADADDR(.ov_gpio);
    __ov_spi_start  = ADDR(.ov_spi);
    __ov_spi_end    = ADDR(.ov_spi) + SIZEOF(.ov_spi);
    __ov_spi_load   = LOADADDR(.ov_spi);
    __ov_jtag_start = ADDR(.ov_jtag);
    __ov_jtag_end   = ADDR(.ov_jtag) + SIZEOF(.ov_jtag);
    __ov_jtag_load  = LOADADDR(.ov_jtag);
    __flash_end = ALIGN(LOADADDR(.rf_sleep) + SIZEOF(.rf_sleep), 4) + SIZEOF(.ov_gpio) + SIZEOF(.ov_spi) + SIZEOF(.ov_jtag);


    /* BSS precedes the overlays so code growth does not force a second
     * capacity-alignment gap. The overlay storage also counts against RAM. */
    __ram_end = ADDR(.ov_gpio) + MAX(SIZEOF(.ov_gpio), MAX(SIZEOF(.ov_spi), SIZEOF(.ov_jtag)));
    __stack_bottom = ORIGIN(RAM) + LENGTH(RAM) - __stack_size;
    __usb_dfu_app_load = ALIGN(__flash_end, 4);
    .dfu_app_ram ALIGN(__ram_end, 4) : AT(__usb_dfu_app_load) {
        __usb_dfu_app_start = .;
        KEEP(*(.dfu_app_ram))
    } > RAM
    __ram_end = ADDR(.dfu_app_ram) + SIZEOF(.dfu_app_ram);
    __flash_end = LOADADDR(.dfu_app_ram) + SIZEOF(.dfu_app_ram);
    ASSERT(((LOADADDR(.dfu_app_ram) | ADDR(.dfu_app_ram) | SIZEOF(.dfu_app_ram)) & 3) == 0, "DFU preload requires word-aligned load/start/size")
    ASSERT(USB_DfuEnter >= 0x20000000 && USB_DfuEnter < 0x20003000, "DFU entry SRAM")
    ASSERT(CLK_SetSysClock >= 0x20000000 && CLK_SetSysClock < 0x20003000, "DFU clock closure SRAM")
    .stack __stack_bottom (NOLOAD) :
    {
        . += __stack_size;
        _eusrstack = .;
    } > RAM

    /* The standalone host bank link leaves zero fill in the 2-byte alignment
     * gap before the DFU preload. Keep that gap explicit so the merged image
     * stays byte-identical to the programmed bank build. */
    __rf_sleep_end = LOADADDR(.rf_sleep) + SIZEOF(.rf_sleep);
    ASSERT(ALIGN(__rf_sleep_end, 4) - __rf_sleep_end == 2,
           "Dongle bank alignment pad is no longer two bytes")
    .bank_align_pad (__rf_sleep_end) : AT(__rf_sleep_end)
    {
        SHORT(0x0000);
    }

    /DISCARD/ : { *(.comment .note .eh_frame .eh_frame.*) }

    ASSERT(SIZEOF(.rf_uart_queue) >= 1024 && (SIZEOF(.rf_uart_queue) & 511) == 0,
           "RF payload queue overlay must retain whole 512-byte capacity units")
    ASSERT(SIZEOF(.rf_uart_boot) <= SIZEOF(.rf_uart_queue), "UID boot overlay exceeds RF payload queues")
    ASSERT((ADDR(.rf_uart_queue) & 511) == 0, "RF payload queues must retain capacity alignment")
    ASSERT(SIZEOF(.ov_gpio) <= 1536, "GPIO SWD overlay exceeds 1536 bytes")
    ASSERT(SIZEOF(.ov_spi) <= 1536, "SPI SWD overlay exceeds 1536 bytes")
    ASSERT(SIZEOF(.ov_jtag) <= 1536, "GPIO JTAG overlay exceeds 1536 bytes")
    ASSERT(__stack_size >= 768 && (__stack_size & 255) == 0, "Stack reserve must be at least 768 bytes and 256-byte aligned")
    ASSERT((__stack_bottom & 255) == 0 && (_eusrstack & 15) == 0, "Stack bounds are misaligned")
    ASSERT(__ram_end <= __stack_bottom, "RAM data/code overlaps the reserved stack")
    ASSERT(__flash_end <= ORIGIN(FLASH) + LENGTH(FLASH), "application overlaps BootLoader")
    ASSERT(SIZEOF(.init) == 4, "WCH option word requires a 4-byte startup jump")
    ASSERT(LOADADDR(.highcode) == 65540, "WCH vector Flash image must start at address 4")
    ASSERT((_vector_base & 1023) == 0, "RAM interrupt vector must be 1024-byte aligned")
}

ASSERT(MODE_mailbox == 0x20000090 && __role_mailbox_end == 0x200000d0, "Role mailbox/vector ABI")

ASSERT(__flash_end <= 0x20000, "Dongle HOST overlaps the next bank")
ASSERT(SIZEOF(.router_image) > 0 && SIZEOF(.router_image) <= 0x1000, "Router image is missing or oversized")
ASSERT(SIZEOF(.dfu_image) == 8192, "DFU image is missing or oversized")
