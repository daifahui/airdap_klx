/* Modified for AirDAP; see NOTICE for provenance and retained usage notices. */
#include "app_config.h"
/********************************** (C) COPYRIGHT *******************************
 * File Name          : startup.S (adapted CH572 startup)
 * Author             : WCH
 * Description        :
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

    /* Export the header-owned reserve to the native linker configuration. */
    .global __stack_size
    .set __stack_size, STACK_SIZE

	.section	.init,"ax",@progbits
	.global	_start
	.global	_vector_base
	.align	1
_start:
	j	handle_reset

    .section    .vector,"ax",@progbits
    .align  1
_vector_base:
    .option norvc;

    .word   0
    .word   0
    .word   NMI_Handler                 /* NMI Handler */
    .word   HardFault_Handler           /* Hard Fault Handler */
    .word   0xF3F9BDA9
    .word   Ecall_M_Mode_Handler        /* 5 */
    .word   0
    .word   0
    .word   Ecall_U_Mode_Handler		/* 8 */
    .word   Break_Point_Handler			/* 9 */
    .word   0
    .word   0
    .word   SysTick_Handler            	/* SysTick Handler */
    .word   0
    .word   SWI_Handler                	/* SWI Handler */
    .word   0
    /* External Interrupts */
    .word   0           				/* 16 */
    .word   GPIOA_IRQHandler           	/* GPIOA */
    .word   0
    .word   SPI_IRQHandler            	/* SPI */
    .word   BB_IRQHandler              	/* BLEB */
    .word   LLE_IRQHandler             	/* BLEL */
    .word   USB_IRQHandler             	/* USB */
    .word   0
    .word   TMR_IRQHandler            	/* TMR */
    .word   0
    .word   0
    .word   UART_IRQHandler           	/* UART */
    .word   RTC_IRQHandler             	/* RTC */
    .word   CMP_IRQHandler             	/* CMP */
    .word   I2C_IRQHandler 			   	/* I2C */
    .word   PWMX_IRQHandler       		/* PWMX */
    .word   0
    .word   KEYSCAN_IRQHandler      	/* KEYSCAN */
    .word   ENCODER_IRQHandler          /* ENCODER */
    .word   WDOG_BAT_IRQHandler        	/* WDOG_BAT */

    .option rvc;

    .section    .vector_handler, "ax", @progbits
    .weak   NMI_Handler
    .weak   HardFault_Handler
    .weak   Ecall_M_Mode_Handler
    .weak   Ecall_U_Mode_Handler
    .weak   Break_Point_Handler
    .weak   SysTick_Handler
    .weak   SWI_Handler
    .weak   GPIOA_IRQHandler
    .weak   SPI_IRQHandler
    .weak   BB_IRQHandler
    .weak   LLE_IRQHandler
    .weak   USB_IRQHandler
    .weak   TMR_IRQHandler
    .weak   UART_IRQHandler
    .weak   RTC_IRQHandler
    .weak   CMP_IRQHandler
    .weak   I2C_IRQHandler
    .weak   PWMX_IRQHandler
    .weak   KEYSCAN_IRQHandler
    .weak   ENCODER_IRQHandler
    .weak   WDOG_BAT_IRQHandler

NMI_Handler:
HardFault_Handler:
Ecall_M_Mode_Handler:
Ecall_U_Mode_Handler:
Break_Point_Handler:
SysTick_Handler:
SWI_Handler:
GPIOA_IRQHandler:
SPI_IRQHandler:
BB_IRQHandler:
LLE_IRQHandler:
USB_IRQHandler:
TMR_IRQHandler:
UART_IRQHandler:
RTC_IRQHandler:
CMP_IRQHandler:
I2C_IRQHandler:
PWMX_IRQHandler:
KEYSCAN_IRQHandler:
ENDOCE_IRQHandler:
WDOG_BAT_IRQHandler:
1:
	j 1b

	.section	.handle_reset,"ax",@progbits
	.weak	handle_reset
	.align	1
handle_reset:
    /* Cold router and warm bank entries both retain startup protection.
     * No C state/stack exists yet. Reload BEFORE arming reset-only WTR;
     * no feeds in copy/wait loops can hide a failed initialization. */
    li t0, 0x40001000
    sb zero, 0x43(t0)
    li t1, 2
    li t2, 0x57
    li t3, 0xa8
    fence.i
    sb t2, 0x40(t0)
    sb t3, 0x40(t0)
    fence.i
    sb t1, 0x46(t0)
    sb zero, 0x40(t0)
    fence.i
.option push
.option	norelax
	la gp, __global_pointer$
1:
	la sp, _eusrstack
.option	pop

/* Load highcode code  section from flash to RAM */
2:
    la a0, _highcode_lma
    la a1, _highcode_vma_start
    la a2, _highcode_vma_end
    bgeu a1, a2, 2f
1:
    lw t0, (a0)
    /* Preserve the 64-byte controller mailbox after the 144-byte vector. */
    li t1, 0x20000090
    bltu a1, t1, 3f
    li t1, 0x200000d0
    bltu a1, t1, 4f
3:
    sw t0, (a1)
4:
    addi a0, a0, 4
    addi a1, a1, 4
    bltu a1, a2, 1b

/* Load data section from flash to RAM */
2:
	la a0, _data_lma
	la a1, _data_vma
	la a2, _edata
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b
2:
	/* clear bss section */
	la a0, _sbss
	la a1, _ebss
	bgeu a0, a1, 2f
1:
	sw zero, (a0)
	addi a0, a0, 4
	bltu a0, a1, 1b
2:

#if USB_DFU_ENABLE && APP_DFU_BUILD
    call usb_dfu_preload
#endif

	/* Enable user-mode interrupts. */
	/* Enable instruction prefetch. */
	li t0, 0x25
	csrw 0xbc0, t0
	/* Enable nested interrupts and hardware context stacking. */
	li t0, 0x3
	csrw 0x804, t0
	li t0, 0x1
	csrw 0xbc1, t0

    li t0, 0x88
    csrw mstatus, t0
	la t0, _vector_base

	/* Use absolute addresses in the interrupt vector table. */
    ori t0, t0, 3
	csrw mtvec, t0

    fence.i
	la t0, main
	csrw mepc, t0

	mret
