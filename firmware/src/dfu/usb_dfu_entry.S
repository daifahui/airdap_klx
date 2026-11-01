#include "app_config.h"
/* Complete-application adapter. All code before the tail jump is resident RAM.
 * The existing CLK_SetSysClock/CLOCK_SafeOperation RAM closure is used only BEFORE SRAM
 * overwrite, with the valid non-ISR caller stack. No old frame is returned to.
 * Tail dfu_enter independently enters M mode, quiesces everything, then copies.
 */
    .section .dfu_app_ram,"ax",@progbits
    .option norelax
    .balign 4
    .global USB_DfuEnter,USB_DfuEnterAtVector
    .type USB_DfuEnter,@function
USB_DfuEnter:
    li t0,0x88
    csrrc zero,0x800,t0
    la a0,_vector_base
USB_DfuEnterAtVector:
    li t0,0x88
    csrrc zero,0x800,t0
    mv s0,a0
    /* CPU IE stays masked across the SRAM clock leaf. The independent tail
     * masks PFIC before its ECALL/copy; none are re-enabled in this adapter. */
    li s1,0x40001000
    li s2,0x57
    li s3,0xa8
    /* First peripheral operation: ungate USB solely to disconnect it. */
    lbu t1,0xd(s1)
    andi t1,t1,-17
    sb s2,0x40(s1)
    sb s3,0x40(s1)
    sb t1,0xd(s1)
    li t0,0x40008000
    sw zero,0(t0)
    sh zero,0x18(s1) /* remove forced DP pullup and old pin remaps now */
    /* Stop autonomous SRAM writers before any later copy. Ungate their
     * registers first; BLE clock is removed rather than running old RF. */
    sb s2,0x40(s1)
    sb s3,0x40(s1)
    sb zero,0xc(s1)
    li t1,0x80
    sb s2,0x40(s1)
    sb s3,0x40(s1)
    sb t1,0xd(s1)
    sb zero,0x1d(s1)
    sb zero,0x40(s1)
    li t0,0x40002400
    sw zero,0(t0)
    li t0,0x40004000
    sw zero,0(t0)
    li t0,0x40005000
    sw zero,0(t0)
    sb zero,0xc(t0)
    /* Do not record into an abandoned application trace while restoring HSE. */
    lui t0,%hi(CLOCK_sdkTrace)
    sw zero,%lo(CLOCK_sdkTrace)(t0)
    li a0,10
    jal ra,CLK_SetSysClock  /* SRAM closure: HSE/10, including LSI->HSE startup */
    /* Restart PLL from known parameters, then restore Flash SCK/wait strategy
     * at conservative HSE/10 before fetching the tail entry word. */
    li t1,4
    sb s2,0x40(s1)
    sb s3,0x40(s1)
    sb t1,0xa(s1)
    li t1,10
    sb t1,0x4b(s1)
    li t1,0x14
    sb t1,0xa(s1)
    li t0,40000
1:  addi t0,t0,-1
    bnez t0,1b
    sb s2,0x40(s1)
    sb s3,0x40(s1)
    li t0,0x40001800
    li t1,7
    sb t1,7(t0)
    li t1,0x10
    sb t1,5(t0)
    sb zero,0x40(s1)
    mv a0,s0
    li t0,USB_DFU_POINTER_SLOT
    lw t0,0(t0)
    fence.i
    jr t0
    .balign 4
    .global __usb_dfu_app_end
__usb_dfu_app_end:
    .size USB_DfuEnter,.-USB_DfuEnter

    .section .text.usb_dfu_preload,"ax",@progbits
    .global usb_dfu_preload
usb_dfu_preload:
    la t0,__usb_dfu_app_load
    la t1,__usb_dfu_app_start
    la t2,__usb_dfu_app_end
1:  lw a0,0(t0)
    sw a0,0(t1)
    addi t0,t0,4
    addi t1,t1,4
    bltu t1,t2,1b
    fence.i
    ret
