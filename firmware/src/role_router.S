#include "app_config.h"
/* Only a real reset enters this router. Internal handoffs enter a selected
 * role directly in machine mode; reset always forgets USB override policy. */
    .section .init,"ax",@progbits
    .option norvc
    .global _start
_start:
    j MODE_ColdStart
    .org 0x14
    .word 0xF3F9BDA9
    .org 0x20
MODE_ColdStart:
    /* Sample the sticky cause BEFORE protected writes. Arm reset-only WTR
     * before either branch, without a stack, gp or initialized SRAM. A warm
     * handoff bypasses this cause test and arms at the bank reset entry. */
    li t0, 0x40001000
    lbu t2, 0x44(t0)
    andi t2, t2, 7
    sb zero, 0x43(t0)      /* Fsys/131072, full 256-count overflow window. */
    li t1, 2              /* RB_WDOG_RST_EN; watchdog IRQ stays disabled. */
    li t3, 0x57
    li t4, 0xa8
    fence.i
    sb t3, 0x40(t0)
    sb t4, 0x40(t0)
    fence.i
    sb t1, 0x46(t0)
    sb zero, 0x40(t0)
    fence.i
    beq t2, t1, MODE_WatchdogDfu
    li t0, 0x20000090
    li t1, 0x200000d0
1:
    sw zero, 0(t0)
    addi t0, t0, 4
    bltu t0, t1, 1b
    /* HOST bank owns the temporary parallel scan; controller role is NONE. */
    li t0, 0x10000
    jr t0

MODE_WatchdogDfu:
    /* dfu_enter expects a writable absolute-vector table for its ECALL
     * privilege bridge. No old application RAM or stack is used after reset. */
    li sp, 0x20003000       /* ECALL hardware context needs a valid cold stack. */
    li a0, 0x20000000
    ori t0, a0, 3
    csrw mtvec, t0
    li t0, USB_DFU_POINTER_SLOT
    lw t0, 0(t0)
    jr t0
