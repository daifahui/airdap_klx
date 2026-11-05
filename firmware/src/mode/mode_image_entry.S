#include "app_config.h"
/* A synchronous exception leaves user mode. Return once to a Flash machine
 * continuation, clearing the old exception/automatic-save context BEFORE
 * discarding its stack. The original role startup can then install gp/sp,
 * vector, RAM and return to user main without a peripheral/system reset. */
    .section .text.MODE_ImageTrap,"ax",@progbits
    .align 2
    .global MODE_ImageTrap
    .type MODE_ImageTrap,@function
MODE_ImageTrap:
    ecall
1:  j 1b
    .size MODE_ImageTrap, .-MODE_ImageTrap

    .section .text.Ecall_U_Mode_Handler,"ax",@progbits
    .align 2
    .global Ecall_U_Mode_Handler
    .type Ecall_U_Mode_Handler,@function
Ecall_U_Mode_Handler:
    li t0, 0x20000090
    lw t1, 0(t0)
    li t2, 0x324b424d
    bne t1, t2, 9f
    lw t1, 4(t0)

#if BSP_USB_DONGLE
    li t2, 1
    bne t1, t2, 9f
#else
    li t2, 2
    bltu t2, t1, 9f
#endif

    la t0, MODE_MachineStart
    csrw mepc, t0
    li t0, 0x1800              /* MPP=M; MPIE/MIE=0 during RAM replacement. */
    csrw mstatus, t0
    mret
9:  j 9b                      /* Unexpected ECALL: preserve the fault context. */
    .size Ecall_U_Mode_Handler, .-Ecall_U_Mode_Handler

    .section .text.MODE_MachineStart,"ax",@progbits
    .align 2
    .type MODE_MachineStart,@function
MODE_MachineStart:

#if !BSP_USB_DONGLE
    li t0, 0x20000090
    lw t1, 4(t0)
    beqz t1, 1f
    li t2, 1
    beq t1, t2, 2f
    li t0, FLASH_TARGET_BASE
    jr t0
1:  li t0, FLASH_LOCAL_BASE
    jr t0
2:  li t0, FLASH_HOST_BASE
#else
    li t0, FLASH_HOST_BASE
#endif

    jr t0
    .size MODE_MachineStart, .-MODE_MachineStart
