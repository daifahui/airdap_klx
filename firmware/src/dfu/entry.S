/* Application takeover and cold watchdog recovery share this Flash entry.
 * The caller supplies a writable SRAM vector table in a0. No old stack or
 * global state is used after the privilege bridge and peripheral shutdown. */
    .macro SAM_OPEN
    sb t3,0x40(t0)
    sb t4,0x40(t0)
    .endm
    .macro QUIESCE_OLD
    li t0,0x40001000
    li t3,0x57
    li t4,0xa8
    /* First peripheral action after minimum IRQ/privilege handoff:
     * make USB registers accessible, then disconnect BEFORE other cleanup. */
    lbu t1,0xd(t0)
    andi t1,t1,-17
    SAM_OPEN
    sb t1,0xd(t0)
    sb zero,0x40(t0)
    li t1,0x40008000
    sw zero,0(t1)   /* pullup/device connection/DMA off, address/IRQ cleared */
    lhu t1,0x18(t0)
    li t2,-4097
    and t1,t1,t2
    sh t1,0x18(t0)  /* remove forced DP pullup even if old sleep mode set it */
    SAM_OPEN
    sb zero,0x46(t0) /* watchdog reset/IRQ off; KEEP0x47 untouched */
    sb zero,0xc(t0)  /* ungate UART/CMP/TMR so disable writes are effective */
    sb zero,0x40(t0)
    li t1,0x80
    SAM_OPEN
    sb t1,0xd(t0)   /* BLE clock off; USB/I2C/PWM/SPI clocks on for quiesce */
    sb zero,0x1d(t0) /* Flash HCLK/XROM clocks on */
    sb zero,0x40(t0)
    lbu t1,0x24(t0)
    andi t1,t1,-13   /* battery IRQ/monitor off; retain LDO power bit */
    SAM_OPEN
    sb t1,0x24(t0)
    sb zero,0xe(t0)  /* no old wake sources */
    sb zero,0x40(t0)
    li t1,0xe000f000
    sw zero,0(t1)   /* SysTick off before SRAM copy */
    li t1,0x40002400
    sw zero,0(t1)   /* timer mode + DMA + IRQ off */
    sh zero,0x20(t1) /* encoder + IRQ off */
    li t1,0x40004000
    sw zero,0(t1)   /* SPI output/DMA/IRQ off */
    sw zero,8(t1)   /* extra SPI FIFO IRQ enables off */
    li t1,0x40005000
    sw zero,0(t1)   /* all PWM outputs/config/DMA off */
    sb zero,0xc(t1)
    li t1,0x40003400
    sw zero,0(t1)   /* UART TX pin enable, BREAK, FIFO, IRQ off */
    li t1,0x40004800
    li t2,0x8000
    sh t2,0(t1)    /* I2C software reset, then disabled */
    sh zero,0(t1)
    sh zero,4(t1)
    sh zero,0x64(t0) /* keyscan pins/start off */
    sb zero,0x66(t0)
    sh zero,0x54(t0) /* comparator enable/output-capture/IRQ off */
    sw zero,0x90(t0) /* GPIO interrupt enables off */
    sw zero,0xa0(t0) /* ALL GPIO floating inputs, not just USB PA0/1 */
    sw zero,0xa8(t0)
    sw zero,0xb0(t0)
    sw zero,0xb4(t0)
    sw zero,0x18(t0) /* USB analog/forced pullup/debug/25MHz/remaps off */
    li t1,-1
    sw t1,0x9c(t0) /* GPIO pending W1C */
    li t1,0xe000e280
    li t2,-1
    sw t2,0(t1)
    sw t2,4(t1)    /* clear PFIC pending after sources stopped */
    fence iorw,iorw
    .endm

    .section .dfu_entry,"ax",@progbits
    .option norelax
    .balign 4
    .global dfu_enter
    .type dfu_enter,@function
dfu_enter:
    li t0,0x88
    csrrc zero,0x800,t0
    li t0,0xe000e180
    li t1,-1
    sw t1,0(t0)
    sw t1,4(t0)
    andi t0,a0,3
    bnez t0,.Lentry_bad
    li t0,0x20000000
    bltu a0,t0,.Lentry_bad
    li t0,0x20002fdc
    bltu t0,a0,.Lentry_bad
    la t0,dfu_machine_trap
    sw t0,20(a0)
    sw t0,32(a0)
    fence.i
    ecall
.Lentry_bad:
    j .Lentry_bad
    .balign 4
dfu_machine_trap:
    li t0,0x20
    csrs 0x804,t0
    la t0,dfu_machine_start
    csrw mepc,t0
    li t0,0x1800
    csrw mstatus,t0
    mret
dfu_machine_start:
    csrw 0x804,zero
    QUIESCE_OLD
    la t0,__ram_load
    la t1,__ram_start
    la t2,__ram_initialized_end
1:  bgeu t1,t2,2f
    lw a1,0(t0)
    sw a1,0(t1)
    addi t0,t0,4
    addi t1,t1,4
    j 1b
2:  la t2,__bss_end
3:  bgeu t1,t2,4f
    sw zero,0(t1)
    addi t1,t1,4
    j 3b
4:
    .option push
    .option norelax
    la gp,__global_pointer$
    .option pop
    li sp,0x20003000
    la t0,dfu_trap
    csrw mtvec,t0
    li t0,0x25
    csrw 0xbc0,t0
    fence.i
    tail DFU_Main
    .size dfu_enter,.-dfu_enter

    .section .text.dfu_trap,"ax",@progbits
    .balign 4
    .global dfu_trap
dfu_trap:
    addi sp,sp,-16
    sw t0,0(sp)
    sw t1,4(sp)
    sw t2,8(sp)
    csrr t0,mcause
    li t1,5 /* genuine CPU Flash read fault -> upload failure, not fake ff */
    bne t0,t1,9f
    la t0,DFU_readGuard
    lw t1,0(t0)
    beqz t1,9f
    la t0,DFU_readFault
    li t1,1
    sw t1,0(t0)
    csrr t0,mepc
    lhu t1,0(t0)
    andi t1,t1,3
    li t2,3
    beq t1,t2,6f
    addi t0,t0,2
    j 7f
6:  addi t0,t0,4
7:  csrw mepc,t0
    lw t0,0(sp)
    lw t1,4(sp)
    lw t2,8(sp)
    addi sp,sp,16
    mret
9:  j 9b /* unexpected fault remains in DFU; never fall into old application */
