/* SPDX-License-Identifier: Apache-2.0 */
#include "app_config.h"
#include "dap/gpio_timing.h"
/* SRAM bit engines, caller-saved registers only, no compressed instructions.
 * Dispatch once per batch: TX, RX, JTAG full duplex, or clocks only.
 * Phases are balanced using measured QingKe/MMIO cycles, not instruction count.
 * Slow delays check cancellation on every iteration; fast batches contain
 * at most 32 bits and check cancellation at entry. An ISR can release the
 * target immediately; these loops never change GPIO direction or ownership.
 * At 100 MHz the TX/RX/JTAG/clock/ACK half-periods are 5/6/8/4/6
 * cycles. Each delay iteration adds six cycles; see docs/dap_timing.md.
 * Final returns and batch/protocol boundaries can stretch HIGH. */
    .section .highcode.GPIO_Shift,"ax",@progbits
    .option push
    .option norvc
    .balign 4

/* t4 is scratch here: input capture follows the low delay, and has already
 * been accumulated before the high delay. a6 is strictly positive. */
    .macro phase_delay enabled
    .if \enabled
    mv a7, a6               /* The limit was loaded once at batch entry. */
.Ldelay\@:
    lbu t4, 0(t6)           /* Cancellation flag, not the counter/limit. */
    bnez t4, gpio_shift_return
    addi a7, a7, -1
    bnez a7, .Ldelay\@
    .endif
    .endm

    .macro low_padding
    .rept GPIO_LOW_EXTRA_NOPS
    nop
    .endr
    .endm
    .macro high_padding
    .rept GPIO_HIGH_EXTRA_NOPS
    nop
    .endr
    .endm

/* The taken bit-loop branch costs an extra cycle. TX needs five LOW and four
 * HIGH instructions to obtain five measured cycles in each phase. */
    .macro tx_loop name, delayed
\name:
    sw t1, 0(t0)             /* CLK falls and data clears in one atomic write. */
    sw t3, 12(t0)
    srli a0, a0, 1
    andi t3, a0, 1
    neg t3, t3
    phase_delay \delayed
    low_padding
    sw t2, 12(t0)
    and t3, t3, a2
    phase_delay \delayed
    addi a1, a1, -1
    high_padding
    bnez a1, \name
    j gpio_shift_return
    .endm

    .macro rx_loop name, delayed
\name:
    sw t2, 0(t0)
    phase_delay \delayed
    srli a4, a4, 1           /* Let the target settle before reading PA_PIN. */
    lw t4, -8(t0)
    and t4, t4, a3
    sll t4, t4, a5           /* Place the sampled pin at bit 31. */
    low_padding
    sw t2, 12(t0)
    or a4, a4, t4
    phase_delay \delayed
    addi a1, a1, -1
    nop
    high_padding
    bnez a1, \name
    j gpio_shift_return
    .endm

    .macro jtag_loop name, delayed
\name:
    sw t1, 0(t0)
    sw t3, 12(t0)
    srli a0, a0, 1
    andi t3, a0, 1
    neg t3, t3
    phase_delay \delayed
    lw t4, -8(t0)
    and t4, t4, a3
    low_padding
    sw t2, 12(t0)
    and t3, t3, a2
    srli a4, a4, 1
    sll t4, t4, a5
    or a4, a4, t4
    phase_delay \delayed
    addi a1, a1, -1
    high_padding
    bnez a1, \name
    j gpio_shift_return
    .endm

    .macro clock_loop name, delayed, fallthrough=0
\name:
    sw t2, 0(t0)
    phase_delay \delayed
    nop
    nop
    nop
    low_padding
    sw t2, 12(t0)
    phase_delay \delayed
    addi a1, a1, -1
    high_padding
    bnez a1, \name
    .if !\fallthrough
    j gpio_shift_return
    .endif
    .endm

    .global GPIO_Shift
    .type GPIO_Shift,@function
GPIO_Shift:
    beqz a1, gpio_shift_zero
    la t6, BSP_stopRequested
    lbu t4, 0(t6)
    bnez t4, gpio_shift_zero
    li t0, 0x400010ac
    sb zero, -105(t0)       /* WTR feed at completed-batch admission only. */
    li t2, 32
    li a4, 0
    li t5, 32
    sub t5, t5, a1           /* Align the returned LSB-first partial word once. */
    la a6, SWJ_delayLoops
    lw t4, 0(a6)
    lui a7, 1
    bltu t4, a7, 1f
    tail GPIO_ShiftSlow      /* slow half-periods use bounded C delays */
1:
    beqz a2, gpio_shift_no_drive
    or t1, a2, t2
    andi t3, a0, 1
    neg t3, t3
    and t3, t3, a2
    beqz a3, gpio_shift_tx
    lw a6, 4(a6)
    j gpio_shift_sample

gpio_shift_no_drive:
    beqz a3, gpio_shift_clock
    li t4, 3
    bne a1, t4, gpio_shift_rx_normal
    li t4, 128
    bne a3, t4, gpio_shift_rx_normal
    lw a6, 12(a6)
    li a5, 24
    beqz a6, gpio_ack_fast
    j gpio_ack_slow
gpio_shift_rx_normal:
    lw a6, 0(a6)
gpio_shift_sample:
    li a5, 24               /* SWDIO is PA7. */
    li t4, 128
    beq a3, t4, gpio_shift_sample_ready
    li a5, (31 - BSP_TDO_GPIO)
gpio_shift_sample_ready:
    beqz a2, gpio_shift_rx
    beqz a6, gpio_jtag_fast_loop
    j gpio_jtag_slow_loop
gpio_shift_rx:
    beqz a6, gpio_rx_fast_loop
    j gpio_rx_slow_loop
gpio_shift_tx:
    lw a6, 0(a6)
    beqz a6, gpio_tx_fast_loop
    j gpio_tx_slow_loop
gpio_shift_clock:
    lw a6, 8(a6)
    beqz a6, gpio_clock_fast_loop
    j gpio_clock_slow_loop

/* General data and JTAG paths stay compact. Fast batches read neither SRAM
 * nor their delay parameters inside the bit loop; count/branch use registers. */
    tx_loop gpio_tx_fast_loop, 0
    rx_loop gpio_rx_fast_loop, 0
    jtag_loop gpio_jtag_fast_loop, 0
    clock_loop gpio_clock_fast_loop, 0

/* Only the fixed 3-bit ACK is unrolled. Request headers use the ordinary TX
 * loop, including its balanced counter/branch budget and cached delay limit. */
    .macro ack name, delayed
\name:
    .rept 3
    sw t2, 0(t0)
    srli a4, a4, 1           /* Do independent work before the input load. */
    phase_delay \delayed
    lw t4, -8(t0)
    and t4, t4, a3
    sll t4, t4, a5
    low_padding
    sw t2, 12(t0)
    or a4, a4, t4
    nop
    nop
    nop
    nop
    phase_delay \delayed
    high_padding
    .endr
    j gpio_shift_return
    .endm

    ack gpio_ack_fast, 0
    ack gpio_ack_slow, 1

    tx_loop gpio_tx_slow_loop, 1
    rx_loop gpio_rx_slow_loop, 1
    jtag_loop gpio_jtag_slow_loop, 1
    /* Last engine falls straight into return, saving a redundant jump. */
    clock_loop gpio_clock_slow_loop, 1, 1

gpio_shift_return:
    srl a0, a4, t5
    ret
gpio_shift_zero:
    li a0, 0
    ret
    .size GPIO_Shift,.-GPIO_Shift
    .option pop
