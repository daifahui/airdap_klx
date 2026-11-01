/* One foreground clock owner, complete DAP command fences and real time. */
#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "bsp/board.h"
#include "bsp/timebase.h"
#include "ch572/clock_transition.h"

#if APP_HAS_DAP
#include "dap/swj.h"
#endif

#if (APP_HAS_DAP && !APP_HAS_RF)
#define __BSP_DAP_CODE __HIGH_CODE
#else
#define __BSP_DAP_CODE
#endif

#define CLOCK_DAP     1
#define CLOCK_BUSY    2
#define CLOCK_PENDING 4
#define CLOCK_CHANGED 8

#if (APP_HAS_RF_UART || RF_DAP_TARGET)
/* This profile reads microseconds only in foreground configuration fences.
 * Its RF ISR timestamps are compiled out; keep the tick's RAM path intact. */
#define __CLOCK_READ_CODE
#else
#define __CLOCK_READ_CODE __HIGH_CODE
#endif

volatile uint32_t BSP_clockHz = CPU_CLOCK;
volatile uint32_t BSP_clockEpoch;
volatile uint32_t BSP_clockUncertaintyUnits;
static timebase_t CLOCK_timebase;

/* board_time_init publishes these before enabling the tick IRQ. Keeping
 * them zero-initialized avoids pushing RF's aligned boot/queue bank. */
static uint32_t         CLOCK_cyclesPerMs;
static uint32_t         CLOCK_cycleUnits;
static volatile uint8_t CLOCK_clockGate;

/* Only foreground changes DAP ownership. An ISR may post CLOCK_PENDING, but
 * never changes HCLK; it cannot overwrite this separate ownership byte. */
static volatile bool DAP_active;

static __HIGH_CODE void CLOCK_UpdateTime(uint32_t counter) {
    uint32_t delta = counter - CLOCK_timebase.counter;
    uint32_t whole = delta / CLOCK_cyclesPerMs;
    uint32_t phase = CLOCK_timebase.phase + (delta - whole * CLOCK_cyclesPerMs) * CLOCK_cycleUnits;

    if (phase >= TIMEBASE_UNITS_PER_MS) {
        phase -= TIMEBASE_UNITS_PER_MS;
        ++whole;
    }
    CLOCK_timebase.counter  = counter;
    CLOCK_timebase.phase    = phase;
    BSP_ms                 += whole;
}

static __HIGH_CODE void CLOCK_ScheduleTick(void) {
    uint32_t irq = BSP_IrqSave();
    uint32_t now = SysTick->CNTL;

    CLOCK_UpdateTime(now);
    uint32_t cycles =
        (TIMEBASE_UNITS_PER_MS - CLOCK_timebase.phase + CLOCK_cycleUnits - 1U) / CLOCK_cycleUnits;

    /* Leave enough time to finish the register writes even near a boundary.
     * board_ms follows elapsed cycles, not the number of these interrupts. */
    if (cycles < 64U) {
        cycles = 64U;
    }
    uint32_t next = now + cycles;

    if (next < now) {
        next = UINT32_MAX; /* Service the 32-bit counter rollover once. */
    }
    SysTick->CMP = next;
    SysTick->SR  = 0;

    /* If a delayed write missed the comparison, request a bounded catch-up
     * ISR rather than waiting an entire counter revolution. */
    uint32_t after = SysTick->CNTL;

    if ((uint32_t)(after - now) >= (uint32_t)(next - now)) {
        PFIC_SetPendingIRQ(SysTick_IRQn);
    }
    BSP_IrqRestore(irq);
}

void BSP_TimeInit(void) {
    SysTick->CTLR     = 0;
    SysTick->CNT      = 0;
    CLOCK_timebase    = (timebase_t){0, 0};
    CLOCK_cyclesPerMs = BSP_clockHz / 1000U;
    CLOCK_cycleUnits  = TIMEBASE_UNITS_PER_SECOND / BSP_clockHz;
    BSP_ms            = 0;
    SysTick->CMP      = BSP_clockHz / 1000U;
    SysTick->SR       = 0;
    PFIC_ClearPendingIRQ(SysTick_IRQn);
    PFIC_EnableIRQ(SysTick_IRQn);
    SysTick->CTLR = SysTick_CTLR_STCLK | SysTick_CTLR_STIE | SysTick_CTLR_STE;
}

void BSP_TimeSuspend(void) {
    CLOCK_UpdateTime(SysTick->CNTL);
    SysTick->CTLR = 0;
    SysTick->SR   = 0;
    PFIC_ClearPendingIRQ(SysTick_IRQn);
}

void BSP_TimeResume(void) {
    /* HCLK has been restored before this call. Do not interpret cycles
     * spent in the HSI/wake/PLL transition using the old HCLK divisor. */
    SysTick->CNT           = 0;
    CLOCK_timebase.counter = 0;
    SysTick->CMP =
        (TIMEBASE_UNITS_PER_MS - CLOCK_timebase.phase + CLOCK_cycleUnits - 1U) / CLOCK_cycleUnits;
    SysTick->SR = 0;
    PFIC_ClearPendingIRQ(SysTick_IRQn);
    SysTick->CTLR = SysTick_CTLR_STCLK | SysTick_CTLR_STIE | SysTick_CTLR_STE;
}

void BSP_TimeTick(void) __ALIAS("CLOCK_ScheduleTick");

__CLOCK_READ_CODE uint32_t BSP_TimeUs(void) {
    uint32_t irq = BSP_IrqSave();

    CLOCK_UpdateTime(SysTick->CNTL);
    uint32_t value = BSP_ms * 1000U + CLOCK_timebase.phase / TIMEBASE_UNITS_PER_US;

    BSP_IrqRestore(irq);
    return value;
}

#if DAP_TIMESTAMP_ENABLE
uint32_t BSP_TimeStamp(void) {
    uint32_t ms, counter, phase, now;

    /* Tick IRQ is the only concurrent writer. A complete snapshot is retried
     * if it ran during the reads; a DAP command already holds the HCLK gate. */
    do {
        ms      = BSP_ms;
        counter = CLOCK_timebase.counter;
        phase   = CLOCK_timebase.phase;
        now     = SysTick->CNTL;
        __asm__ volatile("" ::: "memory");
    } while (ms != BSP_ms || counter != CLOCK_timebase.counter);
    uint64_t units = phase + (uint64_t)(uint32_t)(now - counter) * CLOCK_cycleUnits;

    return ms * 1000U + (uint32_t)(units / TIMEBASE_UNITS_PER_US);
}
#endif

void BSP_ClockQueue(bool pending) {
    uint32_t irq = BSP_IrqSave();

    if (pending) {
        CLOCK_clockGate |= CLOCK_PENDING;
    } else {
        CLOCK_clockGate &= ~CLOCK_PENDING;
    }
    BSP_IrqRestore(irq);
}

bool BSP_ClockAcquire(void) {
    uint32_t irq   = BSP_IrqSave();
    bool     ready = !DAP_active && !(CLOCK_clockGate & CLOCK_BUSY) && !BSP_stopRequested;

    if (ready) {
        CLOCK_clockGate |= CLOCK_BUSY;
    }
    BSP_IrqRestore(irq);
    return ready;
}

void BSP_ClockRelease(void) {
    uint32_t irq = BSP_IrqSave();

    if (CLOCK_clockGate & CLOCK_CHANGED) {
        ++BSP_clockEpoch;
    }
    CLOCK_clockGate &= ~(CLOCK_BUSY | CLOCK_CHANGED | CLOCK_PENDING);
    BSP_IrqRestore(irq);
}

__BSP_DAP_CODE bool BSP_DapEnter(void) {
    bool ready = !DAP_active && !CLOCK_clockGate && !BSP_stopRequested;

    if (ready) {
        DAP_active = true;
    }
    return ready;
}

__BSP_DAP_CODE void BSP_DapLeave(void) {
    DAP_active = false;
}

uint8_t BSP_ClockState(void) {
    return CLOCK_clockGate | (DAP_active ? CLOCK_DAP : 0);
}

/* The SDK has returned to a verified legal stable register configuration
 * before this foreground Flash code does any arithmetic. Its SAM-window
 * leaf only brackets writes and records to the owner's stack in SRAM. */
static bool CLOCK_SetAccounted(uint8_t cfg) {
    clock_trace_t trace;

    trace.count     = 0;
    uint8_t current = R8_CLK_SYS_CFG;

    CLOCK_sdkTrace = &trace;
    CLK_SetSysClock((SYS_CLKTypeDef)cfg);
    CLOCK_sdkTrace  = 0;
    uint8_t  actual = R8_CLK_SYS_CFG;
    uint32_t after  = SysTick->CNTL;

    if (!trace.count || trace.count > CLOCK_TRACE_CAPACITY || !CLOCK_CfgValid(actual) ||
        (actual & 0xc0U) == 0xc0U || trace.samples[trace.count - 1U].cfg != actual) {
        BSP_Fault();
    }
    uint32_t ms = BSP_ms;

    for (uint32_t i = 0; i < trace.count; ++i) {
        clock_sample_t *sample = &trace.samples[i];

        if (!CLOCK_CfgValid((uint8_t)sample->cfg) || (sample->cfg & 0xc0U) == 0xc0U) {
            BSP_Fault();
        }
        TIME_AdvanceCfg(&CLOCK_timebase, &ms, sample->before, current);
        BSP_clockUncertaintyUnits = TIME_BoundaryBudget(
            BSP_clockUncertaintyUnits, sample->after - sample->before, current, sample->cfg);

        /* Only the bracket around the write uses the old-period convention;
         * the whole intermediate HSE/N span uses its exact rational period. */
        TIME_AdvanceCfg(&CLOCK_timebase, &ms, sample->after, current);
        current = sample->cfg;
    }
    TIME_AdvanceCfg(&CLOCK_timebase, &ms, after, current);
    BSP_ms = ms;
    return actual == cfg;
}

/* Only the acquired foreground owner calls this. UART/RF are quiesced by
 * that owner; the SDK's HCLK/Flash register sequence itself resides in RAM.
 * Backend copying follows with IRQs enabled but no new DAP admission. */
static uint8_t CLOCK_Config(uint32_t hz) {
    switch (hz) {
        case 100000000U:
            return 0x46;
        case 75000000U:
            return 0x48;
        case 60000000U:
            return 0x4A;
        case 40000000U:
            return 0x4F;
        case 24000000U:
            return 0x59;
        case 16000000U:
            return 0x02;
        default:
            return 0xff;
    }
}

static bool CLOCK_Apply(uint32_t hz, bool restore) {
    uint8_t cfg = CLOCK_Config(hz);

    if (cfg == 0xff) {
        return false;
    }
    uint32_t irq = BSP_IrqSave();

    if (!(CLOCK_clockGate & CLOCK_BUSY) || DAP_active || (BSP_stopRequested && !restore)) {
        BSP_IrqRestore(irq);
        return false;
    }
    if (hz == BSP_clockHz) {
        BSP_IrqRestore(irq);
        return true;
    }
    uint32_t oldHz  = BSP_clockHz;
    uint8_t  oldCfg = CLOCK_Config(oldHz);

    /* Exactly the six runtime rates are admitted above. Sampling must never
     * cover the SDK's oscillator-start/LSI path (more than two CLK writes),
     * and an uninitialized timebase must never be advanced. */
    if (oldCfg == 0xff || !CLOCK_cycleUnits || CLOCK_sdkTrace || R8_CLK_SYS_CFG != oldCfg ||
        (R8_HFCK_PWR_CTRL & (RB_CLK_XT32M_PON | RB_CLK_PLL_PON)) !=
            (RB_CLK_XT32M_PON | RB_CLK_PLL_PON)) {
        BSP_IrqRestore(irq);
        return false;
    }
    uint32_t before = SysTick->CNTL;

    CLOCK_UpdateTime(before);
    bool applied = CLOCK_SetAccounted(cfg);

    if (!applied) {
        if (!CLOCK_SetAccounted(oldCfg)) {
            BSP_Fault();
        }
        CLOCK_UpdateTime(SysTick->CNTL);
        CLOCK_ScheduleTick();
        BSP_IrqRestore(irq);
        return false;
    }
    BSP_clockHz        = hz;
    CLOCK_cyclesPerMs  = hz / 1000U;
    CLOCK_cycleUnits   = TIMEBASE_UNITS_PER_SECOND / hz;
    CLOCK_clockGate   |= CLOCK_CHANGED;
    CLOCK_ScheduleTick();
    BSP_IrqRestore(irq);

#if APP_HAS_DAP
    if (BSP_LocalSelected()) {
        SWJ_Clock(SWJ_requestedHz);
    }
#endif

    return true;
}

bool BSP_ClockApply(uint32_t hz) {
    return CLOCK_Apply(hz, false);
}

bool BSP_ClockRestore(uint32_t hz) {
    return CLOCK_Apply(hz, true);
}
