/* Modified for AirDAP; see NOTICE for provenance and retained usage notices. */
/********************************** (C) COPYRIGHT *******************************
 * File Name          : clock.c (adapted CH572 clock implementation)
 * Author             : WCH
 * Description        : source file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "ch572/ch572.h"
#include "ch572/attributes.h"
#include "ch572/clock_transition.h"

clock_trace_t *volatile CLOCK_sdkTrace;

/* SAM-window leaf: no division, tables, callbacks or Flash dependencies. */
static __ALWAYS_INLINE void CLOCK_WriteConfig(uint8_t cfg) {
    clock_trace_t *trace  = CLOCK_sdkTrace;
    uint32_t       before = trace ? SysTick->CNTL : 0;
    R8_CLK_SYS_CFG        = cfg;
    if (trace) {
        uint32_t after = SysTick->CNTL;
        __asm__ volatile("" ::: "memory");
        uint32_t n = trace->count;
        if (n < CLOCK_TRACE_CAPACITY) {
            trace->samples[n].before = before;
            trace->samples[n].after  = after;
            trace->samples[n].cfg    = R8_CLK_SYS_CFG;
            trace->count             = n + 1;
        } else {
            trace->count = CLOCK_TRACE_CAPACITY + 1;
        }
    }
}

/* Share the original SAM windows without duplicating their entry/exit
 * instruction sequences in scarce RAM. Operations retain the SDK's exact
 * register order; every window still uses both original safe-access macros. */
#define CLOCK_OP_POWER        0U
#define CLOCK_OP_WRITE        1U
#define CLOCK_OP_RESTORE_TUNE 2U
#define CLOCK_OP_PLL          3U
#define CLOCK_OP_FLASH        4U
#define CLOCK_OP_FINAL        5U

static __HIGH_CODE __NO_INLINE __OPTIMIZE("no-jump-tables") void CLOCK_SafeOperation(uint8_t op,
                                                                                     uint8_t value,
                                                                                     uint8_t tune) {
    CH572_SAFE_ACCESS_ENABLE();
    if (op == CLOCK_OP_POWER) {
        R8_XT32M_TUNE    |= 0x03;
        R8_HFCK_PWR_CTRL |= RB_CLK_XT32M_PON;
    } else if (op == CLOCK_OP_PLL) {
        R8_HFCK_PWR_CTRL |= RB_CLK_PLL_PON;
        R8_FLASH_CFG      = 0x01;
        R8_FLASH_SCK     |= 1 << 4; // SDK 50M setting, unchanged
    } else if (op == CLOCK_OP_FLASH) {
        R8_FLASH_CFG = value;
    } else {
        if (op == CLOCK_OP_FINAL) {
            R8_SLP_POWER_CTRL |= 0x40;
        }
        CLOCK_WriteConfig(value);
        if (op == CLOCK_OP_RESTORE_TUNE) {
            R8_XT32M_TUNE = tune;
        }
    }
    CH572_SAFE_ACCESS_DISABLE();
}

/*********************************************************************
 * @fn      CLK_SetSysClock
 * @brief   Original SDK clock/Flash strategy, with source and DIV writes
 *          separated when crossing PLL/HSE. LSI startup behavior is kept.
 */
__HIGH_CODE void CLK_SetSysClock(SYS_CLKTypeDef sc) {
    uint32_t i; /* Nine NOPs; native word counter avoids byte truncation. */
    if (!CLOCK_CfgValid((uint8_t)sc)) {
        return;
    }
    if (sc == RB_CLK_SYS_MOD) {
        CLOCK_SafeOperation(CLOCK_OP_FINAL, R8_CLK_SYS_CFG | RB_CLK_SYS_MOD, 0);
    } else {
        if (!(R8_HFCK_PWR_CTRL & RB_CLK_XT32M_PON)) {
            uint8_t savedTune = R8_XT32M_TUNE;
            CLOCK_SafeOperation(CLOCK_OP_POWER, 0, 0);
            uint8_t savedConfig = R8_CLK_SYS_CFG;
            CLOCK_SafeOperation(CLOCK_OP_WRITE, savedConfig | 0xC0, 0);
            for (i = 0; i < 9; i++) {
                __nop();
            }
            CLOCK_SafeOperation(CLOCK_OP_RESTORE_TUNE, savedConfig, savedTune);
        }

        /* This stage must follow the original HSE-ready sequence above. */
        uint8_t oldConfig = R8_CLK_SYS_CFG;
        uint8_t bridge    = CLOCK_CfgBridge(oldConfig, (uint8_t)sc);
        if (bridge != oldConfig) {
            CLOCK_SafeOperation(CLOCK_OP_WRITE, bridge, 0);
            for (i = 0; i < 9; i++) {
                __nop();
            }
        }
        if ((sc & RB_CLK_SYS_MOD) == 0x40) {
            CLOCK_SafeOperation(CLOCK_OP_PLL, 0, 0);
        } else {
            CLOCK_SafeOperation(CLOCK_OP_FLASH, (sc & 0x1F) ? 0x02 : 0x07, 0);
        }
        CLOCK_SafeOperation(CLOCK_OP_FINAL, (uint8_t)sc, 0);
    }
}

void CLK_SetHseCapacitance(HSECapTypeDef c) {
    uint8_t savedTune;

    savedTune = R8_XT32M_TUNE;
    savedTune = (savedTune & 0x0f) | (c << 4);

    CH572_SAFE_ACCESS_ENABLE();
    R8_XT32M_TUNE = savedTune;
    CH572_SAFE_ACCESS_DISABLE();
}
