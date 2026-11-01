#include "dfu/dfu.h"
#include "ch572/ch572sfr.h"
#include "ch572/ch572_clock.h"
#include "ch572/flash_io.h"

#define DFU_TIMER_DISCONNECT_CYCLES 6000000U

#define DFU_TIMER_ALL_FLAGS                                                                        \
    (RB_TMR_IF_FIFO_OV | RB_TMR_IF_DMA_END | RB_TMR_IF_FIFO_HF | RB_TMR_IF_DATA_ACT |              \
     RB_TMR_IF_CYC_END)

#define DFU_HSE_STARTUP_BIAS    3U
#define DFU_HSE_STARTUP_CYCLES  9U
#define DFU_HSE_BRIDGE_DIVISOR  10U
#define DFU_PLL_SETTLE_CYCLES   40000U
#define DFU_STATUS_ACK_CYCLES   120000U
#define DFU_FLASH_60MHZ_CONFIG  1U
#define DFU_FLASH_CLOCK_CONFIG  0x10U
#define DFU_RAM_RETENTION_MODE1 0x40U
#define DFU_USB_ENDPOINT_NAK    ((uint32_t)(UEP_R_RES_NAK | UEP_T_RES_NAK) << 16)

static void DFU_DelayCycles(uint32_t n) {
    while (n--)
        __asm__ volatile("nop");
}

static void DFU_Unlock(void) {
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2;
}

/* TMR counts Fsys cycles. Used only during entry/exit, never as a USB/DFU
 * scheduler. All timer IRQs stay off; SysTick stays stopped throughout DFU. */
static void DFU_DisconnectWait(void) {
    DFU_Unlock();
    R8_SLP_CLK_OFF0    &= ~RB_SLP_CLK_TMR;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG0;
    R8_TMR_CTRL_MOD     = RB_TMR_ALL_CLEAR;
    R8_TMR_CTRL_DMA     = 0;
    R8_TMR_INTER_EN     = 0;
    R32_TMR_CNT_END     = DFU_TIMER_DISCONNECT_CYCLES; /* 100ms at the established60MHz */
    R8_TMR_INT_FLAG     = DFU_TIMER_ALL_FLAGS;
    R8_TMR_CTRL_MOD     = RB_TMR_COUNT_EN;
    while (!(R8_TMR_INT_FLAG & RB_TMR_IF_CYC_END)) {
    }
    R8_TMR_CTRL_MOD = 0;
    R8_TMR_INT_FLAG = DFU_TIMER_ALL_FLAGS;
    DFU_Unlock();
    R8_SLP_CLK_OFF0    |= RB_SLP_CLK_TMR;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG0;
}

static void DFU_ClockInit(void) {
    /* With HSE off, a legally executing CPU is on LSI. Nine slow periods
     * replace a frequency-dependent long delay; then select a known HSE
     * divider before enabling PLL. Never retain LSI's source bits as the
     * supposed PLL bridge. All protected windows contain at most2 writes. */
    if (!(R8_HFCK_PWR_CTRL & RB_CLK_XT32M_PON)) {
        uint8_t tune = R8_XT32M_TUNE;

        DFU_Unlock();
        R8_XT32M_TUNE      |= DFU_HSE_STARTUP_BIAS;
        R8_HFCK_PWR_CTRL   |= RB_CLK_XT32M_PON;
        R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG0;
        DFU_DelayCycles(DFU_HSE_STARTUP_CYCLES);
        DFU_Unlock();
        R8_XT32M_TUNE      = tune;
        R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0;
    }
    uint8_t div = R8_CLK_SYS_CFG & RB_CLK_SYS_MOD; /* old source, DIV32; never stale DIV1 */
    DFU_Unlock();
    R8_CLK_SYS_CFG     = div;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0;
    DFU_Unlock();
    R8_CLK_SYS_CFG     = DFU_HSE_BRIDGE_DIVISOR;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0;
    DFU_Unlock();
    R8_HFCK_PWR_CTRL   |= RB_CLK_PLL_PON;
    R8_FLASH_CFG        = DFU_FLASH_60MHZ_CONFIG;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG0;
    DFU_Unlock();
    R8_FLASH_SCK       = DFU_FLASH_CLOCK_CONFIG;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0;
    DFU_DelayCycles(DFU_PLL_SETTLE_CYCLES); /* now clocked from known HSE/10, even on cold entry */
    DFU_Unlock();
    R8_SLP_POWER_CTRL  = DFU_RAM_RETENTION_MODE1;
    R8_CLK_SYS_CFG     = CLK_SOURCE_HSE_PLL_60MHz;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0;
}

static void DFU_StopPeripherals(void) {
    /* All pin drivers and autonomous writers were stopped BEFORE the copy.
     * Finish resetting dormant event sources, then gate every unused module.
     * Keep crystal/LSI load and board power calibration; no blanket SYS memset. */
    R32_GPIO_INT_MODE = 0;
    R8_RTC_FLAG_CTRL  = RB_RTC_TRIG_CLR | RB_RTC_TMR_CLR;
    DFU_Unlock();
    R8_RTC_MODE_CTRL   = 0;
    R8_OSC_CAL_CTRL    = 0;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0;
    DFU_Unlock();
    R8_LONG_RST_CFG    = 0;
    R16_SLP_WAKE_CFG   = RB_ACAUTO_ENABLE;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0;
    DFU_Unlock();
    R8_SLP_CLK_OFF0 = RB_SLP_CLK_UART | RB_SLP_CLK_CMP | RB_SLP_CLK_TMR;
    R8_SLP_CLK_OFF1 =
        RB_SLP_CLK_BLE | RB_SLP_CLK_I2C | RB_SLP_CLK_PWMX | RB_CLK_OFF_AESCCM | RB_SLP_CLK_SPI;
    R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0;
}

void DFU_Reboot(void) {
    /* Let the final status ACK leave the SIE before disconnect. */
    DFU_DelayCycles(DFU_STATUS_ACK_CYCLES);
    R8_USB_CTRL        = 0;
    R8_UDEV_CTRL       = 0;
    R16_PIN_ALTERNATE &= ~RB_UDP_PU_EN;
    DFU_DisconnectWait(); /* host-visible disconnect100ms, not a NOP estimate */

    /* Reset the Flash chip (66/99) is distinct from MCU reset bit0. */
    DFU_Unlock();
    R8_GLOB_ROM_CFG    |= RB_ROM_CTRL_EN;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG0;
    R8_FLASH_CTRL       = FLASH_CTRL_IDLE;
    R8_FLASH_CTRL       = FLASH_CTRL_COMMAND;
    __asm__ volatile("nop;nop");
    R8_FLASH_DATA = FLASH_CMD_RESET_ENABLE;
    while (R8_FLASH_CTRL & RB_FLASH_CTRL_BUSY) {
    }
    R8_FLASH_CTRL = FLASH_CTRL_IDLE;
    R8_FLASH_CTRL = FLASH_CTRL_COMMAND;
    __asm__ volatile("nop;nop");
    R8_FLASH_DATA = FLASH_CMD_RESET;
    while (R8_FLASH_CTRL & RB_FLASH_CTRL_BUSY) {
    }
    R8_FLASH_CTRL = FLASH_CTRL_IDLE;
    DFU_Unlock();
    R8_GLOB_ROM_CFG    &= RB_ROM_CODE_OFS;
    R8_RST_WDOG_CTRL   |= RB_SOFTWARE_RESET;
    R8_SAFE_ACCESS_SIG  = SAFE_ACCESS_SIG0;
    for (;;) {
    }
}

static void DFU_Connect(void) {
    R8_USB_CTRL        = 0;
    R8_UDEV_CTRL       = 0;
    R16_PIN_ALTERNATE &= ~RB_UDP_PU_EN;

    /* Shared by runtime DETACH and cold WTR entry: a real 100 ms logical
     * unplug before the new DFU descriptor/PID is attached. */
    DFU_DisconnectWait();
    R16_PIN_ALTERNATE = RB_PIN_USB_EN;

    /* Reset SIE/FIFO even if the old application was a USB host or had
     * configured endpoints. No DMA is enabled until the fresh EP0 is ready. */
    R8_USB_CTRL      = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
    R8_USB_CTRL      = 0;
    R32_USB_EP0_CTRL = DFU_USB_ENDPOINT_NAK;
    R16_UEP0_DMA     = 0;
    R32_USB_EP1_CTRL = DFU_USB_ENDPOINT_NAK;
    R16_UEP1_DMA     = 0;
    R32_USB_EP2_CTRL = DFU_USB_ENDPOINT_NAK;
    R16_UEP2_DMA     = 0;
    R32_USB_EP3_CTRL = DFU_USB_ENDPOINT_NAK;
    R16_UEP3_DMA     = 0;
    R32_USB_EP4_CTRL = DFU_USB_ENDPOINT_NAK; /* EP4 shares EP0 DMA. */
    R32_USB_EP5_CTRL = DFU_USB_ENDPOINT_NAK;
    R16_UEP5_DMA     = 0;
    R32_USB_EP6_CTRL = DFU_USB_ENDPOINT_NAK;
    R16_UEP6_DMA     = 0;
    R32_USB_EP7_CTRL = DFU_USB_ENDPOINT_NAK;
    R16_UEP7_DMA     = 0;
    R16_UEP0_DMA     = (uint16_t)(uintptr_t)USB_ep0Dma;
    R8_UEP4_1_MOD    = 0;
    R8_UEP2_3_MOD    = 0;
    R8_UEP567_MOD    = 0;
    USB_ControlReset();
    R8_USB_DEV_AD = 0;
    R8_USB_INT_FG = UINT8_MAX;
    R8_UEP0_T_LEN = 0;
    R8_UEP0_CTRL  = USB_ep0Control;
    R8_USB_INT_EN = 0; /* flags latch independently; NO peripheral IRQ request */
    R8_UDEV_CTRL  = RB_UD_PD_DIS | RB_UD_PORT_EN;
    /* Connect last, with fresh EP0 and DMA already ready. */
    R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
}

void DFU_Main(void) {
    DFU_StopPeripherals();
    DFU_ClockInit();
    DFU_Connect();
    for (;;) {
        if (DFU_reconnect) {
            DFU_reconnect = false;

            /* As in manifestation Leave: allow the last status ACK and the
             * host control-completion callback to retire before disconnect. */
            DFU_DelayCycles(DFU_STATUS_ACK_CYCLES);
            DFU_Connect(); /* remain DFU; accepted-once latch survives */
        }
        uint8_t flags = R8_USB_INT_FG;

        if (flags & RB_UIF_BUS_RST) {
            USB_ControlReset();
            R8_USB_DEV_AD = 0;
            R8_USB_INT_FG = RB_UIF_BUS_RST;
        } else if (flags & RB_UIF_TRANSFER) {
            uint8_t status = R8_USB_INT_ST;
            uint8_t setup[USB_SETUP_PACKET_SIZE];
            bool    setupPending = (status & RB_UIS_SETUP_ACT) != 0;

            if (setupPending)
                for (unsigned i = 0; i < USB_SETUP_PACKET_SIZE; i++)
                    setup[i] = USB_ep0Dma[i];
            if (!(status & MASK_UIS_ENDP)) {
                if ((status & MASK_UIS_TOKEN) == UIS_TOKEN_IN)
                    USB_ControlIn();
                else if ((status & MASK_UIS_TOKEN) == UIS_TOKEN_OUT && !setupPending)
                    USB_ControlOut(USB_ep0Dma, R8_USB_RX_LEN, (status & RB_UIS_TOG_OK) != 0);
            }

            /* CH572 keeps the last IN/OUT token alongside SETUP_ACT. Clear
             * that completion FIRST, then consume SETUP and clear again. */
            if (setupPending) {
                /* Freeze before clearing a previous completion. Publishing
                 * the new toggle/length AFTER clearing INT_BUSY races the
                 * next host token and causes intermittent multi-packet loss. */
                R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK;
                if ((status & MASK_UIS_TOKEN) != UIS_TOKEN_SETUP)
                    R8_USB_INT_FG = RB_UIF_TRANSFER;
                USB_ControlSetup(setup);
            }
            R8_USB_DEV_AD = USB_address;
            R8_UEP0_T_LEN = USB_ep0Length;
            R8_UEP0_CTRL  = USB_ep0Control;
            R8_USB_INT_FG = RB_UIF_TRANSFER;
        } else if (flags)
            R8_USB_INT_FG = flags;
        R8_UEP0_T_LEN = USB_ep0Length;
        R8_UEP0_CTRL  = USB_ep0Control;

        /* Only a fully ACKed GETSTATUS releases this job, never SETUP/IN data alone. */
        DFU_Work();
        if (DFU_reboot) {
            uint8_t error = FLASH_Finish();

            if (error) {
                DFU_reboot = false;
                DFU_error  = error;
                DFU_state  = DFU_ERROR;
            } else
                DFU_Reboot();
        }
    }
}
