#include "mode/app_mode.h"
#include "app_config.h"
bool MODE_peerSleeping;

#define AUTO_ENABLED_FLAG 0x80U

/* Internal bits in unused available-mask space preserve six-word ABI. */
#define USB_TIMELY  0x08U
#define BOTH_TIMELY 0x10U

static struct {
    uint32_t sequence;
    uint32_t completed_sequence;
    uint32_t phase_started;
    uint32_t window_ms;
    uint8_t  available_mask;
    uint8_t  current;
    uint8_t  target;
    uint8_t  phase;
    uint8_t  source;
    uint8_t  result;
    uint8_t  error;
    uint8_t  flags;
} MODE_state;

_Static_assert(sizeof(MODE_state) == 24, "Mode controller SRAM budget");

/* Loaded from the Host binding record. Kept outside MODE_state so the saved
 * six-word handoff ABI is unchanged. */
static bool MODE_bound;

static bool MODE_WindowOpen(uint32_t now) {
    return (uint32_t)(now - MODE_state.phase_started) < MODE_state.window_ms;
}

static void MODE_SelectMode(uint8_t mode, uint8_t source) {
    MODE_state.flags &= (uint8_t)~APP_MODE_FLAG_TARGET_RETURN;
    ++MODE_state.sequence;
    MODE_state.source = source;
    MODE_state.target = mode;
    MODE_state.error  = APP_MODE_ERROR_NONE;
    if (!((MODE_state.available_mask & APP_MODE_BIT(mode)) != 0)) {
        MODE_state.phase               = APP_MODE_READY;
        MODE_state.result              = APP_MODE_FAILED;
        MODE_state.error               = APP_MODE_ERROR_UNSUPPORTED;
        MODE_state.completed_sequence  = MODE_state.sequence;
        MODE_state.flags              &= (uint8_t)~APP_MODE_FLAG_MATCHING;
    } else if (mode == MODE_state.current) {
        MODE_state.phase              = APP_MODE_READY;
        MODE_state.result             = APP_MODE_SUCCESS;
        MODE_state.completed_sequence = MODE_state.sequence;
    } else {
        MODE_state.phase  = APP_MODE_PENDING;
        MODE_state.result = APP_MODE_IN_PROGRESS;
    }
}

/* One boot decision only. phase_started/window_ms are reused for the
 * known-clock disconnect and then the shared USB/RF window, preserving ABI24. */
static bool MODE_Automatic(void) {
    return (MODE_state.flags & AUTO_ENABLED_FLAG) &&
           !(MODE_state.flags & (APP_MODE_FLAG_USB_OVERRIDE | APP_MODE_FLAG_FROZEN));
}

static void MODE_Decide(uint32_t now) {
    if (MODE_bound || !MODE_Automatic() || MODE_state.phase != APP_MODE_PAIRING)
        return;
    if (MODE_WindowOpen(now) && (MODE_state.available_mask & USB_TIMELY) &&
        (MODE_state.flags & APP_MODE_FLAG_PEER_MATCHED))
        MODE_state.available_mask |= BOTH_TIMELY;
    if (MODE_state.flags & APP_MODE_FLAG_AUTO_HELD)
        return;
    if ((MODE_state.available_mask & BOTH_TIMELY) &&
        (MODE_state.flags & APP_MODE_FLAG_USB_CONFIGURED)) {
        MODE_state.flags &= (uint8_t)~APP_MODE_FLAG_MATCHING;
        MODE_SelectMode(APP_WORK_RF_HOST, APP_MODE_AUTOMATIC);
    } else if (!MODE_WindowOpen(now)) {
        bool usb = (MODE_state.available_mask & USB_TIMELY) &&
                   (MODE_state.flags & APP_MODE_FLAG_USB_CONFIGURED);
        MODE_state.flags &= (uint8_t)~(APP_MODE_FLAG_MATCHING | APP_MODE_FLAG_PEER_MATCHED);
        MODE_SelectMode(usb ? APP_WORK_LOCAL : APP_WORK_RF_TARGET, APP_MODE_AUTOMATIC);
    }
}

void MODE_AppModeInit(uint32_t now, uint32_t availableMask, uint32_t bootWindowMs) {
    MODE_peerSleeping = false;
    (void)bootWindowMs; /* Complete policy fixed at one 2000ms window. */
    MODE_state.sequence = MODE_state.completed_sequence = 0;
    MODE_state.phase_started                            = now;
    MODE_state.window_ms                                = APP_MODE_PRE_DISCONNECT_MS;

#if BSP_USB_DONGLE
    (void)availableMask;
    MODE_state.window_ms      = 0;
    MODE_state.available_mask = APP_MODE_BIT(APP_WORK_RF_HOST);
    MODE_state.current = MODE_state.target = APP_WORK_RF_HOST;
    MODE_state.source                      = APP_MODE_AUTOMATIC;
    MODE_state.error                       = APP_MODE_ERROR_NONE;
    MODE_state.flags                       = 0;
    MODE_state.phase                       = APP_MODE_READY;
    MODE_state.result                      = APP_MODE_SUCCESS;
#else
    if (MODE_bound) {
        /* The stored peer is the only possible Host session. Keep LOCAL and
         * TARGET unavailable to the automatic policy while waiting. */
        MODE_state.available_mask = APP_MODE_ALL;
        MODE_state.window_ms      = 0;
        MODE_state.current = MODE_state.target = APP_WORK_RF_HOST;
        MODE_state.source                      = APP_MODE_AUTOMATIC;
        MODE_state.error                       = APP_MODE_ERROR_NONE;
        MODE_state.flags                       = 0;
        MODE_state.phase                       = APP_MODE_READY;
        MODE_state.result                      = APP_MODE_SUCCESS;
    } else {
        MODE_state.available_mask = (uint8_t)(availableMask & APP_MODE_ALL);
        bool radio                = (availableMask & APP_MODE_BIT(APP_WORK_RF_HOST)) != 0;

        MODE_state.current = radio ? APP_WORK_NONE : APP_WORK_LOCAL;
        MODE_state.target  = MODE_state.current;
        MODE_state.source  = APP_MODE_AUTOMATIC;
        MODE_state.error   = APP_MODE_ERROR_NONE;
        MODE_state.flags   = radio ? AUTO_ENABLED_FLAG : 0;
        MODE_state.phase   = radio ? APP_MODE_PRE_DISCONNECT : APP_MODE_READY;
        MODE_state.result  = radio ? APP_MODE_NO_RESULT : APP_MODE_SUCCESS;
    }
#endif
}

void MODE_AppModeSetBound(bool bound) {
    MODE_bound = bound;
    if (bound && MODE_Automatic() && MODE_state.phase != APP_MODE_READY) {
        /* A runtime scan-only request can arrive while the boot window is
         * still open; pin the already-running Host before that window ends. */
        MODE_state.flags &=
            (uint8_t)~(AUTO_ENABLED_FLAG | APP_MODE_FLAG_MATCHING | APP_MODE_FLAG_PEER_MATCHED);
        MODE_state.current = MODE_state.target = APP_WORK_RF_HOST;
        MODE_state.phase                       = APP_MODE_READY;
        MODE_state.window_ms                   = 0;
        MODE_state.result                      = APP_MODE_SUCCESS;
        MODE_state.error                       = APP_MODE_ERROR_NONE;
        ++MODE_state.sequence;
        MODE_state.completed_sequence = MODE_state.sequence;
    }
}

bool MODE_AppModePreDisconnect(void) {
    return MODE_Automatic() && MODE_state.phase == APP_MODE_PRE_DISCONNECT;
}

bool MODE_AppModeStartScan(uint32_t now) {
    if (!MODE_AppModePreDisconnect() || MODE_WindowOpen(now))
        return false;
    MODE_state.phase_started  = now;
    MODE_state.window_ms      = APP_MODE_PAIR_WINDOW_MS;
    MODE_state.phase          = APP_MODE_PAIRING;
    MODE_state.flags         |= APP_MODE_FLAG_MATCHING;
    return true;
}

void MODE_AppModeBootRadioFailed(void) {
#if BSP_USB_DONGLE
    MODE_state.result = APP_MODE_FAILED;
    MODE_state.error  = APP_MODE_ERROR_START_FAILED;
#else
    if (!MODE_AppModePreDisconnect())
        return;
    MODE_SelectMode(APP_WORK_LOCAL, APP_MODE_AUTOMATIC);
    MODE_state.phase = APP_MODE_SWITCHING;
    MODE_AppModeFinish(MODE_state.sequence, false, APP_MODE_ERROR_START_FAILED);
#endif
}

void MODE_AppModeNoteUsbUnconfigured(void) {
    MODE_state.flags          &= (uint8_t)~APP_MODE_FLAG_USB_CONFIGURED;
    MODE_state.available_mask &= APP_MODE_ALL;
    if (MODE_Automatic() && MODE_state.phase == APP_MODE_PENDING &&
        MODE_state.target == APP_WORK_RF_HOST) {
        MODE_state.phase   = APP_MODE_PAIRING;
        MODE_state.flags  |= APP_MODE_FLAG_MATCHING;
        MODE_state.target  = MODE_state.current;
    }
}

void MODE_AppModeNoteUsbConfigured(uint32_t now) {
    /* Expire BEFORE accepting a late ACK: it cannot combine with an old OFFER.
     * Both event APIs decide immediately, so in-window facts survive a delayed
     * foreground poll and either event ordering has the same result. */
    if (MODE_state.phase == APP_MODE_PRE_DISCONNECT ||
        (MODE_state.phase == APP_MODE_PAIRING &&
         (uint32_t)(now - MODE_state.phase_started) >= 0x80000000U))
        return;
    MODE_Decide(now);
    MODE_state.flags |= APP_MODE_FLAG_USB_CONFIGURED;
    if (MODE_state.phase == APP_MODE_PAIRING && MODE_WindowOpen(now))
        MODE_state.available_mask |= USB_TIMELY;
    MODE_Decide(now);
}

bool MODE_AppModeAutoDiscovery(void) {
    return MODE_Automatic() &&
           ((MODE_state.flags & APP_MODE_FLAG_MATCHING) || MODE_state.current == APP_WORK_RF_HOST ||
            MODE_state.current == APP_WORK_RF_TARGET);
}

bool MODE_AppModeBootScanning(void) {
    return MODE_Automatic() &&
           (MODE_state.phase == APP_MODE_PAIRING ||
            (MODE_state.current == APP_WORK_NONE && MODE_state.target == APP_WORK_RF_HOST));
}

void MODE_AppModeForgetCandidate(void) {
    if (!MODE_AppModeBootScanning())
        return;
    MODE_state.flags          &= (uint8_t)~APP_MODE_FLAG_PEER_MATCHED;
    MODE_state.available_mask &= (uint8_t)~BOTH_TIMELY;
    if (MODE_state.phase == APP_MODE_PENDING) {
        MODE_state.phase   = APP_MODE_PAIRING;
        MODE_state.flags  |= APP_MODE_FLAG_MATCHING;
        MODE_state.target  = MODE_state.current;
    }
}

bool MODE_AppModeRfCommitAllowed(void) {
    if (!MODE_Automatic())
        return true; /* explicit role/session policy */
    return (MODE_state.flags & APP_MODE_FLAG_USB_CONFIGURED) &&
           (MODE_state.current == APP_WORK_RF_HOST ||
            (MODE_state.phase == APP_MODE_PENDING && MODE_state.target == APP_WORK_RF_HOST));
}

bool MODE_AppModeReady(void) {
    return MODE_state.phase == APP_MODE_READY && MODE_state.current != APP_WORK_NONE &&
           (MODE_state.current == APP_WORK_RF_TARGET ||
            (MODE_state.flags & APP_MODE_FLAG_USB_CONFIGURED));
}

bool MODE_AppModeActionPending(void) {
    return MODE_state.phase == APP_MODE_PENDING;
}

bool MODE_AppModeNotePeerMatched(uint32_t now) {
    if (!MODE_Automatic() || MODE_state.phase != APP_MODE_PAIRING || !MODE_WindowOpen(now))
        return false;
    MODE_state.flags |= APP_MODE_FLAG_PEER_MATCHED;
    MODE_Decide(now);
    return true;
}

bool MODE_AppModeTargetLinkPoll(uint32_t now, bool paired) {
    if (MODE_peerSleeping)
        return false;
    if (MODE_state.current != APP_WORK_RF_TARGET || MODE_state.phase != APP_MODE_READY ||
        MODE_state.source != APP_MODE_USB ||
        (MODE_state.flags & (APP_MODE_FLAG_USB_OVERRIDE | APP_MODE_FLAG_FROZEN)) !=
            APP_MODE_FLAG_USB_OVERRIDE ||
        !(MODE_state.available_mask & APP_MODE_BIT(APP_WORK_LOCAL))) {
        return false;
    }
    if (paired) {
        MODE_state.flags &= (uint8_t)~APP_MODE_FLAG_TARGET_RETURN;
    } else if (!(MODE_state.flags & APP_MODE_FLAG_TARGET_RETURN)) {
        /* The boot/pairing window is inactive under a USB override. Reuse its
         * fields and one spare flag, preserving the six-word mailbox ABI. */
        MODE_state.phase_started  = now;
        MODE_state.window_ms      = APP_MODE_TARGET_RETURN_MS;
        MODE_state.flags         |= APP_MODE_FLAG_TARGET_RETURN;
    } else if (!MODE_WindowOpen(now)) {
        MODE_SelectMode(APP_WORK_LOCAL, APP_MODE_USB);
        return true;
    }
    return false;
}

void MODE_AppModePoll(uint32_t now) {
    MODE_Decide(now);
}

void MODE_AppModeHoldAuto(bool hold) {
    if (hold) {
        MODE_state.flags |= APP_MODE_FLAG_AUTO_HELD;
    } else {
        MODE_state.flags &= (uint8_t)~APP_MODE_FLAG_AUTO_HELD;
    }
}

uint8_t MODE_AppModeValidate(uint8_t mode) {
    if (mode > APP_WORK_AUTO) {
        return APP_MODE_ERROR_INVALID;
    }
    if (mode == APP_WORK_AUTO) {
        if (!(MODE_state.available_mask & APP_MODE_BIT(APP_WORK_RF_HOST)))
            return APP_MODE_ERROR_UNSUPPORTED;
    } else if (!((MODE_state.available_mask & APP_MODE_BIT(mode)) != 0)) {
        return APP_MODE_ERROR_UNSUPPORTED;
    }
    if (MODE_state.flags & APP_MODE_FLAG_FROZEN) {
        return APP_MODE_ERROR_FROZEN;
    }
    if (MODE_state.phase != APP_MODE_READY) {
        return APP_MODE_ERROR_BUSY;
    }
    return APP_MODE_ERROR_NONE;
}

uint8_t MODE_AppModeCommit(uint8_t mode, uint32_t *sequence) {
    uint8_t error = MODE_AppModeValidate(mode);

    if (error != APP_MODE_ERROR_NONE) {
        return error;
    }
    MODE_state.flags |= APP_MODE_FLAG_USB_OVERRIDE;
    MODE_state.flags &= (uint8_t)~(APP_MODE_FLAG_MATCHING | APP_MODE_FLAG_PEER_MATCHED);
    if (mode == APP_WORK_AUTO) {
#if BSP_USB_DONGLE
        MODE_state.flags &= (uint8_t)~APP_MODE_FLAG_USB_OVERRIDE;
        MODE_SelectMode(APP_WORK_RF_HOST, APP_MODE_AUTOMATIC);
#else
        /* The foreground stops current owners, then reloads the boot HOST
         * with an AUTO controller marker. Boot reinitializes the existing scan. */
        ++MODE_state.sequence;
        MODE_state.target = APP_WORK_AUTO;
        MODE_state.source = APP_MODE_USB;
        MODE_state.phase  = APP_MODE_PENDING;
        MODE_state.result = APP_MODE_IN_PROGRESS;
        MODE_state.error  = APP_MODE_ERROR_NONE;
#endif
    } else {
        MODE_SelectMode(mode, APP_MODE_USB);
    }
    if (sequence) {
        *sequence = MODE_state.sequence;
    }
    return APP_MODE_ERROR_NONE;
}

bool MODE_AppModeTakeAction(app_mode_action_t *action) {
    if (!action || MODE_state.phase != APP_MODE_PENDING ||
        (MODE_state.flags & APP_MODE_FLAG_FROZEN)) {
        return false;
    }
    action->sequence = MODE_state.sequence;
    action->previous = MODE_state.current;
    action->target   = MODE_state.target;
    MODE_state.phase = APP_MODE_SWITCHING;
    return true;
}

bool MODE_AppModeFinish(uint32_t sequence, bool success, uint8_t error) {
    if (MODE_state.phase != APP_MODE_SWITCHING || sequence != MODE_state.sequence) {
        return false;
    }
    if (success) {
        MODE_state.current = MODE_state.target;
        MODE_state.result  = APP_MODE_SUCCESS;
        MODE_state.error   = APP_MODE_ERROR_NONE;
    } else {
        MODE_state.current  = ((MODE_state.available_mask & APP_MODE_BIT(APP_WORK_LOCAL)) != 0)
                                  ? APP_WORK_LOCAL
                                  : APP_WORK_NONE;
        MODE_state.result   = APP_MODE_FAILED;
        MODE_state.error    = error ? error : APP_MODE_ERROR_START_FAILED;
        MODE_state.flags   &= (uint8_t)~(APP_MODE_FLAG_MATCHING | APP_MODE_FLAG_PEER_MATCHED);
    }
    MODE_state.phase              = APP_MODE_READY;
    MODE_state.completed_sequence = MODE_state.sequence;
    return true;
}

void MODE_AppModeFreeze(void) {
    MODE_state.flags |= APP_MODE_FLAG_FROZEN;
    MODE_state.flags &=
        (uint8_t)~(APP_MODE_FLAG_AUTO_HELD | APP_MODE_FLAG_MATCHING | APP_MODE_FLAG_TARGET_RETURN);
    if (MODE_state.phase == APP_MODE_PENDING || MODE_state.phase == APP_MODE_PAIRING) {
        MODE_state.phase              = APP_MODE_READY;
        MODE_state.result             = APP_MODE_FAILED;
        MODE_state.error              = APP_MODE_ERROR_FROZEN;
        MODE_state.completed_sequence = MODE_state.sequence;
    } else if (MODE_state.phase == APP_MODE_BOOT) {
        MODE_state.phase  = APP_MODE_READY;
        MODE_state.result = APP_MODE_NO_RESULT;
    }
}

void MODE_AppModeSnapshot(app_mode_status_t *status) {
    status->sequence           = MODE_state.sequence;
    status->completed_sequence = MODE_state.completed_sequence;
    status->available_mask     = MODE_state.available_mask & APP_MODE_ALL;
    status->boot_deadline      = MODE_state.phase_started + MODE_state.window_ms;
    status->current            = MODE_state.current;
    status->target             = MODE_state.target;
    status->phase              = MODE_state.phase;
    status->source             = MODE_state.source;
    status->result             = MODE_state.result;
    status->error              = MODE_state.error;
    status->auto_enabled =
        ((MODE_state.flags & AUTO_ENABLED_FLAG) &&
         !(MODE_state.flags & (APP_MODE_FLAG_USB_OVERRIDE | APP_MODE_FLAG_FROZEN)));
    status->flags = MODE_state.flags & (uint8_t)~AUTO_ENABLED_FLAG;
}

void MODE_AppModeSave(uint32_t words[6]) {
    const uint8_t *src = (const uint8_t *)&MODE_state;
    uint8_t       *dst = (uint8_t *)words;

    for (unsigned i = 0; i < sizeof(MODE_state); ++i) {
        dst[i] = src[i];
    }
}

void MODE_AppModeRestore(const uint32_t words[6]) {
    const uint8_t *src = (const uint8_t *)words;
    uint8_t       *dst = (uint8_t *)&MODE_state;

    for (unsigned i = 0; i < sizeof(MODE_state); ++i) {
        dst[i] = src[i];
    }
}
