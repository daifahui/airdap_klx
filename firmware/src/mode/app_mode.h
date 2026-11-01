#ifndef APP_MODE_H
#define APP_MODE_H

#include <stdbool.h>
#include <stdint.h>

#define APP_WORK_LOCAL     0
#define APP_WORK_RF_HOST   1
#define APP_WORK_RF_TARGET 2
#define APP_WORK_AUTO      3

/* USB-only policy request; never a loaded bank. */
#define APP_WORK_NONE 0xFF

/* A validated RF peer sleep notification holds USB-selected TARGET until a
 * new handshake. */
extern bool MODE_peerSleeping;

#define APP_MODE_BIT(mode)         (1U << (mode))
#define APP_MODE_ALL               0x07U
#define APP_MODE_PAIR_WINDOW_MS    2000U
#define APP_MODE_PRE_DISCONNECT_MS 100U

/* USB-selected TARGET returns to LOCAL after the link timeout. */
#define APP_MODE_TARGET_RETURN_MS 10000U

#define APP_MODE_BOOT      0
#define APP_MODE_READY     1
#define APP_MODE_PENDING   2
#define APP_MODE_SWITCHING 3
#define APP_MODE_PAIRING   4

/* concurrent USB/RF boot scan, not a final role */
#define APP_MODE_PRE_DISCONNECT 5

#define APP_MODE_AUTOMATIC 0
#define APP_MODE_USB       1

#define APP_MODE_NO_RESULT   0
#define APP_MODE_IN_PROGRESS 1
#define APP_MODE_SUCCESS     2
#define APP_MODE_FAILED      3

#define APP_MODE_ERROR_NONE         0
#define APP_MODE_ERROR_INVALID      1
#define APP_MODE_ERROR_UNSUPPORTED  2
#define APP_MODE_ERROR_BUSY         3
#define APP_MODE_ERROR_FROZEN       4
#define APP_MODE_ERROR_START_FAILED 5

#define APP_MODE_FLAG_USB_OVERRIDE   0x01U
#define APP_MODE_FLAG_FROZEN         0x02U
#define APP_MODE_FLAG_USB_CONFIGURED 0x04U
#define APP_MODE_FLAG_BOOT_USB       APP_MODE_FLAG_USB_CONFIGURED
#define APP_MODE_FLAG_AUTO_HELD      0x08U
#define APP_MODE_FLAG_MATCHING       0x10U
#define APP_MODE_FLAG_PEER_MATCHED   0x20U
#define APP_MODE_FLAG_TARGET_RETURN  0x40U

/* Fixed-width query fields; the USB owner serializes its own wire format. */
typedef struct {
    uint32_t sequence;
    uint32_t completed_sequence;
    uint32_t available_mask;
    uint32_t boot_deadline; /* Current boot-enumeration or pairing deadline. */
    uint8_t  current;
    uint8_t  target;
    uint8_t  phase;
    uint8_t  source;
    uint8_t  result;
    uint8_t  error;
    uint8_t  auto_enabled;
    uint8_t  flags;
} app_mode_status_t;

typedef struct {
    uint32_t sequence;
    uint8_t  previous;
    uint8_t  target;
} app_mode_action_t;

/* One shared boot window: PRE disconnect100ms, then USB/RF scan2000ms.
 * Both timely -> HOST; deadline USB yes -> LOCAL; USB no -> TARGET.
 * Events at the deadline are late. Bus reset clears current USB evidence,
 * never restarts a window. Manual selection is admitted only after READY. */

bool MODE_AppModePreDisconnect(void);

bool MODE_AppModeStartScan(uint32_t now);

void MODE_AppModeBootRadioFailed(void);

void MODE_AppModeNoteUsbUnconfigured(void);

bool MODE_AppModeRfCommitAllowed(void);

bool MODE_AppModeBootScanning(void);

void MODE_AppModeForgetCandidate(void);

void MODE_AppModeInit(uint32_t now, uint32_t availableMask, uint32_t bootWindowMs);

void MODE_AppModeNoteUsbConfigured(uint32_t now);

void MODE_AppModePoll(uint32_t now);

/* USB data endpoints stay closed while a boot decision or switch is pending. */

bool MODE_AppModeReady(void);

bool MODE_AppModeActionPending(void);

/* Enable discovery after starting an automatically owned RF bank, or when
 * recovering its transport. Do not restart the RF handshake on every poll. */

bool MODE_AppModeAutoDiscovery(void);

/* A validated OFFER before the shared deadline is candidate evidence only.
 * USB configuration ACK is required before choosing HOST or sending COMMIT. */

bool MODE_AppModeNotePeerMatched(uint32_t now);

/* Foreground, about every 100 ms, only in a role-banked TARGET. A disconnected
 * USB-selected TARGET returns to sticky LOCAL after 10 s, including initial
 * pairing wait. A confirmed link cancels the countdown. True means a LOCAL
 * action is pending and may be taken by the same foreground caller. */

bool MODE_AppModeTargetLinkPoll(uint32_t now, bool paired);

/* Hold a validated mode SETUP until its status ACK or cancellation. The boot
 * deadline still advances, and USB configuration/matching is remembered.
 * On ACK commit first, then release the hold. New SETUP/reset/timeout cancels
 * the hold; a canceled control request must never call commit. */

void MODE_AppModeHoldAuto(bool hold);

uint8_t MODE_AppModeValidate(uint8_t mode);

uint8_t MODE_AppModeCommit(uint8_t mode, uint32_t *sequence);

/* Foreground consumes one action, stops/cleans the old mode, and initializes
 * the target. On failure restore safe LOCAL (or stop all if unavailable) before
 * finish(false). A stale or duplicate finish cannot change controller state. */

bool MODE_AppModeTakeAction(app_mode_action_t *action);

bool MODE_AppModeFinish(uint32_t sequence, bool success, uint8_t error);

/* DFU is terminal for this startup. Cancel any action not yet consumed; an
 * already running action may finish its cleanup, but no further one starts. */

void MODE_AppModeFreeze(void);

void MODE_AppModeSnapshot(app_mode_status_t *status);

/* Role images share this exact six-word controller ABI. These are
 * used only after data owners stop and before enabling the next role. */

void MODE_AppModeSave(uint32_t words[6]);

void MODE_AppModeRestore(const uint32_t words[6]);
#endif
