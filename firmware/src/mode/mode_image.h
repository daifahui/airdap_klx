#ifndef MODE_IMAGE_H
#define MODE_IMAGE_H

#include <stdbool.h>
#include <stdint.h>

/* Last 24 bytes of the existing warm-handoff mailbox. No extra role RAM.
 * Cold startup clears the mailbox; role replacement preserves RF identity. */
typedef struct {
    uint8_t  peer_uid[8];
    uint32_t peer_aa, own_aa, bound, reserved;
} rf_binding_t;

typedef struct {
    uint32_t     magic, role, milliseconds, discardedTransitions;
    uint32_t     controller[6];
    rf_binding_t binding;
} mode_mailbox_t;

_Static_assert(sizeof(mode_mailbox_t) == 64, "Role mailbox ABI");
extern mode_mailbox_t MODE_mailbox;

/* Flash-resident role replacement; never called inside an ISR or DAP command. */

bool MODE_ImageWarm(void);

void MODE_ImageBoot(void);

void MODE_ImageFinish(bool success);

void MODE_ImageRequest(uint8_t role);

bool MODE_ImagePoll(void);
#endif
