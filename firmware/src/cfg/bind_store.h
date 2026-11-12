#ifndef CFG_BIND_STORE_H
#define CFG_BIND_STORE_H

#include <stdbool.h>
#include <stdint.h>

/* HOST/DONGLE peer binding persisted in two alternating Flash sectors.
 * BIND_Load runs once per bank entry. Requests only stage an action; the
 * foreground BIND_Poll stops RF, verifies DAP is idle, then erases/programs. */

#define BIND_UID_SIZE 8U

void BIND_Load(void);

/* Copies the bound peer UID. False means no valid record is bound. */

bool BIND_Get(uint8_t uid[BIND_UID_SIZE]);

/* The current UID is already stored: no write is scheduled. */

bool BIND_RequestSave(const uint8_t uid[BIND_UID_SIZE]);

bool BIND_RequestClear(void);

/* Read-only scan: stop probing and report every discovered peer. */

bool BIND_RequestScanOnly(void);

/* Foreground only, after the RF DAP poll. Never called from an ISR. */

void BIND_Poll(void);
#endif
