#ifndef RF_BRIDGE_H
#define RF_BRIDGE_H

#include "rf/rf_link.h"
#include "dap/dap_config.h"

/* A missing peer releases the private session before the USB scan times out. */
#define RF_DISCOVERY_LEASE_MS 900U

/* Foreground only: stop/start requires the USB owner to retire/cancel its DMA
 * ownership first. Every start clears the session; configure a fresh shared
 * nonzero session before submitting data. Failure leaves RF stopped. */

bool RF_BridgeStart(uint8_t role);

/* Foreground, after start: discard queues and discover the opposite role.
 * Caller supplies a fresh nonzero nonce per attempt. Host discovery must be
 * bounded by the controller's enumeration window. Targets broadcast ADVERT
 * on the public AA every20..32ms and listen between packets; hosts scan
 * silently, then select that exact UID/nonce via PROBE/OFFER/COMMIT/ACK.
 * The ACK completes on public AA; both peers confirm READY on the advertised
 * UID-derived private AA before business is enabled. AA is stable, session
 * rotates. Lease recovery revisits public AA without reassigning private AA.
 * Zero cancels discovery: discard an unconfirmed candidate, preserve an
 * established/manual raw session. Manual session/flush also exits discovery.
 * Confirming/paired auto sessions expire after RF_DISCOVERY_LEASE_MS without
 * valid same-session RU traffic, including empty polls. Expiry discards both
 * queues and advances the nonce before discovery resumes. The optional
 * transport-reset hook cancels old USB DMA ownership before queues are reset.
 * paired() reports only automatic handshake evidence. No authentication. */

bool RF_BridgeDiscover(uint64_t nonce);

bool RF_BridgePaired(void);

/* First successful handshake time; meaningful only while paired() is true.
 * Millisecond zero is valid. Renewals do not change this timestamp. */

uint32_t RF_BridgePairedAt(void);

/* True while automatic discovery owns the session, including paired state. */

bool RF_BridgeDiscovering(void);

/* Optional transport owner implementation. Called with interrupts masked,
 * before discovery resets queues; must cancel old RF endpoint ownership.
 * False defers discovery without changing RF state; absent hook means ready. */

bool RF_BridgeTransportReset(void);

void RF_BridgePoll(void);

#if RF_TX_PREFILL
void RF_BridgePrefill(void);
#endif

void RF_BridgeReplyPrefill(void);

void RF_BridgeStop(void);

/* Foreground USB suspend: bounded private-session control/ACK exchange,
 * independent of stream credit/backlog. No payload buffer is allocated. */

bool RF_BridgePrepareSuspend(void);

/* Foreground clock fence, preserving the reliable session and its queues.
 * False means a published event or TX must be serviced first. */

bool RF_BridgeClockPause(void);

void RF_BridgeClockResume(void);

void RF_BridgeFlush(void);

unsigned RF_BridgeSpace(void);

/* Header + payload share one preflight/foreground generation ownership. */

bool RF_BridgeSubmitPair(const uint8_t *first, unsigned firstLength, const uint8_t *second,
                         unsigned secondLength);

unsigned RF_BridgeRead(uint8_t *data, unsigned limit);

/* Peek one complete payload after its already inspected header. The caller
 * retains the whole record until queue publication succeeds. */

bool RF_BridgeReadAfter(uint8_t *data, unsigned prefix, unsigned length);

void RF_BridgeRetire(unsigned length);

bool RF_BridgeActive(void);

bool RF_BridgeEventPending(void);
#endif
