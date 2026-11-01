#ifndef RF_POLICY_H
#define RF_POLICY_H

#include "rf/rf_link.h"

#define RF_POLICY_CHANNELS     3U
#define RF_POLICY_SURVEY_MS    250U
#define RF_POLICY_HOP_MS       48U
#define RF_POLICY_RECOVER_MS   300U
#define RF_POLICY_SCORE_MARGIN 8U /* About 3 percentage points of reply loss. */

/* Channel indexes are internal. Wire and USB fields use BLE channel numbers. */
typedef struct {
    uint32_t last_rx, sample_until, hop_until, next_survey, hold_until;
    uint32_t switches, recoveries, surveys;
    uint16_t attempts[RF_POLICY_CHANNELS], replies[RF_POLICY_CHANNELS];
    uint8_t  channel, target, phase, epoch, role, sample, failures, waiting;
} rf_policy_t;

void RF_PolicyInit(rf_policy_t *policy, uint8_t role, uint32_t now);

uint8_t RF_PolicyChannel(const rf_policy_t *policy);

uint8_t RF_PolicyTarget(const rf_policy_t *policy);

unsigned RF_PolicyScore(const rf_policy_t *policy, unsigned index);

void RF_PolicyPoll(rf_policy_t *policy, uint32_t now);

void RF_PolicyTx(rf_policy_t *policy);

void RF_PolicyTxDone(rf_policy_t *policy);

void RF_PolicyTimeout(rf_policy_t *policy, uint32_t now);

void RF_PolicyReceive(rf_policy_t *policy, uint8_t flags, uint8_t channel, uint8_t epoch,
                      uint32_t now);
#endif
