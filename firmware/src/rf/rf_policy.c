/* Small, foreground-only channel policy. Link byte offsets survive every hop. */
#include "rf/rf_policy.h"
#include <string.h>

static const uint8_t RF_channels[RF_POLICY_CHANNELS] = {36, 35, 34};

static bool RF_Expired(uint32_t now, uint32_t end) {
    return (int32_t)(now - end) >= 0;
}

static void RF_Change(rf_policy_t *p, uint8_t channel) {
    if (p->channel != channel) {
        p->channel = channel;
        ++p->switches;
    }
    p->waiting = p->failures = 0;
}

static void RF_Propose(rf_policy_t *p, uint8_t target, uint32_t now) {
    p->target = target;
    p->phase  = RF_LINK_PROPOSE;
    ++p->epoch;
    p->hop_until = now + RF_POLICY_HOP_MS;
    p->waiting = p->failures = 0;
}

void RF_PolicyInit(rf_policy_t *p, uint8_t role, uint32_t now) {
    memset(p, 0, sizeof(*p));
    p->role         = role;
    p->last_rx      = now;
    p->hold_until   = now;
    p->sample       = role == RF_ROLE_HOST ? 0 : UINT8_MAX - 1U;
    p->sample_until = now + RF_POLICY_SURVEY_MS;
    p->next_survey  = now + 30000U;
}

uint8_t RF_PolicyChannel(const rf_policy_t *p) {
    return RF_channels[p->channel];
}

uint8_t RF_PolicyTarget(const rf_policy_t *p) {
    return RF_channels[p->phase == RF_LINK_PROPOSE || p->phase == RF_LINK_READY ? p->target
                                                                                : p->channel];
}

unsigned RF_PolicyScore(const rf_policy_t *p, unsigned index) {
    return p->attempts[index]
               ? ((p->attempts[index] - p->replies[index]) * 256U) / p->attempts[index]
               : 256U;
}

void RF_PolicyTx(rf_policy_t *p) {
    p->waiting = p->role == RF_ROLE_HOST && p->phase == 0 && p->sample < RF_POLICY_CHANNELS;
    if (p->waiting) {
        ++p->attempts[p->channel];
    }
}

void RF_PolicyTxDone(rf_policy_t *p) {
    if (p->role == RF_ROLE_TARGET && p->phase == RF_LINK_READY) {
        RF_Change(p, p->target);
        p->phase = 0;
    }
}

void RF_PolicyReceive(rf_policy_t *p, uint8_t flags, uint8_t channel, uint8_t epoch, uint32_t now) {
    if (p->role == RF_ROLE_TARGET) {
        if (flags == RF_LINK_PROPOSE) {
            unsigned target;

            for (target = 0; target < RF_POLICY_CHANNELS; target++) {
                if (RF_channels[target] == channel) {
                    break;
                }
            }
            if (target == RF_POLICY_CHANNELS) {
                return;
            }
            p->target = target;
            p->phase  = RF_LINK_READY;
        } else if (channel != RF_PolicyChannel(p)) {
            return;
        }
        p->epoch = epoch;
    } else {
        if (epoch != p->epoch) {
            return;
        }
        if (p->phase == RF_LINK_PROPOSE && flags == RF_LINK_READY &&
            channel == RF_channels[p->target]) {
            RF_Change(p, p->target);
            p->phase     = RF_LINK_COMMIT;
            p->hop_until = now + RF_POLICY_HOP_MS;
        } else if (channel == RF_PolicyChannel(p) && !flags) {
            if (p->phase == RF_LINK_COMMIT) {
                p->phase        = 0;
                p->sample_until = now + RF_POLICY_SURVEY_MS;
                p->hold_until   = now + 500U;
            }
            if (p->waiting) {
                ++p->replies[p->channel];
                p->waiting = 0;
            }
        } else {
            return;
        }
    }
    p->last_rx  = now;
    p->failures = 0;
}

void RF_PolicyTimeout(rf_policy_t *p, uint32_t now) {
    p->waiting = 0;
    if (p->failures < UINT8_MAX) {
        ++p->failures;
    }
    if (p->role == RF_ROLE_HOST && !p->phase && p->failures >= 8 &&
        RF_Expired(now, p->hold_until)) {
        p->sample = p->role == RF_ROLE_HOST ? UINT8_MAX : UINT8_MAX - 1U;
        RF_Propose(p, (p->channel + 1) % RF_POLICY_CHANNELS, now);
    }
}

void RF_PolicyPoll(rf_policy_t *p, uint32_t now) {
    /* A lost proposal or READY can split the peers. Both return to the
     * agreed rendezvous channel after silence, without clearing stream data. */
    if ((uint32_t)(now - p->last_rx) >= RF_POLICY_RECOVER_MS && (p->channel || p->phase)) {
        RF_Change(p, 0);
        p->phase  = 0;
        p->sample = p->role == RF_ROLE_HOST ? UINT8_MAX : UINT8_MAX - 1U;
        ++p->recoveries;
        if (p->role == RF_ROLE_HOST) {
            ++p->epoch;
        }
        p->last_rx     = now;
        p->hold_until  = now + 500U;
        p->next_survey = now + 1000U;
    }
    if (p->role != RF_ROLE_HOST) {
        return;
    }
    if (p->phase == RF_LINK_PROPOSE && RF_Expired(now, p->hop_until)) {
        /* READY may be lost after the responder moves. Probe the announced
         * channel anyway; the rendezvous timeout covers a lost proposal. */
        RF_Change(p, p->target);
        p->phase = RF_LINK_COMMIT;
    }
    if (p->phase) {
        return;
    }
    if (p->sample == UINT8_MAX && RF_Expired(now, p->next_survey)) {
        memset(p->attempts, 0, sizeof(p->attempts));
        memset(p->replies, 0, sizeof(p->replies));
        p->sample       = 0;
        p->sample_until = now + RF_POLICY_SURVEY_MS;
        if (p->channel) {
            RF_Propose(p, 0, now);
        }
    } else if (p->sample < RF_POLICY_CHANNELS && RF_Expired(now, p->sample_until)) {
        if (++p->sample < RF_POLICY_CHANNELS) {
            RF_Propose(p, p->sample, now);
        } else {
            unsigned best = 0;

            for (unsigned i = 1; i < RF_POLICY_CHANNELS; i++) {
                if (RF_PolicyScore(p, i) + RF_POLICY_SCORE_MARGIN < RF_PolicyScore(p, best)) {
                    best = i;
                }
            }
            ++p->surveys;

            /* Startup is finished. Only a recovery starts another survey. */
            p->sample     = UINT8_MAX - 1U;
            p->hold_until = now + 500U;
            if (p->channel != best) {
                RF_Propose(p, best, now);
            }
        }
    }
}
