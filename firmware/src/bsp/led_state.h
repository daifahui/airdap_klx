#ifndef LED_STATE_H
#define LED_STATE_H

#include "app_config.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t until;
    uint8_t  status, active;
} led_state_t;

static inline void LED_SetActivity(volatile led_state_t *led, uint32_t now) {
#if !APP_COMBINED_HOST
    /* A continuous stream alternates a pulse and a quiet interval. Extending
     * every pulse on every byte would leave the status permanently inverted. */
    if (!led->active || (int32_t)(now - led->until) >= (int32_t)BSP_LED_PULSE_MS) {
        led->until  = now + BSP_LED_PULSE_MS;
        led->active = 1;
    }
#elif APP_COMBINED_HOST
    led->until  = now + BSP_LED_PULSE_MS;
    led->active = 1;
#else
    (void)led;
    (void)now;
#endif
}

/* Caller excludes interrupts while inspecting and expiring an activity pulse. */
static inline bool LED_IsOn(volatile led_state_t *led, uint32_t now) {
#if APP_COMBINED_HOST
    if (led->active && (int32_t)(led->until - now) > 0) {
        return true;
    }
    led->active = 0;
    return false;
#else
    bool pulse = led->active && (int32_t)(led->until - now) > 0;

    if (led->active && (int32_t)(now - led->until) >= (int32_t)BSP_LED_PULSE_MS) {
        led->active = 0;
    }
    return (led->status != 0) ^ pulse;
#endif
}
#endif
