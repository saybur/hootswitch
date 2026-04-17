/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __LED_H__
#define __LED_H__

#include "hardware.h"

/*
 * LED control. Drivers and handlers should not use these functions directly.
 * Instead, tie into the LEDs via the notification system.
 */

/**
 * Activates or deactivates the activity light. This is assumed to be yellow
 * and is toggled based on bus activity.
 */
static inline void led_activity_on(void)
{
	gpio_set_mask(1UL << LED_ACT_PIN);
}
static inline void led_activity_off(void)
{
	gpio_clr_mask(1UL << LED_ACT_PIN);
}

/**
 * Activates or deactivates the error light. This is assumed to be red and
 * represents some exceptional condition.
 */
static inline void led_error_on(void)
{
	gpio_set_mask(1UL << LED_ERR_PIN);
}

static inline void led_error_off(void)
{
	gpio_clr_mask(1UL << LED_ERR_PIN);
}

/**
 * Activates one of the machine indicator LEDs at a chosen intensity level.
 *
 * @param mach   machine indicator from 0-3.
 * @param level  brightness level, from 0 (off) to 255 (max intensity).
 */
void led_machine(uint8_t mach, uint8_t level);

/**
 * Sets up the LEDs. Called during init, do not invoke as a user.
 */
void led_init(void);

#endif /* __LED_H__ */
