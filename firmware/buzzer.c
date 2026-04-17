/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "buzzer.h"
#include "hardware.h"

#define FREQ_MIN        50
#define FREQ_MAX        8000
#define PWM_DIV         16

static volatile bool buzzer_enabled = true;
static uint8_t slice;
static uint8_t chan;

void buzzer_enable(bool enabled)
{
	buzzer_enabled = enabled;
}

void buzzer_play(uint16_t freq, uint8_t vol)
{
	if (vol == 0) {
		// stop any ongoing playback
		pwm_set_chan_level(slice, chan, 0);
		pwm_set_wrap(slice, 1);
		return;
	}

	if (freq < FREQ_MIN) freq = FREQ_MIN;
	if (freq > FREQ_MAX) freq = FREQ_MAX;
	if (vol > 7) vol = 7;

	/*
	 * Rewriting the formula from datasheet 4.5.2.6 should yield this:
	 *
	 * top = fclk / ((CSR_PH_CORRECT + 1) * fPWM * DIV_INT)
	 *
	 * Do wish I'd paid more attention in algebra instead of messing around on
	 * my graphing calculator :(
	 */
	uint16_t top = clock_get_hz(clk_sys) / (2 * PWM_DIV * freq);

#ifndef BUZZER_DISABLE
	// even if disabled go through most of the motions to keep timing similar
	if (buzzer_enabled) {
		pwm_set_wrap(slice, top);
		pwm_set_chan_level(slice, chan, top >> (8 - vol));
	}
#endif
}

void buzzer_init(void)
{
	gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
	slice = pwm_gpio_to_slice_num(BUZZER_PIN);
	chan = pwm_gpio_to_channel(BUZZER_PIN);

	pwm_set_chan_level(slice, chan, 0);
	pwm_set_phase_correct(slice, true);
	pwm_set_clkdiv_int_frac(slice, PWM_DIV, 0);
	pwm_set_wrap(slice, 1);
	pwm_set_enabled(slice, true);
}
