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
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

#include "buzzer.h"
#include "hardware.h"

#define FREQ_MIN        400
#define FREQ_MAX        8000
#define PWM_LENGTH      512

#define CHIRP_FREQ      3200
#define CHIRP_DURATION  100
#define CHIRP_VOLUME    64

static volatile bool buzzer_enabled = true;
static uint8_t slice;
static uint8_t chan;

static void buzzer_callback(void)
{
	pwm_set_chan_level(slice, chan, 0);
	timer_hw->intr = 1U << BUZZER_TIMER;
}

void buzzer_chirp(void)
{
	buzzer_play(CHIRP_FREQ, CHIRP_DURATION, CHIRP_VOLUME);
}

void buzzer_enable(bool enabled)
{
	buzzer_enabled = enabled;
}

void buzzer_play(uint16_t freq, uint16_t duration_ms, uint8_t vol)
{
	if (duration_ms == 0) return;
	if (freq < 50) freq = FREQ_MIN;
	if (freq > FREQ_MAX) freq = FREQ_MAX;

	float div = clock_get_hz(clk_sys) / (float) (freq * PWM_LENGTH);
#ifndef BUZZER_DISABLE
	// even if disabled go through most of the motions to keep timing similar
	if (buzzer_enabled) {
		pwm_set_clkdiv(slice, div);
		pwm_set_chan_level(slice, chan, vol);
	}
#endif

	uint32_t isr = save_and_disable_interrupts();
	uint32_t future = time_us_32() + duration_ms * 1000;
	timer_hw->alarm[BUZZER_TIMER] = future;
	restore_interrupts(isr);
}

void buzzer_init(void)
{
	gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
	slice = pwm_gpio_to_slice_num(BUZZER_PIN);
	chan = pwm_gpio_to_channel(BUZZER_PIN);

	pwm_set_chan_level(slice, chan, 0);
	pwm_set_clkdiv_int_frac(slice, 125, 0);
	pwm_set_wrap(slice, PWM_LENGTH - 1);
	pwm_set_enabled(slice, true);

	hardware_alarm_claim(BUZZER_TIMER);
	irq_set_exclusive_handler(BUZZER_TIMER_IRQ, buzzer_callback);
	hw_set_bits(&timer_hw->inte, 1U << BUZZER_TIMER);
	irq_set_enabled(BUZZER_TIMER_IRQ, true);
}
