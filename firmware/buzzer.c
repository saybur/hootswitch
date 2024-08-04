/*
 * Copyright (C) 2024 saybur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
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

void buzzer_play(uint16_t freq, uint16_t duration_ms, uint8_t vol)
{
	if (duration_ms == 0) return;
	if (freq < 50) freq = FREQ_MIN;
	if (freq > FREQ_MAX) freq = FREQ_MAX;

	float div = clock_get_hz(clk_sys) / (float) (freq * PWM_LENGTH);
	pwm_set_clkdiv(slice, div);
	pwm_set_chan_level(slice, chan, vol);

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
