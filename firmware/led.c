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

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "hardware.h"
#include "led.h"

#define LED_C_COUNT 4

typedef struct {
	uint8_t gpio;
	uint8_t slice;
	uint8_t chan;
} led_c;
static led_c leds[LED_C_COUNT];

void led_board(bool state)
{
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
}

void led_activity(bool state)
{
	gpio_put(LED_ACT_PIN, state);
}

void led_error(bool state)
{
	gpio_put(LED_ERR_PIN, state);
}

void led_machine(uint8_t mach, uint8_t level)
{
	if (mach >= LED_C_COUNT) return;
	pwm_set_chan_level(leds[mach].slice, leds[mach].chan, level);
}

void led_init(void)
{
	assert(LED_C_COUNT == 4);

	gpio_init(LED_ERR_PIN);
	gpio_set_dir(LED_ERR_PIN, GPIO_OUT);
	gpio_init(LED_ACT_PIN);
	gpio_set_dir(LED_ACT_PIN, GPIO_OUT);

	leds[0].gpio = LED_C1_PIN;
	leds[1].gpio = LED_C2_PIN;
	leds[2].gpio = LED_C3_PIN;
	leds[3].gpio = LED_C4_PIN;

	for (uint8_t i = 0; i < LED_C_COUNT; i++) {
		gpio_set_function(leds[i].gpio, GPIO_FUNC_PWM);
		leds[i].slice = pwm_gpio_to_slice_num(leds[i].gpio);
		leds[i].chan = pwm_gpio_to_channel(leds[i].gpio);

		pwm_set_wrap(leds[i].slice, 0xFF);
		pwm_set_chan_level(leds[i].slice, leds[i].chan, 0);
		pwm_set_enabled(leds[i].slice, true);
	}
}
