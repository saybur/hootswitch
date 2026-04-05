/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
