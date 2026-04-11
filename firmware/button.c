/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "FreeRTOS.h"

#include "button.h"
#include "computer.h"
#include "hardware.h"

#define SAMPLE_RATE_IN_MS 20

#define HOLD_TIME_SCAN   3000000L   // 3s
#define HOLD_TIME_SWITCH 50000L     // 50ms

static void button_apply(uint64_t duration)
{
	if (duration > HOLD_TIME_SCAN) {
		// TODO implement
		computer_switch(255, true);
	} else if (duration > HOLD_TIME_SWITCH) {
		computer_switch(255, true);
	}
}

void button_task(void *parameters)
{
	bool pressed = false;
	uint64_t press_time = 0;

	while (true) {
		vTaskDelay(SAMPLE_RATE_IN_MS / portTICK_PERIOD_MS);
		if (! gpio_get(SWITCH_PIN)) {
			if (! pressed) {
				pressed = true;
				press_time = time_us_64();
			}
		} else {
			if (pressed) {
				pressed = false;
				button_apply(time_us_64() - press_time);
			}
		}
	}
}
