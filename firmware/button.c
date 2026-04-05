/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"

#include "button.h"
#include "computer.h"
#include "hardware.h"

#define SAMPLE_RATE_IN_MS 20

static uint32_t sw_cnt;

void button_task(void *parameters)
{
	const TickType_t delay_time = SAMPLE_RATE_IN_MS / portTICK_PERIOD_MS;

	while (1) {
		vTaskDelay(delay_time);
		if (! gpio_get(SWITCH_PIN)) {
			sw_cnt++;
		} else {
			if (sw_cnt > 3) {
				computer_switch(255, true);
				sw_cnt = 0;
			}
		}
	}
}
