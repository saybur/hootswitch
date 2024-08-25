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
