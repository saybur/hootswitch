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

#include <stdbool.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "tusb.h"

void usb_dev_init()
{
	tud_init(BOARD_TUD_RHPORT);
	stdio_set_driver_enabled(&stdio_usb, true);
}

void usb_dev_task(__unused void *parameters)
{
	while (true) {
		do {
			tud_task();
		} while (tud_cdc_write_flush());

		// affected by https://github.com/raspberrypi/pico-sdk/issues/1326
		// below works around issue by introducing explicit delay
		// remove this hack once there are separate _freertos libraries, maybe
		// with https://github.com/raspberrypi/pico-sdk/pull/1438
		vTaskDelay(1);
	}
}
