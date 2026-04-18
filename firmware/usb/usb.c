/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
