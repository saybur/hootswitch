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

#include "computer.h"
#include "driver.h"
#include "hardware.h"

static dev_driver *device_list[DEVICE_MAX];
static uint8_t device_list_count;

uint8_t driver_count_devices(void)
{
	return device_list_count;
}

bool driver_register(uint8_t *dev_id, dev_driver *driver)
{
	if (driver == NULL) {
		return false;
	}

	if (device_list_count < DEVICE_MAX) {
		if (dev_id != NULL) {
			*dev_id = device_list_count;
		}
		device_list[device_list_count++] = driver;
		return true;
	} else {
		return false;
	}
}

bool driver_get(uint8_t dev_id, dev_driver **driver)
{
	if (dev_id < device_list_count) {
		*driver = device_list[dev_id];
		return true;
	} else {
		return false;
	}
}

void driver_init(void)
{
	/*
	 * ------------------------------------------------------------------------
	 * This space available for drivers to tie into the system. Call init code
	 * from here to set up during boot.
	 * ------------------------------------------------------------------------
	 */
}

void driver_task(__unused void *parameters)
{
	while (true) {
		for (uint8_t i = 0; i < device_list_count; i++) {
			if (device_list[i]->poll_func) {
				device_list[i]->poll_func();
			}
		}
	}
}
