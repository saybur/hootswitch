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

#include "bus.h"
#include "computer.h"
#include "driver.h"
#include "host.h"
#include "hardware.h"

static dev_driver driver_list[DRIVER_MAXIMUM];
static uint8_t driver_list_count;

uint8_t driver_count(void)
{
	return driver_list_count;
}

bool driver_register(uint8_t *dev_id, dev_driver *driver)
{
	if (driver == NULL || dev_id == NULL) {
		return false;
	}

	if (driver_list_count < DRIVER_MAXIMUM) {
		*dev_id = driver_list_count;
		driver_list[driver_list_count++] = *driver;
		return true;
	} else {
		return false;
	}
}

bool driver_get(uint8_t dev, dev_driver **driver)
{
	if (dev < driver_list_count) {
		*driver = &(driver_list[dev]);
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

void driver_poll(void)
{
	for (uint8_t i = 0; i < driver_list_count; i++) {
		driver_list[i].poll_func();
	}
}
