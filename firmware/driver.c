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
#include "device.h"
#include "driver.h"
#include "host.h"
#include "hardware.h"

static volatile dev_driver drivers[DRIVER_MAXIMUM];
static volatile uint8_t driver_count;

uint8_t driver_register(uint8_t *dev_id, dev_driver *driver)
{
	if (driver == NULL || dev_id == NULL) {
		return 2;
	}

	if (driver_count < DRIVER_MAXIMUM) {
		*dev_id = driver_count;
		drivers[driver_count++] = *driver;
		return 0;
	} else {
		return 1;
	}
}

void driver_get(uint8_t dev, dev_driver *driver)
{
	if (dev < driver_count) {
		*driver = drivers[dev];
	} else {
		driver = NULL;
	}
}

void driver_poll(void)
{
	for (uint8_t i = 0; i < driver_count; i++) {
		drivers[i].poll();
	}
}
