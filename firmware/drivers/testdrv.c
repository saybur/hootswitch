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

#include "../debug.h"
#include "../driver.h"

static void testdrv_reset(uint8_t comp, uint8_t device)
{
	// do nothing
}

static void testdrv_get_handle(uint8_t comp, uint8_t device, uint8_t *hndl)
{
	*hndl = 0x01;
}

static dev_driver drvr = {
	.default_addr = 0x02,
	.reset_func = testdrv_reset,
	.switch_func = NULL,
	.talk_func = NULL,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = testdrv_get_handle,
	.set_handle_func = NULL,
	.poll_func = NULL
};

void testdrv_init(void)
{
	driver_register(NULL, &drvr);
}
