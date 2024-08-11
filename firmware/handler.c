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

#include "handler.h"

#include "drivers/keyboard.h"
#include "drivers/mouse.h"

// the full list of possible device handlers
static ndev_handler handler_list[HANDLER_MAX];
static uint8_t handler_list_count;

uint8_t handler_count()
{
	return handler_list_count;
}

bool handler_register(ndev_handler *handler)
{
	if (handler == NULL) {
		return false;
	}
	if (handler_list_count < HANDLER_MAX) {
		handler_list[handler_list_count++] = *handler;
		return true;
	} else {
		return false;
	}
}

bool handler_get(uint8_t id, ndev_handler **handler)
{
	if (id < handler_list_count) {
		*handler = &(handler_list[id]);
		return true;
	} else {
		return false;
	}
}

void handler_init(void)
{
	/*
	 * ------------------------------------------------------------------------
	 * This space available for handlers to tie into the system. Call init code
	 * from here to set up during boot.
	 * ------------------------------------------------------------------------
	 */

	keyboard_init();
	mouse_init();
}

void handler_poll(void)
{
	for (uint8_t i = 0; i < handler_list_count; i++) {
		if (handler_list[i].poll_func != NULL) {
			handler_list[i].poll_func();
		}
	}
}
