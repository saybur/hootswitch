/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "pico/stdlib.h"

#include "debug.h"
#include "handler.h"

#include "drivers/joystick_handler.h"
#include "drivers/keyboard_handler.h"
#include "drivers/mouse_handler.h"
#include "drivers/trackball_handler.h"

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
		dbg_err("rej null hndl!");
		return false;
	}
	if (handler_list_count < HANDLER_MAX) {
		dbg("reg handler '%s'", handler->name);
		handler_list[handler_list_count++] = *handler;
		return true;
	} else {
		dbg_err("hdnl cnt!");
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
	 * -----------------------------------------------------------------------
	 *
	 * This space available for handlers to tie into the system. Call init
	 * code from here to set up during boot.
	 *
	 * Devices are interviewed in reverse order of handler registration. Put
	 * specialty handlers at the bottom of the list and the generic handlers
	 * at the top.
	 *
	 * -----------------------------------------------------------------------
	 */

	// generic handlers
	keyboard_handler_init();
	mouse_handler_init();

	// specialty handlers
	trackball_handler_init();
	joystick_handler_init();
}
