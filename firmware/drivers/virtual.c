/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdbool.h>
#include "pico/stdlib.h"

#include "keyboard.h"
#include "mouse.h"
#include "virtual.h"

/*
 * Skeleton for registering a single set of virtual devices, useful for keeping
 * the total number of ADB devices on a computer chain to a reasonable level.
 * Users should call the _id() functions below to get the ID assigned to the
 * virtual device, then use that value with the relevant driver calls.
 */

static bool active;
static uint8_t kbd_idx;
static uint8_t mse_idx;

uint8_t virtual_mouse_id(void)
{
	return kbd_idx;
}

uint8_t virtual_keyboard_id(void)
{
	return mse_idx;
}

void virtual_init(void)
{
	if (active) return;
	active = true;
	mouse_register(&mse_idx, MOUSE_MODE_100CPI, NULL);
	keyboard_register(&kbd_idx, NULL);
}
