/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "hardware.h"
#include "util.h"

#include "serial.h"
#include "virtual.h"

/*
 * ----------------------------------------------------------------------------
 * --- Serial Listener from Host Computer -------------------------------------
 * ----------------------------------------------------------------------------
 */

static void serial_kbd_send(uint8_t *data, uint8_t length)
{
	for (uint8_t i = 1; i < length; i++) {
		virtual_keyboard_offer(data[i] & 0x80, data[i] & 0x7F);
	}
}

static void serial_mse_send(uint8_t *data, uint8_t length)
{
	if (length < 6) return;

	virtual_mouse_data mse;
	mse.buttons = data[1];
	mse.x = (data[2] << 8) + data[3];
	mse.y = (data[4] << 8) + data[5];
	virtual_mouse_offer(&mse);
}

void serial_enqueue(uint8_t *data, uint8_t length) {
	if (!data || length < 1) return;

	switch (data[0]) {
		case SER_CMD_SWITCH:
			if (length >= 2) {
				computer_switch(data[1], true);
			}
			break;
		case SER_CMD_KBD:
			serial_kbd_send(data, length);
			break;
		case SER_CMD_MSE:
			serial_mse_send(data, length);
			break;
	}
}
