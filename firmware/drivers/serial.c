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

#include "keyboard.h"
#include "mouse.h"
#include "serial.h"
#include "virtual.h"

/*
 * Driver for sending UART keystrokes via the standard mouse protocol.
 *
 * Bytes >= 0x80 are commands, < 0x80 are data. Commands must be followed by 0
 * to 1 data bytes, which are committed upon receipt. Commands requiring data
 * to follow are aborted if a new command is sent. See serial.h for the command
 * list.
 */

static uint8_t command;
static uint8_t mse_cache[2];

/*
 * ----------------------------------------------------------------------------
 * --- Serial Listener from Host Computer -------------------------------------
 * ----------------------------------------------------------------------------
 */

static void serial_kbd_send(bool up, uint8_t c)
{
	keyboard_message msg;
	msg.length = 2;
	msg.data[0] = (up ? 0x80 : 0x00) | (c & 0x7F);
	msg.data[1] = 0xFF;
	keyboard_enqueue(virtual_keyboard_id(), &msg);
}

static void serial_mse_send()
{
	int16_t x, y;
	uint8_t btn;
	util_mouse_decode(mse_cache, 2, &x, &y, &btn);
	mouse_update(virtual_mouse_id(), x, y, btn);
}

void serial_enqueue(uint8_t c) {
	if (c >= 0x80) {
		command = c;
		switch (command) {
		case SER_CMD_MSE_DOWN:
			mse_cache[0] &= ~0x80;
			serial_mse_send();
			break;
		case SER_CMD_MSE_UP:
			mse_cache[0] |= 0x80;
			serial_mse_send();
			break;
		case SER_CMD_MSE_APPLY:
			serial_mse_send();
			break;
		}
	} else {
		switch (command) {
		case SER_CMD_MSE_X:
			mse_cache[1] = (mse_cache[1] & 0x80) | c;
			break;
		case SER_CMD_MSE_Y:
			mse_cache[0] = (mse_cache[0] & 0x80) | c;
			break;
		case SER_CMD_KBD_DOWN:
			serial_kbd_send(false, c);
			break;
		case SER_CMD_KBD_UP:
			serial_kbd_send(true, c);
			break;
		case SER_CMD_SWITCH:
			computer_switch(c, true);
			break;
		}
	}
}
