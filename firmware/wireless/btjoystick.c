/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>

#include <pico/stdlib.h>

#include "sdkconfig.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "joystick.h"
#include "keyboard.h"
#include "virtual.h"
#include "btjoystick.h"

/*
 * PS5 DualSense:
 *
 * - x/y: left joystick, [-512, 511], left/up negative
 * - rx/ry: right joystick, as above
 * - brake: left shoulder, [0, 1023]
 * - throttle: right shoulder, [0, 1023]
 * - dpad: directional / hat buttons; 0:up, 1: down, 2: right,3: left
 * - buttons: general buttons; 0:X, 1:circle, 2:square, 3: triangle, 4: left
 *            shoulder, 5:right shoulder, 6:unused, 7:unused, 8:left
 *            thumbstick, 9:right thumbstick
 * - misc: special buttons: 0:PS, 1:sparkle(?), 2:hamburger, 3:mute
 */

static uint8_t joy_idx;
static QueueHandle_t report_queue;
static uni_gamepad_t report;
static joystick_data last;

void bt_joystick_set(uni_gamepad_t *new_report)
{
	xQueueOverwrite(report_queue, new_report);
}

bool bt_joystick_waiting(void)
{
	return joystick_waiting(joy_idx);
}

static void bt_joystick_task(void *parameters)
{
	while (true) {
		if (pdPASS != xQueueReceive(report_queue, &report, portMAX_DELAY)) {
			continue;
		}

		if (joystick_enabled(joy_idx)) {
			joystick_data j;

			/*
			 * Docs for uni_gamepad_t say joystick axis is on [-512, 511] but
			 * 512 is sometimes returned; compensate with special case.
			 */
			if (report.axis_x > 511) {
				j.x = 127;
			} else {
				j.x = (((uint16_t)(report.axis_x)) >> 2) & 0xFF;
			}
			if (report.axis_y > 511) {
				j.y = 127;
			} else {
				j.y	= (((uint16_t)(report.axis_y)) >> 2) & 0xFF;
			}
			j.throttle = (((uint16_t)(report.throttle)) >> 2) & 0xFF;
			j.brake = (((uint16_t)(report.brake)) >> 2) & 0xFF;

			// Firebird has these all over the place, see
			// https://github.com/lampmerchant/tashnotes
			// under macintosh/adb/protocols/gravis_firebird.md
			uint32_t b = 0;
			if (report.dpad & 0x4)    b |= 0x1;
			if (report.buttons & 0x1) b |= 0x2;
			if (report.buttons & 0x2) b |= 0x4;
			if (report.dpad & 0x1)    b |= 0x8;
			if (report.dpad & 0x2)    b |= 0x10;
			if (report.buttons & 0x4) b |= 0x20;
			if (report.buttons & 0x8) b |= 0x40;
			if (report.dpad & 0x8)    b |= 0x80;
			if (report.buttons & 0x10) b |= 0x100;
			if (report.buttons & 0x20) b |= 0x200;
			if (report.buttons & 0x40) b |= 0x400;
			if (report.buttons & 0x100) b |= 0x800;
			if (report.buttons & 0x200) b |= 0x1000;
			if (report.misc_buttons & 0x1) b |= 0x2000;
			if (report.misc_buttons & 0x2) b |= 0x4000;
			if (report.misc_buttons & 0x4) b |= 0x8000;
			if (report.misc_buttons & 0x8) b |= 0x10000L;
			j.buttons = ~b;

			/*
			 * At this point check if there is a difference from the last
			 * report; actual Firebird devices seem to only trigger a Talk
			 * update if the joystick has moved.
			 */
			if (last.x == j.x
					&& last.y == j.y
					&& last.brake == j.brake
					&& last.throttle == j.throttle
					&& last.buttons == j.buttons) {
				// no change, veto
			} else {
				last = j;
				joystick_update(joy_idx, &j);
			}
		} else {
			/*
			 * Use the _sequence() call to send a set of keystrokes to the
			 * virtual keyboard. This only covers the low 4 bits of buttons but
			 * should still be useful when having a full Mac driver present is
			 * undesirable.
			 *
			 * See https://github.com/tmk/tmk_keyboard/wiki/Apple-Desktop-Bus
			 * for the keys used below.
			 */
			uint8_t keys[8] = { 0x3E, 0x3D, 0x3C, 0x3B, 0x00, 0x01, 0x02, 0x03 };
			for (uint8_t i = 0; i < 4; i++) {
				uint8_t mask = 1U << i;
				if ((report.dpad & mask) == 0) keys[i] |= 0x80;
				if ((report.buttons & mask) == 0) keys[i+4] |= 0x80;
			}
			keyboard_sequence(virtual_keyboard_index(), keys, sizeof(keys));
		}
	}
}

void bt_joystick_init(void)
{
	joystick_register(&joy_idx, MODE_FIREBIRD);
	report_queue = xQueueCreate(1, sizeof(uni_gamepad_t));
	xTaskCreate(bt_joystick_task,
			"btjoystick",
			configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY, NULL);
}
