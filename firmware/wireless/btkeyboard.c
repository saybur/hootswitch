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

#include "debug.h"
#include "keymap.h"
#include "keyboard.h"
#include "virtual.h"

#include "btkeyboard.h"

/*
 * bluepad32 has these as a separate bitmask:
 *
 * 0/4: Left/Right Control
 * 1/5: Left/Right Shift
 * 2/6: Left/Right Alt (Command)
 * 3/7: Left/Right Meta (Option)
 */
const static uint8_t modifier_keys[8] = {
	0x36, 0x38, 0x37, 0x3A, 0x7D, 0x7B, 0x37, 0x7C
};

// leave lots of spare room for huge changes and modifier keys
static uint8_t last_keys[UNI_KEYBOARD_PRESSED_KEYS_MAX];
static uint8_t last_key_count;
static uint8_t last_modifier;

static void send_key(uint8_t code, bool up)
{
	static keyboard_message m = {
		.data[0] = 0xFF,
		.data[1] = 0xFF,
		.length = 2
	};

	uint8_t c;
	if (code == 0xFF) {
		// hint to send a partially enqueued report, if one is present
		if (m.data[0] != 0xFF) {
			keyboard_enqueue(virtual_keyboard_index(), &m);
			m.data[0] = 0xFF;
		}
		return;
	} else if (code >= 0xE0) {
		// meta-key, look up via the internal array using low bits only
		c = modifier_keys[code & 0xF];
	} else {
		// regular key, look up in keymap
		c = keymap_decode(code);
		if (c == 0xFF) return;
	}

	if (up) c |= 0x80;

	// set in the appropriate slot of the keyboard report
	if (c == 0x7F || c == 0xFF) {
		// special case power button
		if (m.data[0] != 0xFF) {
			// need space, shove the partially filled item into the queue
			keyboard_enqueue(virtual_keyboard_index(), &m);
		}
		// then send the power button 2x as required
		m.data[0] = c;
		m.data[1] = c;
		keyboard_enqueue(virtual_keyboard_index(), &m);
		m.data[0] = 0xFF;
		m.data[1] = 0xFF;
	} else {
		if (m.data[0] == 0xFF) {
			m.data[0] = c;
		} else if (m.data[1] == 0xFF) {
			m.data[1] = c;
			keyboard_enqueue(virtual_keyboard_index(), &m);
			m.data[0] = 0xFF;
			m.data[1] = 0xFF;
		}
	}
}

/*
 * Insertion sort over a set of key updates; these arrays can be up to
 * UNI_KEYBOARD_PRESSED_KEYS_MAX long but are usually much shorter, so this
 * short-circuits once the first 0x00 non-update is found. Returns the number
 * of valid keycodes in the array.
 */
static uint8_t sort_keys(uint8_t *a)
{
	// don't bother sorting at all if there isn't anything in the array
	if (a[0] == 0) return 0;

	// perform insertion sort
	uint8_t i = 1;
	uint8_t j, tmp;
	while (i < UNI_KEYBOARD_PRESSED_KEYS_MAX) {
		if (a[i] == 0) {
			return i; // end of key updates
		}
		j = i;
		while (j > 0 && a[j-1] > a[j]) {
			tmp = a[j-1];
			a[j-1] = a[j];
			a[j] = tmp;
			j--;
		}
		i++;
	}
	return UNI_KEYBOARD_PRESSED_KEYS_MAX;
}

void bt_keyboard_update(uni_keyboard_t *report)
{
	/*
	 * Bluepad32 remaps the 0xE0 usage IDs into a bitmask, handle those first.
	 */
	uint8_t i;
	uint8_t meta = last_modifier ^ report->modifiers;
	if (meta) {
		for (uint8_t i = 0; i < 8; i++) {
			if (meta & (1U << i)) {
				if (last_modifier & (1U << i)) {
					send_key(0xE0 + i, true);
				} else {
					send_key(0xE0 + i, false);
				}
			}
		}
		last_modifier = report->modifiers;
	}

	/*
	 * Only key-down events are sent, so we need to track which keys were
	 * pressed last time and send key-up events to the Mac(s) appropriately.
	 * Sort the array first to limit the later brute-force check.
	 */
	uint8_t *new_keys = report->pressed_keys;
	uint8_t new_key_count = sort_keys(new_keys);

	/*
	 * Perform a array comparison on the two sorted arrays, hunting for
	 * differences. Values in the new array that aren't in the old one are
	 * key-downs, and values in the old array that aren't in the new one are
	 * key-ups.
	 */
	uint8_t n = 0;
	uint8_t l = 0;
	while (l < last_key_count && n < new_key_count) {
		if (last_keys[l] < new_keys[n]) {
			// a previous key has been released
			send_key(last_keys[l++], true);
		} else if (last_keys[l] > new_keys[n]) {
			// new key has been pressed
			send_key(new_keys[n++], false);
		} else {
			// no change
			l++; n++;
		}
	}
	while (l < last_key_count) {
		// any remaining are key-up events
		send_key(last_keys[l++], true);
	}
	while (n < new_key_count) {
		// any remaining are key-down events
		send_key(new_keys[n++], false);
	}

	// save new keys for the next run
	if (new_key_count > 0) {
		memcpy(last_keys, new_keys, new_key_count);
	}
	last_key_count = new_key_count;

	// send a pending single key if present before being done
	send_key(0xFF, false);
}
