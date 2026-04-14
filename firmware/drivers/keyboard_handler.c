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
#include "hardware.h"
#include "host.h"
#include "host_err.h"

#include "keyboard.h"
#include "keyboard_handler.h"

/*
 * Handler (real ADB device) implementation for keyboards following the
 * standard keyboard protocol.
 */

// sets the most number of passthru keyboards permitted
#define MAX_KEYBOARDS 3

typedef struct {
	uint8_t hdev;
	uint8_t idx;
	bool extended;
} keyboard;

static keyboard keyboards[MAX_KEYBOARDS];
static uint8_t keyboard_count;

/*
 * ----------------------------------------------------------------------------
 * --- Utility Functions ------------------------------------------------------
 * ----------------------------------------------------------------------------
 */

/*
 * Sends a blind Listen Register 2 to the keyboard with the given handler ID,
 * updating the low 3 bits of Register 2 with new LED information.
 */
static void send_host_reg2(uint8_t idx, uint16_t reg2)
{
	uint32_t id;
	uint8_t send[2];
	send[0] = (reg2 >> 8) & 0xFF;
	send[1] = reg2 & 0xFF;
	host_cmd(keyboards[idx].hdev, COMMAND_LISTEN_2, &id, send, 2);
}

/*
 * ----------------------------------------------------------------------------
 * --- Handler for Real ADB Keyboards -----------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool hndl_interview(volatile ndev_info *info, bool (*handle_change)(uint8_t, bool))
{
	if (keyboard_count >= MAX_KEYBOARDS) return false;
	if (info->address_def != 0x02) return false;

	dbg("    kdb assoc to %d at $%X", info->hdev, info->address_cur);
	keyboard *kbd = &keyboards[keyboard_count];
	kbd->hdev = info->hdev;
	kbd->extended = handle_change(0x03, true);

	if (keyboard_register(&kbd->idx, send_host_reg2)) {
		keyboard_count++;
		return true;
	} else {
		return true;
	}
}

static void hndl_talk(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	uint8_t i;
	for (i = 0; i < keyboard_count; i++) {
		if (keyboards[i].hdev == hdev) break;
	}
	if (i == keyboard_count) return;

	if (data_len >= 2) {
		dbg_trace("kbd_h: %d %d", data[0], data[1]);

		// enqueue data, dropping if queue is full
		keyboard_message kb;
		kb.length = 2;
		kb.data[0] = data[0];
		kb.data[1] = data[1];
		keyboard_enqueue(keyboards[i].idx, &kb);
	}
}

static ndev_handler keyboard_handler = {
	.name = "kbd",
	.accept_noop_talks = false,
	.interview_func = hndl_interview,
	.talk_func = hndl_talk,
	.listen_func = NULL,
	.flush_func = NULL,
};

void keyboard_handler_init(void)
{
	handler_register(&keyboard_handler);
}
