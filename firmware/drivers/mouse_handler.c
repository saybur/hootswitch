/*
 * Copyright (C) 2024-2026 saybur
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

#include <string.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "debug.h"
#include "handler.h"
#include "hardware.h"
#include "host.h"
#include "host_err.h"
#include "host_sync.h"
#include "util.h"

#include "mouse.h"
#include "mouse_handler.h"

/*
 * Mouse driver for ADB relative motion devices following either the standard
 * (0x01/0x02) or extended (0x04) mouse protocols. See Technote HW01 (Space
 * Aliens Ate My Mouse) for protocol details and reference materials.
 *
 * The "classic" protocol uses handlers 0x01 (100cpi) or 0x02 (200cpi). For
 * simplicity only the former is used: if a computer switches the mouse to
 * 200cpi the data from a non-extended mouse is simply scaled when responding.
 */

#define DEFAULT_ADDRESS 3
#define DEFAULT_HANDLER 1

// sets the most number of passthru mice permitted
#define MAX_MICE 3

typedef struct {
	uint8_t hdev;
	uint8_t idx;
	uint8_t dhi[COMPUTER_COUNT];
	bool extended;     // true if hardware mouse is in extended (0x04) mode
} mouse;

static mouse mice[MAX_MICE];
static uint8_t mouse_count;

/*
 * ----------------------------------------------------------------------------
 * --- Handler for Real ADB Mice ----------------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool hndl_interview(volatile ndev_info *info, bool (*handle_change)(uint8_t, bool))
{
	if (mouse_count >= MAX_MICE) return false;
	if (info->address_def != DEFAULT_ADDRESS) return false;

	// probe mouse for a general Talk 3 response
	// if nothing returned it's likely a disabled Kensington secondary device
	host_err err;
	uint8_t dev_reg[8];
	uint8_t dev_reg_len;
	if (err = host_sync_cmd(info->hdev, COMMAND_TALK_3, dev_reg, &dev_reg_len)) {
		dbg("    id %d t3 resp %d, skip", info->hdev, err);
		return false;
	}

	// past this point we assume adoption unless it errors out
	mouse *mse = &mice[mouse_count];
	mse->hdev = info->hdev;

	// try to change the device to the extended mouse protocol
	uint8_t *reg1 = NULL;
	if (handle_change(0x04, true)) {
		// read and store register 1
		uint8_t dev_reg1[8];
		uint8_t dev_reg1_len;
		host_sync_cmd(info->hdev, COMMAND_TALK_1, dev_reg1, &dev_reg1_len);
		if (dev_reg1_len == 8) {
			reg1 = dev_reg1;
			mse->extended = true;
		} else {
			// not valid extended response, reset to original handler
			dbg_err("mse: dev %d dhid 4 bad reg1", info->hdev);
			handle_change(info->dhid_def, true);
		}
	}

	// make sure device is in a valid mode
	if (! (info->dhid_cur == 0x01
			|| info->dhid_cur == 0x02
			|| info->dhid_cur == 0x04)) {
		// already tried extended, move to basic protocol
		if (! handle_change(0x01, true)) {
			// failed to accept, must not be a mouse?
			dbg_err("mse: dev %d reject dhid 1, dropped", info->hdev);
			return false;
		}
	}

	// for emulation, start at device handler 1 until changed
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		mse->dhi[c] = DEFAULT_HANDLER;
	}

	// finally register with the mouse driver
	mouse_register(&mse->idx, reg1);
	mouse_count++;

	return true;
}

static void hndl_talk(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	// select the correct device mapping
	uint8_t i;
	for (i = 0; i < mouse_count; i++) {
		if (mice[i].hdev == hdev) break;
	}
	if (i == mouse_count) return;

	if (reg == 0 && data_len >= 2) {
		// decode incoming data from the mouse
		int16_t xt, yt;
		uint8_t buttons;
		util_mouse_decode(data, data_len, &xt, &yt, &buttons);

		// then send it
		mouse_update(mice[i].idx, xt, yt, buttons);

		dbg("mse_h: %d %d", data[0], data[1]);
	}
}

static ndev_handler mouse_handler = {
	.name = "mse",
	.accept_noop_talks = false,
	.interview_func = hndl_interview,
	.talk_func = hndl_talk,
	.listen_func = NULL,
	.flush_func = NULL
};

void mouse_handler_init(void)
{
	handler_register(&mouse_handler);
}
