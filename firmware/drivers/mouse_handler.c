/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
	mouse_mode mode;
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
	if (handle_change(MOUSE_MODE_EXTENDED, true)) {
		// read and store register 1
		uint8_t dev_reg1[8];
		uint8_t dev_reg1_len;
		host_sync_cmd(info->hdev, COMMAND_TALK_1, dev_reg1, &dev_reg1_len);
		if (dev_reg1_len == 8) {
			reg1 = dev_reg1;
			mse->mode = MOUSE_MODE_EXTENDED;
		} else {
			// not valid extended response, reset to original handler and
			// drop the device, it likely (?) is not a standard mouse
			dbg_err("mse: dev %d dhid 4 bad reg1, dropped", info->hdev);
			handle_change(info->dhid_def, true);
			return false;
		}
	} else if (handle_change(MOUSE_MODE_200CPI, true)) {
		mse->mode = MOUSE_MODE_200CPI;
	} else if (handle_change(MOUSE_MODE_100CPI, true)) {
		mse->mode = MOUSE_MODE_100CPI;
	} else {
		dbg_err("mse: dev %d reject dhid 1, dropped", info->hdev);
		return false;
	}

	// finally register with the mouse driver
	mouse_register(&mse->idx, mse->mode, reg1);
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

		dbg_trace("mse_h: %d %d", data[0], data[1]);
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
