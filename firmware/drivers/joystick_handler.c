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

#include "debug.h"
#include "handler.h"
#include "hardware.h"
#include "host.h"
#include "host_err.h"
#include "host_sync.h"

#include "joystick.h"
#include "joystick_handler.h"

/*
 * Driver for physical ADB joysticks following the Gravis protocol.
 */

#define DEFAULT_ADDRESS 3
#define MAX_DEVICES 2

typedef struct {
	uint8_t hdev;
	uint8_t idx;
	joystick_mode mode;
} joystick;

static joystick devices[MAX_DEVICES];
static uint8_t device_count;

/*
 * ----------------------------------------------------------------------------
 * --- Handler for the Physical Device ----------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool hndl_interview(volatile ndev_info *info, bool (*handle_change)(uint8_t, bool))
{
	if (device_count >= MAX_DEVICES) return false;
	if (info->address_def != DEFAULT_ADDRESS) return false;

	// try supported device handlers
	uint8_t handle = 0;
	if (handle_change(MODE_FIREBIRD, true)) {
		// Firebird/Blackhawk
		handle = MODE_FIREBIRD;
	} else if (handle_change(MODE_MOUSESTICK, true)) {
		// Mousestick II
		handle = MODE_MOUSESTICK;
	} else {
		// unsupported device
		return false;
	}

	// setup storage
	joystick *dev = &devices[device_count];
	dev->hdev = info->hdev;

	// read register 1 to determine specific model info
	uint8_t dev_reg1[8];
	uint8_t dev_reg1_len;
	host_sync_cmd(info->hdev, COMMAND_TALK_1, dev_reg1, &dev_reg1_len);
	switch (handle) {
		case MODE_FIREBIRD:
			if (dev_reg1_len == 3) {
				// allow all variants reported by device
				dev->mode = MODE_FIREBIRD;
			} else {
				// not valid extended response, reset to original handler
				dbg_err("gjoy: dev %d dhid 0x4e bad reg1", info->hdev);
				handle_change(info->dhid_def, true);
				return false;
			}
			break;

		case MODE_MOUSESTICK:
			if (dev_reg1_len == 2 && dev_reg1[0] == 0x04) {
				// only support 0x0400 for 3-byte for the moment
				dev->mode = MODE_MOUSESTICK;
			} else {
				// not valid extended response, reset to original handler
				dbg_err("gjoy: dev %d dhid 0x23 bad reg1", info->hdev);
				handle_change(info->dhid_def, true);
				return false;
			}
			break;

		default:
			dbg_err("gjoy: coding error reg1 %d", handle);
			return false;
	}

	if (joystick_register(&dev->idx, dev->mode)) {
		device_count++;
		return true;
	} else {
		dbg_err("gjoy: too many joysticks!");
		handle_change(info->dhid_def, true);
		return false;
	}
}

static void hndl_talk(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	// select the correct device mapping
	uint8_t i;
	for (i = 0; i < device_count; i++) {
		if (devices[i].hdev == hdev) break;
	}
	if (i == device_count) return;

	// we only use register 0
	if (reg != 0) {
		dbg_err("gjoy: reg %d, ignoring", reg);
		return;
	}

	// remap data to the intermediate format
	joystick_data jdata;
	switch (devices[i].mode) {
		case MODE_FIREBIRD:
			if (data_len != 8) {
				dbg_err("gjoy: r0 len != 8, %d", data_len);
				return;
			}
			jdata.buttons = 0xFF000000L
					| ((uint32_t) (data[0] << 16))
					| ((uint32_t) (data[1] << 8))
					| ((uint32_t) data[2]);
			jdata.x1 = ((int16_t) data[3]) - 0x80;
			jdata.y1 = ((int16_t) data[4]) - 0x80;
			jdata.y2 = ((int16_t) data[5]) - 0x80;
			jdata.x2 = ((int16_t) data[6]) - 0x80;
			break;

		case MODE_MOUSESTICK:
			// TODO this needs to be reworked for 0x0300 support in the future
			if (data_len != 3) {
				dbg_err("gjoy: r0 len != 3, %d", data_len);
				return;
			}
			jdata.x1 = ((int16_t) data[0]) - 0x80;
			jdata.y1 = ((int16_t) data[1]) - 0x80;
			jdata.buttons = 0xFFFFFF00L
					| ((uint32_t) data[3]);
			break;

		default:
			// we don't run any joysticks in mouse mode, this is a bug!
			dbg_err("gjoy: r0 talk bug");
			return;
	}

	joystick_update(devices[i].idx, &jdata);
}

static ndev_handler joystick_handler = {
	.name = "gjoy",
	.accept_noop_talks = false,
	.interview_func = hndl_interview,
	.talk_func = hndl_talk,
	.listen_func = NULL,
	.flush_func = NULL
};

void joystick_handler_init(void)
{
	handler_register(&joystick_handler);
}
