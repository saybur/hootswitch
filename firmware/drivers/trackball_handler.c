/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <string.h>
#include <pico/stdlib.h>

#include "debug.h"
#include "handler.h"
#include "hardware.h"
#include "host.h"
#include "host_err.h"
#include "host_sync.h"
#include "util.h"

#include "trackball.h"
#include "trackball_handler.h"

#define DEFAULT_ADDRESS       3
#define DEFAULT_PRI_HANDLER   0x32
#define REGISTER_2_LEN        7
#define MAX_DEVICES           2

typedef struct {
	uint8_t hdev;
	uint8_t idx;
	uint8_t reg2_native[REGISTER_2_LEN];
	trackball_mode mode;
} trackball;

static trackball devices[MAX_DEVICES];
static uint8_t device_count;

/*
 * ----------------------------------------------------------------------------
 * --- Handler for Real Device ------------------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool hndl_interview(volatile ndev_info *info, bool (*handle_change)(uint8_t, bool))
{
	if (device_count >= MAX_DEVICES) return false;
	if (info->address_def != DEFAULT_ADDRESS) return false;
	if (info->dhid_cur != DEFAULT_PRI_HANDLER) return false;

	// attempt to read vendor-specific register 2 data
	host_err err = HOSTERR_OK;
	uint8_t tmp[8];
	uint8_t tmp_len;
	err = host_sync_cmd(info->hdev, COMMAND_TALK_2, tmp, &tmp_len);
	if (err != HOSTERR_OK || tmp_len != REGISTER_2_LEN) {
		dbg_err("track_h: dev %d bad reg2 err:%d", info->hdev, err);
		return false;
	}

	// setup device data
	trackball *dev = &devices[device_count];
	dev->hdev = info->hdev;

	// determine device type
	if (tmp[2] == 0x52 && tmp[3] == 0x00) {
		dbg("    dev %d detect TM5", info->hdev);
		dev->mode = TRACKBALL_MODE_KENS_TM5;
	} else {
		dbg("    dev %d unknown 0x%2X%2X", info->hdev, tmp[2], tmp[3]);
		// for now treat device as a TM4 for compatibility
		dev->mode = TRACKBALL_MODE_KENS_TM4;
	}

	// store the register 2 response we got from the device for later reference
	memcpy(dev->reg2_native, tmp, REGISTER_2_LEN);

	// activate the device
	// TODO determine how this step varies between native device types!
	tmp[0] = 0xA5;
	tmp[1] = 0x14;
	tmp[2] = 0x00;
	tmp[3] = 0x00;
	tmp[4] = 0x69;
	tmp[5] = 0xFF;
	tmp[6] = 0xFF;
	tmp[7] = 0x27;
	tmp_len = 8;
	err = host_sync_cmd(info->hdev, COMMAND_LISTEN_2, tmp, &tmp_len);
	if (err != HOSTERR_OK) {
		dbg_err("track_h: dev %d can't activate err:%d", info->hdev, err);
		return false;
	}

	// finally register with the driver
	if (trackball_register(&dev->idx, dev->mode)) {
		device_count++;
		return true;
	} else {
		dbg_err("track_h: dev %d registration rejected");
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

	if (reg == 0 && data_len >= 2) {
		// decode incoming data
		int16_t xt, yt;
		uint8_t buttons;
		util_mouse_decode(data, data_len, &xt, &yt, &buttons);

		// then send it
		trackball_push(devices[i].idx, xt, yt, buttons);

		if (data_len >= 3) {
			dbg_trace("track_h (%d): %d %d %d", data_len,
					data[0], data[1], data[2]);
		} else {
			dbg_trace("track_h: %d %d", data[0], data[1]);
		}
	}
}

static ndev_handler trackball_handler = {
	.name = "track",
	.accept_noop_talks = false,
	.interview_func = hndl_interview,
	.talk_func = hndl_talk,
	.listen_func = NULL,
	.flush_func = NULL
};

void trackball_handler_init(void)
{
	handler_register(&trackball_handler);
}
