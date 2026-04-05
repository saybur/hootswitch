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

#include "computer.h"
#include "config.h"
#include "debug.h"
#include "driver.h"
#include "handler.h"
#include "hardware.h"
#include "host.h"
#include "host_err.h"
#include "host_sync.h"
#include "util.h"

#include "kensington.h"

#define DEFAULT_ADDRESS       3
#define DEFAULT_PRI_HANDLER   0x32
#define DEFAULT_SEC_HANDLER   0x01
#define REGISTER_1_LEN        8
#define REGISTER_2_LEN        7

// sets the most number of passthru mice permitted
#define MAX_DEVICES 2

typedef enum {
	EMULATE_NATIVE = 0,
	EMULATE_LEGACY,      // Turbo Mouse 4.0
	EMULATE_EXTENDED     // Turbo Mouse 5.0
} emulation;

static uint8_t emul_legacy_reg2[REGISTER_2_LEN] =
		{ 0x20, 0x09, 0x40, 0x01, 0x14, 0x3B, 0xFF };
static uint8_t emul_extended_reg2[REGISTER_2_LEN] =
		{ 0x25, 0x11, 0x52, 0x00, 0x11, 0xFF, 0xFF };
static uint8_t emul_extended_reg1[REGISTER_1_LEN] =
		{ 0x4B, 0x4D, 0x4C, 0x31, 0x00, 0xC8, 0x02, 0x04 };

typedef struct {
	uint8_t hdev;
	uint8_t drv_idx_pri, drv_idx_sec;
	uint8_t dhi[COMPUTER_COUNT];
	SemaphoreHandle_t sem;
	bool pending;      // true if motion cache is valid
	int16_t x, y;      // accumulated X/Y movement data
	uint8_t buttons;   // last seen button data, 0=pressed, 1=released

	// chooses whether a particular 'fake' response type should be generated
	// for each computer or if a native device information should be passed
	emulation mode[COMPUTER_COUNT];
	// native register 1 value read from the device at initialization time;
	// may be set to all-zeroes if the native device had no register for this
	uint8_t reg1_native[REGISTER_1_LEN];
	// native register 2 value read from the device at initialization time
	uint8_t reg2_native[REGISTER_2_LEN];
	// computer-side control flags as set by the remote system
	uint8_t reg2_flags[COMPUTER_COUNT];
} kmouse;

static volatile uint8_t active = 255;
static kmouse mice[MAX_DEVICES];
static uint8_t device_count;

/*
 * ----------------------------------------------------------------------------
 * --- Utility Functions ------------------------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool reg2_check(volatile uint8_t* data)
{
	uint8_t c = 0;
	for (uint8_t i = 0; i < REGISTER_2_LEN; i++) {
		c = ~(c ^ data[i]);
	}
	return c == data[REGISTER_2_LEN];
}

// handles storing register 2 information for altering device behavior
static void reg2_apply(uint8_t comp, uint8_t dev, uint8_t* data)
{
	mice[dev].reg2_flags[comp] = data[0];
}

// returns true if the primary device has been activated
static bool use_primary(uint8_t comp, uint8_t dev)
{
	return mice[dev].reg2_flags[comp] & 0x80;
}

// true if a Talk 0 response to a given computer from a given device is capable
// of using the extended protocol (3 bytes), false for basic protocol (2 bytes)
static bool extended_supported(uint8_t comp, uint8_t dev)
{
	if (use_primary(comp, dev)) {
		// using primary device
		switch (mice[dev].mode[comp]) {
			case EMULATE_EXTENDED:
				return true;
			case EMULATE_LEGACY:
				return false;
			default:
				return mice[dev].reg1_native[0] != 0x00;
		}
	} else {
		// using secondary device, only device handler ID controls
		return mice[dev].dhi[comp] == 0x04;
	}
}

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Driver ---------------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	if (mice[ref].mode[comp] == EMULATE_LEGACY) {
		// set register 2, register 1 does not exist for this device type
		computer_data_set(comp, mice[ref].drv_idx_pri, 1,
				NULL, 0, true);
		computer_data_set(comp, mice[ref].drv_idx_pri, 2,
				emul_legacy_reg2, REGISTER_2_LEN, true);
		reg2_apply(comp, ref, emul_legacy_reg2);
	} else if (mice[ref].mode[comp] == EMULATE_EXTENDED) {
		// set both register 1 and 2
		computer_data_set(comp, mice[ref].drv_idx_pri, 1,
				emul_extended_reg1, REGISTER_1_LEN, true);
		computer_data_set(comp, mice[ref].drv_idx_pri, 2,
				emul_extended_reg2, REGISTER_2_LEN, true);
		reg2_apply(comp, ref, emul_extended_reg2);
	} else {
		// use the native device information to set
		if (mice[ref].reg1_native != 0x00) {
			computer_data_set(comp, mice[ref].drv_idx_pri, 1,
					mice[ref].reg1_native, REGISTER_1_LEN, true);
		} else {
			computer_data_set(comp, mice[ref].drv_idx_pri, 1,
					NULL, 0, true);
		}
		computer_data_set(comp, mice[ref].drv_idx_pri, 2,
				mice[ref].reg2_native, REGISTER_2_LEN, true);
		reg2_apply(comp, ref, mice[ref].reg2_native);
	}

	// reset secondary handler and clear register 1 response
	mice[ref].dhi[comp] = DEFAULT_SEC_HANDLER;
	computer_data_set(comp, mice[ref].drv_idx_sec, 1, NULL, 0, false);

	// clear motion data if the current computer is in control
	if (active == comp) {
		if (xSemaphoreTake(mice[ref].sem, portMAX_DELAY)) {
			mice[ref].pending = false;
			xSemaphoreGive(mice[ref].sem);
		}
	}
}

static void drvr_switch(uint8_t comp)
{
	active = comp;
}

static void drvr_pri_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = DEFAULT_PRI_HANDLER;
}

static void drvr_sec_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	if (use_primary(comp, ref)) {
		// primary is active, do not reply
		*hndl = 0xFF;
	} else {
		// secondary is active, give back handler
		*hndl = mice[ref].dhi[comp];
	}
}

static void drvr_sec_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	if (hndl == 0x01 || hndl == 0x02) {
		mice[ref].dhi[comp] = hndl;
	} else if (hndl == 0x04 && extended_supported(comp, ref)) {
		// supports extended mode and is being asked to go into it
		mice[ref].dhi[comp] = hndl;

		// clone the regular device register 1
		if (mice[ref].mode[comp] == EMULATE_EXTENDED) {
			computer_data_set(comp, mice[ref].drv_idx_sec, 1,
					emul_extended_reg1, REGISTER_1_LEN, true);
		} else {
			// assume native passthrough
			computer_data_set(comp, mice[ref].drv_idx_sec, 1,
					mice[ref].reg1_native, REGISTER_1_LEN, true);
		}
	}
}

static void drvr_talk(uint8_t comp, uint32_t ref, uint8_t reg, bool pri)
{
	if (active != comp) return;
	if (use_primary(comp, ref) != pri) return;

	uint8_t drv_idx = pri ? mice[ref].drv_idx_pri : mice[ref].drv_idx_sec;
	bool extended = extended_supported(comp, ref)
			&& (pri ? true : mice[ref].dhi[comp] == 0x04);

	uint8_t data[5];
	uint8_t len = 0;
	if (reg == 0 && xSemaphoreTake(mice[ref].sem, portMAX_DELAY)) {
		if (mice[ref].pending) {

			util_mouse_encode(data, 0, mice[ref].x, mice[ref].y, mice[ref].buttons);
			len = (extended || pri) ? 3 : 2;
			if (computer_data_offer(active, drv_idx, 0, data, len)) {
				mice[ref].pending = false;
			}
		}
		xSemaphoreGive(mice[ref].sem);

		if (pri) {
			if (len == 3) {
				dbg("kens-pri (tlk): %d %d %d", data[0], data[1], data[2]);
			} else if (len == 2) {
				dbg("kens-pri (tlk): %d %d", data[0], data[1]);
			}
		} else {
			if (len == 3) {
				dbg("kens-sec (tlk): %d %d %d", data[0], data[1], data[2]);
			} else if (len == 2) {
				dbg("kens-sec (tlk): %d %d", data[0], data[1]);
			}
		}
	} else {
		if (pri) {
			dbg("kens-pri (tlk) skip");
		} else {
			dbg("kens-sec (tlk) skip");
		}
	}
}

static void drvr_pri_talk(uint8_t comp, uint32_t ref, uint8_t reg)
{
	drvr_talk(comp, ref, reg, true);
}

static void drvr_sec_talk(uint8_t comp, uint32_t ref, uint8_t reg)
{
	drvr_talk(comp, ref, reg, false);
}

static void drvr_pri_listen(uint8_t comp, uint32_t ref, uint8_t reg,
		volatile uint8_t* data, uint8_t data_len)
{
	if (reg != 2) return;
	if (data_len != 8) return;
	if (! reg2_check(data)) return;

	// construct new register 2 data block
	uint8_t reg2[REGISTER_2_LEN];
	reg2[0] = data[0];
	reg2[1] = data[1];
	switch (mice[ref].mode[comp]) {
		case EMULATE_LEGACY:
			reg2[2] = emul_legacy_reg2[2];
			reg2[3] = emul_legacy_reg2[3];
			break;
		case EMULATE_EXTENDED:
			reg2[2] = emul_extended_reg2[2];
			reg2[3] = emul_extended_reg2[3];
			break;
		default:
			reg2[2] = mice[ref].reg2_native[2];
			reg2[3] = mice[ref].reg2_native[3];
	}
	reg2[4] = data[4];
	reg2[5] = data[5];
	reg2[6] = data[6];

	// store just the relevant data affecting local device behavior
	reg2_apply(comp, ref, reg2);

	// store the updated register information for talking back
	computer_data_set(comp, mice[ref].drv_idx_pri, 2,
			reg2, REGISTER_2_LEN, true);

	// report messaging
	dbg("kens L2 %02X%02X%02X%02X%02X%02X%02X",
			reg2[0], reg2[1], reg2[2], reg2[3], reg2[4], reg2[5], reg2[6]);
}

static dev_driver primary_driver = {
	.default_addr = DEFAULT_ADDRESS,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = drvr_pri_talk,
	.listen_func = drvr_pri_listen,
	.flush_func = NULL, // TODO is issued, uncertain impact on device
	.get_handle_func = drvr_pri_get_handle,
	.set_handle_func = NULL
};

static dev_driver secondary_driver = {
	.default_addr = DEFAULT_ADDRESS,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = drvr_sec_talk,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_sec_get_handle,
	.set_handle_func = drvr_sec_set_handle
};

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

	// attempt to read required register 2 data
	host_err err = HOSTERR_OK;
	uint8_t tmp[8];
	uint8_t tmp_len;
	err = host_sync_cmd(info->hdev, COMMAND_TALK_2, tmp, &tmp_len);
	if (err != HOSTERR_OK || tmp_len != REGISTER_2_LEN) {
		dbg_err("kens: dev %d bad reg2 err:%d", info->hdev, err);
		return false;
	}

	// setup device data
	kmouse *mse = &mice[device_count];
	mse->hdev = info->hdev;
	if (mse->sem == NULL) {
		mse->sem = xSemaphoreCreateMutex();
	}
	assert(mse->sem != NULL);
	mse->buttons = 0xFF;

	// store the register 2 response we got from the device for later reference
	memcpy(mse->reg2_native, tmp, REGISTER_2_LEN);

	// determine device type
	if (tmp[2] == 0x52 && tmp[3] == 0x00) {
		dbg("    dev %d detect TM5", info->hdev);
	} else {
		dbg("    dev %d unknown 0x%2X%2X", info->hdev, tmp[2], tmp[3]);
	}

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
		dbg_err("kens: dev %d can't activate err:%d", info->hdev, err);
		return false;
	}

	// attempt to read register 1 data
	err = host_sync_cmd(info->hdev, COMMAND_TALK_1, tmp, &tmp_len);
	if (err == HOSTERR_OK) {
		if (tmp_len == 8) {
			memcpy(mse->reg1_native, tmp, 8);
		} else {
			dbg_err("kens: dev %d bad reg1 len:", info->hdev, tmp_len);
			return false;
		}
	} else if (err == HOSTERR_TIMEOUT) {
		// expected with older devices
	} else {
		dbg_err("kens: dev %d bad reg1 err:%d", info->hdev, err);
		return false;
	}

	// for secondary emulation, start at device handler 0x01 until changed
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		mse->dhi[c] = DEFAULT_SEC_HANDLER;
	}

	driver_register(&mse->drv_idx_pri, &primary_driver, device_count);
	driver_register(&mse->drv_idx_sec, &secondary_driver, device_count);

	device_count++;
	return true;
}

static void hndl_talk(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	// ignore until there is a computer to send data to
	if (active >= COMPUTER_COUNT) return;

	// select the correct device mapping
	uint8_t i;
	for (i = 0; i < device_count; i++) {
		if (mice[i].hdev == hdev) break;
	}
	if (i == device_count) return;

	if (reg == 0 && data_len >= 2) {
		// decode incoming data
		int16_t xt, yt;
		uint8_t buttons;
		util_mouse_decode(data, data_len, &xt, &yt, &buttons);

		// figure out where to send it and what format is expected
		uint8_t drv_idx;
		uint8_t data_out_len;

		if (use_primary(active, i)) {
			// primary
			drv_idx = mice[i].drv_idx_pri;
			data_out_len = extended_supported(active, i) ? 3 : 3;
		} else {
			// secondary
			drv_idx = mice[i].drv_idx_sec;
			data_out_len = mice[i].dhi[active] == 0x04 ? 3 : 2;
		}

		if (xSemaphoreTake(mice[i].sem, portMAX_DELAY)) {
			// include any pending motion data
			if (mice[i].pending) {
				xt += mice[i].x;
				yt += mice[i].y;
			}

			// encode the resulting output
			uint8_t data_out[5];
			util_mouse_encode(data_out, 0, xt, yt, buttons);

			// try to send data, or if send can't be done, store
			if (computer_data_offer(active, drv_idx, 0,
					data_out, data_out_len)) {
				mice[i].pending = false;
			} else {
				mice[i].pending = true;
				mice[i].x = xt;
				mice[i].y = yt;
				mice[i].buttons = buttons;
			}
			xSemaphoreGive(mice[i].sem);

			if (drv_idx == mice[i].drv_idx_pri) {
				if (data_out_len == 3) {
					dbg("kens-pri: %d %d %d", data[0], data[1], data[2]);
				} else if (data_out_len == 2) {
					dbg("kens-pri: %d %d", data[0], data[1]);
				}
			} else {
				if (data_out_len == 3) {
					dbg("kens-sec: %d %d %d", data[0], data[1], data[2]);
				} else if (data_out_len == 2) {
					dbg("kens-sec: %d %d", data[0], data[1]);
				}
			}
		} else {
			if (drv_idx == mice[i].drv_idx_pri) {
				dbg("kens-pri: skip");
			} else {
				dbg("kens-sec: skip");
			}
		}
	}
}

static ndev_handler kensington_handler = {
	.name = "kens",
	.accept_noop_talks = false,
	.interview_func = hndl_interview,
	.talk_func = hndl_talk,
	.listen_func = NULL,
	.flush_func = NULL
};

void kensington_init(void)
{
	handler_register(&kensington_handler);

	// try to pull device configuration information
	uint8_t config[COMPUTER_COUNT] = {0};
	config_read(CONFIG_OFFSET_KENS, config, COMPUTER_COUNT);

	// inject mode based on configuration
	for (uint8_t j = 0; j < COMPUTER_COUNT; j++) {
		if (config[j] == 0x04) {
			dbg("kens: port %d emulate legacy", j + 1);
			for (uint8_t i = 0; i < MAX_DEVICES; i++) {
				mice[i].mode[j] = EMULATE_LEGACY;
			}
		} else if (config[j] == 0x05) {
			dbg("kens: port %d emulate extended", j + 1);
			for (uint8_t i = 0; i < MAX_DEVICES; i++) {
				mice[i].mode[j] = EMULATE_EXTENDED;
			}
		} else {
			for (uint8_t i = 0; i < MAX_DEVICES; i++) {
				mice[i].mode[j] = EMULATE_NATIVE;
			}
		}
	}
}
