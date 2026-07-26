/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <string.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <semphr.h>

#include "fixedptc.h"

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "hardware.h"
#include "util.h"

#include "trackball.h"

/*
 * This implements virtual trackballs. At the moment the only emulated devices
 * follow the Kensington protocol. The basic idea is there are two devices
 * presented to the computer, a "primary" with handler 0x32 and a "secondary"
 * that behaves like a regular mouse. Until the Kensington extension loads and
 * configures things the primary does (basically) nothing and the secondary
 * translates trackball behavior as though it was a regular mouse. Once the
 * extension loads the primary starts returning motion data and the secondary
 * goes quiet.
 *
 * This unit copies code extensively from mouse.c, refer there for some
 * additional details.
 */

#define DEFAULT_ADDRESS       3
#define DEFAULT_PRI_HANDLER   0x32
#define DEFAULT_SEC_HANDLER   0x01
#define REGISTER_1_LEN        8
#define REGISTER_2_LEN        7
#define MAX_DEVICES           2

static uint8_t reg2_default_kens_tm4[REGISTER_2_LEN] =
		{ 0x20, 0x09, 0x40, 0x01, 0x14, 0x3B, 0xFF };
static uint8_t reg2_default_kens_tm5[REGISTER_2_LEN] =
		{ 0x25, 0x11, 0x52, 0x00, 0x11, 0xFF, 0xFF };
static uint8_t reg1_default_kens_tm5[REGISTER_1_LEN] =
		{ 0x4B, 0x4D, 0x4C, 0x31, 0x00, 0xC8, 0x02, 0x04 };

// used to prepare trackball motion update parameters
typedef struct {
	bool primary;
	uint8_t drv_idx;
	uint8_t downscale;
	uint8_t length;
} trackball_update;

typedef struct {
	uint8_t drv_idx_pri, drv_idx_sec;
	uint8_t dhi_sec[COMPUTER_COUNT];
	trackball_mode mode;
	uint8_t reg2_flags[COMPUTER_COUNT];

	SemaphoreHandle_t sem;
	bool pending; // true if motion cache is valid
	fixedpt x, y;
	uint8_t buttons; // last seen button data, 0=pressed, 1=released
} trackball;

static volatile uint8_t active = 255;
static trackball devices[MAX_DEVICES];
static uint8_t device_count;

/*
 * ----------------------------------------------------------------------------
 * --- Utility Functions ------------------------------------------------------
 * ----------------------------------------------------------------------------
 */

/*
 * The extension sends the last byte of Listen Register 2 as an error-check
 * value. This performs the check (just the XOR/NOT sum of the previous bytes)
 * and returns true if the values matched. See associated documentation for
 * specifics.
 */
static bool reg2_check(volatile uint8_t* data)
{
	uint8_t c = 0;
	for (uint8_t i = 0; i < REGISTER_2_LEN; i++) {
		c = ~(c ^ data[i]);
	}
	return c == data[REGISTER_2_LEN];
}

/*
 * Saves a Listen Register 2 update into both internal storage (for controlling
 * device behavior) and sets the data for return in the relevant computer
 * storage.
 */
static void reg2_apply(trackball *dev, uint8_t comp, uint8_t* data)
{
	dev->reg2_flags[comp] = data[0];
	computer_data_set(comp, dev->drv_idx_pri, 2,
			data, REGISTER_2_LEN, true);
}

/*
 * True if using the primary (0x32) device, false if using the secondary.
 */
static bool use_primary(trackball *dev, uint8_t comp)
{
	return dev->reg2_flags[comp] & 0x80;
}

/*
 * Returns true if a Talk 0 response to a given computer from a given device is
 * capable of using the extended protocol (3 bytes), false for basic protocol
 * (2 bytes).
 */
static bool extended_supported(trackball *dev, uint8_t comp)
{
	if (use_primary(dev, comp)) {
		switch (dev->mode) {
			case TRACKBALL_MODE_KENS_TM4:
				return false;
			case TRACKBALL_MODE_KENS_TM5:
				return true;
			default:
				dbg_err("track developer error: dev mode %d", dev->mode);
				return false;
		}
	} else {
		// using secondary device, defer to device handler ID
		return dev->dhi_sec[comp] == 0x04;
	}
}

/*
 * Fills the update structure with relevant data for pushing an update, either
 * from internal data in response to a completed Talk command or when a user
 * wants to push new motion data from a real/virtual device. This must only be
 * called when a computer is active!
 */
static bool trackball_prepare_update(trackball *dev, trackball_update *update)
{
	update->primary = use_primary(dev, active);
	update->drv_idx = update->primary ? dev->drv_idx_pri : dev->drv_idx_sec;
	bool extended = extended_supported(dev, active)
			&& (update->primary ? true : dev->dhi_sec[active] == 0x04);
	update->length = (extended || update->primary) ? 3 : 2;
	update->downscale = (!(update->primary) && dev->dhi_sec[active] == 0x01);
}

/*
 * Internal function for offering data to the active computer, either in
 * response to a Talk or when new data has been sent into the system. This
 * must be called only when valid data is present AND when the device semaphore
 * is locked!
 *
 * Provided data array must have 5+ members for util_mouse_encode().
 */
static void trackball_offer(trackball *dev,
		trackball_update *update,
		uint8_t *data)
{
	int32_t x, y;
	fixedpt dx, dy;
	fixedpt ds = fixedpt_fromint(update->downscale);
	if (update->downscale > 1) {
		x = fixedpt_toint(fixedpt_div(dev->x, ds));
		y = fixedpt_toint(fixedpt_div(dev->y, ds));
		dx = fixedpt_mul(fixedpt_fromint(x), ds);
		dy = fixedpt_mul(fixedpt_fromint(y), ds);
	} else {
		x = fixedpt_toint(dev->x);
		y = fixedpt_toint(dev->y);
		dx = fixedpt_fromint(x);
		dy = fixedpt_fromint(y);
	}

	util_mouse_encode(data, x, y, dev->buttons);

	// try to send data, or if send can't be done, store
	if (computer_data_offer(active, update->drv_idx, 0,
			data, update->length)) {
		dev->pending = false;
		dev->x -= dx;
		dev->y -= dy;
	} else {
		dev->pending = true;
	}
}

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Driver ---------------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	trackball *dev = &devices[ref];

	// reset secondary handler and clear register 1 response
	dev->dhi_sec[comp] = DEFAULT_SEC_HANDLER;
	computer_data_set(comp, dev->drv_idx_sec, 1, NULL, 0, false);

	// reset register 1 / 2 data
	switch (dev->mode) {
		case TRACKBALL_MODE_KENS_TM4:
			computer_data_set(comp, dev->drv_idx_pri, 1,
					NULL, 0, true);
			reg2_apply(dev, comp, reg2_default_kens_tm4);
			break;
		case TRACKBALL_MODE_KENS_TM5:
			computer_data_set(comp, dev->drv_idx_pri, 1,
					reg1_default_kens_tm5, REGISTER_1_LEN, true);
			reg2_apply(dev, comp, reg2_default_kens_tm5);
			break;
		default:
			dbg_err("track developer error: dev mode %d", devices[ref].mode);
	}

	/*
	 * Clear motion data if the current computer is in control; if this gets
	 * missed (semaphore locked) not a big deal, don't wait for it.
	 */
	if (active == comp) {
		if (xSemaphoreTake(dev->sem, 0)) {
			dev->pending = false;
			xSemaphoreGive(dev->sem);
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
	trackball *dev = &devices[ref];

	if (use_primary(dev, comp)) {
		// primary is active, do not reply
		*hndl = 0xFF;
	} else {
		// secondary is active, give back handler
		*hndl = dev->dhi_sec[comp];
	}
}

static void drvr_sec_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	trackball *dev = &devices[ref];

	if (hndl == 0x01 || hndl == 0x02) {
		dev->dhi_sec[comp] = hndl;
	} else if (hndl == 0x04 && extended_supported(dev, comp)) {
		// supports extended mode and is being asked to go into it
		dev->dhi_sec[comp] = hndl;

		switch (dev->mode) {
			case TRACKBALL_MODE_KENS_TM5:
				computer_data_set_isr(comp, dev->drv_idx_sec, 1,
						reg1_default_kens_tm5, REGISTER_1_LEN, true);
				break;
		}
	}
}

static void drvr_talk(uint8_t comp, uint32_t ref, uint8_t reg, bool pri)
{
	if (active != comp) return;
	if (reg != 0) return;

	trackball *dev = &devices[ref];
	trackball_update update;
	trackball_prepare_update(dev, &update);

	// veto if asked to provide data on the wrong pri/sec device
	if (use_primary(dev, comp) != update.primary) return;

	// otherwise store results
	uint8_t data[5];
	if (xSemaphoreTake(dev->sem, portMAX_DELAY)) {
		if (dev->pending) {
			trackball_offer(dev, &update, data);
		}
		xSemaphoreGive(dev->sem);
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
	// looks weird but is not a bug, last byte is error checking
	if (data_len != REGISTER_2_LEN + 1) return;
	if (! reg2_check(data)) return;

	trackball *dev = &devices[ref];

	// construct new register 2 data block
	uint8_t reg2[REGISTER_2_LEN];
	reg2[0] = data[0];
	reg2[1] = data[1];
	// next 2 bytes are model information, do not copy from Listen
	switch (dev->mode) {
		case TRACKBALL_MODE_KENS_TM4:
			reg2[2] = reg2_default_kens_tm4[2];
			reg2[3] = reg2_default_kens_tm4[3];
			break;
		case TRACKBALL_MODE_KENS_TM5:
			reg2[2] = reg2_default_kens_tm5[2];
			reg2[3] = reg2_default_kens_tm5[3];
			break;
		default:
			dbg_err("track developer error: dev mode %d", dev->mode);
	}
	reg2[4] = data[4];
	reg2[5] = data[5];
	reg2[6] = data[6];

	// store just the relevant data affecting local device behavior
	reg2_apply(dev, comp, reg2);

	// store the updated register information for talking back
	computer_data_set(comp, dev->drv_idx_pri, 2,
			reg2, REGISTER_2_LEN, true);

	// report messaging
	dbg_trace("track L2 %02X%02X%02X%02X%02X%02X%02X",
			reg2[0], reg2[1], reg2[2], reg2[3], reg2[4], reg2[5], reg2[6]);
}

static dev_driver primary_driver = {
	.default_addr = DEFAULT_ADDRESS,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = drvr_pri_talk,
	.listen_func = drvr_pri_listen,
	.flush_func = NULL, // TODO is issued, uncertain impact on device?
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

bool trackball_register(uint8_t *id, trackball_mode mode)
{
	if (device_count >= MAX_DEVICES) return false;

	*id = device_count++;
	trackball *dev = &devices[*id];

	dev->sem = xSemaphoreCreateMutex();
	assert(dev->sem != NULL);
	dev->mode = mode;
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		dev->dhi_sec[c] = DEFAULT_SEC_HANDLER;
	}
	dev->buttons = 0xFF;

	if (! driver_register(&dev->drv_idx_pri, &primary_driver, *id)) {
		return false;
	}
	if (! driver_register(&dev->drv_idx_sec, &secondary_driver, *id)) {
		dbg_err("track partial driver reg, too many devices!");
		return false;
	}
	return true;
}

bool trackball_push(uint8_t id, int16_t x, int16_t y, uint8_t btn)
{
	if (active >= COMPUTER_COUNT) return false;
	if (id >= device_count) return false;

	fixedpt dx = fixedpt_fromint(x);
	fixedpt dy = fixedpt_fromint(y);

	trackball *dev = &devices[id];
	trackball_update update;
	trackball_prepare_update(dev, &update);
	uint8_t data[5];

	if (xSemaphoreTake(dev->sem, portMAX_DELAY)) {
		// with data locked, update with new values
		dev->x += dx;
		dev->y += dy;
		dev->buttons = btn;

		// send data if possible
		trackball_offer(dev, &update, data);

		xSemaphoreGive(dev->sem);

		if (dbg_trace_is_enabled()) {
			if (update.primary) {
				if (update.length == 3) {
					dbg_trace("track-pri (tlk): %d %d %d",
							data[0], data[1], data[2]);
				} else if (update.length == 2) {
					dbg_trace("track-pri (tlk): %d %d",
							data[0], data[1]);
				}
			} else {
				if (update.length == 3) {
					dbg_trace("track-sec (tlk): %d %d %d",
							data[0], data[1], data[2]);
				} else if (update.length == 2) {
					dbg_trace("track-sec (tlk): %d %d",
							data[0], data[1]);
				}
			}
		}
		return true;
	} else {
		dbg("track: dropped rpt!");
		return false;
	}
}
