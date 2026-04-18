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

#include "fixedptc.h"

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "hardware.h"
#include "util.h"

#include "mouse.h"

/*
 * Driver (computer-side) for ADB relative motion devices following either the
 * standard (0x01/0x02) or extended (0x04) mouse protocols. See Technote HW01
 * (Space Aliens Ate My Mouse) for protocol details and reference materials.
 *
 * The "classic" protocol uses handlers 0x01 (100cpi) or 0x02 (200cpi). For
 * simplicity only the former is used: if a computer switches the mouse to
 * 200cpi the data from a non-extended mouse is simply scaled when responding.
 *
 * TODO this implementation needs the 100/200 cpi stuff evaluated, I don't
 * think it is right.
 */

#define DEFAULT_ADDRESS 3
#define MAX_MICE 4

typedef struct {
	uint8_t drv_idx;
	uint8_t dhi[COMPUTER_COUNT];
	SemaphoreHandle_t sem;
	mouse_mode mode;
	/*
	 * Division required to scale input from its nominal CPI down to 100cpi for
	 * extended mice operating at DHID 1; this value is ignored if not in
	 * extended mode.
	 */
	uint8_t downscale;
	bool pending; // true if motion cache is valid
	fixedpt x, y;
	uint8_t buttons; // last seen button data, 0=pressed, 1=released
} mouse;

static volatile uint8_t active = 255;
static mouse mice[MAX_MICE];
static uint8_t mouse_count;

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Mouse Driver ---------------------------------------------
 * ----------------------------------------------------------------------------
 */

/**
 * Provides the amount of down-scaling needed to convert from the mouse
 * update native motion format to the level the active computer is using.
 */
static uint8_t mouse_downscale(mouse *mse)
{
	if (active >= COMPUTER_COUNT) return 1;

	if (mse->mode == MOUSE_MODE_EXTENDED
			&& mse->dhi[active] != MOUSE_MODE_EXTENDED) {
		return mse->downscale;
	} else {
		return 1;
	}
}

/*
 * Internal function for offering data to the active computer, either in
 * response to a Talk or when new data has been sent into the system. This
 * must be called only when valid data is present AND when the mouse semaphore
 * is locked!
 */
static void mouse_offer(mouse *mse, uint8_t downscale)
{
	/*
	 * Prior to hitting the semaphore, store both the value to send and the
	 * value to deduct from the registers if successful at sending motion.
	 */
	int32_t x, y;
	fixedpt dx, dy;
	fixedpt ds = fixedpt_fromint(downscale);
	if (downscale > 1) {
		x = fixedpt_toint(fixedpt_div(mse->x, ds));
		y = fixedpt_toint(fixedpt_div(mse->y, ds));
		dx = fixedpt_mul(fixedpt_fromint(x), ds);
		dy = fixedpt_mul(fixedpt_fromint(y), ds);
	} else {
		x = fixedpt_toint(mse->x);
		y = fixedpt_toint(mse->y);
		dx = fixedpt_fromint(x);
		dy = fixedpt_fromint(y);
	}

	uint8_t data[5];
	util_mouse_encode(data, x, y, mse->buttons);
	uint8_t data_len = mse->dhi[active] == MOUSE_MODE_EXTENDED ? 5 : 2;

	// try to send data, or if send can't be done, store
	if (computer_data_offer(active, mse->drv_idx, 0,
			data, data_len)) {
		mse->pending = false;
		mse->x -= dx;
		mse->y -= dy;
	} else {
		mse->pending = true;
	}
}

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Mouse Driver ---------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	if (mice[ref].mode == MOUSE_MODE_EXTENDED) {
		// extended mice start at DHID 0x01 per HW01
		mice[ref].dhi[comp] = MOUSE_MODE_100CPI;
	} else {
		// non-extended mice start at their default CPI
		mice[ref].dhi[comp] = mice[ref].mode;
	}

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

static void drvr_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = mice[ref].dhi[comp];
}

static void drvr_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	// only extended mice are willing to change their device handler ID
	if (hndl == MOUSE_MODE_EXTENDED
			&& mice[ref].mode == MOUSE_MODE_EXTENDED) {
		mice[ref].dhi[comp] = MOUSE_MODE_EXTENDED;
	}
}

static void drvr_talk(uint8_t comp, uint32_t ref, uint8_t reg)
{
	if (active != comp) return;

	mouse *mse = &mice[ref];
	uint8_t downscale = mouse_downscale(mse);

	if (reg == 0x00 && xSemaphoreTake(mse->sem, portMAX_DELAY)) {
		if (mse->pending) {
			mouse_offer(mse, downscale);
		}
		xSemaphoreGive(mse->sem);
	}
}

static dev_driver mouse_driver = {
	.default_addr = DEFAULT_ADDRESS,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = drvr_talk,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_get_handle,
	.set_handle_func = NULL
};

bool mouse_update(uint8_t id, int16_t x, int16_t y, uint8_t btn)
{
	if (active >= COMPUTER_COUNT) return false;
	if (id >= mouse_count) return false;

	fixedpt dx = fixedpt_fromint(x);
	fixedpt dy = fixedpt_fromint(y);

	mouse *mse = &mice[id];
	uint8_t downscale = mouse_downscale(mse);
	dbg_trace("mse: x:%d, y:%d, btn:0x%02X", x, y, btn);

	if (xSemaphoreTake(mse->sem, portMAX_DELAY)) {
		// with data locked, update with new values
		mse->x += dx;
		mse->y += dy;
		mse->buttons = btn;

		// send data if possible
		mouse_offer(mse, downscale);

		xSemaphoreGive(mse->sem);
		return true;
	} else {
		dbg("mse: drop rpt!");
		return false;
	}
}

bool mouse_register(uint8_t *id, mouse_mode mode, uint8_t *reg1)
{
	if (mouse_count >= MAX_MICE) return false;

	*id = mouse_count++;
	mouse *mse = &mice[*id];

	mse->sem = xSemaphoreCreateMutex();
	assert(mse->sem != NULL);
	mse->mode = mode;
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		if (mode == MOUSE_MODE_EXTENDED) {
			mse->dhi[c] = MOUSE_MODE_100CPI;
		} else {
			mse->dhi[c] = mode;
		}
	}
	mse->buttons = 0xFF;

	if (! driver_register(&mse->drv_idx, &mouse_driver, *id)) {
		return false;
	}

	if (mode == MOUSE_MODE_EXTENDED) {
		if (reg1) {
			// pick a scalar value approximately correct for adjusting
			// true CPI down to the 100cpi used when not in extended mode
			// use powers of 2 to reduce division complexity later
			uint16_t cpi = (reg1[4] << 8) + reg1[5];
			if (cpi > 9600) {
				mse->downscale = 128;
			} else if (cpi > 4800) {
				mse->downscale = 64;
			} else if (cpi > 2400) {
				mse->downscale = 32;
			} else if (cpi > 1200) {
				mse->downscale = 16;
			} else if (cpi > 600) {
				mse->downscale = 8;
			} else if (cpi > 300) {
				mse->downscale = 4;
			} else if (cpi > 150) {
				mse->downscale = 2;
			} else {
				mse->downscale = 1;
			}

			for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
				computer_data_set(c, mse->drv_idx, 1, reg1, 8, true);
			}
		} else {
			// no reg1, revert to basic mode per contract
			mse->mode = MOUSE_MODE_100CPI;
		}
	}
}
