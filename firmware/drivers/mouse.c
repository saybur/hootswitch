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
	 * An internal representation for the number of right-shifts required to
	 * get the input data from its nominal CPI down to 100cpi for extended mice
	 * operating at DHID 1; this value is ignored if not in extended mode.
	 */
	uint8_t rshift;
	bool pending; // true if motion cache is valid
	int32_t x, y;
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
 * Provides the amount of right-shifting needed to convert from the mouse
 * update native motion format to the level the active computer is using.
 */
static uint8_t mouse_rshift(mouse *mse)
{
	if (active >= COMPUTER_COUNT) return 0;

	if (mse->mode == MOUSE_MODE_EXTENDED
			&& mse->dhi[active] != MOUSE_MODE_EXTENDED) {
		return mse->rshift;
	} else {
		return 0;
	}
}

/*
 * Internal function for offering data to the active computer, either in
 * response to a Talk or when new data has been sent into the system. This
 * must be called only when valid data is present AND when the mouse semaphore
 * is locked!
 */
static void mouse_offer(mouse *mse, uint8_t rshift)
{
	uint8_t data[5];
	util_mouse_encode(data,
			rshift,
			mse->x,
			mse->y,
			mse->buttons);
	uint8_t data_len = mse->dhi[active] == MOUSE_MODE_EXTENDED ? 5 : 2;

	// try to send data, or if send can't be done, store
	if (computer_data_offer(active, mse->drv_idx, 0,
			data, data_len)) {
		mse->pending = false;
		mse->x = 0;
		mse->y = 0;
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
	uint8_t rshift = mouse_rshift(mse);

	if (reg == 0x00 && xSemaphoreTake(mse->sem, portMAX_DELAY)) {
		if (mse->pending) {
			mouse_offer(mse, rshift);
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

bool mouse_update(uint8_t id, int32_t dx, int32_t dy, uint8_t btn)
{
	if (active >= COMPUTER_COUNT) return false;
	if (id >= mouse_count) return false;

	mouse *mse = &mice[id];
	uint8_t rshift = mouse_rshift(mse);
	dbg("mse: x:%d, y:%d, btn:0x%02X, rs:%d", dx, dy, btn, rshift);

	if (xSemaphoreTake(mse->sem, portMAX_DELAY)) {
		// with data locked, update with new values
		mse->x += dx;
		mse->y += dy;
		mse->buttons = btn;

		// send data if possible
		mouse_offer(mse, rshift);

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
			// pick a right-shift value approximately correct for adjusting
			// true CPI down to the 100cpi used when not in extended mode
			uint16_t cpi = (reg1[4] << 8) + reg1[5];
			if (cpi > 9600) {
				mse->rshift = 7;
			} else if (cpi > 4800) {
				mse->rshift = 6;
			} else if (cpi > 2400) {
				mse->rshift = 5;
			} else if (cpi > 1200) {
				mse->rshift = 4;
			} else if (cpi > 600) {
				mse->rshift = 3;
			} else if (cpi > 300) {
				mse->rshift = 2;
			} else if (cpi > 150) {
				mse->rshift = 1;
			} else {
				mse->rshift = 0;
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
