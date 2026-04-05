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
#define DEFAULT_HANDLER 1
#define EXTENDED_HANDLER 4
#define MAX_MICE 4

typedef struct {
	uint8_t drv_idx;
	uint8_t dhi[COMPUTER_COUNT];
	SemaphoreHandle_t sem;
	bool extended_ok;  // true if extended (0x04) DHID should be allowed
	bool pending;      // true if motion cache is valid
	int16_t x, y;      // accumulated X/Y movement data
	uint8_t buttons;   // last seen button data, 0=pressed, 1=released
} mouse;

static volatile uint8_t active = 255;
static mouse mice[MAX_MICE];
static uint8_t mouse_count;

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Mouse Driver ---------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	mice[ref].dhi[comp] = DEFAULT_HANDLER;

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
	if (hndl == EXTENDED_HANDLER && mice[ref].extended_ok) {
		mice[ref].dhi[comp] = hndl;
	}
	if (hndl == 0x01 || hndl == 0x02) {
		mice[ref].dhi[comp] = hndl;
	}
}

static void drvr_talk(uint8_t comp, uint32_t ref, uint8_t reg)
{
	if (active != comp) return;

	mouse *mse = &mice[ref];

	if (reg == 0x00 && xSemaphoreTake(mse->sem, portMAX_DELAY)) {
		if (mse->pending) {
			uint8_t data[5];
			util_mouse_encode(data, mse->x, mse->y, mse->buttons);
			uint8_t len = (mse->dhi[comp] == EXTENDED_HANDLER ? 5 : 2);
			if (computer_data_offer(active, mse->drv_idx, 0, data, len)) {
				mse->pending = false;
				mse->x = 0;
				mse->y = 0;
			}
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

bool mouse_update(uint8_t id, int16_t dx, int16_t dy, uint8_t btn)
{
	if (active >= COMPUTER_COUNT) return false;
	if (id >= mouse_count) return false;

	dbg("mse: x:%d, y:%d, btn:0x%02X", dx, dy, btn);

	mouse *mse = &mice[id];

	if (xSemaphoreTake(mse->sem, portMAX_DELAY)) {
		mse->x += dx;
		mse->y += dy;
		mse->buttons = btn;

		// encode the resulting output
		uint8_t data_out[5];
		util_mouse_encode(data_out, mse->x, mse->y, mse->buttons);
		uint8_t data_out_len = mse->dhi[active] == EXTENDED_HANDLER ? 5 : 2;

		// try to send data, or if send can't be done, store
		if (computer_data_offer(active, mse->drv_idx, 0,
				data_out, data_out_len)) {
			mse->pending = false;
			mse->x = 0;
			mse->y = 0;
		} else {
			mse->pending = true;
		}

		xSemaphoreGive(mse->sem);
		return true;
	} else {
		dbg("mse: drop rpt!");
		return false;
	}
}

bool mouse_register(uint8_t *id, uint8_t *reg1)
{
	if (mouse_count >= MAX_MICE) return false;

	*id = mouse_count++;
	mouse *mse = &mice[*id];

	mse->sem = xSemaphoreCreateMutex();
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		mse->dhi[c] = DEFAULT_HANDLER;
	}
	assert(mse->sem != NULL);
	mse->buttons = 0xFF;

	if (! driver_register(&mse->drv_idx, &mouse_driver, *id)) {
		return false;
	}

	if (reg1) {
		mse->extended_ok = true;
		for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
			computer_data_set(c, mse->drv_idx, 1, reg1, 8, true);
		}
	}
}
