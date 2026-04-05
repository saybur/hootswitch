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

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "hardware.h"
#include "util.h"

#include "joystick.h"

/*
 * Driver for ADB joysticks following the Gravis protocol. There are two
 * handlers supported: 0x4E (Firebird/Blackhawk) and 0x23 (Mousestick II, using
 * the 3-byte format only). These are described in good detail here:
 *
 * <https://github.com/lampmerchant/tashnotes/tree/main/macintosh/adb/protocols>
 */

#define DEFAULT_ADDRESS 3
#define DEFAULT_HANDLER 1
#define MAX_DEVICES 3

typedef struct {
	uint8_t drv_idx;
	joystick_mode mode;
	uint8_t dhi[COMPUTER_COUNT];
} joystick;

static volatile uint8_t active = 255;
static joystick devices[MAX_DEVICES];
static uint8_t device_count;

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Virtual Device Driver ------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	devices[ref].dhi[comp] = DEFAULT_HANDLER;

	// Mac driver will ignore us if this isn't set early
	uint8_t reg1[3];
	uint8_t reg1_len = 0;
	switch (devices[ref].mode) {
		case MODE_FIREBIRD:
			reg1[0] = 0x0A;
			reg1[1] = 0x01;
			reg1[2] = 0x30;
			reg1_len = 3;
			break;
		case MODE_MOUSESTICK:
			reg1[0] = 0x04;
			reg1[1] = 0x00;
			reg1_len = 2;
			break;
	}
	if (reg1_len > 0) {
		computer_data_set(comp, devices[ref].drv_idx, 1, reg1, reg1_len, true);
	}
}

static void drvr_switch(uint8_t comp)
{
	active = comp;
}

static void drvr_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = devices[ref].dhi[comp];
}

static void drvr_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	joystick *dev = &devices[ref];

	switch (devices[ref].mode) {
		case MODE_FIREBIRD:
			if (hndl == DEFAULT_HANDLER || hndl == MODE_FIREBIRD) {
				dev->dhi[comp] = hndl;
				dbg("gjoy %d set to dhid %d", ref, hndl);
				if (hndl == MODE_FIREBIRD) {
					// driver expects data immediately, push something for Talk0
					uint8_t reg0[8] = { 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x00, 0x00, 0x00 };
					computer_data_set(comp, dev->drv_idx,
							0, reg0, sizeof(reg0), false);
				}
			}
			break;

		case MODE_MOUSESTICK:
			if (hndl == DEFAULT_HANDLER || hndl == MODE_MOUSESTICK) {
				dev->dhi[comp] = hndl;
				dbg("gjoy %d set to dhid %d", ref, hndl);
			}
			break;
	}
}

static dev_driver joystick_driver = {
	.default_addr = DEFAULT_ADDRESS,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = NULL,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_get_handle,
	.set_handle_func = drvr_set_handle
};

bool joystick_register(uint8_t *id, joystick_mode mode)
{
	if (device_count >= MAX_DEVICES) return false;

	*id = device_count++;
	joystick *dev = &devices[*id];
	dev->mode = mode;

	return driver_register(&dev->drv_idx, &joystick_driver, *id);
}

void joystick_update(uint8_t id, joystick_data *jdata)
{
	if (active >= COMPUTER_COUNT) return;
	if (id >= device_count) return;

	dbg("gjoy %d: x1:%d, y1:%d, x2:%d, y2:%d btn:%d",
			id, jdata->x1, jdata->y1, jdata->x2, jdata->y2, jdata->buttons);

	// remap data from the real device to the virtual handler
	uint8_t odata[8];
	uint8_t odata_len;
	switch (devices[id].dhi[active]) {
		case MODE_FIREBIRD:
			odata[0] = ((jdata->buttons) >> 16) & 0xFF;
			odata[1] = ((jdata->buttons) >> 8) & 0xFF;
			odata[2] = (jdata->buttons) & 0xFF;
			odata[3] = jdata->x1 + 0x80;
			odata[4] = jdata->y1 + 0x80;
			odata[5] = jdata->y2 + 0x80;
			odata[6] = jdata->x2 + 0x80;
			odata[7] = 0;
			odata_len = 8;
			break;
		case MODE_MOUSESTICK:
			odata[0] = jdata->x1 + 0x80;
			odata[1] = jdata->y1 + 0x80;
			odata[2] = (jdata->buttons) & 0xFF;
			odata_len = 3;
			break;
		default:
			int8_t x = jdata->x1;
			int8_t y = jdata->y1;
			// decrease mouse movement radius to avoid wild pointer behavior
			uint8_t rshift = 3;
			// store as mouse movement
			util_mouse_encode(odata, rshift, x, y, jdata->buttons);
			odata_len = 2;
	}

	// store input data directly into the response registers
	computer_data_set(active, devices[id].drv_idx, 0, odata, odata_len, false);
}
