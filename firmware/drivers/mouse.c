/*
 * Copyright (C) 2024 saybur
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

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "handler.h"
#include "hardware.h"
#include "host_err.h"

#include "mouse.h"

#define DEFAULT_HANDLER 1

// sets the most number of passthru mice permitted
// obviously an absurd limit
#define MAX_MICE 4

typedef struct {
	uint8_t hdev;
	uint8_t dev;
	uint8_t dhi[COMPUTER_COUNT];
	SemaphoreHandle_t sem;
	bool pending;
	uint8_t x, y, dx, dy;
	bool b1, b2; // b2 is likely always false, see Ch8 2nd Ed Mac Hardware
} mouse_type;

static volatile uint8_t active = 255;
static mouse_type mice[MAX_MICE];
static uint8_t mouse_count;

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Mouse Driver ---------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint8_t device)
{
	for (uint8_t i = 0; i < mouse_count; i++) {
		mice[i].dhi[comp] = DEFAULT_HANDLER;

		if (active == comp && mice[i].dev == device) {
			if (xSemaphoreTake(mice[i].sem, portMAX_DELAY)) {
				mice[i].x = 0;
				mice[i].y = 0;
				mice[i].dx = 0;
				mice[i].dy = 0;
				mice[i].b1 = false;
				mice[i].b2 = false;
				xSemaphoreGive(mice[i].sem);
			}
		}
	}
}

static void drvr_switch(uint8_t comp)
{
	active = comp;
}

static void drvr_get_handle(uint8_t comp, uint8_t device, uint8_t *hndl)
{
	for (uint8_t i = 0; i < mouse_count; i++) {
		if (mice[i].dev == device) {
			*hndl = mice[i].dhi[comp];
			return;
		}
	}
}

static void drvr_set_handle(uint8_t comp, uint8_t device, uint8_t hndl)
{
	if (hndl != 1 || hndl != 2) return;

	for (uint8_t i = 0; i < mouse_count; i++) {
		if (mice[i].dev == device) {
			mice[i].dhi[comp] = hndl;
			return;
		}
	}
}

static void drvr_talk(uint8_t comp, uint8_t device, uint8_t reg)
{
	if (active != comp || reg != 0) return;

	for (uint8_t i = 0; i < mouse_count; i++) {
		if (mice[i].dev == device) {
			if (xSemaphoreTake(mice[i].sem, portMAX_DELAY)) {
				mice[i].x = (mice[i].x - mice[i].dx) & 0x7F;
				mice[i].y = (mice[i].y - mice[i].dy) & 0x7F;
				mice[i].dx = 0;
				mice[i].dy = 0;
				mice[i].pending = false;
				xSemaphoreGive(mice[i].sem);
			}
		}
	}
}

static dev_driver mouse_driver = {
	.default_addr = 0x03,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = drvr_talk,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_get_handle,
	.set_handle_func = NULL
};

/*
 * ----------------------------------------------------------------------------
 * --- Handler for Real ADB Mice ----------------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool hndl_interview(volatile ndev_info *info, bool (*handle_change)(uint8_t))
{
	if (mouse_count >= MAX_MICE) return false;
	if (info->address_def != 0x03) return false;
	if (! (info->dhid_cur == DEFAULT_HANDLER || info->dhid_cur == 0x02)) return false;

	mice[mouse_count].hdev = info->hdev;
	mice[mouse_count].sem = xSemaphoreCreateMutex();
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		mice[mouse_count].dhi[c] = DEFAULT_HANDLER;
	}
	assert(mice[mouse_count].sem != NULL);

	driver_register(&mice[mouse_count].dev, &mouse_driver);
	mouse_count++;
	return true;
}

static void hndl_talk(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	// ignore until there is a computer to send data to
	if (active >= COMPUTER_COUNT) return;

	// select the correct device mapping
	uint8_t i;
	for (i = 0; i < mouse_count; i++) {
		if (mice[i].hdev == hdev) break;
	}
	if (i == mouse_count) return;

	// update data and enqueue it in the active computer
	if (reg == 0 && data_len >= 2) {
		uint8_t y = data[0] & 0x7F;
		uint8_t x = data[1] & 0x7F;
		mice[i].b1 = data[0] & 0x80;
		mice[i].b2 = data[1] & 0x80;

		if (xSemaphoreTake(mice[i].sem, portMAX_DELAY)) {
			mice[i].x = (mice[i].x + x) & 0x7F;
			mice[i].y = (mice[i].y + y) & 0x7F;
			if (! mice[i].pending) {
				mice[i].dx = mice[i].x;
				mice[i].dy = mice[i].y;
				uint8_t buf[2];
				buf[0] = (mice[i].b1 ? 0x80 : 0x00) | mice[i].dy;
				buf[1] = (mice[i].b2 ? 0x80 : 0x00) | mice[i].dx;
				computer_data_offer(active, mice[i].dev, 0, buf, 2, false);
			}
			xSemaphoreGive(mice[i].sem);
		}

		dbg("mse: %d %d %d", x, y, (uint8_t) mice[i].b1);
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

void mouse_init(void)
{
	handler_register(&mouse_handler);
}
