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

/*
 * Basic mouse driver for relative motion devices following the standard mouse
 * protocol. The mouse is sensitive to lag and doesn't like buffering queues
 * so this uses an internal semaphore structure to manage updates combined
 * with directly writing register data to the storage system.
 */

#define DEFAULT_HANDLER 1

// sets the most number of passthru mice permitted
#define MAX_MICE 4

typedef struct {
	uint8_t hdev;
	uint8_t drv_idx;
	uint8_t dhi[COMPUTER_COUNT];
	SemaphoreHandle_t sem;
	bool pending;      // true if values below are valid
	uint8_t accum[2];  // accumulated register data
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
	if (hndl != 1 || hndl != 2) return;

	mice[ref].dhi[comp] = hndl;
}

static void drvr_talk(uint8_t comp, uint32_t ref, uint8_t reg)
{
	if (active != comp || reg != 0) return;

	if (xSemaphoreTake(mice[ref].sem, portMAX_DELAY)) {
		if (mice[ref].pending) {
			// try to set the accumulated data to get rid of it
			if (computer_data_offer(active, mice[ref].drv_idx,
					0, mice[ref].accum, 2)) {
				mice[ref].pending = false;
			}
		}
		xSemaphoreGive(mice[ref].sem);
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

	mouse *mse = &mice[mouse_count];
	mse->hdev = info->hdev;
	mse->sem = xSemaphoreCreateMutex();
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		mse->dhi[c] = DEFAULT_HANDLER;
	}
	assert(mse->sem != NULL);

	driver_register(&mse->drv_idx, &mouse_driver, mouse_count);
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
		if (xSemaphoreTake(mice[i].sem, portMAX_DELAY)) {
			if (mice[i].pending) {
				// merge existing data with current data
				data[0] = (data[0] & 0x80)
						| ((data[0] + mice[i].accum[0]) & 0x7F);
				data[1] = (data[1] & 0x80)
						| ((data[1] + mice[i].accum[1]) & 0x7F);
			}
			// try to send data, or if send can't be done, store
			if (computer_data_offer(active, mice[i].drv_idx, 0, data, 2)) {
				mice[i].pending = false;
			} else {
				mice[i].pending = true;
				mice[i].accum[0] = data[0];
				mice[i].accum[1] = data[1];
			}
			xSemaphoreGive(mice[i].sem);
		}

		dbg("mse: %d %d", data[0], data[1]);
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
