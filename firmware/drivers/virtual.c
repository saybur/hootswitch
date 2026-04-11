/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdbool.h>
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "debug.h"

#include "joystick.h"
#include "keyboard.h"
#include "mouse.h"
#include "virtual.h"

/*
 * Skeleton for registering a single set of virtual devices, useful for keeping
 * the total number of ADB devices on a computer chain to a reasonable level.
 * Users should call the _id() functions below to get the ID assigned to the
 * virtual device, then use that value with the relevant driver calls.
 */

#define MSE_QUEUE_DEPTH 16

static bool active;
static uint8_t kbd_idx;
static uint8_t mse_idx;
static uint8_t joy_idx;

static QueueHandle_t mse_queue;

static void virtual_device_task(__unused void *parameters)
{
	bool valid = false;
	virtual_mouse_data scratch, send;
	dbg("virt-mse: queue start");

	while (true) {
		/*
		 * Unless we already have something to work with, block until there
		 * is data; otherwise proceed to the compaction stage.
		 */
		if (!valid) {
			if (pdPASS != xQueueReceive(mse_queue, &send, portMAX_DELAY)) {
				// failed to get a result, try again
				continue;
			}
		}

		// setup for reusing validity below
		valid = false;

		/*
		 * Drain the queue until 1) nothing is left, or 2) mouse buttons
		 * change state, which can't be compacted into a single report.
		 */
		while (pdPASS == xQueueReceive(mse_queue, &scratch, 0)) {
			if (send.buttons != scratch.buttons) {
				// result incompressible, prepare to set aside
				valid = true;
				break;
			} else {
				send.x += scratch.x;
				send.y += scratch.y;
			}
		}

		// send the results
		if (mouse_update(mse_idx, send.x, send.y, send.buttons)) {
			// good response, keep incompressible data for next loop if present
			if (valid) {
				send = scratch;
			}
		} else {
			/*
			 * Unable to enqueue 'send'; leaving the value alone for the next
			 * iteration is fine unless there is also an incompressible item
			 * from the queue. If there is one, put it back into the queue if
			 * there's space. If there isn't, drop that report and emit a
			 * warning (if this happens we're obviously running way faster than
			 * the ADB side or there's a programming problem).
			 */
			if (valid) {
				if (pdPASS != xQueueSendToFront(mse_queue, &scratch, 0)) {
					// just drop the second report
					dbg_err("virt-mse: queue overflow");
				}
			} else {
				// leave send alone for enqueuing next time
				valid = true;
			}
		}
	}
}

bool virtual_keyboard_offer(bool up, uint8_t c)
{
	keyboard_message msg;
	msg.length = 2;
	msg.data[0] = (up ? 0x80 : 0x00) | (c & 0x7F);
	msg.data[1] = 0xFF;
	if (msg.data[0] == 0x7F) {
		// protocol requires power key be sent twice in one packet
		msg.data[1] = msg.data[0];
	}
	return keyboard_enqueue(kbd_idx, &msg);
}

uint8_t virtual_keyboard_index(void)
{
	return kbd_idx;
}

bool virtual_mouse_offer(virtual_mouse_data *data)
{
	if (!data) return false;
	return pdPASS == xQueueSend(mse_queue, data, 0);
}

void virtual_init(void)
{
	if (active) return;
	active = true;
	uint8_t mse_reg1[8] = { 'H', 'o', 'o', 't', 1, 144, 1, 8 }; // 400cpi
	mouse_register(&mse_idx, MOUSE_MODE_EXTENDED, mse_reg1);
	keyboard_register(&kbd_idx, NULL);

	mse_queue = xQueueCreate(MSE_QUEUE_DEPTH,
				sizeof(virtual_mouse_data));
	assert(mse_queue != NULL);

	xTaskCreate(virtual_device_task, "virtual_dev", configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY + 2, NULL);
}
