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

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "hardware.h"

#include "serial.h"

/*
 * Driver for sending UART keystrokes via the standard mouse protocol.
 *
 * Bytes >= 0x80 are commands, < 0x80 are data. Commands must be followed by 0
 * to 1 data bytes, which are committed upon receipt. Commands requiring data
 * to follow are aborted if a new command is sent. See serial.h for the command
 * list.
 */

#define DEFAULT_MSE_HANDLER   1
#define DEFAULT_KBD_HANDLER   2
#define KEYBOARD_QUEUE_DEPTH  8

typedef struct {
	uint8_t length;
	uint8_t data[2];
} serial_message;

typedef struct {
	uint8_t drv_idx;
	uint8_t dhi[COMPUTER_COUNT];
	SemaphoreHandle_t sem;
	bool pending;
	uint8_t accum[2];
	uint8_t cache[2];
} serial_mouse;

typedef struct {
	uint8_t drv_idx;
	QueueHandle_t queue;
} serial_keyboard;

static volatile uint8_t active = 255;
static uint8_t command;
static serial_mouse mouse;
static serial_keyboard keyboard;

/*
 * ----------------------------------------------------------------------------
 * --- Serial Driven Mouse Driver ---------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_mse_reset(uint8_t comp, uint32_t ref)
{
	mouse.dhi[comp] = DEFAULT_MSE_HANDLER;
	if (active == comp) {
		if (xSemaphoreTake(mouse.sem, portMAX_DELAY)) {
			mouse.pending = false;
			xSemaphoreGive(mouse.sem);
		}
	}
}

static void drvr_mse_switch(uint8_t comp)
{
	active = comp;
}

static void drvr_mse_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = mouse.dhi[comp];
}

static void drvr_mse_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	if (hndl != 1 || hndl != 2) return;
	mouse.dhi[comp] = hndl;
}

static void drvr_mse_talk(uint8_t comp, uint32_t ref, uint8_t reg)
{
	if (active != comp || reg != 0) return;

	if (xSemaphoreTake(mouse.sem, portMAX_DELAY)) {
		if (mouse.pending) {
			if (computer_data_offer(active, mouse.drv_idx, 0, mouse.accum, 2)) {
				mouse.pending = false;
			}
		}
		xSemaphoreGive(mouse.sem);
	}
}

static dev_driver serial_mse_driver = {
	.default_addr = 0x03,
	.reset_func = drvr_mse_reset,
	.switch_func = drvr_mse_switch,
	.talk_func = drvr_mse_talk,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_mse_get_handle,
	.set_handle_func = drvr_mse_set_handle
};

/*
 * ----------------------------------------------------------------------------
 * --- Serial Driven Keyboard Driver ------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_kbd_reset(uint8_t comp, uint32_t ref)
{
	if (active == comp) {
		xQueueReset(keyboard.queue);
		computer_queue_set(comp, keyboard.drv_idx, keyboard.queue);
	}
}

static void drvr_kbd_switch(uint8_t comp)
{
	computer_queue_set(active, keyboard.drv_idx, NULL);
	computer_queue_set(comp, keyboard.drv_idx, keyboard.queue);
	active = comp;
}

static void drvr_kbd_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = DEFAULT_KBD_HANDLER;
}

static dev_driver serial_kdb_driver = {
	.default_addr = DEFAULT_KBD_HANDLER,
	.reset_func = drvr_kbd_reset,
	.switch_func = drvr_kbd_switch,
	.talk_func = NULL,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_kbd_get_handle,
	.set_handle_func = NULL
};

/*
 * ----------------------------------------------------------------------------
 * --- Serial Listener from Host Computer -------------------------------------
 * ----------------------------------------------------------------------------
 */

static void serial_kbd_send(bool up, uint8_t c)
{
	serial_message msg;
	msg.length = 2;
	msg.data[0] = (up ? 0x80 : 0x00) | (c & 0x7F);
	msg.data[1] = 0xFF;
	if (c == 0x7F) {
		msg.data[1] = msg.data[0];
		computer_psw(active, !up);
	}
	xQueueSend(keyboard.queue, &msg, 0);
}

static void serial_mse_send()
{
	if (xSemaphoreTake(mouse.sem, portMAX_DELAY)) {
		if (mouse.pending) {
			// merge existing data with current data
			mouse.cache[0] = (mouse.cache[0] & 0x80)
					| ((mouse.cache[0] + mouse.accum[0]) & 0x7F);
			mouse.cache[1] = (mouse.cache[1] & 0x80)
					| ((mouse.cache[1] + mouse.accum[1]) & 0x7F);
		}
		// try to send data, or if send can't be done, store
		if (computer_data_offer(active, mouse.drv_idx, 0, mouse.cache, 2)) {
			mouse.pending = false;
		} else {
			mouse.pending = true;
			mouse.accum[0] = mouse.cache[0];
			mouse.accum[1] = mouse.cache[1];
		}
		xSemaphoreGive(mouse.sem);
	}

	mouse.cache[0] &= 0x80;
	mouse.cache[1] &= 0x80;
}

static void serial_enqueue(uint8_t c) {
	if (c >= 0x80) {
		command = c;
		switch (command) {
		case SER_CMD_MSE_DOWN:
			mouse.cache[0] &= ~0x80;
			serial_mse_send();
			break;
		case SER_CMD_MSE_UP:
			mouse.cache[0] |= 0x80;
			serial_mse_send();
			break;
		case SER_CMD_MSE_APPLY:
			serial_mse_send();
			break;
		}
	}
	else {
		switch (command) {
		case SER_CMD_MSE_X:
			mouse.cache[1] = (mouse.cache[1] & 0x80) | c;
			break;
		case SER_CMD_MSE_Y:
			mouse.cache[0] = (mouse.cache[0] & 0x80) | c;
			break;
		case SER_CMD_KBD_DOWN:
			serial_kbd_send(false, c);
			break;
		case SER_CMD_KBD_UP:
			serial_kbd_send(true, c);
			break;
		case SER_CMD_SWITCH:
			computer_switch(c, true);
			break;
		}
	}
}

static void serial_task(__unused void *parameters)
{
	unsigned char c;
	while (true) {
		c = getc(stdin);
		serial_enqueue(c);
	}
}

void serial_init(void)
{
	mouse.sem = xSemaphoreCreateMutex();
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		mouse.dhi[c] = DEFAULT_MSE_HANDLER;
	}
	assert(mouse.sem != NULL);
	mouse.cache[0] = 0x80;
	mouse.cache[1] = 0x80;

	keyboard.queue = xQueueCreate(KEYBOARD_QUEUE_DEPTH,
				sizeof(serial_message));

	xTaskCreate(serial_task, "serial", configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY, NULL);

	driver_register(&mouse.drv_idx, &serial_mse_driver, 1);
	driver_register(&keyboard.drv_idx, &serial_kdb_driver, 1);
}
