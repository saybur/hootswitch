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
#include "queue.h"

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "handler.h"
#include "hardware.h"
#include "host_err.h"

#include "keyboard.h"

// sets the most number of passthru keyboards permitted
#define MAX_KEYBOARDS         4

// standard keyboard handler
#define DEFAULT_HANDLER       2

// how many Talk 0s from a keyboard are queued for sending to computers?
#define KEYBOARD_QUEUE_DEPTH  8

/*
 * Register 2 tracks the state of various meta keys and the LEDs on the
 * Extended Keyboard. These match the Standard Keyboard where applicable (not
 * sure about the IIgs or Design keyboards but this assumes they follow similar
 * codes).
 *
 * The handler will attempt to switch the keyboard from handler $2 to $3 to
 * detect if it is an Extended Keyboard or compatible. If so it will then
 * echo through changes to the LEDs it gets from the host, but will otherwise
 * lie and show the meta keys according to what has passed through Talk 0 from
 * the real keyboard.
 */
#define DELETE_KEY            0x33
#define DELETE_BIT            (1U << 14)
#define CAPS_LOCK_KEY         0x39
#define CAPS_LOCK_BIT         (1U << 13)
#define CAPS_LOCK_LED_BIT     (1U << 1)
#define RESET_KEY             0x7F7F
#define RESET_BIT             (1U << 12)
#define CONTROL_KEY           0x36
#define CONTROL_KEY_RIGHT     0x7D
#define CONTROL_BIT           (1U << 11)
#define SHIFT_KEY             0x38
#define SHIFT_KEY_RIGHT       0X7B
#define SHIFT_BIT             (1U << 10)
#define OPTION_KEY            0x3A
#define OPTION_KEY_RIGHT      0x7C
#define OPTION_BIT            (1U << 9)
#define COMMAND_KEY           0x37
#define COMMAND_BIT           (1U << 8)
#define NUM_LOCK_KEY          0x47
#define NUM_LOCK_BIT          (1U << 7)
#define NUM_LOCK_LED          (1U << 0)
#define SCROLL_LOCK_KEY       0x6B
#define SCROLL_LOCK_BIT       (1U << 6)
#define SCROLL_LOCK_LED       (1U << 2)

/*
 * Magic sequence to trigger a computer switch. This is a 4-byte value shifted
 * up on each up-key when the low byte is 0xFF; when the low byte is not 0xFF
 * it is reset. Only up-keys count for this. Once the sequence is met, any
 * number key 1-9 (0x12-0x19) will switch to that computer port.
 *
 * Sequence is Control->Option->Command->Shift.
 */
#define SWITCH_SEQUENCE       0xB6BAB7B8
#define ONE_KEY_DOWN          0x12

// remap keycodes to computer index numbers for 0x12-0x1D
const uint8_t codes_to_comp_idx[] = {
	0, 1, 2, 3, 5, 4, 254, 8, 6, 255, 7, 9
};

typedef struct {
	uint8_t length;
	uint8_t data[2];
} keyboard_data;

typedef struct {
	uint8_t hdev;
	uint8_t drv_idx;
	bool extended;
	uint8_t dhi[COMPUTER_COUNT];
	QueueHandle_t queue;
	uint16_t reg2;
	uint32_t sw_seq;
} keyboard;

static volatile uint8_t active;
static keyboard keyboards[MAX_KEYBOARDS];
static uint8_t keyboard_count;

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Keyboard Driver ------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	keyboards[ref].dhi[comp] = DEFAULT_HANDLER;
	if (active == comp) {
		keyboards[ref].reg2 = 0xFFFF;
		keyboards[ref].sw_seq = 0;
		xQueueReset(keyboards[ref].queue);
		computer_queue_set(comp, keyboards[ref].drv_idx, keyboards[ref].queue);
	}
}

static void drvr_switch(uint8_t comp)
{
	for (uint8_t i = 0; i < keyboard_count; i++) {
		computer_psw(active, false);
		keyboards[i].sw_seq = 0;
		computer_queue_set(active, keyboards[i].drv_idx, NULL);
		xQueueReset(keyboards[i].queue);
		computer_queue_set(comp, keyboards[i].drv_idx, keyboards[i].queue);
	}
	active = comp;
}

static void drvr_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = keyboards[ref].dhi[comp];
}

static void drvr_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	if (hndl != 2 || hndl != 3) return;
	// TODO handle extended better

	keyboards[ref].dhi[comp] = hndl;
}

static dev_driver keyboard_driver = {
	.default_addr = 0x02,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = NULL,
	.listen_func = NULL,
	.flush_func = NULL,
	.get_handle_func = drvr_get_handle,
	.set_handle_func = drvr_set_handle
};

/*
 * ----------------------------------------------------------------------------
 * --- Handler for Real ADB Keyboards -----------------------------------------
 * ----------------------------------------------------------------------------
 */

static bool hndl_interview(volatile ndev_info *info, bool (*handle_change)(uint8_t))
{
	if (keyboard_count >= MAX_KEYBOARDS) return false;
	if (info->address_def != 0x02) return false;
	if (! (info->dhid_cur == DEFAULT_HANDLER || info->dhid_cur == 0x03)) return false;

	keyboard *kbd = &keyboards[keyboard_count];
	kbd->hdev = info->hdev;
	if (info->dhid_cur == 0x03) {
		kbd->extended = true;
	} else {
		kbd->extended = handle_change(0x03);
	}
	kbd->queue = xQueueCreate(KEYBOARD_QUEUE_DEPTH, sizeof(keyboard_data));
	assert(kbd->queue != NULL);
	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		kbd->dhi[c] = DEFAULT_HANDLER;
	}

	driver_register(&kbd->drv_idx, &keyboard_driver, keyboard_count);
	keyboard_count++;
	return true;
}

static void hndl_talk(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	if (active >= COMPUTER_COUNT) return;

	uint8_t i;
	for (i = 0; i < keyboard_count; i++) {
		if (keyboards[i].hdev == hdev) break;
	}
	if (i == keyboard_count) return;

	if (data_len >= 2 && active < COMPUTER_COUNT) {
		dbg("kbd: %d %d", data[0], data[1]);

		// handle power switch activation
		if (data[0] == 0x7F && data[1] == 0x7F) {
			computer_psw(active, true);
		} else if (data[0] == 0xFF && data[1] == 0xFF) {
			computer_psw(active, false);
		}

		// handle switching
		// hi==lo seems to happen sometimes on meta key up
		if (data[1] == 0xFF || data[0] == data[1]) {
			if (data[0] > 0x80) {
				keyboards[i].sw_seq <<= 8;
				keyboards[i].sw_seq += data[0];
			} else if (keyboards[i].sw_seq == SWITCH_SEQUENCE
					&& data[0] >= ONE_KEY_DOWN
					&& data[0] < ONE_KEY_DOWN + sizeof(codes_to_comp_idx)) {
				// match, veto keystroke and switch instead
				computer_switch(codes_to_comp_idx[data[0] - ONE_KEY_DOWN]);
				return;
			}
		} else {
			keyboards[i].sw_seq = 0;
		}

		// enqueue data, dropping if queue is full
		keyboard_data kb;
		kb.length = 2;
		kb.data[0] = data[0];
		kb.data[1] = data[1];
		xQueueSend(keyboards[i].queue, &kb, 0);
	}
}

static ndev_handler keyboard_handler = {
	.name = "kbd",
	.accept_noop_talks = false,
	.interview_func = hndl_interview,
	.talk_func = hndl_talk,
	.listen_func = NULL,
	.flush_func = NULL,
};

void keyboard_init(void)
{
	handler_register(&keyboard_handler);
}
