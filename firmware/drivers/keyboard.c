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
#include "host.h"
#include "host_err.h"

#include "keyboard.h"

// sets the most number of passthru keyboards permitted
#define MAX_KEYBOARDS         4

// standard keyboard handler
#define DEFAULT_HANDLER       2

// how many Talk 0s from a keyboard are queued for sending to computers?
#define KEYBOARD_QUEUE_DEPTH  8

// bootup/reset state for Register 2
#define DEFAULT_REGISTER_2    0xFFFF

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
#define RESET_KEY             0x7F
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
} keyboard_message;

typedef struct {
	uint8_t dhi;
	QueueHandle_t queue;
	uint16_t reg2;
} keyboard_memory;

typedef struct {
	uint8_t hdev;
	uint8_t drv_idx;
	bool extended;
	keyboard_memory mem[COMPUTER_COUNT];
	uint32_t sw_seq;
} keyboard;

static volatile uint8_t active;
static keyboard keyboards[MAX_KEYBOARDS];
static uint8_t keyboard_count;

/*
 * ----------------------------------------------------------------------------
 * --- Utility Functions ------------------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void reg2_update(uint8_t code, uint16_t *reg2)
{
	uint8_t key = code & 0x7F;
	uint16_t mask = 0;

	switch (key)
	{
	case DELETE_KEY:
		mask = DELETE_BIT;
		break;
	case CAPS_LOCK_KEY:
		mask = CAPS_LOCK_BIT;
		break;
	case RESET_KEY:
		// always in pairs
		// this approach should keep the bit state consistent anyway
		mask = RESET_BIT;
		break;
	case CONTROL_KEY:
	case CONTROL_KEY_RIGHT:
		mask = CONTROL_BIT;
		break;
	case SHIFT_KEY:
	case SHIFT_KEY_RIGHT:
		mask = SHIFT_BIT;
		break;
	case OPTION_KEY:
	case OPTION_KEY_RIGHT:
		mask = OPTION_BIT;
		break;
	case COMMAND_KEY:
		mask = COMMAND_BIT;
		break;
	case NUM_LOCK_KEY:
		mask = NUM_LOCK_BIT;
		break;
	case SCROLL_LOCK_KEY:
		mask = SCROLL_LOCK_BIT;
		break;
	default:
		// no change required
		return;
	}

	if (code & 0x80) {
		// up key
		*reg2 |= mask;
	} else {
		// down key
		*reg2 &= ~mask;
	}
}

/*
 * Sends a blind Listen Register 2 to the keyboard with the given handler ID,
 * updating the low 3 bits of Register 2 with new LED information.
 */
static void send_host_reg2(uint8_t hdev, uint16_t reg2)
{
	uint32_t id;
	uint8_t send[2];
	send[0] = (reg2 >> 8) & 0xFF;
	send[1] = reg2 & 0xFF;
	host_cmd(hdev, COMMAND_LISTEN_2, &id, send, 2);
}

/*
 * Sets the Register 2 data for the given computer/device combination.
 */
static void set_comp_reg2(uint8_t comp, uint8_t drv_idx, uint16_t reg2)
{
	uint8_t set[2];
	set[0] = (reg2 >> 8) & 0xFF;
	set[1] = reg2 & 0xFF;
	computer_data_set(comp, drv_idx, 2, set, 2, true);
}

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Keyboard Driver ------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	// reset the keyboard memory for the affected virtual keyboard
	keyboards[ref].mem[comp].dhi = DEFAULT_HANDLER;
	xQueueReset(keyboards[ref].mem[comp].queue);
	keyboards[ref].mem[comp].reg2 = DEFAULT_REGISTER_2;
	// (re)assign the queue, not done until the first reset for a system
	computer_queue_set(comp, keyboards[ref].drv_idx,
			keyboards[ref].mem[comp].queue);
	set_comp_reg2(comp, keyboards[ref].drv_idx, DEFAULT_REGISTER_2);

	if (active == comp) {
		keyboards[ref].sw_seq = 0;
		send_host_reg2(keyboards[ref].hdev, DEFAULT_REGISTER_2);
	}
}

static void drvr_switch(uint8_t comp)
{
	for (uint8_t i = 0; i < keyboard_count; i++) {
		computer_psw(active, false);
		keyboards[i].sw_seq = 0;
		send_host_reg2(keyboards[i].hdev, keyboards[i].mem[comp].reg2);
	}
	active = comp;
}

static void drvr_listen(uint8_t comp, uint32_t ref, uint8_t reg,
		volatile uint8_t* data, uint8_t length)
{
	if (reg == 2 && length == 2) {
		// only thing we let the computer change is LED state
		uint16_t *reg2 = &keyboards[ref].mem[comp].reg2;
		uint8_t leds = data[1] & 0x7;
		*reg2 = (*reg2 & 0xFFF8) | leds;

		// if it is the active computer, update now
		if (comp == active) {
			send_host_reg2(keyboards[ref].hdev, *reg2);
		}
	}
}

static void drvr_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = keyboards[ref].mem[comp].dhi;
}

static void drvr_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	if (hndl != 2 || hndl != 3) return;
	keyboards[ref].mem[comp].dhi = hndl;
}

static dev_driver keyboard_driver = {
	.default_addr = 0x02,
	.reset_func = drvr_reset,
	.switch_func = drvr_switch,
	.talk_func = NULL,
	.listen_func = drvr_listen,
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

	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		kbd->mem[c].dhi = DEFAULT_HANDLER;
		kbd->mem[c].queue = xQueueCreate(KEYBOARD_QUEUE_DEPTH,
				sizeof(keyboard_message));
		assert(kbd->mem[c].queue != NULL);
		kbd->mem[c].reg2 = DEFAULT_REGISTER_2;
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

		// set appropriate bits in Register 2 based on key state
		uint16_t old_reg2 = keyboards[i].mem[active].reg2;
		if (data[0] != 0xFF) {
			reg2_update(data[0], &keyboards[i].mem[active].reg2);
		}
		if (data[1] != 0xFF) {
			reg2_update(data[1], &keyboards[i].mem[active].reg2);
		}

		// update the real register if needed
		if (old_reg2 != keyboards[i].mem[active].reg2) {
			set_comp_reg2(active, keyboards[i].drv_idx,
					keyboards[i].mem[active].reg2);
		}

		// enqueue data, dropping if queue is full
		keyboard_message kb;
		kb.length = 2;
		kb.data[0] = data[0];
		kb.data[1] = data[1];
		xQueueSend(keyboards[i].mem[active].queue, &kb, 0);
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
