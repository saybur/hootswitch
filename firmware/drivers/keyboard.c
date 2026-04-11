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

#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "hardware.h"

#include "keyboard.h"

/*
 * Driver (computer-side) implementation of an extended ADB keyboard.
 *
 * TODO: evaluate the extended part of this implementation for errors.
 */

// maximum number of supported computer-facing keyboards
#define MAX_KEYBOARDS         4

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
	1, 2, 3, 4, 6, 5, 255, 9, 7, 0, 8, 0
};

typedef struct {
	uint8_t dhi;
	QueueHandle_t queue;
	uint16_t reg2;
} keyboard_memory;

typedef struct {
	uint8_t drv_idx;
	keyboard_memory mem[COMPUTER_COUNT];
	uint32_t down[4];
	uint32_t sw_seq;
	void (*reg2_callback)(uint8_t, uint16_t);
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
 * Sets the Register 2 data for the given computer/device combination.
 */
static void set_comp_reg2(uint8_t comp, uint8_t drv_idx, uint16_t reg2)
{
	uint8_t set[2];
	set[0] = (reg2 >> 8) & 0xFF;
	set[1] = reg2 & 0xFF;
	computer_data_set(comp, drv_idx, 2, set, 2, true);
}

/**
 * Sets/resets tracking bits corresponding to physical keys pressed on the real
 * keyboard. It appears a real keyboard will send fresh key-down events
 * following a reset, which this facilitates.
 */
static void down_update(uint8_t code, uint32_t *down)
{
	uint8_t key = code & 0x7F;
	uint8_t idx = key >> 5;
	uint32_t mask = 1U << (key & 0x1F);

	if (code & 0x80) {
		down[idx] &= ~mask;
	} else {
		down[idx] |= mask;
	}
}

/**
 * Returns true if the down bit is set, false otherwise.
 */
static bool key_is_down(uint8_t code, uint32_t *down)
{
	uint8_t key = code & 0x7F;
	uint8_t idx = key >> 5;
	uint32_t mask = 1U << (key & 0x1F);
	return down[idx] & mask;
}

/**
 * Called following a computer reset to send any key-down events and set
 * register 2 appropriately for the virtual keyboard. This only happens when
 * the computer is the *active* system, otherwise the keyboard reverts to an
 * all-keys-up state in those functions. This is a hack to work around issue #1
 * and probably needs more attention.
 */
static void send_down_keys(uint8_t ref, uint8_t comp)
{
	bool p = false;
	uint8_t key = 0;

	keyboard_message kb;
	kb.length = 2;

	for (uint8_t idx = 0; idx < 4; idx++) {
		uint32_t mask = 1;
		for (uint8_t pos = 0; pos < 32; pos++) {
			// special case: we skip reset key, as a nasty hack to avoid
			// double-key-send shenanagins
			if (idx == 3 && pos == 31) continue;

			// check if corresponding key is pressed, if it is enqueue a down
			// event and update register 2 if needed
			if (keyboards[ref].down[idx] & mask) {
				if (p) {
					kb.data[1] = key;
					xQueueSend(keyboards[ref].mem[comp].queue, &kb, 0);
					p = false;
				} else {
					kb.data[0] = key;
					p = true;
				}
				reg2_update(key, &keyboards[ref].mem[comp].reg2);
			}

			mask <<= 1;
			key++;
		}
	}

	// send remaining single key if present
	if (p) {
		kb.data[1] = 0xFF;
		xQueueSend(keyboards[ref].mem[comp].queue, &kb, 0);
	}
}

/*
 * ----------------------------------------------------------------------------
 * --- Computer-Side Keyboard Driver ------------------------------------------
 * ----------------------------------------------------------------------------
 */

static void drvr_reset(uint8_t comp, uint32_t ref)
{
	// reset the keyboard memory for the affected virtual keyboard
	keyboards[ref].mem[comp].dhi = 0x01;
	xQueueReset(keyboards[ref].mem[comp].queue);
	keyboards[ref].mem[comp].reg2 = DEFAULT_REGISTER_2;
	// (re)assign the queue, not done until the first reset for a system
	computer_queue_set(comp, keyboards[ref].drv_idx,
			keyboards[ref].mem[comp].queue);

	if (active == comp) {
		keyboards[ref].sw_seq = 0;
		// re-send down-key events, which updates register 2 based on the
		// physical keyboard's current pressed keys
		send_down_keys(ref, comp);
		// update keyboard LEDs accordingly (TODO eval how well this is done)
		if (keyboards[ref].reg2_callback) {
			keyboards[ref].reg2_callback(ref, keyboards[ref].mem[comp].reg2);
		}
	}

	set_comp_reg2(comp, keyboards[ref].drv_idx, keyboards[ref].mem[comp].reg2);
}

static void drvr_switch(uint8_t comp)
{
	for (uint8_t i = 0; i < keyboard_count; i++) {
		computer_psw(active, false);
		keyboards[i].sw_seq = 0;
		if (keyboards[i].reg2_callback) {
			keyboards[i].reg2_callback(i, keyboards[i].mem[comp].reg2);
		}
	}
	active = comp;
}

static void drvr_listen(uint8_t comp, uint32_t ref, uint8_t reg,
		volatile uint8_t* data, uint8_t length)
{
	dbg("kbd L%d %d", reg, length);
	if (reg == 2 && length == 2) {
		// only thing we let the computer change is LED state
		uint16_t *reg2 = &keyboards[ref].mem[comp].reg2;
		uint8_t leds = data[1] & 0x7;
		*reg2 = (*reg2 & 0xFFF8) | leds;

		// if it is the active computer, update now
		if (comp == active && keyboards[ref].reg2_callback) {
			keyboards[ref].reg2_callback(ref, *reg2);
		}
	}
}

static void drvr_get_handle(uint8_t comp, uint32_t ref, uint8_t *hndl)
{
	*hndl = keyboards[ref].mem[comp].dhi;
}

static void drvr_set_handle(uint8_t comp, uint32_t ref, uint8_t hndl)
{
	if (! (hndl >= 0x01 && hndl <= 0x03)) return;
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

bool keyboard_enqueue(uint8_t id, keyboard_message *m)
{
	if (active >= COMPUTER_COUNT) return false;
	if (id >= keyboard_count) return false;

	uint8_t hi = m->data[1];
	uint8_t lo = m->data[0];

	dbg("kbd: lo:0x%02X, hi:0x%02X", lo, hi);

	// handle power switch activation
	if (lo == 0x7F && hi == 0x7F) {
		computer_psw(active, true);
	} else if (lo == 0xFF && hi == 0xFF) {
		computer_psw(active, false);
	}

	// handle switching
	// hi==lo seems to happen sometimes on meta key up
	if (hi == 0xFF || lo == hi) {
		if (lo > 0x80) {
			keyboards[id].sw_seq <<= 8;
			keyboards[id].sw_seq += lo;
		} else if (keyboards[id].sw_seq == SWITCH_SEQUENCE
				&& lo >= ONE_KEY_DOWN
				&& lo < ONE_KEY_DOWN + sizeof(codes_to_comp_idx)) {
			// match, veto keystroke and switch instead
			computer_switch(codes_to_comp_idx[lo - ONE_KEY_DOWN], true);
			return false;
		}
	} else {
		keyboards[id].sw_seq = 0;
	}

	// set appropriate bits in Register 2 based on key state
	// along with the map of keys pressed
	uint16_t old_reg2 = keyboards[id].mem[active].reg2;
	if (lo != 0xFF) {
		reg2_update(lo, &keyboards[id].mem[active].reg2);
		down_update(lo, keyboards[id].down);
	}
	if (hi != 0xFF) {
		reg2_update(hi, &keyboards[id].mem[active].reg2);
		down_update(hi, keyboards[id].down);
	}

	// update the real register if needed
	if (old_reg2 != keyboards[id].mem[active].reg2) {
		set_comp_reg2(active, keyboards[id].drv_idx,
				keyboards[id].mem[active].reg2);
	}

	// enqueue data, dropping if queue is full
	return pdPASS == xQueueSend(keyboards[id].mem[active].queue, m, 0);
}

bool keyboard_register(uint8_t *id, void (*reg2_callback)(uint8_t, uint16_t))
{
	if (keyboard_count >= MAX_KEYBOARDS) return false;

	*id = keyboard_count++;
	keyboard *kbd = &keyboards[*id];

	for (uint8_t c = 0; c < COMPUTER_COUNT; c++) {
		kbd->mem[c].dhi = 0x02;
		kbd->mem[c].queue = xQueueCreate(KEYBOARD_QUEUE_DEPTH,
				sizeof(keyboard_message));
		assert(kbd->mem[c].queue != NULL);
		kbd->mem[c].reg2 = DEFAULT_REGISTER_2;
	}

	kbd->reg2_callback = reg2_callback;
	return driver_register(&kbd->drv_idx, &keyboard_driver, *id);
}

void keyboard_sequence(uint8_t id, uint8_t *c, uint8_t len)
{
	if (active >= COMPUTER_COUNT) return;
	if (id >= keyboard_count) return;

	keyboard_message m;
	m.length = 0;

	for (uint16_t i = 0; i < len; i++) {
		bool is_down = key_is_down(c[i], keyboards[id].down);
		bool ask_down = (c[i] & 0x80) == 0;
		if (is_down != ask_down) {
			if (m.length == 0) {
				m.data[0] = c[i];
				m.length = 1;
			} else if (m.length == 1) {
				m.data[1] = c[i];
				m.length = 2;
				keyboard_enqueue(id, &m);
				m.length = 0;
			}
		}
	}

	// send residual
	if (m.length == 1) {
		m.data[1] = 0xFF;
		keyboard_enqueue(id, &m);
	}
}
