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

/*
 * This implements a simple 'relay' device that echos commands from the
 * selected system to any attached devices. This has numerous problems but
 * possesses the virtue of simplicity: the switch doesn't need to understand
 * everything the devices are saying, it just needs to track the protocol
 * enough to know who should be talking, and then echo the line state across
 * the barrier.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "hardware.h"
#include "relay.h"

static bool locked = false;
static uint8_t button = 1;
static uint32_t cdi_pin = C1_DI_PIN;
static uint32_t cdo_pin = C1_DO_PIN;

static inline void relay_dev_assert(bool value)
{
	if (locked) {
		gpio_put(A_DO_PIN, value);
	}
}

static inline void relay_comp_assert(bool value)
{
	if (locked) {
		gpio_put(cdo_pin, value);
	}
}

static void relay_advance(void)
{
	locked = false;
	switch(button)
	{
		case 1:
			button = 2;
			cdi_pin = C2_DI_PIN;
			cdo_pin = C2_DO_PIN;
			gpio_put(LED3_PIN, 0);
			gpio_put(LED4_PIN, 1);
			break;
		case 2:
			button = 3;
			cdi_pin = C3_DI_PIN;
			cdo_pin = C3_DO_PIN;
			gpio_put(LED4_PIN, 0);
			gpio_put(LED5_PIN, 1);
			break;
		case 3:
			button = 4;
			cdi_pin = C4_DI_PIN;
			cdo_pin = C4_DO_PIN;
			gpio_put(LED5_PIN, 0);
			gpio_put(LED6_PIN, 1);
			break;
		default:
			button = 1;
			cdi_pin = C1_DI_PIN;
			cdo_pin = C1_DO_PIN;
			gpio_put(LED6_PIN, 0);
			gpio_put(LED3_PIN, 1);
	}
}

static void relay_data_cycle(uint32_t in, uint32_t out)
{
	uint64_t time = time_us_64();
	while (time_us_64() - time < 240 && gpio_get(in));
	if (gpio_get(in)) return;
	if (locked) gpio_put(out, 1);

	// keep cycling as systems are exchanging bits
	while (true) {
		while (! gpio_get(in));
		if (locked) gpio_put(out, 0);

		time = time_us_64();
		while (time_us_64() - time < 125 && gpio_get(in));
		if (! gpio_get(in)) {
			if (locked) gpio_put(out, 1);
		} else {
			return;
		}
	}
}

static bool relay_loop(void)
{
	// wait for the line to go low
	uint64_t time;
	while (gpio_get(cdi_pin));
	relay_dev_assert(true);
	time = time_us_64();

	// attention; wait for the line to go high
	while (! gpio_get(cdi_pin));
	relay_dev_assert(false);
	uint32_t td = time_us_64() - time;
	if (td < 775) {
		// faulty selection
		return false;
	} else if (td > 825) {
		// reset, we mostly ignore
		return false;
	}

	// in sync phase
	uint8_t cmd = 0;
	while (gpio_get(cdi_pin));
	relay_dev_assert(true);
	time = time_us_64();
	for (uint8_t i = 0; i < 8; i++) {
		while (! gpio_get(cdi_pin));
		relay_dev_assert(false);

		td = (time_us_64() - time) % 100;
		if (td < 20 || td > 80) {
			// protocol fault
			return false;
		} else if (td > 50) {
			cmd |= 1 << i;
		}

		while (! gpio_get(cdi_pin));
		relay_dev_assert(true);
	}

	// command stop bit; drive computer low and wait to see if SRQ happens
	relay_comp_assert(true);
	time = time_us_64();
	while (time_us_64() - time < 65);
	relay_dev_assert(false);
	busy_wait_us_32(1);
	// allow device to assert srq if needed
	if (! gpio_get(A_DI_PIN));
	relay_comp_assert(false);

	// who will talk next?
	cmd &= 0xC;
	if (cmd == 0xC) {
		// talk command, device
		relay_data_cycle(A_DI_PIN, cdo_pin);
	} else if (cmd == 0x8) {
		// listen command, computer
		relay_data_cycle(cdi_pin, A_DO_PIN);
	} else {
		// no further communication needed
	}
	return true;
}

void relay_main(void)
{
	gpio_put(LED3_PIN, 1);
	uint8_t button = 0;

	while (1) {
		if (relay_loop()) {
			locked = true;
		}

		if (gpio_get(SWITCH_PIN)) {
			if (button == 1) {
				relay_advance();
			}
			button++;
		} else {
			button = 0;
		}
	}
}
