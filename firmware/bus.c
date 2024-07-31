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
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "bus.h"
#include "hardware.h"
#include "bus.pio.h"

#define MACHINE_PIO       pio0
#define HOSTING_PIO       pio1
#define HOSTING_PIO_SM    0

#define MACHINE_COUNT     4
#define DEVICE_MAX        8

typedef enum {
	PHASE_IDLE,
	PHASE_ATTENTION,
	PHASE_SYNC,
	PHASE_COMMAND,
	PHASE_SRQ,
	PHASE_TLT,
	PHASE_DATA_OUT,
	PHASE_DATA_IN
} BusPhase;

typedef enum {
	STATUS_FAULT,             // lost track of bus phase
	STATUS_RESET,             // machine issued reset
	STATUS_COMMANDED,         // pending command
	STATUS_DATA_RECV,         // pending data to parse
	STATUS_NORMAL
} BusStatus;

typedef struct {
	uint8_t id;
	uint16_t reg3;
	uint16_t aliases[MACHINE_COUNT];
} BusDevice;

typedef struct {
	bool active;              // true if this is the user-selected machine
	pio_sm_config sm_tx;
	pio_sm_config sm_rx;
	BusPhase phase;
	BusStatus status;
	uint32_t pin_out;         // GPIO number of line driving pin
	uint32_t pin_in;          // GPIO number of line listening pin
	uint16_t device_mask;     // bitmask for devices active on this machine
	uint64_t time;            // multiple use, generally when the phase started
	uint8_t command;          // last command received if STATUS_COMMANDED
	uint16_t message;         // last message received if STATUS_DATA_RECV
} BusMachine;

static BusDevice devices[DEVICE_MAX];
static BusMachine machines[MACHINE_COUNT];

static uint32_t off_m_pio_tx, off_m_pio_rx,
		off_h_pio_tx, off_h_pio_rx;
static pio_sm_config sm_host_tx, sm_host_rx;

static void bus_machine_tick(uint8_t id)
{
	BusMachine *mach = &(machines[id]);
	uint64_t time = time_us_64();
	uint32_t elapsed = time - mach->time;
	bool pin = gpio_get(mach->pin_in);

// TODO not clear how non-normal status should be handled

	switch (mach->phase)
	{
		case PHASE_IDLE:
			if (! pin) {
				mach->time = time;
				mach->phase = PHASE_ATTENTION;
			}
			break;
		case PHASE_ATTENTION:
			if (pin) {
				if (elapsed > BUS_ATN_MAX) {
					mach->status = STATUS_RESET;
					mach->phase = PHASE_IDLE;
				} else if (elapsed > BUS_ATN_MIN) {
					mach->time = time;
					mach->phase = PHASE_COMMAND;
					if (mach->status == STATUS_FAULT) {
						// good attention enough to clear fault... ?
						mach->status = STATUS_NORMAL;
					}
					pio_sm_init(MACHINE_PIO, id, off_m_pio_rx, &(mach->sm_rx));
					pio_sm_put(MACHINE_PIO, id, 8);
					pio_sm_set_enabled(MACHINE_PIO, id, true);
				} else {
					mach->status = STATUS_FAULT;
					mach->phase = PHASE_IDLE;
				}
			} else {
				// ADUM1201 should idle high when off but this still needs
				// something if a weird condition occurs, TODO
			}
			break;
		case PHASE_COMMAND:
			if (! pio_sm_is_rx_fifo_empty(MACHINE_PIO, id)) {
				mach->command = pio_sm_get(MACHINE_PIO, id);
				// look at top 4 bits and see if they match a device
				if (mach->device_mask & (1U << ((mach->command) >> 4))) {
					mach->status = STATUS_COMMANDED;
				}

			} else if (elapsed > BUS_CMD_MAX) {
				pio_sm_set_enabled(MACHINE_PIO, id, false);
				mach->status = STATUS_FAULT;
				mach->phase = PHASE_IDLE;
			}
			break;





			
	}
}

void bus_init(void)
{
	const uint32_t machine_do_pins[] = {
		C1_DO_PIN, C2_DO_PIN, C3_DO_PIN, C4_DO_PIN
	};
	assert(sizeof(machine_do_pins) == MACHINE_COUNT);
	const uint32_t machine_di_pins[] = {
		C1_DI_PIN, C2_DI_PIN, C3_DI_PIN, C4_DI_PIN
	};
	assert(sizeof(machine_di_pins) == MACHINE_COUNT);

	// setup storage
	for (uint8_t i = 0; i < MACHINE_COUNT; i++) {
		machines[i] = (BusMachine) {
			.phase = PHASE_IDLE, .status = STATUS_NORMAL,
			.pin_out = machine_do_pins[i], .pin_in = machine_di_pins[i],
			.active = false
		};
	}

	// try to slow down the edge rates on the bus data drivers
	gpio_set_slew_rate(A_DO_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_slew_rate(C1_DO_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_slew_rate(C2_DO_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_slew_rate(C3_DO_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_slew_rate(C4_DO_PIN, GPIO_SLEW_RATE_SLOW);
	// same for the power switch drivers
	gpio_set_slew_rate(C1_PSW_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_slew_rate(C2_PSW_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_slew_rate(C3_PSW_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_slew_rate(C4_PSW_PIN, GPIO_SLEW_RATE_SLOW);

	// set the power switches to drive low (idle) by default
	gpio_put(C1_PSW_PIN, 0);
	gpio_set_dir(C1_PSW_PIN, GPIO_OUT);
	gpio_init(C1_PSW_PIN);
	gpio_put(C2_PSW_PIN, 0);
	gpio_set_dir(C2_PSW_PIN, GPIO_OUT);
	gpio_init(C2_PSW_PIN);
	gpio_put(C3_PSW_PIN, 0);
	gpio_set_dir(C3_PSW_PIN, GPIO_OUT);
	gpio_init(C3_PSW_PIN);
	gpio_put(C4_PSW_PIN, 0);
	gpio_set_dir(C4_PSW_PIN, GPIO_OUT);
	gpio_init(C4_PSW_PIN);

	// setup bus data drivers to push low (idle) by default
	const uint32_t outpin_masks = A_DO_PIN_bm
			| C1_DO_PIN_bm
			| C2_DO_PIN_bm
			| C3_DO_PIN_bm
			| C4_DO_PIN_bm;
	pio_sm_set_pins_with_mask(MACHINE_PIO, 0, 0, outpin_masks);
	pio_sm_set_pins_with_mask(HOSTING_PIO, 0, 0, outpin_masks);
	pio_sm_set_pindirs_with_mask(MACHINE_PIO, 0, outpin_masks, outpin_masks);
	pio_sm_set_pindirs_with_mask(HOSTING_PIO, 0, outpin_masks, outpin_masks);

	// place all bus drivers in their PIO instance
	pio_gpio_init(MACHINE_PIO, C1_DO_PIN);
	pio_gpio_init(MACHINE_PIO, C2_DO_PIN);
	pio_gpio_init(MACHINE_PIO, C3_DO_PIN);
	pio_gpio_init(MACHINE_PIO, C4_DO_PIN);
	pio_gpio_init(HOSTING_PIO, A_DO_PIN);

	// setup reading pins
	gpio_init(A_DI_PIN);
	gpio_init(C1_DI_PIN);
	gpio_init(C2_DI_PIN);
	gpio_init(C3_DI_PIN);
	gpio_init(C4_DI_PIN);

	// install PIO instances
	off_m_pio_tx = pio_add_program(MACHINE_PIO, &bus_tx_dev_program);
	off_m_pio_rx = pio_add_program(MACHINE_PIO, &bus_rx_dev_program);
	off_h_pio_tx = pio_add_program(HOSTING_PIO, &bus_tx_host_program);
	off_h_pio_rx = pio_add_program(HOSTING_PIO, &bus_rx_host_program);

	// setup PIO SMs for the machines
	for (uint8_t i = 0; i < MACHINE_COUNT; i++) {
		 bus_tx_dev_pio_config(&(machines[i].sm_tx),
				off_m_pio_tx, machine_do_pins[i], machine_di_pins[i]);
		 bus_rx_dev_pio_config(&(machines[i].sm_rx),
				off_m_pio_rx, machine_do_pins[i], machine_di_pins[i]);
	}

	// setup PIO SMs for the device host
	bus_tx_host_pio_config(&(sm_host_tx), off_h_pio_tx, A_DO_PIN);
	bus_rx_host_pio_config(&(sm_host_rx), off_h_pio_rx, A_DI_PIN);
}

void bus_main(void)
{
	while(true) {
		for (uint8_t i = 0; i < MACHINE_COUNT; i++) {
			bus_machine_tick(i);
		}
	}
}
