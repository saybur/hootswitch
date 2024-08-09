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

#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/rand.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

#include "bus.h"
#include "bus.pio.h"
#include "device.h"
#include "driver.h"
#include "hardware.h"

/*
 * Main unit interfacing between the remote machines and the local devices.
 * This manages the low-level signal tracking, as well as most of the "core"
 * device functions like:
 *
 * 1) Bus resets, and the associated addressing requirements,
 * 2) Resolving virtual addresses to their associated devices,
 * 2) Maintaining register 3 for required Talk/Listen Register commands,
 *    including when the machine issuing them is not the one the user has
 *    selected,
 * 3) Dispatching Talk/Listen/Flush commands to the associated handlers,
 * 4) Similar functions global to all devices.
 *
 */

#define DEVICE_WATCHDOG_TICKS   1000

typedef enum {
	PHASE_IDLE,
	PHASE_ATTENTION,
	PHASE_COMMAND,
	PHASE_SRQ,
	PHASE_TLT,
	PHASE_DATA_OUT,
	PHASE_DATA_IN
} bus_phase;

typedef enum {
	STATUS_FAULT,             // lost track of bus phase
	STATUS_RESET,             // machine issued reset
	STATUS_COMMANDED,         // pending command
	STATUS_DATA_RECV,         // pending data to parse
	STATUS_NORMAL
} bus_status;

typedef struct {
	bool active;              // true if this is the user-selected machine
	bus_phase phase;
	bus_status status;
	uint32_t pin_out;         // GPIO number of line driving pin
	uint32_t pin_in;          // GPIO number of line listening pin
	dev_driver *drivers[DRIVER_MAXIMUM];
	uint64_t time;            // multiple use, generally when the phase started
	uint32_t timeout;         // threshold to register a timeout (non-idle)
	uint8_t command;          // last command received if STATUS_COMMANDED
	uint8_t message[8];       // message buffer for commands
	uint8_t message_len;
} machine;

uint32_t const machine_do_pins[] = {
	C1_DO_PIN, C2_DO_PIN, C3_DO_PIN, C4_DO_PIN
};
uint32_t const machine_di_pins[] = {
	C1_DI_PIN, C2_DI_PIN, C3_DI_PIN, C4_DI_PIN
};
uint8_t const dma_channels[] = {
	MACHINE_0_DMA, MACHINE_1_DMA, MACHINE_2_DMA, MACHINE_3_DMA
};

const uint8_t randt[] = {
	0x67, 0xC6, 0x69, 0x73, 0x51, 0xFF, 0x4A, 0xEC,
	0x29, 0xCD, 0xBA, 0xAB, 0xF2, 0xFB, 0xE3, 0x46,
	0x7C, 0xC2, 0x54, 0xF8, 0x1B, 0xE8, 0xE7, 0x8D,
	0x76, 0x5A, 0x2E, 0x63, 0x33, 0x9F, 0xC9, 0x9A,
	0x66, 0x32, 0x0D, 0xB7, 0x31, 0x58, 0xA3, 0x5A,
	0x25, 0x5D, 0x05, 0x17, 0x58, 0xE9, 0x5E, 0xD4,
	0xAB, 0xB2, 0xCD, 0xC6, 0x9B, 0xB4, 0x54, 0x11,
	0x0E, 0x82, 0x74, 0x41, 0x21, 0x3D, 0xDC, 0x87
};

static volatile machine machines[MACHINE_COUNT];
static uint32_t off_pio_tx, off_pio_rx, off_pio_atn;
static uint8_t rand_idx;

static void device_watchdog(unsigned int alarm_num)
{

}

static void device_pio_isr(void)
{

}

void device_init(void)
{
	assert(sizeof(machine_do_pins) / 4 == MACHINE_COUNT);
	assert(sizeof(machine_di_pins) / 4 == MACHINE_COUNT);

	/*
	 * Per https://www.raspberrypi.com/documentation/pico-sdk/high_level.html
	 * pico_rand can block for 10-20us. For the low quality randomness needed
	 * during address musical chairs, we use a precomputed table and just
	 * randomize where we start in it.
	 */
	rand_idx = (uint8_t) (get_rand_32() % sizeof(randt));

	// setup storage
	uint8_t sm_mask;
	for (uint8_t i = 0; i < MACHINE_COUNT; i++) {
		sm_mask |= (1U << i);
		machines[i] = (machine) {
			.phase = PHASE_IDLE, .status = STATUS_NORMAL,
			.pin_out = machine_do_pins[i], .pin_in = machine_di_pins[i],
			.active = false
		};
	}

	// issue claims for peripherals to avoid accidental conflicts
	hardware_alarm_claim(MACHINE_TIMER);
	pio_claim_sm_mask(MACHINE_PIO, sm_mask);
	dma_channel_claim(MACHINE_0_DMA);
	dma_channel_claim(MACHINE_1_DMA);
	dma_channel_claim(MACHINE_2_DMA);
	dma_channel_claim(MACHINE_3_DMA);

	// try to slow down the edge rates on the bus data drivers
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
	pio_sm_set_pindirs_with_mask(MACHINE_PIO, 0, outpin_masks, outpin_masks);

	// assign outputs to PIO
	pio_gpio_init(MACHINE_PIO, C1_DO_PIN);
	pio_gpio_init(MACHINE_PIO, C2_DO_PIN);
	pio_gpio_init(MACHINE_PIO, C3_DO_PIN);
	pio_gpio_init(MACHINE_PIO, C4_DO_PIN);

	// enable state machine IRQs 0-3
	pio_set_irq0_source_mask_enabled(MACHINE_PIO, 0xF00, true);
	irq_set_exclusive_handler(MACHINE_PIO_IRQ0, device_pio_isr);

	// setup input pins
	gpio_init(C1_DI_PIN);
	gpio_init(C2_DI_PIN);
	gpio_init(C3_DI_PIN);
	gpio_init(C4_DI_PIN);

	// install PIO programs
	off_pio_tx = pio_add_program(MACHINE_PIO, &bus_tx_dev_program);
	off_pio_rx = pio_add_program(MACHINE_PIO, &bus_rx_dev_program);
	off_pio_atn = pio_add_program(MACHINE_PIO, &bus_atn_dev_program);
}

/*
 * Starts listening to the connected devices.
 */
void device_start(void)
{

}
