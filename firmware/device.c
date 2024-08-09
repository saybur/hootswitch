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
#include "util.h"

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
#define PIO_CMD_OFFSET          2
#define PIO_ATN_MIN             775
#define PIO_ATN_RESET_THRESH    4294966421UL

typedef enum {
	PHASE_IDLE,
	PHASE_COMMAND,
	PHASE_SRQ,
	PHASE_TLT,

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
	uint8_t address;
	dev_driver *driver;
} mach_driver;

typedef struct {
	bool active;              // true if this is the user-selected machine

	bus_phase phase;

	bus_status status;

	mach_driver drivers[DRIVER_MAXIMUM];
	uint8_t driver_count;

	uint32_t attention;       // duration value of last attention signal
	uint16_t srq;             // queued service requests

	uint64_t time;            // multiple use, generally when the phase started
	uint32_t timeout;         // threshold to register a timeout (non-idle)

	uint8_t command;          // last command received if STATUS_COMMANDED
	cmd_type command_type;

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
	DEVICE_0_DMA, DEVICE_1_DMA, DEVICE_2_DMA, DEVICE_3_DMA
};
uint32_t const dma_ints_mask[] = {
	1U << DEVICE_0_DMA, 1U << DEVICE_1_DMA,
	1U << DEVICE_2_DMA, 1U << DEVICE_3_DMA
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

static volatile machine machines[DEVICE_COUNT];
static uint32_t off_pio_atn, off_pio_rx, off_pio_tx;
static uint8_t rand_idx;

static void enable_pio_atn(uint8_t i)
{
	pio_sm_config pc;
	bus_atn_dev_pio_config(&pc, off_pio_atn, machine_di_pins[i]);
	pio_sm_init(DEVICE_PIO, i, off_pio_atn, &pc);
	pio_sm_put(DEVICE_PIO, i, PIO_ATN_MIN);
	pio_sm_set_enabled(DEVICE_PIO, i, true);

	dma_channel_config dc = dma_channel_get_default_config(dma_channels[i]);
	channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
	channel_config_set_dreq(&dc, pio_get_dreq(DEVICE_PIO, i, false));
	channel_config_set_read_increment(&dc, false);
	channel_config_set_write_increment(&dc, false);
	dma_channel_configure(dma_channels[i], &dc,
			&(machines[i].attention),
			&(DEVICE_PIO->rxf[i]),
			1,
			true);
	dma_irqn_set_channel_mask_enabled(DEVICE_DMA_IDX, dma_channels[i], true);
}

// used to fire TX at the appropriate time
// more general policing of stuck lines is in the polling handler
static void device_timer_isr(void)
{
	// TODO implement
}

// used to detect the end of SRQ so TX/RX can be queued properly
static void device_gpio_isr(void)
{
	// TODO implement
}

// fires during the end of the attention/reset signal only
static void device_dma_isr(void)
{
	// get and clear DMA interrupt bits
	uint32_t ints = DEVICE_DMA_INTS;
	DEVICE_DMA_INTS = ints;

	for (uint8_t i = 0; i < DEVICE_COUNT; i++) {
		if (! (ints & dma_ints_mask[i])) continue;

		// check whether this was attention or reset
		if (machines[i].attention < PIO_ATN_RESET_THRESH) {
// TODO RESET
		} else {
			machines[i].phase = PHASE_COMMAND;

			pio_sm_set_enabled(DEVICE_PIO, i, false);
			pio_sm_config pc;
			bus_rx_dev_pio_config(&pc, off_pio_rx,
					machine_do_pins[i], machine_di_pins[i]);
			pio_sm_init(DEVICE_PIO, i, off_pio_rx + PIO_CMD_OFFSET, &pc);
			pio_sm_put(DEVICE_PIO, i, 8);
			pio_sm_set_enabled(DEVICE_PIO, i, true);
		}
	}
}

// fires when PIO state machines need servicing by the CPU ('irq wait 0 rel')
static void device_pio_isr(void)
{
	uint32_t irq = DEVICE_PIO->irq;

	for (uint8_t i = 0; i < DEVICE_COUNT; i++) {
		uint32_t irq_bit = 1U << i;
		if (! (irq & irq_bit)) continue;
		pio_sm_set_enabled(DEVICE_PIO, i, false);

		bus_phase phase = machines[i].phase;
		if (phase == PHASE_COMMAND) {
			if (pio_sm_is_rx_fifo_empty(DEVICE_PIO, i)) {
				// command aborted
				machines[i].phase = PHASE_IDLE;
				enable_pio_atn(i);
				continue;
			}

			// otherwise command success, decode
			uint8_t cmd = bus_rx_dev_get(DEVICE_PIO, i);
			cmd_type ctype = util_parse_cmd_type(cmd);

			// find if a device matches the command
			uint8_t devc = machines[i].driver_count;
			uint8_t d = 0;
			for (; d < devc; d++) {
				if (machines[i].drivers[d].address == (cmd >> 4)) {
					break;
				}
			}

			if (d == devc) {
				// no match, but still need to check SRQ
				machines[i].command = 0; // to let future handler skip us
				if (ctype == TYPE_TALK && cmd & 0x3 == 0) {
					// Talk 0, if we have any pending SRQ issue it
					if (machines[i].srq) {
						machines[i].phase = PHASE_SRQ;
					}
				}
				continue;
			}

			// determine next step
			bus_phase next_phase;
			if (ctype == TYPE_TALK) {

				// check if SRQ is needed
				next_phase = PHASE_TLT;
				if (cmd & 0x3 == 0) {
					// Talk 0, SRQ may be needed
					uint16_t srq = machines[i].srq;
					srq &= ~(1U << cmd & (0xF0));
					if (srq) {
						// release IRQ line to allow PIO fire SRQ
						pio_interrupt_clear(HOST_PIO, i);
						next_phase = PHASE_SRQ;
					}
				}

// TODO figure out data

			} else if (ctype == TYPE_LISTEN) {
// Needs a copy of the GPIO interrupt code below
			} else if (ctype == TYPE_FLUSH) {
// TODO log Flush
				// done at this point
				machines[i].phase = PHASE_IDLE;
				enable_pio_atn(i);
			} else {
// TODO error
			}

			// if waiting for Tlt then we need to know when it starts
			if (gpio_get(machine_di_pins[i])) {
				// already high, just guess from here
// TODO schedule timer
			} else {
				// line still low, trigger on rising edge instead
//	GPIO_IRQ_EDGE_RISE
				// try to handle race condition above
				if (gpio_get(machine_di_pins[i])) {
					// sigh, line went high; make sure IRQ is now off
					gpio_acknowledge_irq(machine_di_pins[i], GPIO_IRQ_EDGE_RISE);
				} else {
					// good shape, schedule timer
// TODO schedule timer
				}
			}

		} else if (phase == PHASE_SRQ) {
			if (machines[i].command == 0) {
				// done at this point
				machines[i].phase = PHASE_IDLE;
				enable_pio_atn(i);
			} else {
// TODO schedule timer
			}
		} else {
// TODO need a phase error
		}
	}
}

void device_init(void)
{
	assert(sizeof(machine_do_pins) / 4 == DEVICE_COUNT);
	assert(sizeof(machine_di_pins) / 4 == DEVICE_COUNT);
	assert(sizeof(dma_channels) == DEVICE_COUNT);

	/*
	 * Per https://www.raspberrypi.com/documentation/pico-sdk/high_level.html
	 * pico_rand can block for 10-20us. For the low quality randomness needed
	 * during address musical chairs, we use a precomputed table and just
	 * randomize where we start in it.
	 */
	rand_idx = (uint8_t) (get_rand_32() % sizeof(randt));

	// setup storage
	uint8_t sm_mask;
	for (uint8_t i = 0; i < DEVICE_COUNT; i++) {
		sm_mask |= (1U << i);
		machines[i] = (machine) {
			.phase = PHASE_IDLE, .status = STATUS_NORMAL,
			.active = false
		};
	}

	// issue claims for peripherals to avoid accidental conflicts
	hardware_alarm_claim(DEVICE_TIMER);
	pio_claim_sm_mask(DEVICE_PIO, sm_mask);
	dma_channel_claim(DEVICE_0_DMA);
	dma_channel_claim(DEVICE_1_DMA);
	dma_channel_claim(DEVICE_2_DMA);
	dma_channel_claim(DEVICE_3_DMA);

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

	// setup input pins
	gpio_init(C1_DI_PIN);
	gpio_init(C2_DI_PIN);
	gpio_init(C3_DI_PIN);
	gpio_init(C4_DI_PIN);

	/*		c = dma_channel_get_default_config(dma_channels[i]);
		channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
		channel_config_set_dreq(&c, pio_get_dreq(DEVICE_PIO, i, false));
		channel_config_set_read_increment(&c, false);
		channel_config_set_write_increment(&c, true);
		machines[i].dma_cfg_rx = c;*/

	// setup bus data drivers to push low (idle) by default
	const uint32_t outpin_masks = A_DO_PIN_bm
			| C1_DO_PIN_bm
			| C2_DO_PIN_bm
			| C3_DO_PIN_bm
			| C4_DO_PIN_bm;
	pio_sm_set_pins_with_mask(DEVICE_PIO, 0, 0, outpin_masks);
	pio_sm_set_pindirs_with_mask(DEVICE_PIO, 0, outpin_masks, outpin_masks);

	// assign outputs to PIO
	pio_gpio_init(DEVICE_PIO, C1_DO_PIN);
	pio_gpio_init(DEVICE_PIO, C2_DO_PIN);
	pio_gpio_init(DEVICE_PIO, C3_DO_PIN);
	pio_gpio_init(DEVICE_PIO, C4_DO_PIN);

	// install PIO programs
	off_pio_atn = pio_add_program(DEVICE_PIO, &bus_atn_dev_program);
	off_pio_rx = pio_add_program(DEVICE_PIO, &bus_rx_dev_program);
	off_pio_tx = pio_add_program(DEVICE_PIO, &bus_tx_dev_program);

	// enable state machine IRQs 0-3
	pio_set_irq0_source_mask_enabled(DEVICE_PIO, 0xF00, true);
	irq_set_exclusive_handler(DEVICE_PIO_IRQ0, device_pio_isr);

	// assign DMA interrupt handler
	irq_set_exclusive_handler(DEVICE_DMA_IRQ, device_dma_isr);

	// setup GPIO rising edge alert (not enabled yet)
	irq_set_exclusive_handler(IO_IRQ_BANK0, device_gpio_isr);

	// setup timer
	irq_set_exclusive_handler(DEVICE_TIMER_IRQ, device_timer_isr);
	hw_set_bits(&timer_hw->inte, 1U << DEVICE_TIMER);
	irq_set_enabled(DEVICE_TIMER_IRQ, true);
}

/*
 * Starts listening to the connected devices.
 */
void device_start(void)
{
	for (uint8_t i = 0; i < DEVICE_COUNT; i++) {
		enable_pio_atn(i);
	}

	// enable interrupts on all peripherals
	hw_set_bits(&timer_hw->inte, 1U << DEVICE_TIMER);
	irq_set_enabled(DEVICE_TIMER_IRQ, true);
	irq_set_enabled(DEVICE_PIO_IRQ0, true);
	irq_set_enabled(DEVICE_DMA_IRQ, true);
}
