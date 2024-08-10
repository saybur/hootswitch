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
#include "pico/sync.h"
#include "pico/rand.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

#include "bus.h"
#include "bus.pio.h"
#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "hardware.h"
#include "util.h"

/*
 * Main unit interfacing between the remote computers and the local (virtual)
 * devices. This manages the low-level signal tracking as well as most of the
 * "core" emulated device functions like:
 *
 * 1) Bus resets, and the associated addressing requirements,
 * 2) Resolving virtual addresses to their associated devices,
 * 2) Maintaining register 3 for required Talk/Listen Register commands,
 *    including when the computer issuing them is not the one the user has
 *    selected,
 * 3) Dispatching Talk/Listen/Flush commands to the associated drivers.
 */

#define TIME_TLT                140
#define PIO_CMD_OFFSET          2
#define PIO_ATN_MIN             386
#define TIME_RESET_THRESH       400
#define RX_MAX_BITS             64

typedef enum {
	PHASE_IDLE,
	PHASE_ATTENTION,
	PHASE_COMMAND,
	PHASE_SRQ,
	PHASE_TLT,
	PHASE_LISTEN,
	PHASE_TALK
} bus_phase;

typedef enum {
	STATUS_OFF,               // not yet reset by remote computer
	STATUS_RESET,             // computer issued reset
	STATUS_NORMAL
} comp_status;

// "devices" are virtual ADB peripherals that are ran by an underlying driver.
typedef struct {
	uint8_t address;
	dev_driver *driver;
} comp_device;

typedef struct {
	bool active;              // true if this is the user-selected computer
	bus_phase phase;
	comp_status status;

	comp_device devices[DEVICE_MAX];
	uint8_t device_count;

	bool srq_en[DEVICE_MAX];  // true if SRQ enable flag is set in reg3
	uint16_t srq;             // queued service requests, indexed by virt addr
	uint64_t time;            // multiple use, but mostly when phase started
	bool collision;           // was the previous Talk a collision?

	uint8_t command;          // caches last command received
	uint8_t device;           // caches last active device index
	uint8_t reg;              // caches last active register (or 0)
	cmd_type command_type;    // caches last command's type
	uint8_t listen_data[8];   // sink for DMA data during Listen

	// buffers for Talk data storage on register 0-2
	uint8_t talk_data[3][8 * DEVICE_MAX];
	uint8_t talk_data_len[3][DEVICE_MAX];

	// general values to assist with debugging/info reporting
	uint32_t dbg_atn;          // attention from GPIO ISR
	uint32_t dbg_atn_shrt;     // attention from PIO directly
	uint32_t dbg_rst;          // reset conditions
	uint32_t dbg_abrt;         // any incomplete termination (these are ok)
	uint32_t dbg_err;          // incomplete termination due to coding bugs
} computer;

uint32_t const computer_do_pins[] = {
	C1_DO_PIN, C2_DO_PIN, C3_DO_PIN, C4_DO_PIN
};
uint32_t const computer_di_pins[] = {
	C1_DI_PIN, C2_DI_PIN, C3_DI_PIN, C4_DI_PIN
};
uint8_t const dma_channels[] = {
	COMPUTER_0_DMA, COMPUTER_1_DMA, COMPUTER_2_DMA, COMPUTER_3_DMA
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

static volatile computer computers[COMPUTER_COUNT];
static uint32_t off_pio_atn, off_pio_rx, off_pio_tx;
static volatile uint8_t rand_idx;

/*
 * ----------------------------------------------------------------------------
 *   Command Callback Queue
 * ----------------------------------------------------------------------------
 *
 * Simple ring buffer to store callback requests to the drivers.
 */

#define QUEUE_SIZE 16
#define QUEUE_MASK 15
typedef struct {
	dev_driver *driver;
	cmd_type type;
	uint8_t comp;
	uint8_t device;
	uint8_t reg;
	uint8_t data[8];
	uint8_t length;
} comp_command;
static volatile comp_command queue[QUEUE_SIZE];
static volatile uint8_t queue_tail;
static volatile uint8_t queue_count;

/*
 * Reserves a new command queue item for use in subsequent code, and inserts
 * the basic data from the associated computer entry. The new queue item is
 * provided back if more data needs to be added. This will return false if
 * no room is available.
 */
static bool queue_add(uint8_t *idx, uint8_t c)
{
	if (queue_count >= QUEUE_SIZE) return false;
	*idx = (queue_tail + queue_count) & QUEUE_MASK;
	queue_count++;

	// insert the basic data before returning
	queue[*idx].device = computers[c].device;
	queue[*idx].driver = computers[c].devices[queue[*idx].device].driver;
	queue[*idx].comp = c;
	queue[*idx].type = computers[c].command_type;
	queue[*idx].reg = computers[c].reg;
	return true;
}

/*
 * ----------------------------------------------------------------------------
 *   PIO / GPIO Setup Routines
 * ----------------------------------------------------------------------------
 *
 * All of these that start something also set .time to help track if a
 * condition has stalled beyond allowed levels and/or met certain durations.
 */

/*
 * Configures and activates the PIO SM for the given computer in attention-seek
 * mode. This will cause a PIO interrupt once a long (~770us or so) pulse is
 * detected.
 */
static void dev_pio_atn_start(uint8_t i)
{
	pio_sm_config pc;
	bus_atn_dev_pio_config(&pc, off_pio_atn, computer_di_pins[i]);
	pio_sm_init(COMPUTER_PIO, i, off_pio_atn, &pc);
	pio_sm_put(COMPUTER_PIO, i, PIO_ATN_MIN);
	pio_sm_set_enabled(COMPUTER_PIO, i, true);
	computers[i].time = time_us_64();
}

/*
 * Starts the given PIO SM in command-listen mode.
 */
static void dev_pio_command_start(uint8_t i)
{
	pio_sm_config pc;
	bus_rx_dev_pio_config(&pc, off_pio_rx, computer_do_pins[i], computer_di_pins[i]);
	pio_sm_init(COMPUTER_PIO, i, off_pio_rx + PIO_CMD_OFFSET, &pc);
	pio_sm_put(COMPUTER_PIO, i, 7);
	pio_sm_set_enabled(COMPUTER_PIO, i, true);
	computers[i].time = time_us_64();
}

/*
 * Starts the given PIO SM in transmit mode. This relies on the Tlt start
 * delay to avoid stalling at a bad location: be sure to submit data into this
 * before Tlt timeout.
 */
static void dev_pio_tx_start(uint8_t i)
{
	pio_sm_config pc;
	bus_tx_dev_pio_config(&pc, off_pio_tx, computer_do_pins[i], computer_di_pins[i]);
	pio_sm_init(COMPUTER_PIO, i, off_pio_tx, &pc);
	pio_sm_set_enabled(COMPUTER_PIO, i, true);
	computers[i].time = time_us_64();
}

/*
 * Starts the given PIO SM in receive (Listen) mode.
 */
static void dev_pio_rx_start(uint8_t i)
{
	pio_sm_config pc;
	bus_rx_dev_pio_config(&pc, off_pio_rx, computer_do_pins[i], computer_di_pins[i]);
	pio_sm_init(COMPUTER_PIO, i, off_pio_rx, &pc);
	pio_sm_put(COMPUTER_PIO, i, 63); // max 8 bytes, minus 1 for countdown

	dma_channel_config dc = dma_channel_get_default_config(dma_channels[i]);
	channel_config_set_transfer_data_size(&dc, DMA_SIZE_8);
	channel_config_set_dreq(&dc, pio_get_dreq(COMPUTER_PIO, i, false));
	channel_config_set_read_increment(&dc, false);
	channel_config_set_write_increment(&dc, true);
	dma_channel_configure(dma_channels[i], &dc,
			computers[i].listen_data,
			&(COMPUTER_PIO->rxf[i]),
			8,
			true); // start

	pio_sm_set_enabled(COMPUTER_PIO, i, true);
	computers[i].time = time_us_64();
}

/*
 * Stops the given PIO and clears the IRQ flag. This is _not_ comprehensive,
 * for example a stuck pin will continue to be asserted. This may need tweaks
 * in the future.
 */
static void dev_pio_stop(uint8_t i)
{
	pio_sm_set_enabled(COMPUTER_PIO, i, false);
	pio_interrupt_clear(COMPUTER_PIO, i);
}

/*
 * Activates the rising-edge interrupt for a computer port. If this returns
 * false it indicates the relevant GPIO is already high and the ISR was not
 * scheduled, typical for a slow ISR response and/or short signal of interest.
 * If true the caller should delegate to the GPIO ISR.
 */
static bool setup_gpio_isr(uint8_t i)
{
	// if waiting for Tlt then we need to know when it starts
	computers[i].time = time_us_64();
	uint32_t const pin = computer_di_pins[i];
	if (gpio_get(pin)) {
		return false;
	} else {
		// line still low, trigger on rising edge instead
		gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE, true);
		// try to handle race condition above
		if (gpio_get(pin)) {
			// sigh, line went high; make sure IRQ is now off
			gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE, false);
			gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_RISE);
			return false;
		} else {
			return true;
		}
	}
}

/*
 * ----------------------------------------------------------------------------
 *   Interrupt Routines & Support
 * ----------------------------------------------------------------------------
 */

/*
 * Processes the command byte, updates internal state with the results, and
 * indicates whether this needs to issue a SRQ.
 *
 * This will return false if the command couldn't be processed at all, probably
 * due to an interrupted command and/or mis-phase. It will otherwise return
 * true, even for foreign devices: those will get caught later.
 */
static bool isr_command_process(uint8_t i, bool *srq)
{
	if (pio_sm_is_rx_fifo_empty(COMPUTER_PIO, i)) return false;
	uint8_t cmd = bus_rx_dev_get(COMPUTER_PIO, i);
	cmd_type ctype = util_parse_cmd_type(cmd);

	// find if a driver matches the command
	uint8_t devc = computers[i].device_count;
	uint8_t d = 0;
	uint8_t addr;
	while (d < devc) {
		addr = computers[i].devices[d].address;
		if (addr == (cmd >> 4)) {
			break;
		}
		d++;
	}

	// see if SRQ is warranted
	if (ctype == TYPE_TALK && cmd & 0x3 == 0) {
		// Talk 0, if device exists we need to mask it out
		if (d < devc) {
			computers[i].srq &= ~(1U << addr);
		}
		*srq = computers[i].srq;
	}

	computers[i].command = cmd;
	computers[i].command_type = ctype;
	computers[i].device = d;
	return true;
}

/*
 * Executes the next step needed for the last command from the computer and
 * returns the next phase. If the command isn't addressing one of the virtual
 * devices the opportunity is taken here to stop further processing by
 * returning to idle.
 */
static bus_phase isr_command_execute(uint8_t i)
{
	uint8_t reg;

	// ensure this is a device owned by us
	uint8_t dev = computers[i].device;
	if (dev >= computers[i].device_count) {
		// it is not, abort further processing
		// dbg("cmp: %02X, %d", computers[i].command, dev);
		dev_pio_atn_start(i);
		return PHASE_IDLE;
	}

	switch (computers[i].command_type) {
	case TYPE_TALK:
		dev_pio_tx_start(i); // start immediately, Tlt timing is important

		// we handle register 3 for the drivers
		reg = computers[i].command & 0x3;
		computers[i].reg = reg;
		if (reg == 3) {
			bus_tx_dev_put(COMPUTER_PIO, i,
					((computers[i].srq_en[dev]) ? 0x20 : 0x00)
					| (0x40) // exceptional event, always '1' for us
					| (computers[i].devices[dev].address));
			uint8_t hndl;
			computers[i].devices[dev].driver->get_handle_func(i, dev, &hndl);
			bus_tx_dev_put(COMPUTER_PIO, i, hndl);
			return PHASE_TALK;
		} else {
			uint8_t dev = computers[i].device;
			uint8_t dlen = computers[i].talk_data_len[reg][dev];
			if (dlen == 0) {
				// no data: halt transmission and let it time out
				dev_pio_stop(i);
				dev_pio_atn_start(i);
				return PHASE_IDLE;
			} else {
				bus_tx_dev_putm(COMPUTER_PIO, i,
						computers[i].talk_data[(dev * 8)], dlen);
				return PHASE_TALK;
			}
		}
	case TYPE_LISTEN:
		// just listen to the computer, we'll handle special register 3 later
		computers[i].reg = computers[i].command & 0x3;
		dev_pio_rx_start(i);
		return PHASE_LISTEN;
	case TYPE_FLUSH:
		// no data transfer step is required, just enqueue the callback
		uint8_t qi;
		queue_add(&qi, i);
		dev_pio_atn_start(i);
		return PHASE_IDLE;
	default:
		// illegal command?
		computers[i].dbg_err++;
		dev_pio_atn_start(i);
		return PHASE_IDLE;
	}
}

static void isr_talk_complete(uint8_t i)
{
	// collision?
	if (COMPUTER_PIO->irq & (1U << (i + 4))) {
		computers[i].collision = true;
		pio_interrupt_clear(COMPUTER_PIO, i + 4);
		// leave data alone, we'll need it later
	} else {
		computers[i].collision = false;
		uint8_t dev = computers[i].device;
		uint8_t reg = computers[i].reg;
		uint8_t addr = computers[i].devices[dev].address;

		computers[i].srq &= ~(1U << addr);
		if (reg < 3) {
			computers[i].talk_data_len[computers[i].reg][dev] = 0;
		}
	}
}

static void isr_listen_complete(uint8_t i)
{
	uint8_t dev = computers[i].device;
	uint8_t reg = computers[i].reg;
	uint8_t addr = computers[i].devices[dev].address;

	uint8_t xfer = 8 - dma_channel_hw_addr(dma_channels[i])->transfer_count;
	dma_channel_abort(dma_channels[i]);

	if (reg == 3) {
		// make sure we got the right length
		if (xfer != 2) {
			// for now just fail, but we might want to log this exception
			return;
		}

		uint8_t up = computers[i].listen_data[0];
		uint8_t lw = computers[i].listen_data[1];

		// if there was a previous collision we are to ignore this
		// cheat a bit and assume last collision was targeted at this address!
		if (computers[i].collision && lw == 0xFE) {
			// last was a collision and we're being told to ignore this
		} else if (lw == 0xFD || lw == 0xFF) {
			// outright ignore, we don't use activators or self-tests
		} else if (lw == 0x00 || lw == 0xFE) {
			// change the address, ignore the handler
			if (! lw) computers[i].srq_en[dev] = up & 0x20;
			computers[i].srq &= ~(1U << (computers[i].devices[dev].address));
			computers[i].devices[dev].address = up & 0xF;
		} else {
			// propose new handler
			dev_driver *drvr = computers[i].devices[dev].driver;
			if (drvr->set_handle_func) {
				drvr->set_handle_func(i, dev, lw);
			}
		}
	} else {
		// more traditional data delivery
		uint8_t qi;
		queue_add(&qi, i);
		for (uint8_t di = 0; di < xfer; di++) {
			queue[qi].data[di] = computers[i].listen_data[di];
		}
		queue[qi].length = xfer;
	}
}

/*
 * Used during GPIO interrupts. Two cases:
 *
 * 1) Rising edge at end of attention pulse to detect if signal was long enough
 *    to trigger a reset,
 * 2) Rising edge at end of SRQ, signaling start of Tlt.
 *
 * Both of these are conditional on the line actually being low. See the
 * earlier setup function.
 */
static void computer_gpio_isr(void)
{
	for (uint8_t i = 0; i < COMPUTER_COUNT; i++) {
		if (! (gpio_get_irq_event_mask(computer_di_pins[i]) & GPIO_IRQ_EDGE_RISE)) {
			continue;
		}

		// we don't need it again (right away), disable (clears interrupt)
		gpio_set_irq_enabled(computer_di_pins[i], GPIO_IRQ_EDGE_RISE, false);

		switch (computers[i].phase) {
		case PHASE_ATTENTION:
			uint32_t td = time_us_64() - computers[i].time;
			if (td > TIME_RESET_THRESH) {
				// stop interrupt processing, note reset
				dev_pio_atn_start(i);
				computers[i].dbg_rst++;
				computers[i].phase = PHASE_IDLE;
				computers[i].status = STATUS_RESET;
			} else {
				// start command processing
				dev_pio_command_start(i);
				computers[i].dbg_atn++;
				computers[i].phase = PHASE_COMMAND;
			}
			break;

		case PHASE_SRQ:
			computers[i].phase = isr_command_execute(i);
			break;

		default:
			// probably a coding error, TODO report it
			dev_pio_atn_start(i);
			computers[i].dbg_err++;
			computers[i].phase = PHASE_IDLE;
		}
	}
}

/*
 * Fires from PIO interrupts. This happens in the following instances:
 *
 * 1) Signal of interest in attention phase (min duration attention),
 * 2) End of command, either due to timeout or the requested number of bits
 *    being collected,
 * 3) End of SRQ pulse in the above condition when the IRQ flag is cleared
 *    without stopping the state machine.
 * 4) End of data RX (Listen) due to bit timeout,
 * 5) End of data TX (Talk), either due to data underflow (OK) or collision
 *    state (not OK).
 */
static void computer_pio_isr(void)
{
	uint32_t irq = COMPUTER_PIO->irq;

	for (uint8_t i = 0; i < COMPUTER_COUNT; i++) {
		uint32_t irq_bit = 1U << i;
		if (! (irq & irq_bit)) continue;

		switch (computers[i].phase)
		{
		case PHASE_IDLE:
			dev_pio_stop(i);
			// attention signal present; is it over yet?
			if (setup_gpio_isr(i)) {
				computers[i].phase = PHASE_ATTENTION;
			} else {
				// too late, already high; assume short enough for reg atn
				dev_pio_command_start(i);
				computers[i].dbg_atn_shrt++;
				computers[i].phase = PHASE_COMMAND;
			}
			break;
		case PHASE_COMMAND:
			// process the command and start issuing a SRQ if needed
			bool srq = false;
			if (! isr_command_process(i, &srq)) {
				// abort completely
				dev_pio_stop(i);
				dev_pio_atn_start(i);
				computers[i].dbg_abrt++;
				computers[i].phase = PHASE_IDLE;
			}
			if (srq) {
				// will return into PHASE_SRQ to resolve
				pio_interrupt_clear(COMPUTER_PIO, i);
				computers[i].phase = PHASE_SRQ;
			} else {
				dev_pio_stop(i);
				// PIO will remain stopped, need to transition into GPIO ISR
				// on the rising edge of the (potentially stretched) stop bit
				if (! setup_gpio_isr(i)) {
					// stop bit already over, need to execute now while Tlt
					computers[i].phase = isr_command_execute(i);
				} else {
					// no phase change needed, GPIO IRQ will resolve
					computers[i].phase = PHASE_SRQ;
				}
			}
			break;
		case PHASE_SRQ:
			dev_pio_stop(i);
			if (! setup_gpio_isr(i)) {
				computers[i].phase = isr_command_execute(i);
			} else {
				// no phase change needed, GPIO IRQ will resolve once the
				// other device has stopped issuing a SRQ
			}
			break;
		case PHASE_TALK:
			dev_pio_stop(i);
			isr_talk_complete(i);
			dev_pio_atn_start(i);
			computers[i].phase = PHASE_IDLE;
			break;
		case PHASE_LISTEN:
			dev_pio_stop(i);
			isr_listen_complete(i);
			dev_pio_atn_start(i);
			computers[i].phase = PHASE_IDLE;
			break;
		default:
			// probably a coding error, TODO report it
			dev_pio_stop(i);
			dev_pio_atn_start(i);
			computers[i].dbg_err++;
			computers[i].phase = PHASE_IDLE;
		}
	}
}

/*
 * ----------------------------------------------------------------------------
 *   Data Handling
 * ---------------------------------------------------------------------------
 */

bool computer_data_offer(dev_driver *drv, uint8_t comp, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	if (data_len > 8) return false;
	if (comp >= COMPUTER_COUNT) return false;
	if (reg > 2) return false;

	// no driver list changes are permitted once this is set
	// must be present to protect lower code from concurrent changes
	if (computers[comp].status != STATUS_NORMAL) return false;

	// find the matching driver
	uint8_t devc = computers[comp].device_count;
	uint8_t d = 0;
	for (; d < devc; d++) {
		// compare pointers for match
		if (computers[comp].devices[d].driver == drv) {
			break;
		}
	}
	if (d == devc) return false;

	// remaining steps change internal state, protect from concurrent access
	uint32_t isr = save_and_disable_interrupts();

	// copy data
	for (uint8_t i = 0; i < data_len; i++) {
		computers[comp].talk_data[reg][(d * 8) + i] = data[i];
	}
	computers[comp].talk_data_len[reg][d] = data_len;

	// update SRQ flags if needed
	if (reg == 0) { // only for Talk 0
		computers[comp].srq |= (1U << computers[comp].devices[d].address);
	}

	restore_interrupts(isr);
	return true;
}

/*
 * ----------------------------------------------------------------------------
 *   Reset & Init Logic
 * ----------------------------------------------------------------------------
 */

void computer_init(void)
{
	assert(sizeof(computer_do_pins) / 4 == COMPUTER_COUNT);
	assert(sizeof(computer_di_pins) / 4 == COMPUTER_COUNT);
	assert(sizeof(dma_channels) == COMPUTER_COUNT);

	/*
	 * Per https://www.raspberrypi.com/documentation/pico-sdk/high_level.html
	 * pico_rand can block for 10-20us. For the low quality randomness needed
	 * during address musical chairs, we use a precomputed table and just
	 * randomize where we start in it.
	 */
	rand_idx = (uint8_t) (get_rand_32() % sizeof(randt));

	// setup storage
	uint8_t sm_mask;
	for (uint8_t i = 0; i < COMPUTER_COUNT; i++) {
		sm_mask |= (1U << i);
		computers[i] = (computer) {
			.phase = PHASE_IDLE, .status = STATUS_OFF,
			.active = false
		};
	}

	// issue claims for peripherals to avoid accidental conflicts
	hardware_alarm_claim(COMPUTER_TIMER);
	pio_claim_sm_mask(COMPUTER_PIO, sm_mask);
	dma_channel_claim(COMPUTER_0_DMA);
	dma_channel_claim(COMPUTER_1_DMA);
	dma_channel_claim(COMPUTER_2_DMA);
	dma_channel_claim(COMPUTER_3_DMA);

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

	// setup bus data drivers to push low (idle) by default
	const uint32_t outpin_masks = A_DO_PIN_bm
			| C1_DO_PIN_bm
			| C2_DO_PIN_bm
			| C3_DO_PIN_bm
			| C4_DO_PIN_bm;
	pio_sm_set_pins_with_mask(COMPUTER_PIO, 0, 0, outpin_masks);
	pio_sm_set_pindirs_with_mask(COMPUTER_PIO, 0, outpin_masks, outpin_masks);

	// assign outputs to PIO
	pio_gpio_init(COMPUTER_PIO, C1_DO_PIN);
	pio_gpio_init(COMPUTER_PIO, C2_DO_PIN);
	pio_gpio_init(COMPUTER_PIO, C3_DO_PIN);
	pio_gpio_init(COMPUTER_PIO, C4_DO_PIN);

	// install PIO programs
	off_pio_atn = pio_add_program(COMPUTER_PIO, &bus_atn_dev_program);
	off_pio_rx = pio_add_program(COMPUTER_PIO, &bus_rx_dev_program);
	off_pio_tx = pio_add_program(COMPUTER_PIO, &bus_tx_dev_program);

	// enable state machine IRQs 0-3
	pio_set_irq0_source_mask_enabled(COMPUTER_PIO, 0xF00, true);
	irq_set_exclusive_handler(COMPUTER_PIO_IRQ0, computer_pio_isr);

	// setup GPIO rising edge alert (not enabled yet)
	gpio_add_raw_irq_handler_masked((1U << C1_DI_PIN)
			| (1U << C2_DI_PIN)
			| (1U << C3_DI_PIN)
			| (1U << C4_DI_PIN),
			computer_gpio_isr);
}

void computer_start(void)
{
	for (uint8_t i = 0; i < COMPUTER_COUNT; i++) {
		dev_pio_atn_start(i);
	}

	// enable interrupts on peripherals
	irq_set_enabled(COMPUTER_PIO_IRQ0, true);
	irq_set_enabled(IO_IRQ_BANK0, true);
}

void computer_poll(void)
{
	while (queue_count) {
		volatile comp_command *q = &(queue[queue_tail]);

		// only pass commands back if the device has come out of reset
		if (computers[q->comp].status == STATUS_NORMAL) {
			switch (q->type) {
			case TYPE_TALK:
				if (q->driver->talk_func) {
					q->driver->talk_func(q->comp, q->device, q->reg);
				}
				break;
			case TYPE_FLUSH:
				if (q->driver->flush_func) {
					q->driver->flush_func(q->comp, q->device);
				}
				break;
			case TYPE_LISTEN:
				if (q->driver->listen_func) {
					q->driver->listen_func(q->comp, q->device, q->reg,
						q->data, q->length);
				}
				break;
			}
		}

		// remove item, minding safety
		uint32_t isr = save_and_disable_interrupts();
		queue_tail++;
		if (queue_tail >= QUEUE_SIZE) queue_tail = 0;
		queue_count--;
		restore_interrupts(isr);
	}

	for (uint8_t i = 0; i < COMPUTER_COUNT; i++) {
		// handle reset condition, if present
		if (computers[i].status == STATUS_RESET) {
			// reset count to 0 to stop updates via ISRs
			computers[i].device_count = 0;

			// rebuild list of drivers
			uint8_t dcnt = driver_count_devices();
			for (uint8_t d = 0; d < dcnt; d++) {
				dev_driver *drv;
				driver_get(d, &drv);
				computers[i].devices[d].address = drv->default_addr;
				computers[i].devices[d].driver = drv;

				// while we're here, call the reset handler
				drv->reset_func(i, d);
			}

			// reset length to match new set of drivers
			computers[i].device_count = dcnt;

			// mark reset complete
			computers[i].status = STATUS_NORMAL;
		}
	}
}
