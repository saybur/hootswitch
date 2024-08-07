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
#include "pico/critical_section.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

#include "bus.pio.h"
#include "debug.h"
#include "hardware.h"
#include "host.h"
#include "host_sync.h"
#include "led.h"
#include "util.h"

#define DEVICE_MAX        14
#define DMA_MAX_BITS      65

// see the PIO host definitions for what these values mean
#define PIO_CMD_OFFSET    2
#define PIO_RESET_VAL     59
#define PIO_CMD_VAL       15
#define PIO_RX_TIME_VAL   110

// a few more common timer-based timeouts
#define MIN_TLT           140
#define COMMAND_TIMEOUT   (800 + 70 + 800 + 65 + 300 + 200)
#define RESET_TIMEOUT     (2200 + COMMAND_TIMEOUT)
#define RX_MAX_TIMEOUT    (300 + 66 * 130)
#define TYP_CMD_GAP       1000

// trackers for the PIO state machine used
static uint32_t pio_offset;
static uint8_t pio_sm;
static bool pio_tx_prog;

// DMA data reception from devices
static uint8_t dma_chan;
static dma_channel_config dma_rx_cfg;

typedef enum {
	HOST_IDLE,
	HOST_COMMAND,
	HOST_TALK,
	HOST_LISTEN_TLT,
	HOST_LISTEN
} host_phase;

// devID->address and devID->handler tables
static volatile ndev_info devices[DEVICE_MAX];
static volatile ndev_handler *device_handlers[DEVICE_MAX];
static volatile uint8_t device_count;

/*
 * Queue for requested commands on devices, implemented as a simple ring buffer
 * with a tail pointer and size value. Queue size must be a power of 2. Queue
 * mask must be one less than that to allow for efficient wrap operations.
 *
 * `queue_id` is used to generate `id` values.
 *
 * The state tracker here is used by the async routines to hunt for commands
 * that can be sent on the bus. See below for details.
 */
#define CMD_QUEUE_SIZE 16
#define CMD_QUEUE_MASK 15
typedef enum {
	CMD_EMPTY = 0,
	CMD_PENDING,
	CMD_DONE
} cmd_state;
typedef struct {
	cmd_state state;
	uint16_t id;
	uint8_t device;
	cmd_type type;
	uint8_t command;
	uint8_t data[8];
	uint8_t length;
	host_err error;
} host_command;
static volatile host_command queue[CMD_QUEUE_SIZE];
static volatile uint8_t queue_tail;
static volatile uint8_t queue_count;
static volatile uint16_t queue_id = 1;

/*
 * ----------------------------------------------------------------------------
 *   PIO Support Functions
 * ----------------------------------------------------------------------------
 */

// swaps the PIO program out (if needed)
static void host_pio_load_tx(bool tx)
{
	// important: the timer firing during this would be a problem!
	pio_sm_set_enabled(HOST_PIO, pio_sm, false);
	pio_interrupt_clear(HOST_PIO, pio_sm);
	if (tx != pio_tx_prog) {
		if (tx) {
			pio_remove_program(HOST_PIO, &bus_rx_host_program, pio_offset);
			pio_add_program_at_offset(HOST_PIO, &bus_tx_host_program, pio_offset);
			pio_tx_prog = true;
		} else {
			pio_remove_program(HOST_PIO, &bus_tx_host_program, pio_offset);
			pio_add_program_at_offset(HOST_PIO, &bus_rx_host_program, pio_offset);
			pio_tx_prog = false;
		}
	}
}

static volatile uint16_t rxbuf[DMA_MAX_BITS];

// parses device reception buffer and writes result to supplied byte array
// returns the number of recognized bytes
static uint8_t host_parse_rxbuf(volatile uint8_t *data, uint8_t bits)
{
	if (bits > DMA_MAX_BITS) bits = DMA_MAX_BITS;
	if (bits < 1) return 0;
	bits -= 1; // first bit is start bit, ignore

	uint8_t bytes = bits / 8;
	volatile uint16_t *in = &(rxbuf[1]);
	for (uint i = 0; i < bytes; i++) {
		uint8_t b = 0;
		for (uint j = 0; j < 8; j++) {
			if (j != 0) b = b << 1;
			uint16_t v = *in++;
			if ((v >> 8) > (v & 0xFF)) {
				b |= 1;
			}
		}
		data[i] = b;
	}
	return bytes;
}

/*
 * ----------------------------------------------------------------------------
 *   Interrupt Routines / Async Command Execution
 * ----------------------------------------------------------------------------
 */

static volatile bool idle_poll;
static volatile host_phase phase = HOST_IDLE;
static volatile uint16_t timeout;
static volatile bool srq;

static uint8_t cmd_idx;
static uint8_t cmd_last_dev;

// generic PIO stop routine for the timer
static inline void host_stop_pio(void)
{
	pio_sm_set_enabled(HOST_PIO, pio_sm, false);
	pio_interrupt_clear(HOST_PIO, pio_sm);
	pio_sm_set_pins_with_mask(HOST_PIO, 0, 0, A_DO_PIN_bm);
}

// generic setup for calling back into the timer when a command completes
static inline void host_cmd_complete(void)
{
	phase = HOST_IDLE;
	timer_hw->alarm[HOST_TIMER] = time_us_32() + TYP_CMD_GAP;
}

/*
 * ISR callback for the timer. Used to implement periodic polling, as well as
 * handle timeouts that the PIO unit can't resolve (usually because it has
 * stalled somewhere).
 *
 * This assumes it has the same interrupt priority as the PIO ISR (as in, they
 * can't interrupt each other).
 */
static void host_timer(void)
{
	// clear trigger that brought us here
	timer_hw->intr = 1U << HOST_TIMER;

	switch (phase)
	{
		case HOST_IDLE:
			// scan the queue looking for a pending command
			cmd_idx = queue_tail;
			for (uint8_t i = 0; i < queue_count - 1; i++) {
				if (queue[cmd_idx].state == CMD_PENDING) break;
				cmd_idx = (cmd_idx + i) & CMD_QUEUE_MASK;
			}

			if (queue_count > 0 && queue[cmd_idx].state == CMD_PENDING) {
				cmd_idx = cmd_idx;
				cmd_last_dev = queue[cmd_idx].device;
			} else {
				// none pending, check if there is space to add a command;
				// if idle polling is not enabled then always take this branch
				if (! idle_poll
						|| cmd_last_dev >= device_count
						|| queue_count >= CMD_QUEUE_SIZE) {
					// no, short-circuit to wait a bit for space to free up
					// or the init routines to help us out a bit
					timer_hw->alarm[HOST_TIMER] = time_us_32() + TYP_CMD_GAP;
					return;
				} else {
					// there is space, insert a new Talk 0 prompt for us
					cmd_idx = (queue_tail + queue_count) & CMD_QUEUE_MASK;
					queue[cmd_idx].state = CMD_PENDING;
					queue[cmd_idx].id = 0;
					queue[cmd_idx].device = cmd_last_dev;
					queue[cmd_idx].command =
							((devices[cmd_last_dev].address_cur) << 4)
							| (uint8_t) COMMAND_TALK_0;
					queue[cmd_idx].error = HOSTERR_OK;
				}
			}

			// start command
			host_pio_load_tx(true);
			pio_sm_config c;
			bus_tx_host_pio_config(&c, pio_offset, A_DO_PIN, A_DI_PIN);
			pio_sm_init(HOST_PIO, pio_sm, pio_offset + PIO_CMD_OFFSET, &c);
			if (queue[cmd_idx].command == 0x00) {
				// use alternate long reset pulse
				bus_tx_host_put(HOST_PIO, pio_sm, PIO_RESET_VAL);
				timeout = RESET_TIMEOUT;
			} else {
				// normal attention signal
				bus_tx_host_put(HOST_PIO, pio_sm, PIO_CMD_VAL);
				timeout = COMMAND_TIMEOUT;
			}
			bus_tx_host_put(HOST_PIO, pio_sm, queue[cmd_idx].command);
			pio_sm_set_enabled(HOST_PIO, pio_sm, true);

			// advance and setup fallback timer
			phase = HOST_COMMAND;
			timer_hw->alarm[HOST_TIMER] = time_us_32() + timeout;
			break;

		case HOST_COMMAND:
		case HOST_TALK:
			// should only happen if the line sticks low, should be rare
			host_stop_pio();
			queue[cmd_idx].state = CMD_DONE;
			queue[cmd_idx].error = HOSTERR_LINE_STUCK;
			host_cmd_complete();
			break;

		case HOST_LISTEN_TLT:
			// at end of Tlt, start transmission
			pio_sm_set_enabled(HOST_PIO, pio_sm, true);
			// add timeout safety, calculated in PIO ISR
			timer_hw->alarm[HOST_TIMER] = time_us_32() + timeout;
			phase = HOST_LISTEN;

			break;
		case HOST_LISTEN:
			// line locked up
			host_stop_pio();
			queue[cmd_idx].state = CMD_DONE;
			queue[cmd_idx].error = HOSTERR_LINE_STUCK;
			host_cmd_complete();

			break;
	}
}

// ISR callback for the PIO state machine
static void host_pio_isr(void)
{
	// unconditionally stop PIO
	host_stop_pio();
	// cancel fallback alarm / make sure it isn't waiting to fire
	timer_hw->armed = 1U << HOST_TIMER;
	timer_hw->intr = 1U << HOST_TIMER;

	switch (phase)
	{
		case HOST_COMMAND:
			// triggered by successful end of command

			// load SRQ state and clear it out of the PIO
			srq = HOST_PIO->irq & (1U << (pio_sm + 4));
			pio_interrupt_clear(HOST_PIO, pio_sm + 4);

			// check if there is a follow-on data phase
			pio_sm_config c;
			switch (queue[cmd_idx].type) {
			case TYPE_TALK:
				host_pio_load_tx(false);

				bus_rx_host_pio_config(&c, pio_offset, A_DI_PIN);
				pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
				pio_sm_put(HOST_PIO, pio_sm, PIO_RX_TIME_VAL);
				dma_channel_configure(dma_chan, &dma_rx_cfg,
						rxbuf,
						&(HOST_PIO->rxf[pio_sm]),
						DMA_MAX_BITS, // 8 bytes plus start/stop bits
						true); // start

				// start immediately, the PIO will stall until data comes in
				pio_sm_set_enabled(HOST_PIO, pio_sm, true);

				// move to next phase, include safety valve
				phase = HOST_TALK;
				timer_hw->alarm[HOST_TIMER] = time_us_32() + RX_MAX_TIMEOUT;
				break;
			case TYPE_LISTEN:
				// start timer immediately to avoid missing the window
				timer_hw->alarm[HOST_TIMER] = time_us_32() + MIN_TLT;

				// setup for sending
				bus_tx_host_pio_config(&c, pio_offset, A_DO_PIN, A_DI_PIN);
				pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
				for (uint8_t i = 0; i < queue[cmd_idx].length; i++) {
					bus_tx_host_put(HOST_PIO, pio_sm, queue[cmd_idx].data[i]);
				}
				timeout = 100 + 800 * queue[cmd_idx].length + 250;

				// timer firing will start transfer
				phase = HOST_LISTEN_TLT;
				break;
			default:
				// no data transfer required
				queue[cmd_idx].state = CMD_DONE;
				host_cmd_complete();
			}
			break;

		case HOST_TALK:
			// normal end of Talk, either timeout or data response
			uint8_t dma_remain = dma_channel_hw_addr(dma_chan)->transfer_count;
			dma_channel_abort(dma_chan);
			uint8_t len = host_parse_rxbuf(queue[cmd_idx].data,
					DMA_MAX_BITS - dma_remain);
			queue[cmd_idx].length = len;
			if (len == 0) queue[cmd_idx].error = HOSTERR_TIMEOUT;

			queue[cmd_idx].state = CMD_DONE;
			host_cmd_complete();
			break;

		case HOST_LISTEN:
			queue[cmd_idx].state = CMD_DONE;
			host_cmd_complete();
			break;

		default:
			queue[cmd_idx].state = CMD_DONE;
			queue[cmd_idx].error = HOSTERR_BAD_STATE;
			host_cmd_complete();
	}
}

/*
 * ----------------------------------------------------------------------------
 *   Reset & Init Logic
 * ----------------------------------------------------------------------------
 */

// moves from async->sync (if needed) and sends the actual reset command
static host_err host_reset_bus(void)
{
	// send a reset to all handlers
	for (uint8_t i = 0; i < handler_count(); i++) {
		ndev_handler *handler;
		handler_get(i, &handler);
		if (handler != NULL) {
			handler->reset_func();
		}
	}

	// drain the queue and wait for the system to go idle
	idle_poll = false;
	queue_count = 0;
	while (phase != HOST_IDLE);

	// send reset command
	return host_sync_cmd(0xFF, 0x00, NULL, NULL);
}

// performs ADB address shuffling
static host_err host_reset_addresses(void)
{
	// wipe existing device/handler assignments
	for (uint8_t i = 0; i < DEVICE_MAX; i++) {
		device_handlers[i] = NULL;
	}

	// readdress the devices
	device_count = 0;
	host_err err = HOSTERR_OK;
	uint8_t addr_free = 0xE;
	uint8_t data[2] = { 0 };
	uint8_t datalen;

	for(uint8_t i = 1; i <= 7; i++) {
		dbg("  addr $%X {", i);
		uint8_t dev_at_addr = 0;

		// move all devices at address to free slots
		datalen = 2;
		uint8_t cmd = (i << 4) | ((uint8_t) COMMAND_TALK_3);
		while (! (err = host_sync_cmd(0xFF, cmd, data, &datalen))) {
			// did device return the right amount of data?
			if (datalen != 2) {
				dbg_err("    dev reg3 != 2b, %d", datalen);
				return HOSTERR_BAD_DEVICE;
			}

			// found a device, report the default handler
			dbg("    fnd $%X (id %d)", data[1], device_count);
			// and move it to a free address
			dev_at_addr++;
			uint8_t dhid = data[1];
			dbg("    mov $%X", addr_free);
			data[0] = addr_free;
			data[1] = 0xFE;
			datalen = 2;
			uint8_t cmd_move = (i << 4) | ((uint8_t) COMMAND_LISTEN_3);
			if (err = host_sync_cmd(0xFF, cmd_move, data, &datalen)) {
				dbg_err("    (!) %d", err);
				return err;
			}

			// store data on device
			devices[device_count].address_def = i;
			devices[device_count].address_cur = addr_free;
			devices[device_count].dhid_def = dhid;
			devices[device_count].dhid_cur = dhid;
			devices[device_count].fault = false;
			device_count++;

			// advance for next
			addr_free--;
			if (addr_free < 0x7) {
				dbg_err("    (!) %d", HOSTERR_TOO_MANY_DEVICES);
				return HOSTERR_TOO_MANY_DEVICES;
			}
		}
		if (err != HOSTERR_TIMEOUT) {
			// bail out, all non-timeout conditions are fatal
			dbg_err("  } (!) %d", err);
			return err;
		}

		// move last-moved device back to original address
		// different from the ADB Manager's first-moved procedure, but this
		// seems simpler and should (in theory) be OK with all devices
		if (dev_at_addr > 0) {
			addr_free++;
			dbg("    mov $%X, $%X", i, addr_free);
			data[0] = i;
			data[1] = 0x00;
			datalen = 2;
			cmd = (addr_free << 4) | ((uint8_t) COMMAND_LISTEN_3);
			if (err = host_sync_cmd(0xFF, cmd, data, &datalen)) {
				dbg_err("    (!) %d", err);
				return err;
			}
			devices[(device_count - 1)].address_cur = i;
		}
		dbg("  } got %d", dev_at_addr);
	}

	dbg("re-addr ok!");
	return HOSTERR_OK;
}

static host_err host_handle_setup(void)
{
	// perform a reset on all handlers
	for (uint8_t i = 0; i < handler_count(); i++) {
		ndev_handler *handler;
		handler_get(i, &handler);
		if (handler != NULL) {
			handler->reset_func();
		}
	}

	// scan the devices and assign handlers
	for (uint8_t hid = handler_count(); hid > 0; hid--) {
		ndev_handler *handler;
		handler_get(hid - 1, &handler);
		if (handler == NULL) continue;

		dbg("  handler '%s':", handler->name);
		for (uint8_t did = 0; did < device_count; did++) {
			volatile ndev_info *device = &(devices[did]);

			// disallow if a device is already owned
			if (device_handlers[did] != NULL) continue;
			// disallow if device faulted
			if (device->fault) continue;

			// might be OK, check with the handler and see if they want it
			uint8_t herr = 0;
			bool res = handler->interview_func(device, &herr);
			if (herr) {
				dbg_err("    id %d bad interview (%d), skip", did, herr);
				device->fault = true;
			} else if (res) {
				dbg("    id %d accepted", did);
				device_handlers[did] = handler;
			}
		}
	}
}

// removes faulted devices and devices with no handler from the listing to
// avoid dealing with them later
static host_err host_prune_faulted(void)
{
	for (uint8_t i = 0; i < device_count; i++) {
		if (devices[i].fault || device_handlers[i] == NULL) {
			dbg_err("host drop dev %d", i);
			for (uint8_t j = i; j < device_count - 1; j++) {
				devices[j] = devices[j+1];
				device_handlers[j] = device_handlers[j+1];
			}
			device_count--;
		}
	}
}

host_err host_reset(void)
{
	host_err err;

	dbg("host reset");
	if (err = host_reset_bus()) {
		return err;
	}

	dbg("wait for device reset...");
	busy_wait_ms(2000);

	dbg("host re-addr");
	if (err = host_reset_addresses()) {
		return err;
	}

	dbg("assign handlers");
	if (err = host_handle_setup()) {
		return err;
	}

	// make sure there is at least one working device before resuming
	host_prune_faulted();
	if (device_count == 0) {
		dbg_err("disabling host, no devices!");
		return HOSTERR_NO_DEVICES;
	}

	// re-enable general polling of Talk 0
	idle_poll = true;
}

void host_init(void)
{
	// perform setup of the host pins
	gpio_set_slew_rate(A_DO_PIN, GPIO_SLEW_RATE_SLOW);
	pio_sm_set_pins_with_mask(HOST_PIO, 0, 0, A_DO_PIN_bm);
	pio_sm_set_pindirs_with_mask(HOST_PIO, 0, A_DO_PIN_bm, A_DO_PIN_bm);
	pio_gpio_init(HOST_PIO, A_DO_PIN);
	gpio_init(A_DI_PIN);

	// leave the largest of the two programs in PIO instruction memory,
	// this will be swapped in and out as needed.
	assert(sizeof(bus_tx_host_program_instructions)
			>= sizeof(bus_rx_host_program_instructions));
	pio_offset = pio_add_program(HOST_PIO, &bus_tx_host_program);
	// change the following default if RX ever becomes > TX
	pio_tx_prog = true;

	// issue claims for peripherals to avoid accidental conflicts
	pio_sm = pio_claim_unused_sm(HOST_PIO, true);
	dma_chan = dma_claim_unused_channel(true);
	hardware_alarm_claim(HOST_TIMER);

	// create base DMA configuration
	dma_rx_cfg = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&dma_rx_cfg, DMA_SIZE_16);
	channel_config_set_dreq(&dma_rx_cfg, pio_get_dreq(HOST_PIO, pio_sm, false));
	channel_config_set_read_increment(&dma_rx_cfg, false);
	channel_config_set_write_increment(&dma_rx_cfg, true);

	// install the interrupt handlers
	irq_set_exclusive_handler(HOST_PIO_IRQ0, host_pio_isr);
	irq_set_exclusive_handler(HOST_TIMER_IRQ, host_timer);
	hw_set_bits(&timer_hw->inte, 1U << HOST_TIMER);

	// set PIO to interrupt on the SM IRQ bit
	hw_set_bits(&HOST_PIO->inte0, 1U << (8 + pio_sm));

	// spin up the interrupts and fire the first pulse to get things going
	irq_set_enabled(HOST_TIMER_IRQ, true);
	irq_set_enabled(HOST_PIO_IRQ0, true);
	timer_hw->alarm[HOST_TIMER] = time_us_32() + TYP_CMD_GAP;
}

/*
 * ----------------------------------------------------------------------------
 *   Command Processing
 * ----------------------------------------------------------------------------
 */

host_err host_cmd(uint8_t dev, uint8_t cmd, uint16_t *id,
		uint8_t *data, uint8_t len)
{
	if (dev != 0xFF && dev >= device_count) {
		return HOSTERR_INVALID_PARAM;
	}

	cmd_type type = util_parse_cmd_type(cmd);

	if (type == TYPE_INVALID) {
		return HOSTERR_INVALID_PARAM;
	}
	// only special devices are allowed to issue reset
	if (dev != 0xFF && type == TYPE_RESET) {
		return HOSTERR_INVALID_PARAM;
	}

	// disallow address-based commands when using a device index
	if (dev != 0xFF && cmd & 0xF0) {
		return HOSTERR_INVALID_PARAM;
	}

	// Listen needs parameters and good length
	if (type == TYPE_LISTEN) {
		if (data == NULL || len < 2 || len > 8) {
			return HOSTERR_INVALID_PARAM;
		}
	}

	// rest needs to be in barrier to avoid ISR manipulating values
	host_err result = HOSTERR_OK;
	uint32_t isr = save_and_disable_interrupts();
	if (queue_count < CMD_QUEUE_SIZE) {
		*id = queue_id;

		uint8_t queue_pos = (queue_tail + queue_count) & CMD_QUEUE_MASK;
		queue[queue_pos].state = CMD_PENDING;
		queue[queue_pos].id = queue_id++;
		queue[queue_pos].device = dev;
		if (dev != 0xFF) {
			queue[queue_pos].command = ((devices[dev].address_cur) << 4)
					| (uint8_t) cmd;
		} else {
			queue[queue_pos].command = cmd;
		}
		queue[queue_pos].type = type;
		queue[cmd_idx].error = HOSTERR_OK; // only set on problems

		// submit Listen data if needed
		if (type == TYPE_LISTEN) {
			for (uint8_t i = 0; i < len; i++) {
				queue[queue_pos].data[i] = data[i];
			}
			queue[queue_pos].length = len;
		} else {
			queue[queue_pos].length = 0;
		}

		queue_count++;
	} else {
		result = HOSTERR_FULL;
	}
	restore_interrupts(isr);
	return result;
}

void host_poll(void)
{
	// alias original length when this started
	// shouldn't be able to shrink anywhere but here
	uint8_t ql = (uint8_t) queue_count;

	for (uint8_t i = 0; i < ql; i++) {
		uint8_t idx = (queue_tail + i) & CMD_QUEUE_MASK;
		if (queue[idx].state == CMD_DONE) {

			// determine the endpoint to call back
			volatile ndev_handler *hndl;
			if (queue[idx].device == 0xFF) {
				// special sync handler request, will use that instead
				hndl = NULL;
			} else {
				// call back the associated handler
				hndl = device_handlers[queue[idx].device];
			}

			// perform callback
			if (hndl == NULL) {
				host_sync_cb(
						queue[idx].error,
						queue[idx].id,
						queue[idx].type,
						queue[idx].data,
						queue[idx].length);
			} else {
				switch (queue[idx].type) {
				case TYPE_TALK:
					if (hndl->talk_func) {
						hndl->talk_func(
								queue[idx].device,
								queue[idx].error,
								queue[idx].id,
								queue[idx].command & 0x3,
								(uint8_t *) queue[idx].data,
								queue[idx].length);
					}
					break;
				case TYPE_LISTEN:
					if (hndl->listen_func) {
						hndl->listen_func(
								queue[idx].device,
								queue[idx].error,
								queue[idx].id,
								queue[idx].command & 0x3);
					}
					break;
				case TYPE_FLUSH:
					if (hndl->flush_func) {
						hndl->flush_func(
								queue[idx].device,
								queue[idx].error,
								queue[idx].id);
					}
					break;
				default:
					dbg_err("BUG: cmd %d", queue[idx].type);
				}
			}

			// shrink queue and reset command holder
			uint32_t isr = save_and_disable_interrupts();
			queue[idx].state = CMD_EMPTY;
			queue_tail++;
			if (queue_tail >= CMD_QUEUE_SIZE) queue_tail = 0;
			queue_count--;
			restore_interrupts(isr);
		}
	}
}
