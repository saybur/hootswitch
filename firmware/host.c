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
#include "led.h"

#define DEVICE_MAX        14
#define DMA_MAX_BITS      65

// aliases for header versions for use in sync call
#define TALK_REG3         0xF
#define LISTEN_REG3       0xB

// see the PIO host definitions for what these values mean
#define PIO_CMD_OFFSET    2
#define PIO_RESET_VAL     59
#define PIO_CMD_VAL       15
#define PIO_TIMEOUT_VAL   110

// a few more common timer-based timeouts
#define MIN_TLT           140
#define COMMAND_TIMEOUT   1500
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
	HOST_RX,
	HOST_TX_START,
	HOST_TX_END
} host_phase;

// devID->address and devID->handler tables
static volatile ndev_info devices[DEVICE_MAX];
static volatile ndev_handler *device_handlers[DEVICE_MAX];
static volatile uint8_t device_count;

// queue for requested commands on devices, implemented as a ring buffer
#define CMD_QUEUE_SIZE 16
#define CMD_QUEUE_MASK 15
typedef struct {
	uint16_t id;
	uint8_t device;
	uint8_t command;
} host_command;
static volatile bool queue_enabled;
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

/*
 * Stall until the PIO IRQ sets or timeout occurs. Only used with sync calls.
 * This will stop the state machine after timeout, clear the IRQ, and ensure
 * the line driver has been released. Returns true if flag set, false
 * otherwise.
 */
static bool host_busy_wait_pio_irq(uint16_t timeout)
{
	uint32_t elapsed = 0;
	while (! (HOST_PIO->irq & (1U << pio_sm))
			&& elapsed <= timeout) {
		busy_wait_us_32(5);
		elapsed += 5;
	}

	// stop state machine, get IRQ status, clear, and make sure output is idle
	pio_sm_set_enabled(HOST_PIO, pio_sm, false);
	bool res = HOST_PIO->irq & (1U << pio_sm);
	pio_interrupt_clear(HOST_PIO, pio_sm);
	pio_sm_set_pins_with_mask(HOST_PIO, 0, 0, A_DO_PIN_bm);
	return res;
}

static volatile uint16_t rxbuf[DMA_MAX_BITS];

// parses device reception buffer and writes result to supplied byte array
// returns the number of recognized bytes
static uint8_t host_parse_rxbuf(uint8_t *data, uint8_t bits)
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

static volatile host_phase phase = HOST_IDLE;
static volatile host_command active_command;
static volatile uint8_t data_buf[8];
static volatile uint16_t timeout;
static volatile bool srq;

// generic PIO stop routine for the timer
static inline void host_stop_pio(void)
{
	pio_sm_set_enabled(HOST_PIO, pio_sm, false);
	pio_interrupt_clear(HOST_PIO, pio_sm);
	pio_sm_set_pins_with_mask(HOST_PIO, 0, 0, A_DO_PIN_bm);
}

// generic setup for calling back into the timer when a command completes
static inline void host_sync_complete(void)
{
	host_stop_pio();
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
	switch (phase)
	{
		case HOST_IDLE:
			/*
			 * Three options, in descending order:
			 *
			 * 1) queued command,
			 * 2) SRQ,
			 * 3) poll Talk0 at current device.
			 */
			if (queue_count) {
				active_command = queue[queue_tail];
				queue_tail++;
				queue_count--;
			} else {
				uint8_t dev = active_command.device;
				if (srq) dev++;
				active_command.id = 0;
				active_command.device = dev;
				active_command.command = ((devices[dev].address_cur) << 4)
						| (uint8_t) COMMAND_TALK_0;
			}

			// start command
			host_pio_load_tx(true);
			pio_sm_config c;
			bus_tx_host_pio_config(&c, pio_offset, A_DO_PIN, A_DI_PIN);
			pio_sm_init(HOST_PIO, pio_sm, pio_offset + PIO_CMD_OFFSET, &c);
			bus_tx_host_put(HOST_PIO, pio_sm, PIO_CMD_VAL);
			bus_tx_host_put(HOST_PIO, pio_sm, active_command.command);
			pio_sm_set_enabled(HOST_PIO, pio_sm, true);

			// advance and setup fallback timer
			phase = HOST_COMMAND;
			timer_hw->alarm[HOST_TIMER] = time_us_32() + COMMAND_TIMEOUT;
			break;

		case HOST_COMMAND:
		case HOST_RX: // this one should be very rare
			// if invoked, command step timed out
			host_sync_complete();
			break;

		case HOST_TX_START:
			// at end of Tlt, start transmission
			pio_sm_set_enabled(HOST_PIO, pio_sm, true);
			// and add a timeout safety
			timer_hw->alarm[HOST_TIMER] = time_us_32() + timeout;
			phase = HOST_TX_END;

			break;
		case HOST_TX_END:
			// send failed
			host_sync_complete();

			break;
	}

	// clear before returning
	timer_hw->intr = 1U << HOST_TIMER;
}

// ISR callback for the PIO state machine
static void host_pio_isr(void)
{
	// unconditionally stop PIO
	host_stop_pio();

	switch (phase)
	{
		case HOST_COMMAND:
			// triggered by successful end of command

			// check SRQ flag and clear it
			srq = HOST_PIO->irq & (1U << (pio_sm + 4));
			pio_interrupt_clear(HOST_PIO, pio_sm + 4);

			uint8_t cmd_type = active_command.command & 0xC;
			if (cmd_type == 0xC) {
				// Talk, prepare for reception
				host_pio_load_tx(false);
				pio_sm_config c;
				bus_rx_host_pio_config(&c, pio_offset, A_DI_PIN);
				pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
				pio_sm_put(HOST_PIO, pio_sm, PIO_TIMEOUT_VAL);
				dma_channel_configure(dma_chan, &dma_rx_cfg,
						rxbuf,
						&(HOST_PIO->rxf[pio_sm]),
						DMA_MAX_BITS, // 8 bytes plus start/stop bits
						true); // start

				// start immediately, the PIO will stall until data comes in
				pio_sm_set_enabled(HOST_PIO, pio_sm, true);

				// move to next phase, include safety valve
				phase = HOST_RX;
				timer_hw->alarm[HOST_TIMER] = time_us_32() + RX_MAX_TIMEOUT;

			} else if (cmd_type == 0x8) {
				// Listen; start timer immediately to help avoid this missing
				timer_hw->alarm[HOST_TIMER] = time_us_32() + MIN_TLT;

				// fetch data from the handler
				uint8_t len = 0;
				uint8_t dev = active_command.device;
				device_handlers[dev]->listen_func(
						dev,
						active_command.id,
						active_command.command & 0x3,
						(uint8_t *) data_buf,
						&len);
				if (len == 0) {
					// skip sending
					phase = HOST_IDLE;
					return;
				}

				// setup for sending
				pio_sm_config c;
				bus_tx_host_pio_config(&c, pio_offset, A_DO_PIN, A_DI_PIN);
				pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
				for (uint8_t i = 0; i < len; i++) {
					bus_tx_host_put(HOST_PIO, pio_sm, data_buf[i]);
				}
				timeout = 100 + len * 8 * 100 + 150;
				// timer firing will actually start transfer
			} else {
				// no data transfer required
				host_sync_complete();
			}

			break;
		case HOST_RX:
			// triggered by successful end of Talk
			uint8_t dma_remain = dma_channel_hw_addr(dma_chan)->transfer_count;
			dma_channel_abort(dma_chan);
			uint8_t len = host_parse_rxbuf((uint8_t *) data_buf,
					DMA_MAX_BITS - dma_remain);

			// provide data to proper handler
			uint8_t dev = active_command.device;
			device_handlers[dev]->talk_func(
					dev,
					active_command.id,
					active_command.command & 0x3,
					(uint8_t *) data_buf,
					len);

			// successful completion
			host_sync_complete();
			break;

		case HOST_TX_END:
			host_sync_complete();

			break;
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
	if (queue_enabled) {
		// abort any existing tasks and go async->sync
		irq_set_enabled(HOST_TIMER_IRQ, false);
		irq_set_enabled(HOST_PIO_IRQ0, false);
		pio_sm_set_enabled(HOST_PIO, pio_sm, false);
		// clear local interrupt sources
		pio_interrupt_clear(HOST_PIO, pio_sm);
		timer_hw->armed = 1U << HOST_TIMER;
		// dwell for a bit to time out anything stopped above
		queue_enabled = false;
		busy_wait_us(150);
	}

	// send reset command
	return host_cmd_sync(0xFF, 0x00, NULL, NULL);
}

// performs ADB address shuffling
static host_err host_reset_addresses(void)
{
	if (queue_enabled) {
		return HOSTERR_SYNC_DISABLED;
	}

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
		while (! (err = host_cmd_sync(0xFF, (i << 4) | TALK_REG3, data, &datalen))) {
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
			if (err = host_cmd_sync(0xFF, (i << 4) | LISTEN_REG3, data, &datalen)) {
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
			data[0] = addr_free;
			data[1] = 0x00;
			datalen = 2;
			if (err = host_cmd_sync(0xFF, (addr_free << 4) | LISTEN_REG3, data, &datalen)) {
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

// resets handlers and assigns devices to them
static host_err host_handle_assign(void)
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
	if (err = host_handle_assign()) {
		return err;
	}

	// make sure there is at least one working device before resuming
	host_prune_faulted();
	if (device_count == 0) {
		dbg_err("disabling host, no devices!");
		return HOSTERR_NO_DEVICES;
	}

	// switch back to sync mode if there are devices
	irq_set_enabled(HOST_TIMER_IRQ, false);
	irq_set_enabled(HOST_PIO_IRQ0, false);
	queue_enabled = true;
	phase = HOST_IDLE;
	active_command.id = 0;
	active_command.device = 0;
	active_command.command = ((devices[0].address_cur) << 4)
			| (uint8_t) COMMAND_TALK_0;
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

	// setup PIO IRQ, don't enable in NVIC yet
	pio_set_irq0_source_enabled(HOST_PIO, PIO_INTR_SM0_LSB << pio_sm, true);
}

/*
 * ----------------------------------------------------------------------------
 *   Command Processing
 * ----------------------------------------------------------------------------
 */

host_err host_cmd_sync(uint8_t dev, uint8_t cmd, uint8_t *data, uint8_t *len)
{
	// block sync commands once async has started
	if (queue_enabled) return HOSTERR_SYNC_DISABLED;

	// buffer/length must be present if I/O is needed
	if (cmd & 0xC) {
		if (data == NULL || len == NULL) return HOSTERR_INVALID_PARAM;
		if (*len > 8) return HOSTERR_INVALID_PARAM;
	}

	// special case handling for unspecified devices, see header for details
	if (dev != 0xFF) {
		if (dev >= device_count) return HOSTERR_INVALID_PARAM;
		uint8_t addr = devices[dev].address_cur;
		cmd = (cmd & 0xF) | ((addr & 0xF) << 4);
	}

	// load in correct program for transmitting
	host_pio_load_tx(true);

	// execute the requested command
	uint16_t timeout_base;
	pio_sm_config c;
	bus_tx_host_pio_config(&c, pio_offset, A_DO_PIN, A_DI_PIN);
	pio_sm_init(HOST_PIO, pio_sm, pio_offset + PIO_CMD_OFFSET, &c);
	if (cmd == 0x00) {
		// reset, including long pulse
		bus_tx_host_put(HOST_PIO, pio_sm, PIO_RESET_VAL);
		timeout_base = 3000;
	} else {
		bus_tx_host_put(HOST_PIO, pio_sm, PIO_CMD_VAL);
		timeout_base = 800;
	}
	bus_tx_host_put(HOST_PIO, pio_sm, cmd);
	pio_sm_set_enabled(HOST_PIO, pio_sm, true);

	// wait for the state machine to complete operation, then clear IRQs
	bool cmdres = host_busy_wait_pio_irq(timeout_base + 100 * 9 + 400);
	if (! cmdres) {
		// timed out, should only happen if line gets stuck low
		return HOSTERR_LINE_STUCK;
	}

	// is there a data transfer step?
	host_err cmd_status = HOSTERR_OK;
	uint8_t xfer = (cmd & 0xC) >> 2;
	if (xfer == 0x3) {
		// Talk, we'll be reading from device
		host_pio_load_tx(false);
		bus_rx_host_pio_config(&c, pio_offset, A_DI_PIN);
		pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
		pio_sm_put(HOST_PIO, pio_sm, PIO_TIMEOUT_VAL);
		dma_channel_configure(dma_chan, &dma_rx_cfg,
				rxbuf,
				&(HOST_PIO->rxf[pio_sm]),
				DMA_MAX_BITS, // 8 bytes plus start/stop bits
				true);

		// start immediately, the PIO will stall until data comes in
		pio_sm_set_enabled(HOST_PIO, pio_sm, true);

		// wait for maximum possible legal data length, just in case something
		// gets really stuck
		bool ok = host_busy_wait_pio_irq(300 + 66 * 130);
		uint8_t dma_remain = dma_channel_hw_addr(dma_chan)->transfer_count;
		dma_channel_abort(dma_chan);
		if (! ok) {
			// generally should only happen if the remote keeps toggling the
			// line for longer than it is allowed, treat as stuck
			*len = 0;
			return HOSTERR_LINE_STUCK;
		}

		// if we got anything consider it not to be a timeout, just report
		if (dma_remain == DMA_MAX_BITS) {
			*len = 0;
			return HOSTERR_TIMEOUT;
		} else {
			*len = host_parse_rxbuf(data, DMA_MAX_BITS - dma_remain);
		}

	} else if (xfer == 0x2) {
		// Listen, send data; TX FIFO joined so it can fit all 8 if needed
		bus_tx_host_pio_config(&c, pio_offset, A_DO_PIN, A_DI_PIN);
		pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
		for (uint8_t i = 0; i < *len; i++) {
			bus_tx_host_put(HOST_PIO, pio_sm, data[i]);
		}

		// wait for min Tlt to expire (likely more due to above work)
		busy_wait_us_32(140);
		// then proceed to send
		pio_sm_set_enabled(HOST_PIO, pio_sm, true);

		// wait for the state machine to finish sending
		bool ok = host_busy_wait_pio_irq(100 + *len * 8 * 100 + 150);
		if (! ok) {
			// as before, only occurs if line gets stuck low
			cmd_status = HOSTERR_LINE_STUCK;
		}

	} else {
		// no data transfer, just wait for Tlt to expire
		busy_wait_us_32(140);
	}

	return cmd_status;
}

host_err host_cmd_async(uint8_t dev, bus_command cmd, uint16_t *id)
{
	if (! queue_enabled) {
		return HOSTERR_ASYNC_DISABLED;
	}
	if (dev >= device_count) {
		return HOSTERR_INVALID_PARAM;
	}

	// rest needs to be in barrier to avoid ISR manipulating values
	host_err result = HOSTERR_OK;
	uint32_t isr = save_and_disable_interrupts();
	if (queue_count < CMD_QUEUE_SIZE) {
		*id = queue_id;
		uint8_t queue_pos = (queue_tail + queue_count) & CMD_QUEUE_MASK;
		queue[queue_pos].id = queue_id++;
		queue[queue_pos].device = dev;
		queue[queue_pos].command = ((devices[dev].address_cur) << 4)
				| (uint8_t) cmd;
		queue_count++;
	} else {
		result = HOSTERR_FULL;
	}
	restore_interrupts(isr);
	return result;
}
