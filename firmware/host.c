/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bus.pio.h"
#include "debug.h"
#include "hardware.h"
#include "host.h"
#include "host_sync.h"
#include "led.h"
#include "util.h"

// RTOS delay ticks between device polling; inexact due to callback processing
#define HOST_POLL_RATE    10

#define DEVICE_MAX        14
#define DMA_MAX_BITS      131

// see the PIO host definitions for what these values mean
#define PIO_CMD_OFFSET    2
#define PIO_CMD_VAL       52
#define PIO_RX_TIME_VAL   110

// a few more common timer-based timeouts
#define COMMAND_TIMEOUT   (800 + 70 + 800 + 65 + 300 + 300)
#define RESET_TIMEOUT     (3200 + COMMAND_TIMEOUT)
#define RX_MAX_TIMEOUT    (300 + 66 * 130)
#define TYP_CMD_GAP       8500

// Tlt below is influenced by the end of the PIO command, usually +~50us beyond
// the normal rising edge of the stop bit. Testing is showing that this is
// significantly longer than it should be still. TODO for fixing.
#define LISTEN_TX_WAIT    110

// options for the host TX/RX/reset PIO unit
typedef enum {
	HOST_PIO_TX,
	HOST_PIO_RX,
	HOST_PIO_RST
} host_pio_type;

// trackers for the PIO state machine used
static uint32_t pio_offset;
static uint8_t pio_sm;
static host_pio_type pio_tx_prog;

// DMA data reception from devices
static uint8_t dma_chan;
static dma_channel_config dma_rx_cfg;

typedef enum {
	HOST_IDLE,
	HOST_COMMAND,
	HOST_TALK,
	HOST_LISTEN
} host_phase;

// devID->address and devID->handler tables
static volatile ndev_info devices[DEVICE_MAX];
static volatile ndev_handler *device_handlers[DEVICE_MAX];
static volatile uint8_t device_count;

// table for controlling the order of devices when servicing SRQs
// also used to prune faulted devices out of the above arrays
static volatile uint8_t device_itr[DEVICE_MAX];
static volatile uint8_t device_itr_count;

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
#define CMD_QUEUE_SIZE 8
#define CMD_QUEUE_MASK 7
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
	uint8_t data[DMA_MAX_BITS];
	uint8_t length;
	host_err error;
} host_command;
static volatile host_command queue[CMD_QUEUE_SIZE];
static volatile uint8_t queue_tail;
static volatile uint8_t queue_count;
static volatile uint32_t queue_id = 1;

/*
 * ----------------------------------------------------------------------------
 *   General Functions
 * ----------------------------------------------------------------------------
 */

// sends Talk Register 3 command to an address, returning errors
host_err reg3_sync_talk(uint8_t addr, uint8_t *hi, uint8_t *lo)
{
	uint8_t data[8];
	uint8_t datalen = 2;
	uint8_t cmd = (addr << 4) | (uint8_t) COMMAND_TALK_3;
	host_err err = host_sync_cmd(0xFF, cmd, data, &datalen);
	busy_wait_ms(1);
	if (err) return err;
	if (datalen != 2) return HOSTERR_BAD_RESPONSE;
	*hi = data[0];
	*lo = data[1];
	return HOSTERR_OK;
}

// as above, but retries a given number of times
host_err reg3_sync_talk_retry(uint8_t addr, uint8_t *hi, uint8_t *lo, uint8_t retry)
{
	host_err err = HOSTERR_TIMEOUT;
	for (uint8_t r = 0; r < retry && err == HOSTERR_TIMEOUT; r++) {
		err = reg3_sync_talk(addr, hi, lo);
	}
	return err;
}

// sends Listen Register 3 to an address, returning errors
host_err reg3_sync_listen(uint8_t addr, uint8_t hi, uint8_t lo)
{
	uint8_t data[2];
	uint8_t datalen = 2;
	uint8_t cmd = (addr << 4) | (uint8_t) COMMAND_LISTEN_3;
	data[0] = hi;
	data[1] = lo;
	host_err err = host_sync_cmd(0xFF, cmd, data, &datalen);
	busy_wait_ms(1);
	return err;
}

/*
 * ----------------------------------------------------------------------------
 *   PIO Support Functions
 * ----------------------------------------------------------------------------
 */

// swaps the PIO program out (if needed)
static void host_pio_load_prog(host_pio_type program)
{
	// important: the timer firing during this would be a problem!
	pio_sm_set_enabled(HOST_PIO, pio_sm, false);
	pio_interrupt_clear(HOST_PIO, pio_sm);
	if (program != pio_tx_prog) {
		switch (pio_tx_prog) {
			case HOST_PIO_RX:
				pio_remove_program(HOST_PIO, &bus_rx_host_program, pio_offset);
				break;
			case HOST_PIO_TX:
				pio_remove_program(HOST_PIO, &bus_tx_host_program, pio_offset);
				break;
			case HOST_PIO_RST:
				pio_remove_program(HOST_PIO, &bus_reset_program, pio_offset);
				break;
		}
		switch (program) {
			case HOST_PIO_RX:
				pio_add_program_at_offset(HOST_PIO, &bus_rx_host_program, pio_offset);
				break;
			case HOST_PIO_TX:
				pio_add_program_at_offset(HOST_PIO, &bus_tx_host_program, pio_offset);
				break;
			case HOST_PIO_RST:
				pio_add_program_at_offset(HOST_PIO, &bus_reset_program, pio_offset);
				break;
		}
		pio_tx_prog = program;
	}
}

/*
 * Parses the results of PIO RX bit-time data into bytes. Modifies the array
 * provided and returns the number of valid bytes found. See the host-side
 * RX function in bus.pio for details.
 */
static uint8_t parse_rx_bit_data(volatile uint8_t *data, uint8_t bits)
{
	if (bits > DMA_MAX_BITS) bits = DMA_MAX_BITS;
	if (bits < 2) return 0;
	bits -= 2; // first bit is start bit, ignore

	uint8_t bytes = bits / 16;
	volatile uint8_t *in = &(data[2]);
	for (uint i = 0; i < bytes; i++) {
		uint8_t b = 0;
		for (uint j = 0; j < 8; j++) {
			if (j != 0) b = b << 1;
			uint8_t vl = *in++;
			uint8_t vh = *in++;
			if (vl > vh) {
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

static TaskHandle_t task_handle = NULL;
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
}

// generic setup for calling back into the timer when a command completes
static inline void host_cmd_complete(void)
{
	queue[cmd_idx].state = CMD_DONE;
	phase = HOST_IDLE;
	timer_hw->alarm[HOST_TIMER] = time_us_32() + TYP_CMD_GAP;
	if (task_handle != NULL) {
		vTaskNotifyGiveFromISR(task_handle, NULL);
	}
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

			// was one found?
			if (queue_count > 0 && queue[cmd_idx].state == CMD_PENDING) {
				cmd_idx = cmd_idx;
				cmd_last_dev = queue[cmd_idx].device;
			} else {
				// no, try again after a wait
				timer_hw->alarm[HOST_TIMER] = time_us_32() + TYP_CMD_GAP;
				return;
			}

			// start command
			pio_sm_config c;
			if (queue[cmd_idx].command == 0x00) {
				// send long reset pulse
				host_pio_load_prog(HOST_PIO_RST);
				bus_reset_pio_config(&c, pio_offset, A_DO_PIN);
				pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
				timeout = RESET_TIMEOUT;
			} else {
				// normal attention signal
				host_pio_load_prog(HOST_PIO_TX);
				bus_tx_host_pio_config(&c, pio_offset, A_DO_PIN, A_DI_PIN);
				pio_sm_init(HOST_PIO, pio_sm, pio_offset + PIO_CMD_OFFSET, &c);
				bus_tx_host_put(HOST_PIO, pio_sm, PIO_CMD_VAL);
				timeout = COMMAND_TIMEOUT;
				bus_tx_host_put(HOST_PIO, pio_sm, queue[cmd_idx].command);
			}
			pio_sm_set_enabled(HOST_PIO, pio_sm, true);

			// advance and setup fallback timer
			phase = HOST_COMMAND;
			timer_hw->alarm[HOST_TIMER] = time_us_32() + timeout;
			break;

		default:
			// should only happen if the line sticks low, should be rare
			host_stop_pio();
			queue[cmd_idx].error = HOSTERR_LINE_STUCK;
			host_cmd_complete();
	}
}

// ISR callback for the PIO state machine
static void host_pio_isr(void)
{
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
				host_stop_pio();
				host_pio_load_prog(HOST_PIO_RX);

				bus_rx_host_pio_config(&c, pio_offset, A_DI_PIN);
				pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
				pio_sm_put(HOST_PIO, pio_sm, PIO_RX_TIME_VAL);
				dma_channel_configure(dma_chan, &dma_rx_cfg,
						queue[cmd_idx].data,
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
				pio_interrupt_clear(HOST_PIO, pio_sm);
				// only need to fill TX FIFO, rest will happen automatically
				for (uint8_t i = 0; i < queue[cmd_idx].length; i++) {
					bus_tx_host_put(HOST_PIO, pio_sm, queue[cmd_idx].data[i]);
				}
				// set a timeout to be on the safe side
				phase = HOST_LISTEN;
				timer_hw->alarm[HOST_TIMER] = time_us_32()
						+ 250 + 800 * queue[cmd_idx].length + 250;
				break;
			default:
				// no data transfer required
				host_stop_pio();
				host_cmd_complete();
			}
			break;

		case HOST_TALK:
			// normal end of Talk, either timeout or data response
			host_stop_pio();
			uint8_t dma_remain = dma_channel_hw_addr(dma_chan)->transfer_count;
			dma_channel_abort(dma_chan);
			uint8_t len = DMA_MAX_BITS - dma_remain;
			queue[cmd_idx].length = len;
			if (len == 0) queue[cmd_idx].error = HOSTERR_TIMEOUT;

			host_cmd_complete();
			break;

		case HOST_LISTEN:
			host_stop_pio();
			host_cmd_complete();
			break;

		default:
			host_stop_pio();
			queue[cmd_idx].error = HOSTERR_BAD_STATE;
			host_cmd_complete();
	}
}

/*
 * ----------------------------------------------------------------------------
 *   Reset & Init Logic
 * ----------------------------------------------------------------------------
 */

// calls reset_func on handlers and sends the reset pulse on the bus
host_err host_reset_bus(void)
{
	dbg("host reset bus");

	// drain the queue and wait for the system to go idle
	idle_poll = false;
	queue_count = 0;
	while (phase != HOST_IDLE);

	// send reset command
	uint8_t len = 0;
	return host_sync_cmd(0xFF, 0x00, NULL, &len);
}

/*
 * Used in the following function to move a device from the origin to another
 * address. If more than one device is detected at origin, the subsequent
 * devices are placed at another free address. If there is insufficient free
 * space an error is generated.
 *
 * The second parameter is a list of device origin addresses, which will be
 * updated by this function, where a 0 value means an empty address.
 *
 * The final parameter is the last-used destination address as a return value.
 */
static host_err host_reset_move(uint8_t ao, uint8_t *da, uint8_t *ad)
{
	host_err err = HOSTERR_OK;
	uint8_t hi, lo;

	do {
		// find a free destination address
		*ad = 0;
		if (ao >= 0x8 && da[da[ao]] == 0) {
			// device home address available, use that
			*ad = da[ao];
		}
		if (*ad == 0) {
			// locate the first free address, if there is one
			for (uint8_t i = 0xF; i >= 0x8 && *ad == 0; i--) {
				if (da[i] == 0) *ad = i;
			}
		}
		if (*ad == 0) {
			// error out if no further addresses are available
			dbg_err("  addr pool exhausted!");
			return HOSTERR_TOO_MANY_DEVICES;
		}

		// perform move
		if (err = reg3_sync_listen(ao, *ad, 0xFE)) {
			dbg_err("  L3 err:%d", err);
			return err;
		}

		// check if devices remain at address and reassign table
		err = reg3_sync_talk(ao, &hi, &lo);
		if (err == HOSTERR_TIMEOUT) {
			// expected condition, device moved and nobody is left at origin
			da[*ad] = da[ao];
			da[ao] = 0;
		} else if (err == HOSTERR_OK) {
			// device remains at origin, either second device (hopefully) or
			// original device that didn't hear command; assume former
			dbg("  add dev at $%X, home $%X", ao, da[ao]);
			da[*ad] = da[ao];
		} else {
			// remaining conditions are fatal
			dbg_err("  T3 err:%d", ao, err);
			return err;
		}
	} while (err == HOSTERR_OK);
	return HOSTERR_OK;
}

/*
 * Performs ADB address allocation after a reset. This process deviates quite
 * a bit from what Inside Macintosh: ADB Manager 5-17 describes and instead
 * partially follows behavior seen by the IIci ROM routine. That system does a
 * few things differently:
 *
 * - Top address used is 0xF (IM says 0xE)
 * - A period of back-and-forth swapping happens for each device that IM does
 *   not mention; IIci seems to do this ~50 times (!) for each device.
 * - During "swapping" stage, Listen 3 is followed by Talk 3 to the /original/
 *   address, not the /destination/ address like IM indicates; lack of response
 *   indicates device moved OK and there was no collision.
 * - The IIci does a round-robin movement routine that this does not perform.
 */
static host_err host_reset_addresses(void)
{
	// wipe existing device/handler assignments
	for (uint8_t i = 0; i < DEVICE_MAX; i++) {
		device_handlers[i] = NULL;
	}

	host_err err = HOSTERR_OK;
	uint8_t hi, lo;
	device_count = 0;
	device_itr_count = 0;

	/*
	 * Track which addresses have devices present. A nonzero value means a
	 * device is present, with the value being the device's origin address.
	 */
	uint8_t device_addrs[16] = {0};

	// find addresses with devices present
	for (uint8_t i = 0x0; i <= 0xF; i++) {
		if (err = reg3_sync_talk(i, &hi, &lo)) {
			// only note fault if it was something other than no response
			if (err != HOSTERR_TIMEOUT) {
				dbg_err("  addr $%X scan err:%d", i, err);
				return HOSTERR_BAD_DEVICE;
			}
		} else {
			// mark address as having a device
			device_addrs[i] = i;
			if (i == 0 || i >= 0x8) {
				// illegal default address, fault out
				dbg_err("  addr $%X occ on startup, stuck device?", i);
				return HOSTERR_BAD_DEVICE;
			} else {
				dbg("  addr $%X has device(s)", i);
			}
		}
	}

	// perform the 'address dance' routine to shuffle devices around and find
	// all devices at the starting addreses
	uint8_t addr_free = 0xF;
	for (uint16_t itr = 0; itr < 10; itr++) {
		for (uint8_t ai = 0x1; ai <= 0xF; ai++) {
			// only perform a move if a device is present
			if (device_addrs[ai] == 0) continue;

			// perform device swap
			uint8_t dest;
			if (err = host_reset_move(ai, device_addrs, &dest)) {
				return err;
			}
			if (err = host_reset_move(dest, device_addrs, &dest)) {
				return err;
			}
		}
	}

	// index the devices
	for (uint8_t ai = 0x1; ai <= 0xF; ai++) {
		// skip if no device present
		if (device_addrs[ai] == 0) continue;

		// for each device, find its DHID for storing into the index
		err = reg3_sync_talk_retry(ai, &hi, &lo, 3);
		if (err) {
			if (err == HOSTERR_TIMEOUT) {
				// allow timeouts, just drop with warning
				dbg("  drop $%X during dhid, timeout", ai);
			} else {
				// remaining errors are fatal
				dbg_err("  T3 dhid err:%d", err);
				return err;
			}
		} else {
			// add device to the index
			devices[device_count].hdev = device_count;
			devices[device_count].address_def = device_addrs[ai];
			devices[device_count].address_cur = ai;
			devices[device_count].dhid_def = lo;
			devices[device_count].dhid_cur = lo;
			devices[device_count].fault = false;
			device_count++;
		}
	}

	dbg("re-addr ok!");
	return HOSTERR_OK;
}

// callback provided to interviewers to try and change a device's DHID
static uint8_t handle_change_devid;
static bool host_handle_change(uint8_t dhid, bool srq)
{
	// block special DHID values
	if (dhid == 0x00 || dhid >= 0xFD) {
		dbg("    veto %d", dhid);
		return false;
	}

	volatile ndev_info *device = &(devices[handle_change_devid]);
	uint8_t addr = device->address_cur;
	uint8_t reg3_hi = 0;
	uint8_t device_dhid = 0;

	// get current values
	dbg("    id %d dhid to %d?", handle_change_devid, dhid);
	host_err err;
	if (err = reg3_sync_talk_retry(addr, &reg3_hi, &device_dhid, 3)) {
		dbg_err("    id %d err T3! err:%d", handle_change_devid, err);
		device->fault = true;
		return false;
	}
	device->dhid_cur = device_dhid;
	if (device_dhid == 0) {
		dbg_err("    id %d bad DHID", handle_change_devid);
		device->fault = true;
		return false;
	}

	// try to reassign to the handler requested
	// Note: IM says the low 4 bits of the high byte should be the device
	// address, but some devices don't seem to return their address here; set
	// to address to be on the safe side
	reg3_hi = (reg3_hi & 0xD0) | (addr & 0x0F);
	if (srq) {
		reg3_hi |= 0x20;
	}
	if (err = reg3_sync_listen(addr, reg3_hi, dhid)) {
		dbg_err("    id %d err L3! err:%d", handle_change_devid, err);
		device->fault = true;
		return false;
	}

	// was it accepted?
	if (err = reg3_sync_talk_retry(addr, &reg3_hi, &device_dhid, 3)) {
		dbg_err("    id %d err T3! err:%d", handle_change_devid, err);
		device->fault = true;
		return false;
	}
	device->dhid_cur = device_dhid;
	if (device_dhid == 0) {
		dbg_err("    id %d dhid fault", handle_change_devid);
		device->fault = true;
		return false;
	}

	dbg("    id %d dhid now %d", handle_change_devid, device_dhid);
	return device_dhid == dhid;
}

static host_err host_handle_setup(void)
{
	// scan the devices and assign handlers
	for (uint8_t hid = handler_count(); hid > 0; hid--) {
		ndev_handler *handler;
		handler_get(hid - 1, &handler);
		if (handler == NULL) continue;
		if (handler->interview_func == NULL) {
			dbg("  handler '%s' cannot interview", handler->name);
			continue;
		}

		dbg("  handler '%s':", handler->name);
		for (uint8_t did = 0; did < device_count; did++) {
			volatile ndev_info *device = &(devices[did]);

			// disallow if a device is already owned
			if (device_handlers[did] != NULL) continue;
			// disallow if device faulted
			if (device->fault) continue;

			// might be OK, check with the handler and see if they want it
			handle_change_devid = did;
			bool res = handler->interview_func(device, host_handle_change);
			if (device->fault) {
				dbg_err("    id %d fault (%d), skip", did, device->fault);
			} else if (res) {
				dbg("    id %d ok", did);
				device_handlers[did] = handler;
			}
		}
	}

	return HOSTERR_OK;
}

// removes faulted devices and devices with no handler from the listing to
// avoid dealing with them later
static void host_build_itr_table(void)
{
	// build iterator list
	device_itr_count = 0;
	for (uint8_t i = 0; i < device_count; i++) {
		if (devices[i].fault || device_handlers[i] == NULL) {
			dbg("host drop dev %d", i);
		} else {
			device_itr[device_itr_count++] = i;
		}
	}

	// insertion sort by device address
	uint8_t i = 1;
	for (uint8_t i = 1; i < device_itr_count; i++) {
		uint8_t j = i;
		while (j > 0
				&& devices[device_itr[j - 1]].address_cur
						> devices[device_itr[j]].address_cur) {
			uint8_t t = device_itr[j];
			device_itr[j] = device_itr[j - 1];
			device_itr[j - 1] = t;
			j--;
		}
	}
}

host_err host_reset_devices(void)
{
	host_err err;

	if (idle_poll) {
		dbg_err("host not in reset!");
		return HOSTERR_BAD_STATE;
	}

	dbg("host re-addr");
	if (err = host_reset_addresses()) {
		return err;
	}

	dbg("assign handlers");
	if (err = host_handle_setup()) {
		return err;
	}

	// make sure there is at least one working device before resuming
	host_build_itr_table();
	if (device_itr_count == 0) {
		dbg_err("disabling host, no devices!");
		return HOSTERR_NO_DEVICES;
	}

	return HOSTERR_OK;
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
	pio_tx_prog = HOST_PIO_TX;

	// issue claims for peripherals to avoid accidental conflicts
	pio_sm = pio_claim_unused_sm(HOST_PIO, true);
	dma_chan = dma_claim_unused_channel(true);
	hardware_alarm_claim(HOST_TIMER);

	// create base DMA configuration
	dma_rx_cfg = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&dma_rx_cfg, DMA_SIZE_8);
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

host_err host_cmd(uint8_t dev, uint8_t cmd, uint32_t *id,
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
		uint8_t queue_pos = (queue_tail + queue_count) & CMD_QUEUE_MASK;
		queue[queue_pos].state = CMD_PENDING;

		if (id != NULL) {
			*id = queue_id;
			queue[queue_pos].id = queue_id++;
		} else {
			queue[queue_pos].id = 0;
		}

		queue[queue_pos].device = dev;
		if (dev != 0xFF) {
			queue[queue_pos].command = ((devices[dev].address_cur) << 4)
					| (uint8_t) cmd;
		} else {
			queue[queue_pos].command = cmd;
		}
		queue[queue_pos].type = type;
		queue[queue_pos].error = HOSTERR_OK; // only set on problems

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

			// if Talk, convert bit-times to bytes before returning
			uint8_t dlen = queue[idx].length;
			if (queue[idx].type == TYPE_TALK && dlen > 0) {
				dlen = parse_rx_bit_data(
						queue[idx].data,
						queue[idx].length);
				queue[idx].length = dlen;
			}

			// perform callback
			if (hndl == NULL) {
				host_sync_cb(
						queue[idx].error,
						queue[idx].id,
						queue[idx].type,
						queue[idx].data,
						dlen);
			} else {
				switch (queue[idx].type) {
				case TYPE_TALK:
					if (hndl->talk_func
							&& (queue[idx].length > 0
									|| hndl->accept_noop_talks)) {
						hndl->talk_func(
								queue[idx].device,
								queue[idx].error,
								queue[idx].id,
								queue[idx].command & 0x3,
								(uint8_t *) queue[idx].data,
								dlen);
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

	// if no commands are pending insert an idle poll command
	if (idle_poll && queue_count == 0) {
		uint8_t dev = cmd_last_dev;
		if (srq) {
			// advance to next device
			bool found = false;
			for (uint8_t i = 0; !found && i < device_itr_count; i++) {
				// hunt for device in iterator list
				if (device_itr[i] == dev) {
					// found, pick next device or wrap to beginning
					if (i == device_itr_count - 1) {
						dev = device_itr[0];
					} else {
						dev = device_itr[i + 1];
					}
					found = true;
				}
			}
			if (! found) {
				// host may have been talking to a faulted device
				// fallback to first device
				dev = device_itr[0];
			}
		}
		host_cmd(dev, COMMAND_TALK_0, NULL, NULL, 0);
	}
}

void host_task(__unused void *parameters)
{
	idle_poll = true;
	task_handle = xTaskGetCurrentTaskHandle();
	while (true) {
		ulTaskNotifyTake(pdFALSE, HOST_POLL_RATE);
		host_poll();
	}
}
