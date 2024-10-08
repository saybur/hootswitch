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

#define DEVICE_MAX        14
#define DMA_MAX_BITS      131

// see the PIO host definitions for what these values mean
#define PIO_CMD_OFFSET    2
#define PIO_RESET_VAL     59
#define PIO_CMD_VAL       15
#define PIO_RX_TIME_VAL   110

// a few more common timer-based timeouts
#define COMMAND_TIMEOUT   (800 + 70 + 800 + 65 + 300 + 300)
#define RESET_TIMEOUT     (2200 + COMMAND_TIMEOUT)
#define RX_MAX_TIMEOUT    (300 + 66 * 130)
#define TYP_CMD_GAP       1000

// Tlt below is influenced by the end of the PIO command, usually +~50us beyond
// the normal rising edge of the stop bit. Testing is showing that this is
// significantly longer than it should be still. TODO for fixing.
#define LISTEN_TX_WAIT    110

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
	if (err) return err;
	if (datalen != 2) return HOSTERR_BAD_RESPONSE;
	*hi = data[0];
	*lo = data[1];
	return HOSTERR_OK;
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
	return err;
}

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
				host_pio_load_tx(false);

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
	uint8_t addr_top = 0xE;
	uint8_t addr_free = 0xE;
	uint8_t hi, lo;

	for (uint8_t base_addr = 1; base_addr <= 7; base_addr++) {
		dbg("  addr $%X {", base_addr);
		uint8_t dev_at_addr = 0;

		// move all devices at address to free slots
		while (! (err = reg3_sync_talk(base_addr, &hi, &lo))) {
			// found a device, report the default handler
			uint8_t dhid = lo;
			dbg("    fnd $%X (id %d)", dhid, device_count);

			// move it to a free address
			dev_at_addr++;
			dbg("    mov $%X", addr_free);
			if (err = reg3_sync_listen(base_addr, addr_free, 0xFE)) {
				dbg_err("    (!) %d", err);
				// TODO should this be fatal for entire host side?
				return err;
			}

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

		/*
		 * Now perform a swap between the original address and the new location
		 * for each device. Verifies they are working and sets SRQ flags.
		 */
		uint8_t srq_flag = 0x20;
		for (uint8_t i = 0; i < dev_at_addr; i++) {
			uint8_t cur_addr = addr_top - i;

			// is device still listening to us?
			if (err = reg3_sync_talk(cur_addr, &hi, &lo)) {
				dbg_err("    $%X fail T3 %d", cur_addr, err);
				continue;
			}
			// move back to original address
			dbg("    mov $%X, $%X", base_addr, cur_addr);
			if (err = reg3_sync_listen(cur_addr, base_addr, 0x00)) {
				dbg_err("    $%X fail L3 lo mov %d", cur_addr, err);
				continue;
			}
			// did it make it?
			if (err = reg3_sync_talk(base_addr, &hi, &lo)) {
				dbg_err("    $%X fail T3 lo mov %d", err);
				continue;
			}

			// move back to high address, this time including SRQ flag
			dbg("    mov $%X, $%X", cur_addr, base_addr);
			if (err = reg3_sync_listen(base_addr, cur_addr | srq_flag, 0x00)) {
				dbg_err("    $%X fail L3 hi mov %d", cur_addr, err);
				continue;
			}
			// did it make it?
			if (err = reg3_sync_talk(cur_addr, &hi, &lo)) {
				// uh-oh, this is more problematic, low address is now fouled!
				dbg_err("    $%X fail T3 hi mov %d", err);
				return err; // drop out
			}

			// now treat the device as permanent and add to our records
			devices[device_count].hdev = device_count;
			devices[device_count].address_def = base_addr;
			devices[device_count].address_cur = cur_addr;
			devices[device_count].dhid_def = lo;
			devices[device_count].dhid_cur = lo;
			devices[device_count].fault = false;
			device_count++;
		}

		/*
		 * Take the device at the lowest address moved to and put it back in
		 * the original address. Technically different than ADB Manager but
		 * this seems simpler; there seems to be some diversity in how Macs
		 * initialize the ADB bus.
		 */
		if (dev_at_addr > 0) {
			dbg("    mov $%X, $%X", base_addr, addr_free + 1);
			if (err = reg3_sync_listen(addr_free + 1, base_addr | srq_flag, 0x00)) {
				dbg_err("    (!) %d", err);
				return err;
			}
			if (err = reg3_sync_talk(base_addr, &hi, &lo)) {
				dbg_err("    $%X fail T3 fin mov %d", err);
				return err;
			}
			devices[(device_count - 1)].address_cur = base_addr;

			// adjust for next round
			addr_free++;
			addr_top = addr_free;
		}
		dbg("  } got %d", dev_at_addr);
	}

	dbg("re-addr ok!");
	return HOSTERR_OK;
}

// callback provided to interviewers to try and change a device's DHID
static uint8_t handle_change_devid;
static bool host_handle_change(uint8_t dhid)
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
	if (err = reg3_sync_talk(addr, &reg3_hi, &device_dhid)) {
		dbg_err("    id %d err T3!", handle_change_devid);
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
	reg3_hi = (reg3_hi & 0xF0) | (addr & 0xF);
	if (err = reg3_sync_listen(addr, reg3_hi, dhid)) {
		dbg_err("    id %d err L3!", handle_change_devid);
		device->fault = true;
		return false;
	}

	// was it accepted?
	if (err = reg3_sync_talk(addr, &reg3_hi, &device_dhid)) {
		dbg_err("    id %d err T3!", handle_change_devid);
		device->fault = true;
		return false;
	}
	device->dhid_cur = device_dhid;
	if (device_dhid == 0) {
		dbg_err("    id %d dhid fault", handle_change_devid);
		device->fault = true;
		return false;
	}

	dbg("    id %d dhid:%d", handle_change_devid, device_dhid);
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
static void host_prune_faulted(void)
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
	host_prune_faulted();
	if (device_count == 0) {
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
	pio_tx_prog = true;

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
		if (srq) dev++;
		if (dev >= device_count) dev = 0;
		host_cmd(dev, COMMAND_TALK_0, NULL, NULL, 0);
	}
}

void host_task(__unused void *parameters)
{
	idle_poll = true;
	task_handle = xTaskGetCurrentTaskHandle();
	while (true) {
		ulTaskNotifyTake(pdFALSE, 5);
		host_poll();
	}
}
