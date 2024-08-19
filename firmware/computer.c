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

#include "pico/sem.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/rand.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bus.pio.h"
#include "buzzer.h"
#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "hardware.h"
#include "led.h"
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

#define LED_ACTIVE              70
#define LED_DETECT              40

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

typedef struct {
	uint8_t data[8];
	uint8_t length;
	bool keep;
} register_data;

typedef struct {
	uint8_t address;          // ADB address
	dev_driver *driver;       // pointer to driver in computer struct
	uint8_t ref;              // ref constant provided at driver reg
	bool srq_en;              // true if SRQ enabled in Reg3
	semaphore_t sem;          // semaphore for locking the Talk registers
	volatile register_data talk[3];  // Talk register contents (registers 0-2)
	QueueHandle_t queue;      // optional queue for holding pending Talk 0 data
} comp_device;

typedef struct {
	volatile bus_phase phase;
	volatile comp_status status;

	comp_device devices[DEVICE_MAX];
	uint8_t device_count;

	volatile uint16_t srq;             // queued service requests, bit flags by addr
	volatile uint64_t time;            // multiple use, but mostly when phase started
	volatile bool collision;           // was the previous Talk a collision?

	comp_device *dev;         // caches last active device
	uint8_t command;          // caches last command received
	cmd_type command_type;    // caches last command's type
	uint8_t reg;              // caches last active register
	uint8_t listen[8];        // sink for DMA data during Listen

	// general values to assist with debugging/info reporting
	uint32_t dbg_atn;          // attention from GPIO ISR
	uint32_t dbg_atn_shrt;     // attention from PIO directly
	uint32_t dbg_rst;          // reset conditions
	uint32_t dbg_abrt;         // any incomplete termination (these are ok)
	uint32_t dbg_err;          // incomplete termination due to coding bugs
	uint32_t dbg_lock;         // unable to send due to data semaphore
} computer_t;

uint32_t const computer_do_pins[] = {
	C1_DO_PIN, C2_DO_PIN, C3_DO_PIN, C4_DO_PIN
};
uint32_t const computer_di_pins[] = {
	C1_DI_PIN, C2_DI_PIN, C3_DI_PIN, C4_DI_PIN
};
uint32_t const computer_psw_pins[] = {
	C1_PSW_PIN, C2_PSW_PIN, C3_PSW_PIN, C4_PSW_PIN
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

static TaskHandle_t task_handle = NULL;
static uint8_t active_computer;
static computer_t computers[COMPUTER_COUNT];
static uint32_t off_pio_atn, off_pio_rx, off_pio_tx;
static uint8_t rand_idx;

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
	uint8_t comp;
	comp_device *dev;
	cmd_type type;
	uint8_t reg;
	uint8_t data[8];
	uint8_t length;
} comp_command;
static comp_command queue[QUEUE_SIZE];
static volatile uint8_t queue_tail;
static volatile uint8_t queue_count;

/*
 * Reserves a new command queue item for use in subsequent code, and inserts
 * the basic data from the associated computer entry. The new queue item is
 * provided back if more data needs to be added. This will return false if
 * no room is available or no current device is defined for that computer.
 */
static bool queue_add(uint8_t *idx, uint8_t c)
{
	if (queue_count >= QUEUE_SIZE) return false;
	if (computers[c].dev == NULL) return false;

	*idx = (queue_tail + queue_count) & QUEUE_MASK;
	queue_count++;

	// insert the basic data
	queue[*idx].comp = c;
	queue[*idx].dev = computers[c].dev;
	queue[*idx].type = computers[c].command_type;
	queue[*idx].reg = computers[c].reg;

	// wake up the dispatcher
	if (task_handle != NULL) {
		vTaskNotifyGiveFromISR(task_handle, NULL);
	}

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
			computers[i].listen,
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

	// SRQ issued if anyone wants it; omit device being talked to
	*srq = false;
	if (ctype == TYPE_TALK && (cmd & 0x3) == 0) {
		if (d < devc) {
			*srq = computers[i].srq & ~(1U << addr);
		} else {
			*srq = computers[i].srq;
		}
	}

	if (d < devc) {
		computers[i].dev = &computers[i].devices[d];
	} else {
		computers[i].dev = NULL;
	}
	computers[i].command = cmd;
	computers[i].command_type = ctype;

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
	comp_device *dev = computers[i].dev;
	if (dev == NULL) {
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
			uint8_t rand_addr = randt[rand_idx++] & 0xF;
			if (rand_idx >= sizeof(randt)) rand_idx = 0;
			bus_tx_dev_put(COMPUTER_PIO, i,
					((dev->srq_en) ? 0x20 : 0x00)
					| (0x40) // exceptional event, always '1' for us
					| rand_addr);
			uint8_t hndl = 0;
			dev->driver->get_handle_func(i, dev->ref, &hndl);
			bus_tx_dev_put(COMPUTER_PIO, i, hndl);
			return PHASE_TALK;
		} else {
			if (sem_try_acquire(&dev->sem)) {
				// got the lock
				uint8_t dlen = dev->talk[reg].length;
				if (dlen == 0) {
					// no data: halt transmission and let it time out
					sem_release(&dev->sem);
					dev_pio_stop(i);
					dev_pio_atn_start(i);
					return PHASE_IDLE;
				} else {
					bus_tx_dev_putm(COMPUTER_PIO, i,
							dev->talk[reg].data, dlen);
					// hold the semaphore until data send done
					return PHASE_TALK;
				}
			} else {
				// data being updated, act as though we have nothing
				computers[i].dbg_lock++;
				dev_pio_stop(i);
				dev_pio_atn_start(i);
				return PHASE_IDLE;
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
	comp_device *dev = computers[i].dev;
	if (COMPUTER_PIO->irq & (1U << (i + 4))) {
		computers[i].collision = true;
		pio_interrupt_clear(COMPUTER_PIO, i + 4);
		// leave data alone, we'll need it later
	} else {
		computers[i].collision = false;
		uint8_t reg = computers[i].reg;

		if (reg == 0) {
			computers[i].srq &= ~(1U << dev->address);
		}
		if (reg < 3) {
			if (! dev->talk[reg].keep) {
				dev->talk[reg].length = 0;
			}
			uint8_t qi;
			queue_add(&qi, i);
		}
	}
	sem_release(&dev->sem);
}

static void isr_listen_complete(uint8_t i)
{
	comp_device *dev = computers[i].dev;
	uint8_t reg = computers[i].reg;

	uint8_t xfer = 8 - dma_channel_hw_addr(dma_channels[i])->transfer_count;
	dma_channel_abort(dma_channels[i]);

	if (reg == 3) {
		// make sure we got the right length
		if (xfer != 2) {
			// for now just fail, but we might want to log this exception
			return;
		}

		uint8_t up = computers[i].listen[0];
		uint8_t lw = computers[i].listen[1];

		// if there was a previous collision we are to ignore this
		// cheat a bit and assume last collision was targeted at this address!
		if (computers[i].collision && lw == 0xFE) {
			// last was a collision and we're being told to ignore this
		} else if (lw == 0xFD || lw == 0xFF) {
			// outright ignore, we don't use activators or self-tests
		} else if (lw == 0x00 || lw == 0xFE) {
			// address change command
			dev->address = up & 0xF;
			if (lw == 0x00) {
				// only change SRQ for 0x00, 0xFE apparently assumes unchanged
				dev->srq_en = up & 0x20;
				if (! (up & 0x20)) {
					computers[i].srq &= ~(1U << dev->address);
				}
			}
		} else {
			// propose new handler
			if (dev->driver->set_handle_func) {
				dev->driver->set_handle_func(i, dev->ref, lw);
			}
		}
	} else {
		// more traditional data delivery
		uint8_t qi;
		if (queue_add(&qi, i)) {
			for (uint8_t di = 0; di < xfer; di++) {
				queue[qi].data[di] = computers[i].listen[di];
			}
			queue[qi].length = xfer;
		}
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
				if (task_handle != NULL) {
					vTaskNotifyGiveFromISR(task_handle, NULL);
				}
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

/*
 * Low-level setter for Talk data. This must only be invoked with the data
 * semaphore already locked.
 */
static inline void computer_data_set_talk(uint8_t comp, comp_device *dev,
		uint8_t reg, uint8_t *data, uint8_t data_len, bool keep)
{
	// copy data
	for (uint8_t i = 0; i < data_len; i++) {
		dev->talk[reg].data[i] = data[i];
	}
	dev->talk[reg].length = data_len;
	dev->talk[reg].keep = keep;

	// update SRQ flags if needed
	if (reg == 0 && dev->srq_en) { // only for Talk 0
		computers[comp].srq |= 1U << dev->address;
	}
}

/*
 * Checks if a queue has been set for a particular computer/device combination.
 * If there is one, it will lock the Talk register, check the queue for data,
 * and insert it.
 *
 * This must be reentrant due to the potential for being called from a separate
 * worker thread. This is accomplished by hiding behind the semaphore. This
 * will miss opportunities to drain the queue, which isn't a huge deal and
 * will get resolved on the next poll check.
 */
static inline void computer_dev_queue_drain(uint8_t comp, comp_device *dev)
{
	QueueHandle_t dev_queue = dev->queue; // copy in case it gets changed
	if (dev_queue) {
		if (sem_try_acquire(&dev->sem)) {
			// is data already pending?
			if (dev->talk[0].length == 0) {
				// no data, pop the queue
				talk_data_entry e;
				if (xQueueReceive(dev_queue, &e, 0)) {
					// insert if data exists
					computer_data_set_talk(comp, dev, 0,
							e.data, e.length, false);
				}
			}
			sem_release(&dev->sem);
		}
	}
}

bool computer_data_offer(uint8_t comp, uint8_t drv_idx, uint8_t reg,
		uint8_t *data, uint8_t data_len, bool keep)
{
	if (comp >= COMPUTER_COUNT) return false;
	if (drv_idx >= computers[comp].device_count) return false;
	if (reg > 2) return false;
	if (data_len > 8 || data_len == 0) return false;

	if (computers[comp].status != STATUS_NORMAL) return false;

	comp_device *dev = &computers[comp].devices[drv_idx];
	if (sem_acquire_timeout_us(&dev->sem, 1000)) {
		computer_data_set_talk(comp, dev, reg, data, data_len, keep);
		sem_release(&dev->sem);
		return true;
	} else {
		return false;
	}
}

void computer_queue_set(uint8_t comp, uint8_t drv_idx, QueueHandle_t queue)
{
	if (comp >= COMPUTER_COUNT) return;
	if (drv_idx >= computers[comp].device_count) return;

	computers[comp].devices[drv_idx].queue = queue;
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
	assert(sizeof(computer_psw_pins) / 4 == COMPUTER_COUNT);
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
		computers[i] = (computer_t) {
			.phase = PHASE_IDLE, .status = STATUS_OFF
		};
		for (uint j = 0; j < DEVICE_MAX; j++) {
			computers[i].devices[j].srq_en = true;
			sem_init(&computers[i].devices[j].sem, 1, 1);
		}
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
	gpio_init(C1_PSW_PIN);
	gpio_put(C1_PSW_PIN, 0);
	gpio_set_dir(C1_PSW_PIN, GPIO_OUT);
	gpio_init(C2_PSW_PIN);
	gpio_put(C2_PSW_PIN, 0);
	gpio_set_dir(C2_PSW_PIN, GPIO_OUT);
	gpio_init(C3_PSW_PIN);
	gpio_put(C3_PSW_PIN, 0);
	gpio_set_dir(C3_PSW_PIN, GPIO_OUT);
	gpio_init(C4_PSW_PIN);
	gpio_put(C4_PSW_PIN, 0);
	gpio_set_dir(C4_PSW_PIN, GPIO_OUT);

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

void computer_psw(uint8_t computer, bool assert)
{
	if (computer >= COMPUTER_COUNT) return;
	uint32_t gpio = computer_psw_pins[computer];
	gpio_put(gpio, assert);
}

bool computer_switch(uint8_t target)
{
	// move to either next system _or_ target system
	uint8_t next = active_computer;
	if (target >= COMPUTER_COUNT) {
		dbg("sw next");
		for (uint8_t i = 0; i < COMPUTER_COUNT; i++) {
			next++;
			if (next >= COMPUTER_COUNT) next = 0;
			if (computers[next].status == STATUS_NORMAL) {
				break;
			}
		}
	} else {
		dbg("sw cmp %d", target);
		next = target;
	}

	// cancel if no match and/or out of range
	if (next >= COMPUTER_COUNT) {
		dbg("  sw veto, no cmp %d", next);
		return false;
	} else {
		dbg("  sw to %d", next);
	}

	// switch the drivers over
	dev_driver *drv;
	uint8_t dcnt = driver_count_devices();
	for (uint8_t d = 0; d < dcnt; d++) {
		driver_get(d, &drv, NULL);
		if (drv->switch_func) {
			drv->switch_func(next);
		}
	}

	// finally update
	active_computer = next;
	led_machine(active_computer, LED_ACTIVE);
	buzzer_chirp();
	dbg("sw ok!");
	return true;
}

static void computer_poll(void)
{
	while (queue_count) {
		comp_command *q = &(queue[queue_tail]);

		// only pass commands back if the device has come out of reset
		if (computers[q->comp].status == STATUS_NORMAL) {
			dev_driver *drvr = q->dev->driver;
			switch (q->type) {
			case TYPE_TALK:
				if (drvr->talk_func) {
					drvr->talk_func(q->comp, q->dev->ref, q->reg);
				}
				break;
			case TYPE_FLUSH:
				if (drvr->flush_func) {
					drvr->flush_func(q->comp, q->dev->ref);
				}
				break;
			case TYPE_LISTEN:
				if (drvr->listen_func) {
					drvr->listen_func(q->comp, q->dev->ref, q->reg,
						q->data, q->length);
				}
				break;
			}
		}

		// before disposing ref data, see if device Talk queue exists
		computer_dev_queue_drain(q->comp, q->dev);

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
			dbg("cmp %d rst {", i);
			computers[i].device_count = 0;

			// rebuild list of drivers
			uint8_t dcnt = driver_count_devices();
			for (uint8_t d = 0; d < dcnt; d++) {
				dev_driver *drv;
				uint32_t ref = 0;
				driver_get(d, &drv, &ref);

				comp_device *dev = &computers[i].devices[d];
				dev->address = drv->default_addr;
				dev->driver = drv;
				dev->ref = ref;
				dev->srq_en = true;
				for (uint8_t r = 0; r < 3; r++) {
					dev->talk[r].length = 0;
				}
				computers[i].device_count++;
				dbg("  add %d ($%X)", d, drv->default_addr);

				// while we're here, call the reset handler
				drv->reset_func(i, ref);
			}

			// mark reset complete
			computers[i].status = STATUS_NORMAL;
			dbg("} got %d", dcnt);
		}
	}

	/*
	 * Perform a check of all queues. The lack of semaphore lock on the length
	 * check is intentional, the function will resolve that.
	 */
	for (uint8_t i = 0; i < COMPUTER_COUNT; i++) {
		uint8_t dcnt = driver_count_devices();
		for (uint8_t d = 0; d < dcnt; d++) {
			if (computers[i].devices[d].talk[0].length == 0) {
				computer_dev_queue_drain(i, &computers[i].devices[d]);
			}
		}
	}
}

void computer_task(__unused void *parameters)
{
	task_handle = xTaskGetCurrentTaskHandle();
	while (true) {
		ulTaskNotifyTake(pdFALSE, 1);
		computer_poll();
	}
}
