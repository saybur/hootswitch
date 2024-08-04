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

#define HANDLER_MAX  8
#define DEVICE_MAX   14
#define TYP_CMD_GAP  6000

// see the PIO host definitions for what these values mean
#define PIO_CMD_OFFSET  2
#define PIO_RESET_VAL   59
#define PIO_CMD_VAL     15

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
	HOST_TX,
	HOST_RESET
} host_phase;
static volatile host_phase phase = HOST_IDLE;

// the full list of possible device handlers
static volatile ndev_handler handler_list[HANDLER_MAX];
static volatile uint8_t handler_list_count;

// device ID -> ADB address mapping
static volatile ndev_info devices[DEVICE_MAX];
static volatile ndev_handler device_handlers[DEVICE_MAX];
static volatile uint8_t device_count;

// queue for requested commands on devices
typedef struct {
	uint8_t device;
	uint8_t command;
	uint8_t length;
} host_command;
static volatile bool queue_enabled;
static volatile host_command queue[COMMAND_QUEUE_SIZE];
static volatile uint8_t queue_pos;
static volatile uint8_t queue_count;

// buffer for data talk/listen steps
static volatile uint32_t time;
static volatile uint8_t device;
static volatile uint8_t command;
static volatile uint16_t rxbuf[64];
static volatile uint8_t databuf[8];
static volatile uint8_t databuflen;

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

// parses the device reception buffer and provides the results back
static void host_parse_rxbuf(uint8_t *data, uint8_t bytes)
{
	for (uint i = 0; i < bytes; i++) {
		uint8_t b = 0;
		for (uint j = 0; j < 8; j++) {
			uint16_t v = rxbuf[i * 8 + j];
			if ((v >> 8) > (v & 0xFF)) {
				b |= 1;
			}
			b = b << 1;
		}
		data[i] = b;
	}
}

// ISR callback for the timer
static void host_timer(void)
{
	switch (phase)
	{
		case HOST_IDLE:
			break;
		case HOST_COMMAND:
			break;
		case HOST_RX:
			break;
		case HOST_TX:
			break;
	}

	// TODO implement

	// clear before returning
	irq_clear(HOST_TIMER_IRQ);
}

// ISR callback for the PIO state machine
static void host_pio_isr(void)
{
	// TODO implement
}

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
	pio_sm_config c;
	bus_tx_host_pio_config(&c, pio_offset, A_DO_PIN, A_DI_PIN);
	pio_sm_init(HOST_PIO, pio_sm, pio_offset + PIO_CMD_OFFSET, &c);
	if (cmd == 0x00) {
		// reset, including long pulse
		pio_sm_put(HOST_PIO, pio_sm, PIO_RESET_VAL);
	} else {
		pio_sm_put(HOST_PIO, pio_sm, PIO_CMD_VAL);
	}
	pio_sm_put(HOST_PIO, pio_sm, cmd);
	pio_sm_set_enabled(HOST_PIO, pio_sm, true);

	// wait for the state machine to complete operation, then clear IRQs
	// FIXME stall condition if line stays stuck low, needs clean abort
	while (! (HOST_PIO->irq & (1U << pio_sm))) tight_loop_contents();
	pio_sm_set_enabled(HOST_PIO, pio_sm, false);
	pio_interrupt_clear(HOST_PIO, pio_sm);

	// is there a data transfer step?
	host_err cmd_status = HOSTERR_OK;
	uint8_t xfer = (cmd & 0xC) >> 2;
	if (xfer == 0x3) {
		// Talk, we'll be reading from device
		host_pio_load_tx(false);
		bus_rx_host_pio_config(&c, pio_offset, A_DI_PIN);
		pio_sm_put(HOST_PIO, pio_sm, *len);
		dma_channel_configure(dma_chan, &dma_rx_cfg,
				rxbuf,
				&(HOST_PIO->rxf[pio_sm]),
				*len * 8,
				true);

		// start immediately, the PIO will stall until data comes in
		pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
		pio_sm_set_enabled(HOST_PIO, pio_sm, true);

		// wait for about 300us plus 130us per bit, unless ended sooner
		uint32_t elapsed = 0;
		uint32_t target = 300 + (*len + 2) * 8 * 130;
		while (! (HOST_PIO->irq & (1U << pio_sm))
				|| elapsed < target) {
			// sketchy simple wait
			busy_wait_us_32(20);
			elapsed += 20;
		}

		// stop machine and check status
		pio_sm_set_enabled(HOST_PIO, pio_sm, false);
		if (! (HOST_PIO->irq & (1U << pio_sm))) {
			// stalled out, consider transfer to have failed
			dma_channel_abort(dma_chan);
			cmd_status = HOSTERR_TIMEOUT;
		} else {
			pio_interrupt_clear(HOST_PIO, pio_sm);
			host_parse_rxbuf(data, *len);
		}

	} else if (xfer == 0x2) {
		// Listen, send data; TX FIFO joined so it can fit all 8 if needed
		bus_tx_host_pio_config(&c, pio_offset, A_DO_PIN, A_DI_PIN);
		for (uint8_t i = 0; i < *len; i++) {
			pio_sm_put(HOST_PIO, pio_sm, data[i]);
		}

		// wait for Tlt to expire, then send
		busy_wait_us_32(180);
		pio_sm_init(HOST_PIO, pio_sm, pio_offset, &c);
		pio_sm_set_enabled(HOST_PIO, pio_sm, true);

		// wait for the state machine to become done
		// FIXME stall condition if line stays stuck low, needs clean abort
		while (! (HOST_PIO->irq & (1U << pio_sm))) tight_loop_contents();
		pio_sm_set_enabled(HOST_PIO, pio_sm, false);
		pio_interrupt_clear(HOST_PIO, pio_sm);

	} else {
		// no data transfer, just wait for Tlt to expire
		busy_wait_us_32(180);
	}

	return cmd_status;
}

host_err host_cmd_async(uint8_t dev, bus_command cmd, uint8_t len)
{
	if (! queue_enabled) {
		return HOSTERR_ASYNC_DISABLED;
	}
	if (device >= device_count) {
		return HOSTERR_INVALID_PARAM;
	}
	if (cmd == COMMAND_FLUSH && len != 0) {
		return HOSTERR_INVALID_PARAM;
	}
	if (cmd != COMMAND_FLUSH && (len < 2 || len > 8)) {
		return HOSTERR_INVALID_PARAM;
	}

	// rest needs to be in barrier to avoid ISR manipulating values
	host_err result = HOSTERR_OK;
	uint32_t isr = save_and_disable_interrupts();
	if (queue_count < COMMAND_QUEUE_SIZE) {
		queue[queue_pos].device = device;
		queue[queue_pos].command = ((devices[device].address_cur) << 4)
				| command;
		if (len > 8) len = 8;
		queue[queue_pos++].length = len;
		if (queue_pos == COMMAND_QUEUE_SIZE) {
			queue_pos = 0;
		}
		queue_count++;
	} else {
		result = HOSTERR_FULL;
	}
	restore_interrupts(isr);
	return result;
}

host_err host_register(ndev_handler *handler)
{
	if (handler == NULL) {
		return HOSTERR_INVALID_PARAM;
	}
	if (handler_list_count < HANDLER_MAX) {
		handler_list[handler_list_count++] = *handler;
		return HOSTERR_OK;
	} else {
		return HOSTERR_FULL;
	}
}

host_err host_reset(void)
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
	host_cmd_sync(0xFF, 0x00, NULL, NULL);

	// wait for 1 seconds for devices to reset
	// FIXME should be split up somehow
	busy_wait_ms(2000);
/*
	// lets do the address warp again
	uint8_t top = 0xE;
	uint8_t bottom = 0xE;
	uint8_t data[2];
	uint8_t len = 2;
	for (uint8_t i = 0x10; i <= 0x70; i += 0x10) {
		// Talk Register 3 at the address
		while (host_cmd_sync(0xFF, i | 0xF, data, &len)) {
			// Listen Register 3, move device to new address
			
			
			
			
			host_cmd_sync(0xFF, 0x00, NULL, NULL);
		}
	}
*/

	led_machine(0, 64);
	led_machine(2, 64);

	// switch to async and set up periodic update pulse
// TODO implement
//	pio_set_irq0_source_enabled(HOST_PIO, PIO_INTR_SM0_LSB << pio_sm, true);
//	queue_count = 0;
//	queue_enabled = true;
//	HOST_TIMER_ALRM = time_us_32() + TYP_CMD_GAP;
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
	assert(sizeof(bus_rx_host_program_instructions)
			>= sizeof(bus_tx_host_program_instructions));
	pio_offset = pio_add_program(HOST_PIO, &bus_rx_host_program);
	// change the following default if TX ever becomes > RX
	pio_tx_prog = false;

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

	// setup PIO IRQ, don't enable in NVIC yet
	pio_set_irq0_source_enabled(HOST_PIO, PIO_INTR_SM0_LSB << pio_sm, true);
}
