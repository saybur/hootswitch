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
#include "hardware/dma.h"
#include "hardware/timer.h"

#include "bus.h"
#include "bus.pio.h"
#include "hardware.h"
#include "host.h"

/*
 * This unit contains a basic implementation of an ADB host for communicating
 * with real devices.
 */

static uint32_t pio_offset;
static uint8_t pio_sm;
static uint8_t dma_chan;

static void host_watchdog(unsigned int alarm_num)
{
	
}

static void host_pio_isr(void)
{
	
}

static void host_gpio_isr(void)
{
	
}

void host_init(void)
{
	gpio_set_slew_rate(A_DO_PIN, GPIO_SLEW_RATE_SLOW);
	pio_sm_set_pins_with_mask(HOST_PIO, 0, 0, A_DO_PIN_bm);
	pio_sm_set_pindirs_with_mask(HOST_PIO, 0, A_DO_PIN_bm, A_DO_PIN_bm);
	pio_gpio_init(HOST_PIO, A_DO_PIN);

	gpio_init(A_DI_PIN);

	// leave the (large) bus_rx_host program in PIO instruction memory,
	// this will be swapped in and out as needed.
	assert(sizeof(bus_rx_host_program_instructions)
			>= sizeof(bus_tx_host_program_instructions));
	pio_offset = pio_add_program(HOST_PIO, &bus_rx_host_program);

	// issue claims for peripherals to avoid accidental conflicts
	pio_sm = pio_claim_unused_sm(HOST_PIO, true);
	dma_chan = dma_claim_unused_channel(true);
	hardware_alarm_claim(HOST_TIMER);

	// install line interrupt handler (don't enable yet)
	irq_add_shared_handler(IO_IRQ_BANK0, host_gpio_isr,
			PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
}

/*
 * Starts listening to the connected devices.
 */
void host_start(void)
{
	uint32_t mask = GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE;
	gpio_set_irq_enabled(C1_DI_PIN, mask, true);
	gpio_set_irq_enabled(C2_DI_PIN, mask, true);
	gpio_set_irq_enabled(C3_DI_PIN, mask, true);
	gpio_set_irq_enabled(C4_DI_PIN, mask, true);
}
