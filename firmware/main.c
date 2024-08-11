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

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"

#include "bus.h"
#include "buzzer.h"
#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "host.h"
#include "hardware.h"
#include "led.h"
#include "manager.h"

#define PROGRAM_NAME   "hootswitch-v20240803"

static void init_hardware(void)
{
	led_init();
	led_activity(true);

	buzzer_init();
	host_init();
	computer_init();

	// setup input switch
	gpio_init(SWITCH_PIN);
	gpio_pull_up(SWITCH_PIN);
}

int main(void)
{
	stdio_init_all();
	if (cyw43_arch_init()) {
		panic("unable to init cyw43, is this a Pico W?");
	}
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

	init_hardware();

	// wait for USB enumeration, and for ADB devices to power up themselves up
	busy_wait_ms(1600);
	dbg(PROGRAM_NAME);

	handler_init();

	host_err herr;
	if (herr = host_reset_bus()) {
		dbg_err("host bus reset err %d", herr);
	}
	busy_wait_ms(1000);

	// test: turn computer 1 on
	gpio_put(C1_PSW_PIN, 1);
	busy_wait_ms(1000);
	gpio_put(C1_PSW_PIN, 0);

	if (herr = host_reset_devices()) {
		dbg_err("host device reset err %d", herr);
	} else {
		dbg("host reset ok!");
	}

	driver_init();
	computer_start();

	led_machine(0, 64);
	led_machine(2, 64);

	while (1) {
		host_poll();
		computer_poll();
		handler_poll();
		driver_poll();
	}
}
