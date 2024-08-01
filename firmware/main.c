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
#include "device.h"
#include "host.h"
#include "hardware.h"
#include "manager.h"

static void init_hardware(void)
{
	// setup LEDs
	gpio_init(LED_ERR_PIN);
	gpio_set_dir(LED_ERR_PIN, GPIO_OUT);
	gpio_init(LED_ACT_PIN);
	gpio_set_dir(LED_ACT_PIN, GPIO_OUT);
	gpio_init(LED_C1_PIN);
	gpio_set_dir(LED_C1_PIN, GPIO_OUT);
	gpio_init(LED_C2_PIN);
	gpio_set_dir(LED_C2_PIN, GPIO_OUT);
	gpio_init(LED_C3_PIN);
	gpio_set_dir(LED_C3_PIN, GPIO_OUT);
	gpio_init(LED_C4_PIN);
	gpio_set_dir(LED_C4_PIN, GPIO_OUT);

	// setup input switch
	gpio_init(SWITCH_PIN);
	gpio_pull_up(SWITCH_PIN);

	// setup buzzer
	gpio_init(BUZZER_PIN);
	gpio_set_dir(BUZZER_PIN, GPIO_OUT);
}

int main(void)
{
	stdio_init_all();
	if (cyw43_arch_init()) {
		panic("unable to init cyw43, is this a Pico W?");
	}
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

	init_hardware();
	manager_main();

	while (1) tight_loop_contents();
}
