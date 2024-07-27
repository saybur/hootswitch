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
#include "hardware/gpio.h"

#include "hardware.h"
#include "relay.h"

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
#ifdef PICO_DEFAULT_LED_PIN
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

	// setup input switch
	gpio_init(SWITCH_PIN);
	gpio_pull_up(SWITCH_PIN);

	// setup buzzer
	gpio_init(BUZZER_PIN);
	gpio_set_dir(BUZZER_PIN, GPIO_OUT);

	// setup the 'host' port
	gpio_init(A_DO_PIN);
	gpio_set_slew_rate(A_DO_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_dir(A_DO_PIN, GPIO_OUT);
	gpio_init(A_DI_PIN);

	// computer 1
	gpio_init(C1_DO_PIN);
	gpio_set_slew_rate(A_DO_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_dir(C1_DO_PIN, GPIO_OUT);
	gpio_init(C1_DI_PIN);
	gpio_init(C1_PSW_PIN);

	// computer 2
	gpio_init(C2_DO_PIN);
	gpio_set_slew_rate(A_DO_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_dir(C2_DO_PIN, GPIO_OUT);
	gpio_init(C2_DI_PIN);
	gpio_init(C2_PSW_PIN);

	// computer 3
	gpio_init(C3_DO_PIN);
	gpio_set_slew_rate(A_DO_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_dir(C3_DO_PIN, GPIO_OUT);
	gpio_init(C3_DI_PIN);
	gpio_init(C3_PSW_PIN);

	// computer 4
	gpio_init(C4_DO_PIN);
	gpio_set_slew_rate(A_DO_PIN, GPIO_SLEW_RATE_SLOW);
	gpio_set_dir(C4_DO_PIN, GPIO_OUT);
	gpio_init(C4_DI_PIN);
	gpio_init(C4_PSW_PIN);
}

int main(void)
{
	stdio_init_all();
	init_hardware();

	relay_main();
}
