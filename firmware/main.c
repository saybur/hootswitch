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

#include "FreeRTOS.h"
#include "task.h"

#include "button.h"
#include "buzzer.h"
#include "computer.h"
#include "debug.h"
#include "driver.h"
#include "host.h"
#include "hardware.h"
#include "led.h"
#include "usb.h"

#define PROGRAM_NAME       "hootswitch-v20240803"

#define DEFAULT_STACK      configMINIMAL_STACK_SIZE
#define DEFAULT_PRIORITY   (tskIDLE_PRIORITY + 1U)
#define DISPATCH_PRIORITY  (tskIDLE_PRIORITY + 2U)

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

static void init_task(__unused void *parameters)
{
	// wait for USB enumeration, and for ADB devices to power up themselves up
	busy_wait_ms(1600);
	dbg(PROGRAM_NAME);

	handler_init();

	host_err herr;
	if (herr = host_reset_bus()) {
		dbg_err("host bus reset err %d", herr);
	}
	busy_wait_ms(1000);

	if (herr = host_reset_devices()) {
		dbg_err("host device reset err %d", herr);
	} else {
		dbg("host reset ok!");
	}

	driver_init();
	computer_start();
	computer_switch(1, true);

	xTaskCreate(computer_task, "computer", DEFAULT_STACK,
			NULL, DISPATCH_PRIORITY, NULL);
	xTaskCreate(host_task, "host", DEFAULT_STACK,
			NULL, DISPATCH_PRIORITY, NULL);
	xTaskCreate(button_task, "button", DEFAULT_STACK,
			NULL, DEFAULT_PRIORITY, NULL);

	vTaskDelete(NULL);
}

int main(void)
{
	usb_dev_init();
	stdio_init_all();
	if (cyw43_arch_init()) {
		panic("unable to init cyw43, is this a Pico W?");
	}
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

	init_hardware();

	xTaskCreate(init_task, "init", DEFAULT_STACK,
			NULL, DEFAULT_PRIORITY, NULL);
	xTaskCreate(usb_dev_task, "usb_dev", DEFAULT_STACK * 3,
			NULL, configMAX_PRIORITIES - 1, NULL);

	while (true) {
		vTaskStartScheduler();
	}
}
