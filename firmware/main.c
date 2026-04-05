/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
#include "config.h"
#include "control.h"
#include "debug.h"
#include "driver.h"
#include "host.h"
#include "hardware.h"
#include "led.h"
#include "usb.h"

#define PROGRAM_NAME       "hootswitch-v20250426"

#define DEFAULT_STACK      configMINIMAL_STACK_SIZE
#define DEFAULT_PRIORITY   (tskIDLE_PRIORITY + 1U)
#define DISPATCH_PRIORITY  (tskIDLE_PRIORITY + 2U)

static void init_hardware(void)
{
	led_init();

	buzzer_init();
	host_init();
	computer_init();

	// setup input switch
	gpio_init(SWITCH_PIN);
	gpio_pull_up(SWITCH_PIN);
}

static void init_config(void)
{
	config_setup();

	// read core configuration information, and on failure leave at defaults
	uint8_t core;
	if (config_read(CONFIG_OFFSET_BASE, &core, 1)) {
		return;
	}

	if (! (core & 1)) {
		buzzer_enable(false);
	}
}

static void init_task(__unused void *parameters)
{
	// wait for USB enumeration, and for ADB devices to power up themselves up
	busy_wait_ms(1600);

	// handle alternate startup conditions
	switch (control_check_reset()) {
		case RESET_TYPE_DEBUG:
			// wait for a USB UART to connect
			bool led = false;
			while (! stdio_usb_connected()) {
				vTaskDelay(100);
				cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
				led = !led;
			}
			cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
			break;
	}

	dbg(PROGRAM_NAME);
	init_config();
	led_activity(true);
	handler_init();

	host_err herr;
	if (herr = host_reset_bus()) {
		dbg_err("host bus reset err %d", herr);
		led_error(true);
	}
	busy_wait_ms(1);
	if (herr = host_reset_devices()) {
		dbg_err("host device reset err %d", herr);
		led_error(true);
	} else {
		dbg("host reset ok!");
	}

	driver_init();
	computer_start();
	computer_switch(1, true);
	control_start();

	xTaskCreate(computer_task, "computer", DEFAULT_STACK,
			NULL, DISPATCH_PRIORITY, NULL);
	xTaskCreate(host_task, "host", DEFAULT_STACK,
			NULL, DISPATCH_PRIORITY, NULL);
	xTaskCreate(button_task, "button", DEFAULT_STACK,
			NULL, DEFAULT_PRIORITY, NULL);
	xTaskCreate(control_task, "control", configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY, NULL);

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
