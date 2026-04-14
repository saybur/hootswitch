/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
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

#ifdef HOOTSWITCH_WIRELESS
#include "bt.h"
#endif

#define PROGRAM_NAME       "hootswitch-v20250426"

#define DEFAULT_STACK      configMINIMAL_STACK_SIZE
#define DEFAULT_PRIORITY   (tskIDLE_PRIORITY + 1U)
#define DISPATCH_PRIORITY  (tskIDLE_PRIORITY + 2U)

// https://stackoverflow.com/a/2220565
#pragma GCC push_options
#pragma GCC optimize ("O0")
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	dbg_err("___stack overflow!___ %s", pcTaskName);
	while (1);
}
#pragma GCC pop_options

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
				led_activity(led);
				led = !led;
			}
			dbg_trace_enable();
			led_activity(false);
			break;
	}

	dbg(PROGRAM_NAME);
	led_activity(true);

#ifdef HOOTSWITCH_WIRELESS
	// need btstack loaded to get TLV config
	volatile bool started = false;
	bt_init(&started);
	while (!started) tight_loop_contents();
#endif

	init_config();
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
	control_start();

	xTaskCreate(computer_task, "computer", DEFAULT_STACK,
			NULL, DISPATCH_PRIORITY, NULL);
	xTaskCreate(host_task, "host", DEFAULT_STACK,
			NULL, DISPATCH_PRIORITY, NULL);
	xTaskCreate(button_task, "button", DEFAULT_STACK,
			NULL, DEFAULT_PRIORITY, NULL);
	xTaskCreate(control_task, "control", configMINIMAL_STACK_SIZE,
			NULL, tskIDLE_PRIORITY, NULL);

	// may be vulnerable to drivers not being hooked but we'll try anyway
	computer_switch(1, true);

	vTaskDelete(NULL);
}

int main(void)
{
	usb_dev_init();
	stdio_init_all();

	init_hardware();

	xTaskCreate(init_task, "init", DEFAULT_STACK,
			NULL, DEFAULT_PRIORITY, NULL);
	xTaskCreate(usb_dev_task, "usb_dev", DEFAULT_STACK * 3,
			NULL, configMAX_PRIORITIES - 1, NULL);

	vTaskStartScheduler();
}
