/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>

#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>
#include <uni.h>

#include "sdkconfig.h"

#include "FreeRTOS.h"
#include "task.h"

#include "btscan.h"

#define SAMPLE_RATE_MS  100
#define SCAN_MAX_TICK   (BTSCAN_DURATION_SECONDS * 1000L) / SAMPLE_RATE_MS
#define SCAN_PRIORITY   tskIDLE_PRIORITY + 1U

static TaskHandle_t scan_task;

void bt_scan(void)
{
	// only able to (re)start a scan, should have no effect if already scanning
	vTaskResume(scan_task);
}

static void bt_scan_task(void *parameters)
{
	static uint32_t ticks = 0;
	bool led = false;
	bool scanning = false;

	while (true) {
		vTaskDelay(SAMPLE_RATE_MS / portTICK_PERIOD_MS);
		scanning = uni_bt_is_scanning();

		if (ticks == 0) {
			uni_bt_start_scanning_and_autoconnect_safe();
			led = false;
			cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
			ticks++;
		} else if (!scanning || ticks > SCAN_MAX_TICK) {
			if (scanning) {
				// if not stopped early elsewhere, halt scan now
				uni_bt_stop_scanning_safe();
			}
			cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
			ticks = 0;
			vTaskSuspend(NULL);
		} else {
			led = !led;
			cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
			ticks++;
		}
	}
}

void bt_scan_init(void)
{
	xTaskCreate(bt_scan_task,
			"btscan",
			configMINIMAL_STACK_SIZE,
			NULL, SCAN_PRIORITY, &scan_task);
}
