/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdint.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "buzzer.h"
#include "debug.h"
#include "led.h"
#include "notify.h"

#define NOTIFY_QUEUE_DEPTH  4

#define FREQ_CONN_LOW       235
#define FREQ_CONN_HIGH      320
#define FREQ_GENERIC        370

#define CHIRP_DURATION      100
#define CHIRP_VOLUME        3

#define CONNECT_DURATION    200
#define CONNECT_VOLUME      3

static QueueHandle_t notifications;

static void notify_do_error_flash(uint8_t count)
{
	const TickType_t delay = CHIRP_DURATION / portTICK_PERIOD_MS;
	uint8_t i = count;

	led_error_on();
	vTaskDelay(delay);
	for (int i = 1; i < count; i++) {
		led_error_off();
		vTaskDelay(delay);
		led_error_on();
		vTaskDelay(delay);
	}
	led_error_off();
}

static void notify_do_computer_switch(void)
{
	buzzer_play(FREQ_GENERIC, CHIRP_VOLUME);
	vTaskDelay(CHIRP_DURATION / portTICK_PERIOD_MS);
	buzzer_play(0, 0);
}

static void notify_connect(void)
{
	const TickType_t delay = CONNECT_DURATION / portTICK_PERIOD_MS;
	led_machine_overlay(0xF, 0); // all off
	buzzer_play(FREQ_CONN_LOW, CONNECT_VOLUME);
	vTaskDelay(delay);
	led_machine_overlay(0xF, LED_MACHINE_LEVEL); // all on
	buzzer_play(FREQ_CONN_HIGH, CONNECT_VOLUME);
	vTaskDelay(delay);
	led_machine_reset(); // restore
	buzzer_play(0, 0);
}

static void notify_disconnect(void)
{
	const TickType_t delay = CONNECT_DURATION / portTICK_PERIOD_MS;
	led_machine_overlay(0xF, LED_MACHINE_LEVEL); // all on
	buzzer_play(FREQ_CONN_HIGH, CONNECT_VOLUME);
	vTaskDelay(delay);
	led_machine_overlay(0xF, 0); // all off
	buzzer_play(FREQ_CONN_LOW, CONNECT_VOLUME);
	vTaskDelay(delay);
	led_machine_reset(); // restore
	buzzer_play(0, 0);
}

static void notify_keys_delete(void)
{
	const TickType_t delay = CONNECT_DURATION / portTICK_PERIOD_MS;

	for (int i = 0; i < 3; i++) {
		buzzer_play(FREQ_CONN_LOW, CONNECT_VOLUME);
		led_machine_overlay(0xF, LED_MACHINE_LEVEL);
		vTaskDelay(delay);
		buzzer_play(0, 0);
		if (i < 2) {
			led_machine_overlay(0x0, LED_MACHINE_LEVEL);
			vTaskDelay(delay);
		}
	}
	led_machine_reset();
}

bool notify_user(notify_type type)
{
	if (notifications) {
		return pdPASS == xQueueSend(notifications, &type, 0);
	} else {
		return false;
	}
}

void notify_task(__unused void *parameters)
{
	notifications = xQueueCreate(
				NOTIFY_QUEUE_DEPTH,
				sizeof(notify_type));

	notify_type type;
	while (1) {
		if (pdPASS == xQueueReceive(notifications, &type, portMAX_DELAY)) {
			switch (type) {
				case NOTIFY_HOST_INIT_RESET_FAIL:
					notify_do_error_flash(NOTIFY_HOST_INIT_RESET_FAIL_FLASHES);
					break;
				case NOTIFY_HOST_INIT_NO_DEVICES:
					notify_do_error_flash(NOTIFY_HOST_NO_DEVICES_FLASHES);
					break;
				case NOTIFY_COMPUTER_SWITCH:
					notify_do_computer_switch();
					break;
				case NOTIFY_DEVICE_CONNECT:
					notify_connect();
					break;
				case NOTIFY_DEVICE_DISCONNECT:
					notify_disconnect();
					break;
				case NOTIFY_DELETE_BT_KEYS:
					notify_keys_delete();
					break;
				default:
					dbg_err("unknown notification %d", type);
			}
		}
	}
}
