/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/*
 * See https://github.com/ricardoquesada/bluepad32/blob/main/examples/pico_w/src/main.c
 */

#include <stddef.h>
#include <string.h>

#include <pico/cyw43_arch.h>
#include <pico/time.h>
#include <uni.h>

#include "sdkconfig.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

// sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

static void my_platform_init(int argc, const char** argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	dbg("bt init()");
}

static void my_platform_on_init_complete(void)
{
	dbg("bt on_init_complete()");

	uni_bt_start_scanning_and_autoconnect_unsafe();
	uni_bt_list_keys_unsafe();
	uni_bt_service_set_enabled(true);
	uni_property_dump_all();
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr,
		const char* name, uint16_t cod, uint8_t rssi)
{
	dbg("bt on_device_discovered(): %d");

	return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d)
{
	dbg("bt device connected: %p", d);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d)
{
	dbg("bt device disconnected: %p", d);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d)
{
	dbg("bt: device ready: %p", d);

	return UNI_ERROR_SUCCESS;
}

static void my_platform_on_controller_data(uni_hid_device_t* d,
		uni_controller_t* ctl)
{
	static uint8_t leds = 0;
	static uint8_t enabled = true;
	static uni_controller_t prev = {0};
	uni_gamepad_t* gp;

	// Used to prevent spamming the log, but should be removed in production.
	//	if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
	//		return;
	//	}
	prev = *ctl;

	dbg("bt (%p) id=%d ", d, uni_hid_device_get_idx_for_instance(d));
	uni_controller_dump(ctl);

	switch (ctl->klass) {
		case UNI_CONTROLLER_CLASS_GAMEPAD:
			uni_gamepad_dump(&ctl->gamepad);
			break;

		case UNI_CONTROLLER_CLASS_BALANCE_BOARD:
			uni_balance_board_dump(&ctl->balance_board);
			break;

		case UNI_CONTROLLER_CLASS_MOUSE:
			uni_mouse_dump(&ctl->mouse);
			break;

		case UNI_CONTROLLER_CLASS_KEYBOARD:
			uni_keyboard_dump(&ctl->keyboard);
			break;

		default:
			dbg_err("Unsupported controller class: %d\n", ctl->klass);
			break;
	}
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx)
{
	ARG_UNUSED(idx);
	return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data)
{
	switch (event) {
		case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON:
			dbg("bt on_oob_event: sys btn");
			break;

		case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
			dbg("bt on_oob_event: bt enabled: %d", (bool)(data));
			break;

		default:
			dbg("bt on_oob_event: unsupported event: 0x%04x", event);
	}
}

struct uni_platform* get_my_platform(void)
{
	static struct uni_platform platform = {
		.name = "hootswitch",
		.init = my_platform_init,
		.on_init_complete = my_platform_on_init_complete,
		.on_device_discovered = my_platform_on_device_discovered,
		.on_device_connected = my_platform_on_device_connected,
		.on_device_disconnected = my_platform_on_device_disconnected,
		.on_device_ready = my_platform_on_device_ready,
		.on_oob_event = my_platform_on_oob_event,
		.on_controller_data = my_platform_on_controller_data,
		.get_property = my_platform_get_property,
	};

	return &platform;
}

void bt_task(void *params)
{
	uni_platform_set_custom(get_my_platform());
	uni_init(0, NULL);
	btstack_run_loop_execute();
	while (true) {
		vTaskDelay(1000);
	}
}
