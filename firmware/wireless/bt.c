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

#include "virtual.h"

// sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

static uint32_t stack_high_water;

static virtual_mouse_data mse_data;

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

	uint32_t s = uxTaskGetStackHighWaterMark(NULL);
	if (s != stack_high_water) {
		dbg("bt: %s %d", pcTaskGetName(NULL), s);
		stack_high_water = s;
	}

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
//	dbg("bt (%p) id=%d ", d, uni_hid_device_get_idx_for_instance(d));
//	uni_controller_dump(ctl);

	uint32_t s = uxTaskGetStackHighWaterMark(NULL);
	if (s != stack_high_water) {
		dbg("bt: %s %d", pcTaskGetName(NULL), s);
		stack_high_water = s;
	}

	switch (ctl->klass) {
		case UNI_CONTROLLER_CLASS_GAMEPAD:
			uni_gamepad_dump(&ctl->gamepad);
			break;

		case UNI_CONTROLLER_CLASS_BALANCE_BOARD:
			uni_balance_board_dump(&ctl->balance_board);
			break;

		case UNI_CONTROLLER_CLASS_MOUSE:
//			uni_mouse_dump(&ctl->mouse);

			mse_data.x = ctl->mouse.delta_x;
			mse_data.y = ctl->mouse.delta_y;
			mse_data.buttons = ~(ctl->mouse.buttons);
			virtual_mouse_offer(&mse_data);

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

static void bt_do_work(
		__unused async_context_t *context,
		__unused async_when_pending_worker_t *worker)
{
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
	uni_platform_set_custom(get_my_platform());
	uni_init(0, NULL);
}
static async_when_pending_worker_t bt_worker = { .do_work = bt_do_work };

void bt_init(void)
{
	if (cyw43_arch_init()) {
		panic("unable to init cyw43, is this a Pico W?");
	}

	/*
	 * This required a fair bit of trial-and-error, the mix of btstack,
	 * bluepad32, and FreeRTOS has some confusing quirks. As I understand it,
	 * using the pico_cyw43_arch_sys_freertos library will make the above
	 * cyw43_arch_init() use cyw43_arch_freertos.c and a separate async_context
	 * for thread safety using a created FreeRTOS task. To make btstack and
	 * bluepad32 stay on that same thread the following submits their
	 * initialization to the async_context. Seems (?) like btstack is aware of
	 * this difference and 'just works' without the usual
	 * btstack_run_loop_execute(). Bluepad32 needed a change to its build
	 * information:
	 *
	 * - Remove pico_cyw43_arch_none
	 * - Add pico_cyw43_arch_sys_freertos and FreeRTOS-Kernel-Heap4
	 * - Add target_compile_definitions(bluepad32 PRIVATE CYW43_LWIP=0) to keep
	 *   LWIP from being compiled in.
	 */
	async_context_add_when_pending_worker(cyw43_arch_async_context(), &bt_worker);
	async_context_set_work_pending(cyw43_arch_async_context(), &bt_worker);
}
