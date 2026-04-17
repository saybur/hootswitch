/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "FreeRTOS.h"
#include "task.h"

#include "computer.h"
#include "config.h"
#include "control.h"
#include "debug.h"

#include "drivers/serial.h"

#ifdef HOOTSWITCH_WIRELESS
#include "btscan.h"
#endif

#define WATCHDOG_SCRATCH_REG   0

#define CONTROL_WDRST_DEBUG    0xA5A5A5A5

static volatile control_mode_type mode = CONTROL_MODE_IDLE;

static void control_reboot(bool debug)
{
	// set flag to pause restarting, if needed
	if (debug) {
		watchdog_hw->scratch[WATCHDOG_SCRATCH_REG] = CONTROL_WDRST_DEBUG;
	}

	// perform watchdog reset
	watchdog_enable(1, 1);
	while(1);
}

static void control_enqueue(unsigned char c)
{
	if (c >= 0xE0) {
		switch (c) {
			case SER_CMD_BTSCAN:
#ifdef HOOTSWITCH_WIRELESS
				bt_scan();
#endif
				break;
			case CONTROL_DBG_TRACE:
				dbg_trace_enable(!dbg_trace_is_enabled());
				break;
			case CONTROL_DBG_HEAP:
				dbg_stats(DEBUG_RUNTIME_HEAP);
				break;
			case CONTROL_DBG_LIST:
				dbg_stats(DEBUG_RUNTIME_LIST);
				break;
			case CONTROL_DBG_STATS:
				dbg_stats(DEBUG_RUNTIME_STATS);
				break;
			case CONTROL_REBOOT:
				control_reboot(false);
				break;
			case CONTROL_REBOOT_DEBUG:
				control_reboot(true);
				break;
		}
	} else if (mode == CONTROL_MODE_FLYBYWIRE) {
		serial_enqueue(c);
	}
}

control_reset_type control_check_reset(void)
{
	// retrieve and clear any special flags
	uint32_t w = watchdog_hw->scratch[WATCHDOG_SCRATCH_REG];
	watchdog_hw->scratch[WATCHDOG_SCRATCH_REG] = 0;

	// if the watchdog was responsible for the reset,
	// check if it was a special condition we need to report
	if (watchdog_enable_caused_reboot()) {
		switch (w) {
			case CONTROL_WDRST_DEBUG:
				return RESET_TYPE_DEBUG;
		}
	}

	// fallback to normal
	return RESET_TYPE_NORMAL;
}

void control_start(void)
{
	mode = CONTROL_MODE_FLYBYWIRE;
}

void control_task(__unused void *parameters)
{
	unsigned char c;
	while (true) {
		c = getc(stdin);
		control_enqueue(c);
	}
}
