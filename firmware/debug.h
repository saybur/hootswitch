/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdbool.h>

typedef enum {
	DEBUG_RUNTIME_NONE = 0,  // take no action
	DEBUG_RUNTIME_HEAP,      // vPortGetHeapStats()
	DEBUG_RUNTIME_LIST,      // vTaskList()
	DEBUG_RUNTIME_STATS      // vTaskGetRunTimeStats()
} debug_stats_option;

#define DEBUG_MESSAGE_QUEUE_DEPTH  16
#define DEBUG_MESSAGE_LENGTH_MAX   64

void dbg(const char *format, ...);
void dbg_err(const char *format, ...);
void dbg_trace(const char *format, ...);
void dbg_data(const char *format, ...);

void dbg_trace_enable(bool state);
bool dbg_trace_is_enabled(void);

/**
 * Mechanism to trigger invocation of FreeRTOS runtime task diagnostics and
 * print the results to the USB debug console.
 *
 * These will only be available if RUNTIME_FREERTOS_REPORTING is set during
 * compilation, otherwise passing a value here will do nothing. For details on
 * the data returned refer the FreeRTOS documentation at
 * <https://www.freertos.org/Documentation/02-Kernel/04-API-references/03-Task-utilities/00-Task-utilities>
 *
 * Note: these calls suspend interrupts and mess with the scheduler, so expect
 * system disruption if they are invoked (the reason they're a compile-time
 * option). This runs from the task that calls it instead of queuing like the
 * normal logging functions.
 *
 * @param option  the function to invoke and print.
 */
void dbg_stats(debug_stats_option option);

void dbg_init(void);
void dbg_task(__unused void *parameters);

#endif /* __DEBUG_H__ */
