/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifdef RUNTIME_FREERTOS_REPORTING
#include <stdlib.h>
#endif

#include <stdio.h>
#include <stdarg.h>
#include <pico/stdlib.h>
#include <pico/time.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "debug.h"

static volatile bool trace_on;
static volatile QueueHandle_t messages;

static void dbg_printf(const char *format, ...)
{
	char buf[DEBUG_MESSAGE_LENGTH_MAX];

	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof buf, format, args);
	va_end(args);

	xQueueSend(messages, buf, 0);
}

void dbg(const char *format, ...)
{
	uint32_t time = time_us_64() >> 11; // approx us->ms
	char buf[DEBUG_MESSAGE_LENGTH_MAX - 12];

	// thanks to https://stackoverflow.com/a/20639708 for this technique!
	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof buf, format, args);
	va_end(args);

	dbg_printf("[%8d] %s", time, buf);
}

void dbg_err(const char *format, ...)
{
	uint32_t time = time_us_64() >> 11; // approx us->ms
	char buf[DEBUG_MESSAGE_LENGTH_MAX - 17];

	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof buf, format, args);
	va_end(args);

	dbg_printf("[%8d] ERR: %s", time, buf);
}

void dbg_trace(const char *format, ...)
{
	if (!trace_on) return;

	uint32_t time = time_us_64() >> 11; // approx us->ms
	char buf[DEBUG_MESSAGE_LENGTH_MAX - 15];

	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof buf, format, args);
	va_end(args);

	dbg_printf("[%8d] t: %s", time, buf);
}

void dbg_data(const char *format, ...)
{
	char buf[DEBUG_MESSAGE_LENGTH_MAX - 11];

	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof buf, format, args);
	va_end(args);

	dbg_printf("[    DATA] %s", buf);
}

void dbg_trace_enable(bool state)
{
	trace_on = state;
}

bool dbg_trace_is_enabled(void)
{
	return trace_on;
}

uint32_t debug_time_us_32(void)
{
	return time_us_32();
}

void dbg_stats(debug_stats_option option)
{
#ifdef RUNTIME_FREERTOS_REPORTING
	uint8_t task_count = uxTaskGetNumberOfTasks();
	uint8_t *stats = NULL;

	switch (option) {
		case DEBUG_RUNTIME_HEAP:
			HeapStats_t heap;
			vPortGetHeapStats(&heap);
			dbg("heap: xAvailableHeapSpaceInBytes %d",
					heap.xAvailableHeapSpaceInBytes);
			dbg("heap: xSizeOfLargestFreeBlockInBytes %d",
					heap.xSizeOfLargestFreeBlockInBytes);
			dbg("heap: xSizeOfSmallestFreeBlockInBytes %d",
					heap.xSizeOfSmallestFreeBlockInBytes);
			dbg("heap: xNumberOfFreeBlocks %d",
					heap.xNumberOfFreeBlocks);
			dbg("heap: xMinimumEverFreeBytesRemaining %d",
					heap.xMinimumEverFreeBytesRemaining);
			dbg("heap: xNumberOfSuccessfulAllocations %d",
					heap.xNumberOfSuccessfulAllocations);
			dbg("heap: xNumberOfSuccessfulFrees %d",
					heap.xNumberOfSuccessfulFrees);
			break;
		case DEBUG_RUNTIME_LIST:
		case DEBUG_RUNTIME_STATS:
			uint16_t stats_size = task_count * 48;
			stats = malloc(stats_size);
			if (!stats) {
				dbg_err("malloc() fail on stats %d", option);
				return;
			}
			if (option == DEBUG_RUNTIME_LIST) {
				stdio_puts("Name\tState\tPriority\tStack\tNum");
				stdio_puts("****************************************");
				vTaskListTasks(stats, stats_size);
			} else if (option == DEBUG_RUNTIME_STATS) {
				stdio_puts("Task\tAbs Time\t% Time");
				stdio_puts("****************************************");
				vTaskGetRunTimeStatistics(stats, stats_size);
			}
			stdio_puts(stats);
			free(stats);
			break;
	}
#else
	dbg("need RUNTIME_FREERTOS_REPORTING for dbg_stats %d", option);
#endif
}

void dbg_init(void)
{
	if (!messages) {
		messages = xQueueCreate(
				DEBUG_MESSAGE_QUEUE_DEPTH,
				DEBUG_MESSAGE_LENGTH_MAX);
	}
}

void dbg_task(__unused void *parameters)
{
	char buf[64];
	while (1) {
		if (pdPASS == xQueueReceive(messages, &buf, portMAX_DELAY)) {
			stdio_puts(buf);
		}
	}
}
