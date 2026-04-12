/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdio.h>
#include <stdarg.h>

#include "pico/stdlib.h"

#include "debug.h"

void dbg(const char *format, ...)
{
	uint32_t time = time_us_64() >> 11; // approx us->ms
	char buf[64];

	// thanks to https://stackoverflow.com/a/20639708 for this technique!
	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof buf, format, args);
	va_end(args);

	stdio_printf("[%8d] %s\n", time, buf);
}

void dbg_err(const char *format, ...)
{
	uint32_t time = time_us_64() >> 11; // approx us->ms
	char buf[64];

	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof buf, format, args);
	va_end(args);

	stdio_printf("[%8d] ERR: %s\n", time, buf);
}

void dbg_trace(const char *format, ...)
{
	uint32_t time = time_us_64() >> 11; // approx us->ms
	char buf[64];

	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof buf, format, args);
	va_end(args);

	stdio_printf("[%8d] t: %s\n", time, buf);
}
