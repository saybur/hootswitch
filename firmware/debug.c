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
	printf("[%10d] dbg: ", time_us_32());

	// thanks to https://stackoverflow.com/a/20639708 for this technique!
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);

	puts(""); // newline
}

void dbg_err(const char *format, ...)
{
	printf("[%10d] err: ", time_us_32());

	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);

	puts(""); // newline
}
