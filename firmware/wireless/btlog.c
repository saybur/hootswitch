/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/*
 * Overrides 'weak' variants to integrate with the rest of the program log.
 */

#include <stdio.h>
#include <stdarg.h>
#include <pico/stdlib.h>

#include "debug.h"
#include "uni_log.h"

void uni_log(const char* format, ...)
{
	va_list args;
	va_start(args, format);
	uni_logv(format, args);
	va_end(args);
}

void uni_logv(const char* format, va_list args)
{
	uint32_t time = time_us_64() >> 11; // approx us->ms
	char buf[80];
	vsnprintf(buf, sizeof buf, format, args);
	stdio_printf("[%8d] b32: %s", time, buf);
}
