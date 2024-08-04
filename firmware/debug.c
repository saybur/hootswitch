/*
 * Copyright (C) 2024 saybur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdarg.h>

#include "pico/stdlib.h"

#include "debug.h"

void dbg(const char *format, ...)
{
	printf("[%08x] dbg: ", time_us_32());

	// thanks to https://stackoverflow.com/a/20639708 for this technique!
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);
}
