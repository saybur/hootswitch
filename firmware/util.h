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

#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdint.h>

typedef enum {
	TYPE_RESET,
	TYPE_FLUSH,
	TYPE_LISTEN,
	TYPE_TALK,
	TYPE_INVALID
} cmd_type;

/**
 * Provided a bus command, this responds with a command type. This ignores the
 * command address (apart from $0 during the reset command).
 *
 * @param cmd  the command byte.
 * @return     the command type, including TYPE_INVALID if it is illegal.
 */
cmd_type util_parse_cmd_type(uint8_t cmd);

#endif /* __UTIL_H__ */
