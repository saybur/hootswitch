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

#include "util.h"

cmd_type util_parse_cmd_type(uint8_t cmd)
{
	if (cmd == 0x00) {
		return TYPE_RESET;
	}
	cmd &= 0xF;
	if (cmd == 0x01) {
		return TYPE_FLUSH;
	}
	cmd &= 0xC;
	if (cmd == 0xC) {
		return TYPE_TALK;
	} else if (cmd == 0x8) {
		return TYPE_LISTEN;
	} else {
		return TYPE_INVALID;
	}
}
