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

/**
 * Decodes a mouse protocol register 0 data packet. This supports between 2
 * and 5 bytes corresponding to both the "classic" and "extended" protocols
 * described in Technote HW01.
 *
 * @param data      input array of data values.
 * @param data_len  input length of input array, between 2 and 5.
 * @param x         output X axis movement.
 * @param y         output Y axis movement.
 * @param buttons   output buttons, LSB button 1.
 * @return          nonzero on failure, zero on success.
 */
uint8_t util_mouse_decode(uint8_t* data, uint8_t data_len,
		int16_t* x, int16_t* y, uint8_t* buttons);

/**
 * Encoddes a mouse protocol register 0 data packet. This always encodes based
 * on the "extended" protocol described in Technote HW01; for "classic" mice
 * just use bytes 0 and 1 in the array.
 *
 * @param data      output array of data values, at least 5 bytes long.
 * @param rshift    number of bits to shift motion data to the right prior to
 *                  encoding, to support fixed-point representations and/or
 *                  simple scaling; 0 to not shift.
 * @param x         input X axis movement.
 * @param y         input Y axis movement.
 * @param buttons   input buttons, LSB button 1.
 */
void util_mouse_encode(uint8_t* data, uint8_t rshift,
		int32_t x, int32_t y, uint8_t buttons);

#endif /* __UTIL_H__ */
