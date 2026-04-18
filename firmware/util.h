/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
 * @param x         input X axis movement.
 * @param y         input Y axis movement.
 * @param buttons   input buttons, LSB button 1.
 */
void util_mouse_encode(uint8_t* data, int16_t x, int16_t y, uint8_t buttons);

#endif /* __UTIL_H__ */
