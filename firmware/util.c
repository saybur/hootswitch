/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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

uint8_t util_mouse_decode(uint8_t* data, uint8_t data_len,
		int16_t* x, int16_t* y, uint8_t* buttons)
{
	if (data_len < 2) return 1;

	// store motion from 'classic' registers
	uint16_t ix, iy;
	iy = data[0] & 0x7F;
	ix = data[1] & 0x7F;

	// store buttons from 'classic' registers
	*buttons = 0xFF;
	if (! (data[0] & 0x80)) *buttons &= 0xFE;
	if (! (data[1] & 0x80)) *buttons &= 0xFD;

	// store motion and buttons from the extended registers, if present
	// spec seems to allow for 1, 2, or 3 additional bytes
	uint16_t tmp;
	uint8_t mshift = 7;
	uint16_t bmask = 0x4;
	for (uint8_t i = 2; i < 5 && i < data_len; i++) {
		tmp = data[i] & 0x07;
		ix += tmp << mshift;
		tmp = data[i] & 0x70;
		iy += tmp << (mshift - 4);
		mshift += 3;

		if (! (data[i] & 0x80)) *buttons &= (~bmask);
		bmask <<= 1;
		if (! (data[i] & 0x08)) *buttons &= (~bmask);
		bmask <<= 1;
	}

	// if the highest used bit is set in motion registers (number is negative)
	// fill out remaining bits in our 16-bit representation of the number
	bmask = (0xFFFF << mshift);
	if (ix & (0x01 << (mshift - 1))) {
		ix |= bmask;
	}
	if (iy & (0x01 << (mshift - 1))) {
		iy |= bmask;
	}

	// store in signed types
	*x = ix;
	*y = iy;

	return 0;
}

void util_mouse_encode(uint8_t* data, int16_t x, int16_t y, uint8_t buttons)
{
	uint16_t ix = x;
	uint16_t iy = y;

	data[0] = iy & 0x7F;
	data[1] = ix & 0x7F;
	if (buttons & 0x01) data[0] |= 0x80;
	if (buttons & 0x02) data[1] |= 0x80;

	uint16_t mshift = 7;
	uint8_t bmask = 0x04;
	for (uint8_t i = 2; i < 5; i++) {
		data[i] = (ix >> mshift) & 0x07;
		data[i] |= ((iy >> mshift) & 0x07) << 4;

		if (buttons & bmask) data[i] |= 0x80;
		bmask <<= 1;
		if (buttons & bmask) data[i] |= 0x08;
		bmask <<= 1;
	}
}
