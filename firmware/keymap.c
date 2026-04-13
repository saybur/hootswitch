/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "keymap.h"

const static uint8_t aek_map[100] = {
		0xFF, // 00 : [no event]
		0xFF, // 01 : [rollover error, ignored]
		0xFF, // 02 : [POST fail, ignored]
		0xFF, // 03 : [undefined error, ignored]
		0x00, // 04 : A
		0x0B, // 05 : B
		0x08, // 06 : C
		0x02, // 07 : D
		0x0E, // 08 : E
		0x03, // 09 : F
		0x05, // 0A : G
		0x04, // 0B : H
		0x22, // 0C : I
		0x26, // 0D : J
		0x28, // 0E : K
		0x25, // 0F : L
		0x2E, // 10 : M
		0x2D, // 11 : N
		0x1F, // 12 : O
		0x23, // 13 : P
		0x0C, // 14 : Q
		0x0F, // 15 : R
		0x01, // 16 : S
		0x11, // 17 : T
		0x20, // 18 : U
		0x09, // 19 : V
		0x0D, // 1A : W
		0x07, // 1B : X
		0x10, // 1C : Y
		0x06, // 1D : Z
		0x12, // 1E : 1
		0x13, // 1F : 2
		0x14, // 20 : 3
		0x15, // 21 : 4
		0x17, // 22 : 5
		0x16, // 23 : 6
		0x1A, // 24 : 7
		0x1C, // 25 : 8
		0x19, // 26 : 9
		0x1D, // 27 : 0
		0x24, // 28 : ENTER
		0x35, // 29 : ESCAPE
		0x33, // 2A : DELETE (BACKSPACE)
		0x30, // 2B : TAB
		0x31, // 2C : SPACEBAR
		0x1B, // 2D : -/_
		0x18, // 2E : =/+
		0x21, // 2F : [
		0x1E, // 30 : ]
		0x2A, // 31 : BACKSPACE
		0xFF, // 32 : NON-US # [ignored]
		0x29, // 33 : ;/:
		0x27, // 34 : '/"
		0x32, // 35 : `
		0x2B, // 36 : ,
		0x2F, // 37 : .
		0x2C, // 38 : /
		0x39, // 39 : CAPS LOCK
		0x7A, // 3A : F1
		0x78, // 3B : F2
		0x63, // 3C : F3
		0x76, // 3D : F4
		0x60, // 3E : F5
		0x61, // 3F : F6
		0x62, // 40 : F7
		0x64, // 41 : F8
		0x65, // 42 : F9
		0x6D, // 43 : F10
		0x67, // 44 : F11
		0x6F, // 45 : F12
		0x69, // 46 : PRINTSCREEN
		0x6B, // 47 : SCROLL
		0x48, // 48 : PAUSE
		0x72, // 49 : INSERT
		0x73, // 4A : HOME
		0x74, // 4B : PAGEUP
		0x75, // 4C : DELFWD
		0x77, // 4D : END
		0x79, // 4E : PGDN
		0x3C, // 4F : RIGHT
		0x3B, // 50 : LEFT
		0x3D, // 51 : DOWN
		0x3E, // 52 : UP
		0x47, // 53 : NUMLK
		0x4B, // 54 : KP /
		0x43, // 55 : KP *
		0x4E, // 56 : KP -
		0x45, // 57 : KP +
		0x4C, // 58 : KP ENTER
		0x53, // 59 : KP 1
		0x54, // 5A : KP 2
		0x55, // 5B : KP 3
		0x56, // 5C : KP 4
		0x57, // 5D : KP 5
		0x58, // 5E : KP 6
		0x59, // 5F : KP 7
		0x5B, // 60 : KP 8
		0x5C, // 61 : KP 9
		0x52, // 62 : KP 0
		0x41  // 63 : KP .
	};

uint8_t keymap_decode(uint8_t key)
{
	if (key < sizeof(aek_map)) {
		return aek_map[key];
	} else {
		return 0xFF;
	}
}
