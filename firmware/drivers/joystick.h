/*
 * Copyright (C) 2024-2026 saybur
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

#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

typedef enum {
	MODE_INVALID = 0,
	MODE_FIREBIRD = 0x4E,
	MODE_MOUSESTICK = 0x23
} joystick_mode;

/**
 * A snapshot of position and button data from a joystick. There are several
 * quirks to this format due to the underlying devices being emulated.
 *
 * 1) Position data is -128 == full up/left, 127 == full down/right, 0 center.
 * 2) Least significant bit of button data is button 1.
 * 3) Buttons follow the usual ADB convention where set (1) is up and cleared
 *    (0) is pressed.
 */
typedef struct {
	int8_t x1, y1;
	int8_t x2, y2;
	uint32_t buttons;
} joystick_data;

/**
 * Registers a computer-facing keyboard and assigns it for exclusive use to
 * the caller.
 *
 * @param *id            on success, set to the ID that should be used during
 *                       enqueue operations.
 * @param mode           the preferred joystick mode to be used for emulation
 *                       (all devices start as fallback mouse/keyboards).
 * @return               true if registration was successful, false otherwise.
 */
bool joystick_register(uint8_t *id, joystick_mode mode);

/**
 * Updates joystick position and button information and sets it in the active
 * computer immediately for sending during the next ADB Talk opportunity.
 *
 * Not all data fields may be used, depending on the device being emulated
 * and/or the current state of the particular computer's device handler ID.
 * This tries to degrade cleanly where possible. In general, users are
 * encouraged to just send as much data as possible and let the function try to
 * sort it out.
 *
 * @param id      the ID provided at registration time.
 * @param *jdata  the data to apply, as above.
 */
void joystick_update(uint8_t id, joystick_data *jdata);

#endif /* __JOYSTICK_H__ */
