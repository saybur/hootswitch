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

#ifndef __MOUSE_H__
#define __MOUSE_H__

typedef enum {
	MOUSE_MODE_100CPI = 1,
	MOUSE_MODE_200CPI = 2,
	MOUSE_MODE_EXTENDED = 4
} mouse_mode;

/**
 * Registers a computer-facing mouse and assigns it for exclusive use to the
 * caller. If provided, register 1 contents are checked to determine how
 * _update() should treat motion data submitted later (bytes 4-5).
 *
 * _Note_: to avoid expensive division the scaling implementation here is
 * tailored to work with devices that are some power of 2 multipled by 100.
 * Using other CPI values will cause inaccuracy.
 *
 * @param *id            on success, set to the ID that should be used during
 *                       enqueue operations.
 * @param mode           the starting mouse mode to use.
 * @param *reg1          array of 8 bytes for MOUSE_MODE_EXTENDED, otherwise
 *                       otherwise; providing NULL when mode is extended drops
 *                       the device to MOUSE_MODE_100CPI.
 * @return               true if registration was successful, false otherwise.
 */
bool mouse_register(uint8_t *id, mouse_mode mode, uint8_t *reg1);

/**
 * Updates the mouse position information, appending it to any existing data.
 * This locks an internal semaphore and may block when called.
 *
 * Callers should provide motion data scaled to the native CPI of their
 * device. This will internally handle scaling based on both the current
 * DHID/extended state with the computer(s).
 *
 * @param id   the ID to use from the original registration call.
 * @param dx   change in X-axis position.
 * @param dy   change in Y-axis position.
 * @param btn  button state bitmask following ADB convention: 0=down, LSB b1
 */
bool mouse_update(uint8_t id, int32_t dx, int32_t dy, uint8_t btn);

#endif /* __MOUSE_H__ */
