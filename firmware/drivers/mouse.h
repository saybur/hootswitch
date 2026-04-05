/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
