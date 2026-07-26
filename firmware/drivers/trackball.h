/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __TRACKBALL_H__
#define __TRACKBALL_H__

typedef enum {
	TRACKBALL_MODE_KENS_TM4,
	TRACKBALL_MODE_KENS_TM5
} trackball_mode;

/**
 * Registers a computer-facing trackball and assigns it for exclusive use to
 * the caller.
 *
 * @param *id    on success, set to the ID that should be used during enqueue
 *               operations.
 * @param mode   the trackball mode to use.
 * @return       true if registration was successful, false otherwise.
 */
bool trackball_register(uint8_t *id, trackball_mode mode);

/**
 * Updates the trackball position information, appending it to any existing
 * data. This locks an internal semaphore and may block when called.
 *
 * Callers should provide motion data scaled to the native CPI of their
 * device. This will internally handle scaling based on both the current
 * DHID/extended state with the computer(s).
 *
 * @param id   the ID to use from the original registration call.
 * @param x    change in X-axis position.
 * @param y    change in Y-axis position.
 * @param btn  button state bitmask following ADB convention: 0=down, LSB b1
 */
bool trackball_push(uint8_t id, int16_t x, int16_t y, uint8_t btn);

#endif /* __TRACKBALL_H__ */
