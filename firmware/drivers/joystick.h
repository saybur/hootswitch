/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
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
	int8_t x, y;
	uint8_t brake, throttle;
	uint32_t buttons;
} joystick_data;

/**
 * General indicator for whether the joystick driver has been configured to a
 * non-standard handler by the active computer. This can be used as an
 * indicator of whether a Mac driver has loaded on this system for the device.
 * Even when not enabled mouse emulation should (theoretically) be present.
 *
 * @param id  the ID provided at registration time.
 * @return    true if the joystick has been set to a non-standard handler.
 */
bool joystick_enabled(uint8_t id);

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

/**
 * Indicates whether data is waiting to be sent for the joystick. This can be
 * used to help rate-limit certain devices when needed.
 *
 * This will return true if there is no valid computer to send to.
 *
 * @return  true if data is waiting to be sent, false otherwise.
 */
bool joystick_waiting(uint8_t id);

#endif /* __JOYSTICK_H__ */
