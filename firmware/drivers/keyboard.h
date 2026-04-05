/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

typedef struct {
	uint8_t length;
	uint8_t data[2];
} keyboard_message;

/**
 * Enqueues a message to send to the active computer at the next opportunity.
 *
 * @param id  the ID to use from the original registration call.
 * @param m   the message to enqueue.
 */
void keyboard_enqueue(uint8_t id, keyboard_message *m);

/**
 * Registers a computer-facing keyboard and assigns it for exclusive use to
 * the caller.
 *
 * @param *id            on success, set to the ID that should be used during
 *                       enqueue operations.
 * @param reg2_callback  function called at appropriate times to update
 *                       register 2; this may be null and each user may choose
 *                       how to respect this call (or not).
 * @return               true if registration was successful, false otherwise.
 */
bool keyboard_register(uint8_t *id, void (*reg2_callback)(uint8_t, uint16_t));

#endif /* __KEYBOARD_H__ */
