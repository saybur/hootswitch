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
