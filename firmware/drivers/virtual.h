/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __VIRTUAL_H__
#define __VIRTUAL_H__

#include <stdbool.h>

#include "joystick.h"

typedef struct {
	int16_t x, y;
	uint8_t buttons; // 1=up, 0=down, LSB button 1
} virtual_mouse_data;

bool virtual_keyboard_offer(bool up, uint8_t c);
bool virtual_mouse_offer(virtual_mouse_data *data);

uint8_t virtual_keyboard_index(void);

void virtual_init(void);

#endif /* __VIRTUAL_H__ */
