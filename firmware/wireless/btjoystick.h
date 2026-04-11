/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __BTJOYSTICK_H__
#define __BTJOYSTICK_H__

#include <uni.h>

void bt_joystick_set(uni_gamepad_t *report);
bool bt_joystick_waiting(void);

void bt_joystick_init(void);

#endif /* __BTJOYSTICK_H__ */
