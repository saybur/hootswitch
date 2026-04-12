/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __CONTROL_H__
#define __CONTROL_H__

#define CONTROL_REBOOT                0xF1
#define CONTROL_REBOOT_DEBUG          0xF2

typedef enum {
	RESET_TYPE_NORMAL = 0,
	RESET_TYPE_DEBUG
} control_reset_type;

typedef enum {
	CONTROL_MODE_IDLE = 0,
	CONTROL_MODE_FLYBYWIRE
} control_mode_type;

/**
 * Indicates if there was a special reset condition that should change the
 * device startup mode.
 *
 * @return any special device startup flag, 0 if none is present.
 */
control_reset_type control_check_reset(void);

/**
 * Indicates to the control system that system startup is complete and it may
 * begin performing non-core functions.
 */
void control_start(void);

/**
 * Task responsible for the serial control interface. Users should not call
 * this function.
 */
void control_task(void *parameters);

#endif /* __CONTROL_H__ */
