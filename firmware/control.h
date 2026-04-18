/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __CONTROL_H__
#define __CONTROL_H__

/*
 * Defines a control system that can be used to remotely operate a device. The
 * interface operates using control frames, with each frame containing one
 * command byte followed by zero or more data bytes. Over USB CDC these are
 * encapsulated using SLIP:
 *
 * <https://en.wikipedia.org/wiki/Serial_Line_Internet_Protocol>
 *
 * Commands below CONTROL_CODE_SEGMENT are deferred to the serial interface.
 * The remaining commands are defined below.
 *
 * The SLIP encapsulation is for writing to the device only. Reading from it
 * uses the logging system and ASCII.
 */

#define CONTROL_CODE_SEGMENT          0x80

#define CONTROL_BT_SCAN               0xE0
#define CONTROL_DBG_TRACE             0xE8
#define CONTROL_DBG_HEAP              0xEA
#define CONTROL_DBG_LIST              0xEB
#define CONTROL_DBG_STATS             0xEC
#define CONTROL_REBOOT                0xF1
#define CONTROL_REBOOT_DEBUG          0xF2

typedef enum {
	RESET_TYPE_NORMAL = 0,
	RESET_TYPE_DEBUG
} control_reset_type;

/**
 * Indicates if there was a special reset condition that should change the
 * device startup mode.
 *
 * @return any special device startup flag, 0 if none is present.
 */
control_reset_type control_check_reset(void);

/**
 * Task responsible for the serial control interface. Users should not call
 * this function.
 */
void control_task(void *parameters);

#endif /* __CONTROL_H__ */
