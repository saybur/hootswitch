/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __NOTIFY_H__
#define __NOTIFY_H__

#include <stdbool.h>

typedef enum {
	NOTIFY_HOST_INIT_RESET_FAIL,
	NOTIFY_HOST_INIT_NO_DEVICES,
	NOTIFY_COMPUTER_SWITCH,
	NOTIFY_DEVICE_CONNECT,
	NOTIFY_DEVICE_DISCONNECT
} notify_type;

#define NOTIFY_ERROR_FLASH_FREQUENCY              250
#define NOTIFY_HOST_INIT_RESET_FAIL_FLASHES       5
#define NOTIFY_HOST_NO_DEVICES_FLASHES            4

/**
 * Enqueues a notification for the user. This is safe to call from any thread.
 *
 * 'Notifications' for the device are pre-baked 'messages' sent to the user via
 * the LEDs and the speaker. The idea here is for end users to get a basic
 * sense of what's going on without the USB debug console. Only one can be
 * playing back at any given time, but new ones can be enqueued via calling
 * into this function.
 *
 * @param type  the next notification to play.
 * @return      true if it could be enqueued, false if not (queue full).
 */
bool notify_user(notify_type type);

/**
 * Internal task for the notification (which operates purely on FreeRTOS delays
 * and similar functions). Do not invoke from user code.
 */
void notify_task(__unused void *parameters);

#endif /* __NOTIFY_H__ */
