/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __HOST_ERR_H__
#define __HOST_ERR_H__

typedef enum {
	HOSTERR_OK = 0,
	HOSTERR_TIMEOUT = 1,
	HOSTERR_FULL = 2,
	HOSTERR_INVALID_PARAM = 3,
	HOSTERR_LINE_STUCK = 4,
	HOSTERR_TOO_MANY_DEVICES = 5,
	HOSTERR_BAD_DEVICE = 6,
	HOSTERR_NO_DEVICES = 7,
	HOSTERR_BAD_STATE = 8,
	HOSTERR_BAD_RESPONSE = 9
} host_err;

#endif /* __HOST_ERR_H__ */
