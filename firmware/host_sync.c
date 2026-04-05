/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "pico/stdlib.h"

#include "debug.h"
#include "host_sync.h"

#define MAX_CB_DATA 8

static volatile bool callback;
static volatile cmd_type type_back;
static volatile uint32_t id_back;
static volatile uint8_t data_back[MAX_CB_DATA];
static volatile uint8_t length_back;
static volatile host_err error_back;

void host_sync_cb(host_err err, uint32_t id, cmd_type type,
		volatile uint8_t *data, uint8_t data_len)
{
	callback = true;
	id_back = id;
	type_back = type;

	if (data_len > MAX_CB_DATA) data_len = MAX_CB_DATA;
	if (type == TYPE_TALK) {
		for (uint8_t i = 0; i < data_len; i++) {
			data_back[i] = data[i];
		}
		length_back = data_len;
	} else {
		length_back = 0;
	}

	error_back = err;
}

host_err host_sync_cmd(uint8_t dev, uint8_t cmd,
		uint8_t *data, uint8_t *length)
{
	callback = false;

	// data is OK but we have to dereference length, disallow null
	if (length == NULL) {
		return HOSTERR_INVALID_PARAM;
	}

	uint32_t id;
	host_err res;
	if (res = host_cmd(dev, cmd, &id, data, *length)) {
		return res;
	}

	while (! callback) {
		host_poll();
	}

	// check that callback is valid
	cmd_type type = util_parse_cmd_type(cmd);
	if (type_back != type || id_back != id) {
		return HOSTERR_BAD_STATE;
	}

	if (type == TYPE_TALK) {
		uint8_t lb = length_back;
		for (uint8_t i = 0; i < lb; i++) {
			data[i] = data_back[i];
		}
		*length = lb;
	} else {
		*length = 0;
	}

	return error_back;
}
