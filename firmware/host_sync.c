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

#include "pico/stdlib.h"

#include "debug.h"
#include "host_sync.h"

static volatile bool callback;
static volatile cmd_type type_back;
static volatile uint32_t id_back;
static volatile uint8_t data_back[8];
static volatile uint8_t length_back;
static volatile host_err error_back;

void host_sync_cb(host_err err, uint32_t id, cmd_type type,
		volatile uint8_t *data, uint8_t data_len)
{
	callback = true;
	id_back = id;
	type_back = type;

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
