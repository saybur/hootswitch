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

#ifndef __HOST_SYNC_H__
#define __HOST_SYNC_H__

#include <stdbool.h>
#include <stdint.h>

#include "handler.h"
#include "host.h"
#include "util.h"

/**
 * Executes a synchronous command on the host bus.
 *
 * This will busy-wait and block until the command completes. It is intended
 * for initialization code, which is otherwise quite annoying to write with
 * callbacks. You can _technically_ use it at other times but it is not
 * recommended.
 *
 * The same caveats for the _async version apply: only change things for
 * devices assigned to you and do not mess with the address of a device without
 * updating the associated `ndev_info` struct (and use extreme caution while
 * even doing so, it is not well tested).
 *
 * This will remove the high nibble of the command byte unless you call with
 * special device 0xFF. This should generally only be done in special
 * situations.
 *
 * @param dev     device number, or 0xFF in rare cases to bypass (see next).
 * @param cmd     ADB command to execute (low 4 bits if dev specified,
 *                all 8 bits if dev is 0xFF.
 * @param data    data to send if Listen, an array of at least 8 bytes if Talk,
 *                ignored otherwise.
 * @param length  length to send if Listen, will be set to actual length
 *                received if Talk.
 * @return        status of the response, will return immediately if not OK.
 */
host_err host_sync_cmd(uint8_t dev, uint8_t cmd,
		uint8_t *data, uint8_t *length);

// used during the command callback, do not invoke
void host_sync_cb(host_err err, uint16_t id, cmd_type type,
		volatile uint8_t *data, uint8_t data_len);

#endif /* __HOST_SYNC_H__ */
