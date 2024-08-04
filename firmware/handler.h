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

#ifndef __HANDLER_H__
#define __HANDLER_H__

#include <stdint.h>

/*
 * The initialization routine works like this:
 *
 * 0) handler_init() is called. Add a call there (or in driver_init()) to
 *    register a `ndev_handler` with the host implementation.
 * 1) The host resets all native devices and readdresses them, storing the
 *    information in `ndev_info` structs.
 * 2) The host goes through the handlers in insertion order. For each native
 *    device with a default handler matching the 'seek' value, the host will
 *    attempt to assign the 'set' handler to the device. If the device accepts
 *    it, your setup handler will be called with device information.
 *
 * FIXME COMPLETE
 */

typedef struct {
	char *name;
	uint8_t handle_seek;
	uint8_t handle_set;
} ndev_handler;

void handler_get(uint8_t dev, ndev_handler *handler);
void handler_poll(void);

#endif /* __HANDLER_H__ */
