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

#include <stdbool.h>
#include <stdint.h>

#include "host_err.h"

/*
 * Defines an external interface for interoperating with native devices on the
 * host side of the switch.
 *
 * Initialization of this subsystem works as follows:
 *
 * 0) handler_init() is called. Add a call there to register a `ndev_handler`
 *    with host_register().
 * 1) The host resets all native devices and readdresses them, storing the
 *    information in `ndev_info` structs.
 * 2) The host goes through the handlers in reverse insertion order. The
 *    handlers will be interviewed to see if they want the device. If the
 *    device accepts it, that handler will be called with device information
 *    via `assign_func`.
 * 3) From this point, any time a command is completed on an assigned device
 *    the matching handler will be called back by the host implementation. See
 *    the list of calls for more details.
 *
 * The functions to implement are as follows.
 *
 * - bool interview_func(ndev_info *info, uint8_t *err)
 * - void assign_func(ndev_info *info, uint8_t hdev)
 * - void talk_func(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg,
 *                  uint8_t *data, uint8_t data_len)
 * - void listen_func(uint8_t hdev, host_err err, uint32_t cid, uint8_t reg)
 * - void flush_func(uint8_t hdev, host_err err, uint32_t cid)
 *
 * These share common parameters:
 *
 * - `hdev` is a host device ID value that uniquely identifies a specific piece
 *   of hardware (it is distinct from the device ADB address). Each call
 *   receives this to help you keep track of multiple devices.
 * - `err` receives any errors that happened during execution of the command
 *   or subsequent data phase.
 * - `id` is a per-command identifier that is 1:1 with the value provided by
 *   the async command queue system, to help you keep track of which command
 *   the response is in relation to. This will be 0 if there is no associated
 *   async command (due to how ADB works this implies Talk Register 0).
 * - `reg` is the ADB register, from 0 - 3. Do not mess with the device address
 *   or you will confuse the host implementation (changing the handler of a
 *   device you own is fine as long as you update the struct given in
 *   `assign_func`).
 *
 * Notes for each:
 *
 * - interview_func is called with information about a device, asking the
 *   handler to give a yes/no answer to whether it should be assigned this
 *   device. Simpler handlers can just check the "device handler ID" (DHI)
 *   value in the struct to answer the question. More compliated handlers may
 *   want to query the device and see if it accepts a different DHI, which
 *   can be done by calling the provided function, which will return true if
 *   the proposed DHI was accepted, false otherwise (the struct will auto-
 *   update with any new DHI value). Return true to take control of the device,
 *   false otherwise. If a malfunction occurs that should block further use of
 *   the device by other handlers set `fault` within the given struct.
 * - talk_func is called whenever the device has returned data from a
 *   Talk Register command. If you set `accept_noop_talks` to true, this will
 *   be called with a length of 0 whenever a no-op talk occurs, which can be
 *   frequent depending on the device. If false this is only called when there
 *   is data to be processed.
 * - [n] listen_func is called back to inform the handler that previously-
 *   submitted Listen data has been sent to the remote device.
 * - [n] flush_func as above for completed Flush command.
 *
 * All functions above are invoked from the handler task. The functions marked
 * with [n] may be NULL.
 *
 * Most handlers will also be drivers and thus must deal with the potential for
 * concurrent modification from calls to their driver functions. There are
 * several ways to get around this problem that will depend on the driver, but
 * generally using a Pico SDK mutex will be the most straightforward. See
 * the SDK documentation for details.
 */

// reflects information from register 3
typedef struct {
	uint8_t hdev;         // unique hardware ID (see above)
	uint8_t address_def;  // default address at reset time
	uint8_t address_cur;  // current address, don't change this!
	uint8_t dhid_def;     // default device handler ID at reset time
	uint8_t dhid_cur;     // current device handler ID
	bool fault;           // set if the device should be skipped
} ndev_info;

// struct to submit for registration, see (long) description above.
typedef struct {
	const char *name;
	bool accept_noop_talks;
	bool (*interview_func)(volatile ndev_info*, bool (*handle_change)(uint8_t));
	void (*talk_func)(uint8_t, host_err, uint32_t, uint8_t, uint8_t*, uint8_t);
	void (*listen_func)(uint8_t, host_err, uint32_t, uint8_t);
	void (*flush_func)(uint8_t, host_err, uint32_t);
} ndev_handler;

// maximum number of simultaneous handlers supported
#define HANDLER_MAX       16

/**
 * @return  the current number of registered handlers.
 */
uint8_t handler_count();

/**
 * Register a new handler in the internal list. Must be called before the host
 * initalizes, usually from handler_init().
 *
 * @param *handler  the new handler to add.
 * @return          true if it was added, false otherwise.
 */
bool handler_register(ndev_handler *handler);

/*
 * ----------------------------------------------------------------------------
 * The remaining functions in this unit are used internally, do not invoke
 * these directly from a driver.
 * ----------------------------------------------------------------------------
 */

bool handler_get(uint8_t id, ndev_handler **handler);
void handler_init(void); // see .c file for details

#endif /* __HANDLER_H__ */
