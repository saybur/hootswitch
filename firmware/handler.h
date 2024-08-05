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

/*
 * Defines an external interface for interoperating with native devices on the
 * host side of the switch.
 *
 * Initialization of this subsystem works as follows:
 *
 * 0) handler_init() is called. Add a call there to register a `ndev_handler`
 *    with host_register().
 * 1) The host resets all native devices and readdresses them, storing the
 *    information in `ndev_info` structs. The `reset_func` of all handlers is
 *    called.
 * 2) The host goes through the handlers in reverse insertion order. The
 *    handlers will be interviewed to see if they want the device. If the
 *    device accepts it, that handler will be called with device information
 *    via `assign_func`. This will likely need to update Register 3 with an SRQ
 *    flag [TODO should this be automatic??]
 * 3) From this point, any time a command is completed on an assigned device
 *    the matching handler will be called back by the host implementation. See
 *    the list of calls for more details.
 *
 * The functions to implement are as follows.
 *
 * - bool interview_func(ndev_info *info, uint8_t *err)
 * - void assign_func(uint8_t dev, ndev_info *info)
 * - void talk_func(uint8_t dev, uint16_t cid, uint8_t reg,
 *                  uint8_t *data, uint8_t data_len)
 * - void listen_func(uint8_t dev, uint16_t cid, uint8_t reg)
 *                  uint8_t *data, uint8_t *data_len)
 * - void flush_func(uint8_t dev, uint16_t cid, uint8_t reg)
 * - void reset_func(void)
 * - void poll(void)
 *
 * These share common parameters:
 *
 * - `dev` is a host device ID value that uniquely identifies a specific piece
 *   of hardware (it is distinct from the device ADB address). Each call
 *   receives this to help you keep track of multiple devices.
 * - `id` is a per-command identifier that is 1:1 with the value provided by
 *   the async command queue system, to help you keep track of which command
 *   the response is in relation to. This will be 0 if there is no associated
 *   async command.
 * - `reg` is the ADB register, from 0 - 3. Do not mess with the device address
 *   or you will confuse the host implementation (changing the handler of a
 *   device you own is fine as long as you update the appropriate struct).
 *
 * Notes for each:
 *
 * - (-) reset_func is called whenever a host-side reset occurs. This will
 *   always happen once at the beginning of the program. It may happen again
 *   if someone resets the host later. When received, drop all devices you
 *   own.
 * - (-) interview_func is called with information about a device, asking the
 *   handler to give a yes/no answer to whether it should be assigned this
 *   device. Simpler handlers can just check the ADB "device handler ID" (DHID)
 *   to answer the question. More compliated handlers may want to query the
 *   device and see if it accepts a different DHID; if that is done, be sure to
 *   update the struct members appropriately. Return true to take control of
 *   the device, false otherwise. Return a nonzero value in the int pointer if
 *   a malfunction occurs that should block further use of the device by other
 *   handlers.
 * - (-) assign_func is called to grant control of a particular device to a
 *   handler. The handler will be called back with any information from this
 *   device.
 * - (!) talk_func is called whenever the device has returned data for a
 *   Talk Register command. If you set `accept_noop_talks` to true, this will
 *   be called with a length of 0 whenever a no-op talk occurs (which can be
 *   frequent, depending on the device). If false this is only called when
 *   there is data to be processed.
 * - (!) listen_func is called to fetch data when the system is ready to send
 *   the command to the device, during Tlt. This must return quickly to avoid
 *   problems. Set the data in the array and the length you want to send.
 * - (!) flush_func is called in response to a completed Flush command, to
 *   let the handler know it has been sent. It can usually be ignored.
 * - (-) poll_func is called periodically to give time for your handler to
 *   do whatever it wants. This is cooperative, do not perform excessive
 *   processing here if you can avoid it. See the driver notes for details,
 *   this works identically.
 *
 * The functions with (!) above are invoked from the interrupt context. As a
 * result it is important they return _quickly_ and work with variables set to
 * be `volatile` in your code.
 *
 * The functions with (-) are invoked from the process context. You may still
 * (generally) receive interrupt commands while these are executing.
 *
 * The functions marked with [n] may be NULL.
 *
 * Unless otherwise noted none of these calls are reentrant, are NOT safe for
 * use from ISRs and/or the other CPU core.
 */

// reflects information from register 3
typedef struct {
	uint8_t address_def;  // default address at reset time
	uint8_t address_cur;  // current address, don't change this!
	uint8_t dhid_def;     // default device handler ID at reset time
	uint8_t dhid_cur;     // current device handler ID
	bool fault;           // true if the device is skipped due to HW issues
} ndev_info;

// struct to submit for registration, see (long) description above.
typedef struct {
	const char *name;
	bool accept_noop_talks;
	void (*reset_func)(void);
	bool (*interview_func)(volatile ndev_info*, uint8_t*);
	void (*assign_func)(uint8_t, volatile ndev_info*);
	void (*talk_func)(uint8_t, uint16_t, uint8_t, uint8_t*, uint8_t);
	void (*listen_func)(uint8_t, uint16_t, uint8_t, uint8_t*, uint8_t*);
	void (*flush_func)(uint8_t, uint16_t, uint8_t);
	void (*poll_func)(void);
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
void handler_poll(void);

#endif /* __HANDLER_H__ */
