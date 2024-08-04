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

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * Defines a thin abstraction layer for device emulators ("drivers") mirroring
 * the ADB logical command set. This approach is designed to allow new
 * compilation units to tie into the firmware with minimal changes to the rest
 * of the program code.
 *
 * This is conceptually similar but distinct from "handlers," which are used on
 * the host emulation side. Drivers may want both if they are translating for a
 * real device. See the handler code for details.
 *
 * To use, create an instance of the following struct and register it. This
 * provides several functions which will be called back by the connected
 * computers to perform varous tasks. The functions are as follows:
 *
 * - void get_func(uint8_t dev, uint8_t mach, uint8_t reg, uint8_t *data, uint8_t *data_len)
 * - void set_func(uint8_t dev, uint8_t mach, uint8_t reg, uint8_t *data, uint8_t data_len)
 * - void flush_func(uint8_t dev, uint8_t mach, uint8_t reg)
 * - void reset_func(uint8_t dev, uint8_t mach)
 * - bool srq_func(uint8_t mach)
 * - uint8_t get_handle_func(uint8_t dev, uint8_t mach)
 * - void set_handle_func(uint8_t dev, uint8_t mach, uint8_t handle)
 * - void switch_func(uint8_t mach)
 * - void poll_func()
 *
 * These share some common parameters:
 *
 * - `dev` is the device list ID returned during registration, to differentiate
 *   between multiple devices if more than one gets registered. If you don't
 *   register more than one device you can ignore this.
 * - `mach` is the machine ID from 0 to 3 that the task is coming from (more on
 *   this later).
 * - `reg` is a register value, 1:1 to the ADB register, from 0-2 (register 3
 *   is handled for you).
 * - `data` is an array, generally 8 bytes long, to either write data into or
 *   out of.
 * - `handle` is the ADB device handle value. After reset you should use the
 *   default. If you don't support a given handle you should ignore requests
 *   to change it.
 *
 * Notes for each:
 *
 * - (!) get_func is called in response to a Talk command. Return relevant data
 *   for that register, or zero data length if no response is needed.
 * - (!) set_func is called in response to a Listen command. Set data for that
 *   register if needed.
 * - (!) flush_func is called in response to a Flush command.
 * - (-) reset_func is called in response to a reset pulse. Reset state (and
 *   handler) to defaults.
 * - (!) get_handle_func should return the currently assigned handle. This can
 *   be either the default _or_ a handle from set_handle_func that was actually
 *   accepted.
 * - (!) set_handle_func offers a new handle to assign. This is called both
 *   when the user is switching between systems _and_ when the remote system is
 *   actively changing the driver handle via Listen Register 3. If the handle
 *   is supported it should be assigned internally and then returned each time
 *   from get_handle_func, otherwise leave the old handle alone.
 * - (-) switch_func changes the active machine the user has picked to the one
 *   provided.
 * - (-) poll_func is called periodically to give time for your driver to do
 *   whatever it wants. This is cooperative, do not perform excessive
 *   processing here if you can avoid it. If you _really_ need processing time
 *   core1 is available for your use, but it will be your responsibility to
 *   figure out the various challenges associated with multicore usage on the
 *   Pico! :)
 *
 * The functions with (!) above are invoked from the interrupt context. As a
 * result it is important they return _quickly_ and work with variables set to
 * be `volatile` in your code. The following have particularly sensitive
 * timings:
 *
 * - get_func and get_handle_func are called during ADB Tlt and usually have
 *   less than ~60us to generate results.
 * - srq_func is called during the command stop bit and must share time with
 *   all device drivers; it is _highly_ recommended to store this in a variable
 *   that can be instantly returned.
 *
 * The functions with (-) are invoked from the process context. You may still
 * (generally) receive the interrupt commands while these are executing.
 *
 * There is only one "active" machine at any time, the others are inactive. It
 * is up to the driver to decide how this should be handled. For example, a
 * mouse driver will probably want to only return data to an active machine.
 * However, a virtual modem might want to return data to all machines and
 * maintain state separately for each one, effectively acting like multiple
 * independent devices.
 *
 * ADB initializes at startup and the device table is not updated again (unless
 * something exceptional happens, like ADBReinit). To accomodate this the
 * manager keeps track of which devices are registered as of the time it
 * receives a machine reset. If you are not registered then you will not be
 * called while that machine is active. Restarting the affected computer
 * will cause a new reset event, after which your calls will be invoked
 * normally. This isn't great, but hey, it's ADB, what can you do.
 */

typedef struct {
	void (*get_func)(uint8_t, uint8_t, uint8_t, uint8_t*, uint8_t*);
	void (*set_func)(uint8_t, uint8_t, uint8_t, uint8_t*, uint8_t);
	void (*flush_func)(uint8_t, uint8_t, uint8_t);
	void (*reset_func)(uint8_t, uint8_t);
	bool (*srq_func)(uint8_t);
	uint8_t (*get_handle_func)(uint8_t, uint8_t);
	void (*set_handle_func)(uint8_t, uint8_t, uint8_t);
	void (*switch_func)(uint8_t);
	bool (*poll)(void);
} dev_driver;

/*
 * The maximum number of simultaneous drivers allowed to be installed at any
 * given time. This _cannot_ be higher than 16, but given ADB addressing
 * limitations it is very likely a lower number is the true maximum.
 */
#define DRIVER_MAXIMUM  8

/*
 * Register a new device. This will set a device identifier in the given
 * pointer, which will be used when calling back the functions given.
 *
 * This will return a nonzero result if the device canot be added.
 */
uint8_t driver_register(uint8_t *dev_id, dev_driver *driver);

/*
 * ----------------------------------------------------------------------------
 * The remaining functions in this unit are used internally, do not invoke
 * these directly from a driver.
 * ----------------------------------------------------------------------------
 */

void driver_get(uint8_t dev, dev_driver *driver);
void driver_poll(void);

#endif /* __DRIVER_H__ */
