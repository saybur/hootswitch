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
 * - void get_func(uint8_t reg, uint8_t *data, uint8_t *data_len)
 * - void set_func(uint8_t reg, uint8_t *data, uint8_t data_len)
 * - void flush_func(uint8_t reg)
 * - void reset_func()
 * - uint8_t get_handle()
 * - void set_handle(uint8_t mach, uint8_t handle)
 * - bool poll()
 *
 * These share a common set of parameters:
 *
 * - `reg` is a register value, 1:1 to the ADB register, from 0-2 (register 3
 *   is handled for you).
 * - `data` is an array, generally 8 bytes long, to either write data into or
 *   out of.
 * - `mach` is the connected machine ID, from 0 to 3. This may be useful for
 *   differentiating which machine you're talking to; some drivers may not care
 *   and ignore this, which is fine.
 * - `handle` is the ADB device handle value. After reset you should use the
 *   default. If you don't support a given handle you should ignore set_handle.
 *   These are swapped in and out as the user switches machines, expect them to
 *   move around during normal use more than they would be with a typical
 *   device.
 *
 * Notes for each:
 *
 * - get_func is called in response to a Talk command. Return relevant data for
 *   that register, or zero data length if no response is needed.
 * - set_func is called in response to a Listen command. Set data for that
 *   register if needed.
 * - flush_func is called in response to a Flush command.
 * - reset_func is called in response to a reset pulse. Reset state (and
 *   handler) to defaults.
 * - get_handle_func should return the currently assigned handle. This can be
 *   either the default _or_ a handle from set_handle_func that was actually
 *   accepted.
 * - set_handle_func offers a new handle to assign. This is called both when
 *   the user is switching between systems _and_ when the remote system is
 *   actively changing the driver handle via Listen Register 3. If the handle
 *   is supported it should be assigned internally and then returned each time
 *   from get_handle_func, otherwise leave the old handle alone.
 * - poll_func is called periodically to give time for your driver to do
 *   whatever it wants. This is cooperative, do not perform excessive
 *   processing here if you can avoid it. Return true if you have new data,
 *   false otherwise (aka true to queue up a SRQ). If you _really_ need
 *   processing time core1 is available for your use, but it will be your
 *   responsibility to figure out the various challenges associated with
 *   multicore usage on the Pico! :)
 *
 * Except for poll() these are all invoked from the interrupt context. As a
 * result, it is important they return _quickly_ and work with variables set to
 * be `volatile` in your code. With get_func you are called during ADB Tlt and
 * have ~100us to return before things break. The others generally have looser
 * timing requirements but still should return promptly.
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
	void (*get_func)(uint8_t, uint8_t*, uint8_t*);
	void (*set_func)(uint8_t, uint8_t*, uint8_t);
	void (*flush_func)(uint8_t);
	void (*reset_func)(void);
	uint8_t (*get_handle)(void);
	void (*set_handle)(uint8_t, uint8_t);
	bool (*poll)(void);
} dev_driver;

uint8_t driver_register(dev_driver driver);
void driver_poll(void);

#endif /* __DRIVER_H__ */
