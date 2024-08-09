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
 * implementations to tie into the firmware with minimal changes to the rest
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
 * - void reset_func(uint8_t mach, uint8_t dev)
 * - void switch_func(uint8_t mach)
 * - void talk_func(uint8_t mach, uint8_t dev, uint8_t reg,
 *                 uint8_t *data, uint8_t *data_len)
 * - void listen_func(uint8_t mach, uint8_t dev, uint8_t reg,
 *                 uint8_t *data, uint8_t data_len)
 * - void flush_func(uint8_t mach, uint8_t dev)
 * - bool srq_func(uint8_t mach, uint8_t dev)
 * - void get_handle_func(uint8_t mach, uint8_t dev, uint8_t *handle)
 * - void set_handle_func(uint8_t mach, uint8_t dev, uint8_t handle)
 * - void poll_func(void)
 *
 * These share some common parameters:
 *
 * - `mach` is the machine ID that the task is coming from.
 * - `dev` is the device list ID returned during registration, to differentiate
 *   between multiple devices if more than one gets registered for the same
 *   underlying functions. If you don't register more than one device you can
 *   ignore this.
 * - `reg` is a register value, 1:1 to the ADB register, from 0-2 (register 3
 *   is handled for you).
 * - `data` is an array, generally 8 bytes long, to either write data into or
 *   out of.
 * - `data_len` defines the number of values in the above array, either
 *   supplied or requested.
 * - `handle` is the ADB device handler ID (DHI) value. After reset you should
 *   use your driver's default. If you don't support a given handle you should
 *   ignore requests to change it.
 *
 * Notes for each:
 *
 * - (-) reset_func is called in response to a reset pulse. Reset state (and
 *   DHI) to defaults.
 * - (-) switch_func is called to indicate the active machine the user has
 *   picked is being switched to the one provided.
 * - (!) talk_func is called in response to a Talk command, used when the
 *   machine wants get data from the driver. Return relevant data for that
 *   register, or zero data length if no response is needed.
 * - (!) listen_func is called in response to a Listen command, used when the
 *   machine wants to set data for a register.
 * - (!) flush_func is called in response to a Flush command.
 * - (!) srq_func is called to determine if the driver has data it wants to
 *   return in Register 0 for a given device. The implementation will
 *   automatically figure out if a SRQ is warranted based on a check of this
 *   value, so just return true/false based on whether there is new data.
 * - (!) get_handle_func should return the currently assigned DHI. This can
 *   be either the default _or_ a DHI from set_handle_func that was actually
 *   accepted.
 * - (!) set_handle_func offers a new DHI to assign. If the handle is supported
 *   it should be assigned internally and then returned each time from
 *   get_handle_func, otherwise leave the old handle alone.
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
 * - talk_func and get_handle_func are called during ADB Tlt and usually have
 *   less than ~60us to generate results.
 * - srq_func is called during the command stop bit and must share time with
 *   all device drivers; it is _highly_ recommended to store this in a variable
 *   that can be instantly returned.
 *
 * The functions with (-) are invoked from the process context. You may still
 * (generally) receive interrupt commands while these are executing.
 *
 * There is only one "active" machine at any time, the others are inactive. It
 * is up to the driver to decide how to handle active vs inactive machines. For
 * example, a mouse driver will probably want to only return data to an active
 * machine. However, a virtual modem might want to return data to all machines
 * and maintain state separately for each one, effectively acting like multiple
 * independent devices. `talk_inactive` and `srq_inactive` help guide how a
 * driver should be called: the first controls if Talk 0 instructions should be
 * passed through from inactive machines (if false these are suppressed and the
 * driver is not notified of them), the second controls if SRQs should be
 * checked from the driver for inactive machines.
 *
 * ADB initializes at startup and the device table is not updated again (unless
 * something exceptional happens, like ADBReinit). To accomodate this the
 * manager keeps track of which devices are registered as of the time it
 * receives a machine reset. If you are not registered then you will not be
 * called while that machine is active. Restarting the affected computer
 * will cause a new reset event, after which your calls will be invoked
 * normally. This isn't great, but hey, it's ADB, what can you do.
 *
 * Host initialization and native device assignment happens before this is
 * used, feel free to register in response to an assignment call from there.
 */

typedef struct {
	bool talk_inactive;  // if true, pass Talk 0 requests inactive machines
	bool srq_inactive;
	void (*reset_func)(uint8_t, uint8_t);
	void (*switch_func)(uint8_t);
	void (*talk_func)(uint8_t, uint8_t, uint8_t, uint8_t*, uint8_t*);
	void (*listen_func)(uint8_t, uint8_t, uint8_t, uint8_t*, uint8_t);
	void (*flush_func)(uint8_t, uint8_t);
	bool (*srq_func)(uint8_t);
	void (*get_handle_func)(uint8_t, uint8_t, uint8_t*);
	void (*set_handle_func)(uint8_t, uint8_t, uint8_t);
	bool (*poll_func)(void);
} dev_driver;

/*
 * The maximum number of simultaneous drivers allowed to be installed at any
 * given time. This _cannot_ be higher than 16, but given ADB addressing
 * limitations it is very likely a lower number is the true maximum.
 */
#define DRIVER_MAXIMUM  8

/**
 * @return  the current number of registered drivers.
 */
uint8_t driver_count(void);

/**
 * Register a new device. This will set a device identifier in the given
 * pointer, which will be used when calling back the functions given.
 *
 * @param *dev_id  will be set to the identifier for later callbacks.
 * @param *driver  the new driver to add.
 * @return         true if it was added, false otherwise.
 */
bool driver_register(uint8_t *dev_id, dev_driver *driver);

/*
 * ----------------------------------------------------------------------------
 * The remaining functions in this unit are used internally, do not invoke
 * these directly from a driver.
 * ----------------------------------------------------------------------------
 */

bool driver_get(uint8_t dev, dev_driver **driver);
void driver_init(void); // see .c file for details
void driver_poll(void);

#endif /* __DRIVER_H__ */
