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
 * - void reset_func(uint8_t comp, uint32_t ref)
 * - void switch_func(uint8_t comp)
 * - void talk_func(uint8_t comp, uint32_t ref, uint8_t reg)
 * - void listen_func(uint8_t comp, uint32_t ref, uint8_t reg,
 *                 volatile uint8_t *data, uint8_t data_len)
 * - void flush_func(uint8_t comp, uint32_t ref)
 * - bool srq_func(uint8_t comp, uint32_t ref)
 * - void get_handle_func(uint8_t comp, uint32_t ref, uint8_t *handle)
 * - void set_handle_func(uint8_t comp, uint32_t ref, uint8_t handle)
 * - void poll_func(void)
 *
 * These share some common parameters:
 *
 * - `comp` is the computer ID that the task is coming from.
 * - `ref` is the reference value originally provided during registration.
 * - `reg` is a register value, 1:1 to the ADB register, from 0-2 (register 3
 *   is handled for you).
 * - `handle` is the ADB device handler ID (DHI) value. After reset you should
 *   use your driver's default. If you don't support a given handle you should
 *   ignore requests to change it.
 *
 * Notes for each function:
 *
 * - `reset_func` is called in response to a reset pulse. Reset state (and DHI)
 *   to defaults as appropriate.
 * - [n] `switch_func` is called to indicate the active computer the user has
 *   picked is being switched to the one provided.
 * - [n] `talk_func` is called after a Talk command has completed, letting the
 *   driver know that the last-submitted Talk data was sent to the computer.
 *   The driver may set new data in response to this call.
 * - [n] `listen_func` is called in response to a Listen command, offering up
 *   to 8 bytes of data to be stored in the driver.
 * - [n] `flush_func` is called in response to a Flush command.
 * - (!) `get_handle_func` should return the currently assigned DHI. This can
 *   be either the default _or_ a DHI from set_handle_func that was actually
 *   accepted. Not setting the value will result in a DHI of 0x00, which
 *   the remote system will likely interpret as a failed self-test or error.
 * - (!) [n] `set_handle_func` offers a new DHI to assign. If the handle is
 *   supported it should be assigned internally and then returned each time
 *   from get_handle_func, otherwise leave the old handle alone.
 *
 * The functions marked with (!) above are called from an interrupt. Ensure
 * these return _very quickly_, just update a relevant variable and provide
 * it back, leaving processing to a later non-interrupt call. All other
 * functions will be called from the computer message dispatching task. The
 * functions marked with [n] may be NULL.
 *
 * There is only one "active" computer at any time, the others are inactive. It
 * is up to the driver to decide how to handle active vs inactive computer. For
 * example, a mouse driver will probably want to only send data to an active
 * computer. However, a virtual modem might want to send data to all computers
 * and maintain state separately for each one, effectively acting like multiple
 * independent devices.
 *
 * Sending data is accomplished by calling `computer_offer`. Once this data is
 * successfully sent over the wire `talk_func` is called back. See
 * `computer_offer` for more details.
 *
 * ADB initializes at startup and the device table is not updated again (unless
 * something exceptional happens, like ADBReinit). To accomodate this the
 * manager keeps track of which devices are registered as of the time it
 * receives a computer reset. If you are not registered then you will not be
 * called while that computer is active. Restarting the affected computer
 * will cause a new reset event, after which your calls will be invoked
 * normally. This isn't great, but hey, it's ADB, what can you do.
 *
 * Host initialization and native device assignment happens before this is
 * used, feel free to register in response to an assignment call from there.
 *
 * It is possible for users to switch drivers to active computers that are
 * offline or are in the process of booting and have not yet reset the bus.
 * Drivers need to handle this condition gracefully where possible.
 */

typedef struct {
	uint8_t length;
	uint8_t data[8];
} talk_data_entry;

typedef struct {
	const char *name;     // name of driver for debugging purposes
	uint8_t default_addr; // default bus address to be used at reset
	void (*reset_func)(uint8_t, uint32_t);
	void (*switch_func)(uint8_t);
	void (*talk_func)(uint8_t, uint32_t, uint8_t);
	void (*listen_func)(uint8_t, uint32_t, uint8_t, volatile uint8_t*, uint8_t);
	void (*flush_func)(uint8_t, uint32_t);
	void (*get_handle_func)(uint8_t, uint32_t, uint8_t*);
	void (*set_handle_func)(uint8_t, uint32_t, uint8_t);
} dev_driver;

/*
 * The maximum number of simultaneous devices allowed to be installed at any
 * given time. This definitely can't be higher than 14, but given ADB's
 * addressing scheme it is likely a lower number is the true maximum.
 *
 * This is distinct from the maximum number of total _drivers_ allowed. Some
 * drivers may be creating multiple devices, so the total number of drivers is
 * probably lower than the below number.
 */
#define DEVICE_MAX  8

/**
 * @return  the current number of registered devices.
 */
uint8_t driver_count_devices(void);

/**
 * Register a new device to the given driver. This saves the given device
 * driver pointer internally, along with a value to use when calling back the
 * functions given.
 *
 * @param *index   driver registration index.
 * @param *driver  pointer to the new driver to add.
 * @param ref      a reference constant that will be provided during each
 *                 callback, can be any value useful to the driver (array
 *                 index, pointer, etc).
 * @return         true if it was added, false otherwise.
 */
bool driver_register(uint8_t *index, dev_driver *driver, uint32_t ref);

/*
 * ----------------------------------------------------------------------------
 * The remaining functions in this unit are used internally, do not invoke
 * these directly from a driver.
 * ----------------------------------------------------------------------------
 */

bool driver_get(uint8_t index, dev_driver **driver, uint32_t *ref);
void driver_init(void); // see .c file for details

#endif /* __DRIVER_H__ */
