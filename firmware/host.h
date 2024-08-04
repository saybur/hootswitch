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

#ifndef __HOST_H__
#define __HOST_H__

#include "handler.h"

#define COMMAND_QUEUE_SIZE   16

typedef enum {
	HOSTERR_OK = 0,
	HOSTERR_TIMEOUT = 1,
	HOSTERR_FULL = 2,
	HOSTERR_SYNC_DISABLED = 3,
	HOSTERR_ASYNC_DISABLED = 4,
	HOSTERR_INVALID_PARAM = 5,
	HOSTERR_LINE_STUCK = 6,
	HOSTERR_TOO_MANY_DEVICES = 7,
	HOSTERR_BAD_DEVICE = 8
} host_err;

typedef enum {
	COMMAND_FLUSH =    0x1,
	COMMAND_LISTEN_0 = 0x8,
	COMMAND_LISTEN_1 = 0x9,
	COMMAND_LISTEN_2 = 0xA,
	COMMAND_TALK_0 =   0xC,
	COMMAND_TALK_1 =   0xD,
	COMMAND_TALK_2 =   0xE
} bus_command;

/**
 * Executes an asynchronous command on the host bus.
 *
 * This places the command in a queue and returns. The registered handler
 * callback will be invoked once the host controller has a chance to get to
 * your request.
 *
 * This is not needed for some devices: the controller will auto-poll Register
 * 0 on any device that issues a SRQ, so if you are the handler for a device
 * you'll still get calls whenever the native device issues a SRQ and returns
 * data for Talk 0.
 *
 * @param dev   device number from the register call.
 * @param cmd   ADB command to execute.
 * @param len   number of bytes to exchange in the command (if needed).
 * @param id    will be assigned an identifier, which will match the callback.
 * @return      true if the request could be queued, false otherwise; this is
 *              due to the queue being full, or an invalid device.
 */
host_err host_cmd_async(uint8_t dev, bus_command cmd, uint8_t len, uint16_t *id);

/**
 * Executes a synchronous command on the host bus.
 *
 * This version is intended for initialization code and is only safe to call
 * from the handler reset_func, which is otherwise quite annoying to write with
 * callbacks. After host_reset() completes this will always return false.
 *
 * This is more flexible than the async version by allowing commands to
 * Register 3. This is not really meant for end-users but some devices might
 * require it for particular things. Importantly, _do not_ mess with the
 * address of a device without updating the associated `ndev_info` struct
 * (and use extreme caution while doing so).
 *
 * @param dev   device number, or 0xFF to bypass (see below).
 * @param cmd   ADB command to execute (low 4 bits if dev specified, all 8 bits
 *              if dev is 0xFF.
 * @param data  an array of 8 members (minimum!) to load or save data.
 * @param len   length to send if Listen, expected length to receive if Talk
 *              (updated with real length), ignored otherwise.
 * @return      true if a response was received, false if timed out _or_ if
 *              sync commands are not allowed right now.
 */
host_err host_cmd_sync(uint8_t dev, uint8_t cmd, uint8_t *data, uint8_t *len);

/**
 * Registers a handler with the host. Only handlers registered during reset
 * will have devices assigned to them (if any are on the bus).
 *
 * @param *handler  pointer to a handler, which will be copied locally.
 * @return          any errors.
 */
host_err host_register(ndev_handler *handler);

/**
 * Executes a bus reset on the host side.
 *
 * This will disable async commands and force a reset. Devices may take some
 * time to come out of reset and begin accepting commands again. This will
 * return after executing the (synchronous) reset command.
 *
 * This runs exclusively in polling mode and will hog the CPU. While I've tried
 * to make it safe to execute post-boot, this should generally only be called
 * during startup.
 *
 * @return      any errors.
 */
host_err host_reset(void);

/**
 * Performs the post-reset address resolution step, sets up the host device
 * table, and assigns the handlers to their various devices.
 *
 * Because of the randomness in ADB addresses there are no guarantees that a
 * particular handler will get a particular device if there is competition.
 *
 * This runs exclusively in polling mode and will hog the CPU. While I've tried
 * to make it safe to execute post-boot, this should generally only be called
 * during startup.
 *
 * @return      any errors.
 */
host_err host_readdress(void);

/**
 * Initializes hardware to use with the host. Users should not call this
 * function.
 */
void host_init(void);

#endif /* __HOST_H__ */
