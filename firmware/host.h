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
#include "host_err.h"

typedef enum {
	COMMAND_FLUSH =    0x1,
	COMMAND_LISTEN_0 = 0x8,
	COMMAND_LISTEN_1 = 0x9,
	COMMAND_LISTEN_2 = 0xA,
	COMMAND_LISTEN_3 = 0xB,
	COMMAND_TALK_0 =   0xC,
	COMMAND_TALK_1 =   0xD,
	COMMAND_TALK_2 =   0xE,
	COMMAND_TALK_3 =   0xF
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
 * @param cmd   ADB command to execute (suggest using bus_command above).
 * @param id    will be assigned an identifier, which will match the callback;
 *              some handlers may not care and can freely ignore this.
 * @param data  up to 8 bytes of data if a Listen command, ignored otherwise.
 * @param len   length of data if Listen, ignored otherwise.
 * @return      true if the request could be queued, false otherwise; this is
 *              due to the queue being full or an invalid device.
 */
host_err host_cmd(uint8_t dev, uint8_t cmd, uint16_t *id,
		uint8_t *data, uint8_t len);

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

/**
 * Called as part of periodic processing to send data responses to the
 * handlers. Users should not call this function. It _must_ only be called from
 * process context.
 */
void host_poll(void);

#endif /* __HOST_H__ */
