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

#include "pico/stdlib.h"

#include "../debug.h"
#include "../handler.h"

#include "keyboard.h"

static volatile uint8_t buffer[2];
static volatile uint8_t buffer_count;

static void hndl_reset(void)
{
	// TODO implement
}

static bool hndl_interview(volatile ndev_info *info, uint8_t *err)
{
	return info->dhid_cur == 0x01 || info->dhid_cur == 0x02;
}

static void hndl_assign(volatile ndev_info *info, uint8_t id)
{
	// TODO implement
}

static void hndl_talk(uint8_t dev, uint8_t err, uint16_t cid, uint8_t reg,
		uint8_t *data, uint8_t data_len)
{
	if (data_len > 2) {
		buffer[0] = data[0];
		buffer[1] = data[1];
		buffer_count = 2;
	}
}

static void hndl_listen(uint8_t dev, uint8_t err, uint16_t cid, uint8_t reg)
{
	// TODO implement
}

static void hndl_flush(uint8_t dev, uint8_t err, uint16_t cid)
{
	// TODO implement
}

static void hndl_poll(void)
{
	if (buffer_count == 2) {
		dbg("kbd: %d / %d", buffer[0], buffer[1]);
		buffer_count = 0;
	}
}

static ndev_handler keyboard_handler = {
	.name = "kbd",
	.accept_noop_talks = false,
	.reset_func = hndl_reset,
	.interview_func = hndl_interview,
	.assign_func = hndl_assign,
	.talk_func = hndl_talk,
	.listen_func = hndl_listen,
	.flush_func = hndl_flush,
	.poll_func = hndl_poll
};

void keyboard_init(void)
{
	handler_register(&keyboard_handler);
}
