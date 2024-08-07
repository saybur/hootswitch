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

#ifndef __HOST_ERR_H__
#define __HOST_ERR_H__

typedef enum {
	HOSTERR_OK = 0,
	HOSTERR_TIMEOUT = 1,
	HOSTERR_FULL = 2,
	HOSTERR_INVALID_PARAM = 3,
	HOSTERR_LINE_STUCK = 4,
	HOSTERR_TOO_MANY_DEVICES = 5,
	HOSTERR_BAD_DEVICE = 6,
	HOSTERR_NO_DEVICES = 7,
	HOSTERR_BAD_STATE = 8
} host_err;

#endif /* __HOST_ERR_H__ */
