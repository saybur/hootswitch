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

#ifndef __BUS_H__
#define __BUS_H__

#define BUS_ATN_MIN 750
#define BUS_ATN_MAX 850

typedef enum {
	PHASE_IDLE,
	PHASE_ATTENTION,
	PHASE_COMMAND,
	PHASE_SRQ,
	PHASE_TLT,
	PHASE_DATA_OUT,
	PHASE_DATA_IN
} BusPhase;

typedef enum {
	STATUS_FAULT,             // lost track of bus phase
	STATUS_RESET,             // machine issued reset
	STATUS_COMMANDED,         // pending command
	STATUS_DATA_RECV,         // pending data to parse
	STATUS_NORMAL
} BusStatus;

void bus_init(void);

#endif /* __BUS_H__ */
