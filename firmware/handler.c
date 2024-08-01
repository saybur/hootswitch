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

#include "handler.h"

/*
 * Resolves commands from the connected machines. This acts as a central
 * dispatcher that takes generic bus commands, like "Talk Register 0" to a
 * device with handler 0x03, and translates those to instructions that are
 * more appropriate for an emulated device.
 */

uint16_t handler_talk_reg3(uint8_t mach)
{
	
}

void handler_listen_reg3(uint8_t mach, uint16_t reg3)
{
	
}
