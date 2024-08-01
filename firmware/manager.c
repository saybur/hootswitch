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

#include "bus.h"
#include "device.h"
#include "host.h"
#include "hardware.h"
#include "manager.h"

/*
 * Manager for the ADB subsystem. This is conceptually similar to handler.c but
 * is used in the opposite direction: it actively manages the connected native
 * devices, pulling their information and storing it for use by the connected
 * machines.
 *
 * Because of the periodic polling required, this is also a convenient location
 * for the core1 main(). That will not be enabled until the program is further
 * along but the code is still here as though it will be later.
 */

void manager_main(void)
{
	// perform initialization
	bus_init();
	host_init();
	device_init();

	while (1) tight_loop_contents();
}
