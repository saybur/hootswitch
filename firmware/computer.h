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

#ifndef __COMPUTER_H__
#define __COMPUTER_H__

#include <stdbool.h>
#include <stdint.h>

#include "driver.h"

bool computer_data_offer(dev_driver *drv, uint8_t comp, uint8_t reg,
		uint8_t *data, uint8_t data_len);

void computer_init(void);

#endif /* __COMPUTER_H__ */
