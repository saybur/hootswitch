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

#include "FreeRTOS.h"
#include "queue.h"

#include "driver.h"

bool computer_data_offer(uint8_t comp, uint8_t drv_idx, uint8_t reg,
		uint8_t *data, uint8_t data_len, bool keep);

/**
 * Assigns a queue that will be queried for Talk 0 data automatically.
 *
 * This mechanism allows drivers to use a more asynchronous method to perform
 * data updates than the above method. The queue is drained in two spots:
 * first immediately following the talk callback, and again periodically.
 *
 * For performance, this mechanism is only available for Talk 0 (at least at
 * this point). The queue will be drained by every computer it is assigned to.
 * For devices that only want to send to an active computer, you will need to
 * de-assign the old computer before assigning to the new one.
 *
 * Importantly, do not delete the provided queue before unassigning it!
 *
 * @param computer  the computer index to assign for.
 * @param drv_idx   the index that your driver/device assignment was originally
 *                  given during registration.
 * @param queue     the queue to assign, or NULL to deassign an existing queue.
 */
void computer_queue_set(uint8_t comp, uint8_t drv_idx, QueueHandle_t queue);

/**
 * Switch to the computer matching the target.
 *
 * If the target number is greater than COMPUTER_COUNT this is interpreted as
 * "switch to next computer," otherwise it switches to the computer at the
 * index selected.
 *
 * @param target  the target computer number, 0-COMPUTER_COUNT-1, or higher
 *                than that to switch to the next system.
 * @return        true if switch went OK, false if no switch occurred.
 */
bool computer_switch(uint8_t target);

/**
 * Sets the power switch line to the given value.
 *
 * @param computer  the computer index to set the power line value
 * @param assert    if true, assert (turn on), if false, release.
 */
void computer_psw(uint8_t computer, bool assert);

void computer_init(void);
void computer_start(void);
void computer_task(void *parameters);

#endif /* __COMPUTER_H__ */
