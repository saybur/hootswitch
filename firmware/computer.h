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

/**
 * Places new data into the given Talk register if no existing data is present.
 *
 * This will wait for a short time if the register in question is in use due to
 * an ongoing Talk operation. If it times out it will return false. It will
 * also return false if there is existing data in the register that has not
 * yet been sent.
 *
 * @param computer  the computer index to assign for.
 * @param drv_idx   the index that your driver/device assignment was originally
 *                  given during registration.
 * @param reg       the Talk register to assign, from 0-2.
 * @param data      the data to assign.
 * @param data_len  the length of the data to assign, from 2-8.
 * @return          true if data was assigned, false otherwise (error, register
 *                  was busy, or register already had data).
 */
bool computer_data_offer(uint8_t comp, uint8_t drv_idx, uint8_t reg,
		uint8_t *data, uint8_t data_len);

/**
 * Places new data into the given Talk register, overwriting any existing data
 * present.
 *
 * This version works similarly to the above one but will wipe out old data if
 * it can get to the register. It will still return false if the register is
 * locked due to an ongoing operation that exceeds the timeout.
 *
 * This version also has a `keep` flag allowing for data to be kept between
 * Talk operations. This is useful for "status" registers that always return
 * the same data each time.
 *
 * @param computer  the computer index to assign for.
 * @param drv_idx   the index that your driver/device assignment was originally
 *                  given during registration.
 * @param reg       the Talk register to assign, from 0-2.
 * @param data      the data to assign.
 * @param data_len  the length of the data to assign, from 0-8; anything less
 *                  than 2 will result in existing data being destroyed.
 * @param keep      if true, preserve data in the register between Talks.
 * @return          true if data was assigned, false otherwise (error or
 *                  register was busy).
 */
bool computer_data_set(uint8_t comp, uint8_t drv_idx, uint8_t reg,
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
 * index selected. 255 is interpreted as "do not switch."
 *
 * This is asynchronous and will be picked up during the next computer task
 * polling loop.
 *
 * @param target  the target computer number, 0-COMPUTER_COUNT-1, or higher
 *                than that to switch to the next system, with 255 being a
 *                "do not switch" command.
 */
void computer_switch(uint8_t target);

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
