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

#ifndef __LED_H__
#define __LED_H__

/**
 * Activates or deactivates the activity light. This is assumed to be yellow
 * and indicates normal activity.
 *
 * @param state  true for on, off for false.
 */
void led_activity(bool state);

/**
 * Activates or deactivates the error light. This is assumed to be red and
 * represents some exceptional condition.
 *
 * @param state  true for on, off for false.
 */
void led_error(bool state);

/**
 * Activates one of the machine indicator LEDs at a chosen intensity level.
 *
 * @param mach   machine indicator from 0-3.
 * @param level  brightness level, from 0 (off) to 255 (max intensity).
 */
void led_machine(uint8_t mach, uint8_t level);

/**
 * Sets up the LEDs. Called during init, do not invoke as a user.
 */
void led_init(void);

#endif /* __LED_H__ */
