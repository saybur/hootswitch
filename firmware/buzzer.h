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

#ifndef __BUZZER_H__
#define __BUZZER_H__

/**
 * Shorthand for calling buzzer_play(), using a system default alert noise.
 */
void buzzer_chirp(void);

/**
 * Simple square wave PWM audio on the buzzer.
 *
 * Don't expect much out of this, the code was thrown together quickly. It does
 * OK for making chirps, but improvements here are welcome.
 *
 * @param freq         approximate playback frequency.
 * @param duration_ms  approximate time to play, in milliseconds.
 * @param vol          duty cycle from 0 (0%) to 255 (50%).
 */
void buzzer_play(uint16_t freq, uint16_t duration_ms, uint8_t vol);

/**
 * Sets up the buzzer. Called during init, do not invoke as a user.
 */
void buzzer_init(void);

#endif /* __BUZZER_H__ */
