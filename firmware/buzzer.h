/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __BUZZER_H__
#define __BUZZER_H__

/**
 * Shorthand for calling buzzer_play(), using a system default alert noise.
 */
void buzzer_chirp(void);

/**
 * Enables or disables the buzzer component.
 *
 * @param enabled      if false mute the buzzer.
 */
void buzzer_enable(bool enabled);

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
