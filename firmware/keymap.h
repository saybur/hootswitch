/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __KEYMAP_H
#define __KEYMAP_H

#include <stdint.h>

/**
 * Provides the ADB keycode for a given HID usage ID. Only the low 100 usage
 * IDs are included. This will return either the low 7 bits of the ADB keycode
 * based on the Apple Extended Keyboard or 0xFF if no equivalent keycode
 * exists in the mapping.
 *
 * See https://zmk.dev/docs/keymaps/list-of-keycodes for additional details
 * and some useful links.
 *
 * @param key  the key to decode.
 * @return     equivalent ADB keycode or 0xFF.
 */
uint8_t keymap_decode(uint8_t key);

#endif /* __KEYMAP_H */
