/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#define SER_CMD_SWITCH        0x01
#define SER_CMD_KBD           0x02
#define SER_CMD_MSE           0x03

void serial_enqueue(uint8_t *data, uint8_t length);

#endif /* __SERIAL_H__ */
