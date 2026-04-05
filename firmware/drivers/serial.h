/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#define SER_CMD_MSE_DOWN      0x80
#define SER_CMD_MSE_UP        0x81
#define SER_CMD_MSE_X         0x82
#define SER_CMD_MSE_Y         0x83
#define SER_CMD_MSE_APPLY     0x84
#define SER_CMD_SWITCH        0x85
#define SER_CMD_KBD_DOWN      0x86
#define SER_CMD_KBD_UP        0x87

void serial_enqueue(uint8_t);
void serial_init(void);

#endif /* __SERIAL_H__ */
