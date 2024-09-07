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

void serial_init(void);

#endif /* __SERIAL_H__ */
