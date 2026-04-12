/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * Base addresses within the flash sector where various components have their
 * setting stored.
 */
#define CONFIG_OFFSET_BASE              4
#define CONFIG_OFFSET_KENS              32

typedef enum {
	CONFIG_OK = 0,
	CONFIG_INVALID
} config_err;

/**
 * Loads the given array with configuration data from flash. If the setting
 * does not exist a nonzero error is returned.
 *
 * @param tag       the key to be read.
 * @param data      location to read data into.
 * @param data_len  the number of bytes to read from flash.
 * @return          non-zero if data could not be loaded.
 */
config_err config_read(uint16_t tag, uint8_t* data, uint8_t data_len);

/**
 * Sets up for subsequent calls to the configuration functions. Should only be
 * invoked once during startup.
 *
 * @return          non-zero if the configuration setup failed.
 */
config_err config_setup(void);

/**
 * Saves the given array with configuration data into flash.
 *
 * @param offset    the key to write.
 * @param data      location to write data from.
 * @param data_len  the number of bytes to read from data.
 * @return          non-zero if data could not be written.
 */
config_err config_write(uint16_t tag, uint8_t* data, uint8_t data_len);

#endif /* __CONFIG_H__ */
