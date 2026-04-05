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
 * By default, the configuration is placed at the end of the 2MB flash memory
 * space. This may be changed as needed, but ensure you are not clobbering
 * program code or Bad Things may happen.
 */
#define CONFIG_SECTOR                   0x1FF000

/*
 * Base addresses within the flash sector where various components have their
 * setting stored.
 */
#define CONFIG_OFFSET_BASE              4
#define CONFIG_OFFSET_KENS              32

typedef enum {
	CONFIG_OK = 0,
	CONFIG_INVALID,
	CONFIG_WRITE_IN_PROGRESS,
	CONFIG_WRITE_ERR
} config_err;

/**
 * Loads the given array with configuration data from flash. If the internal
 * flash has not been verified, or if it does not contain values, a nonzero
 * error is returned and no data is written to the given array.
 *
 * This can read back the entire configuration space if needed. The CRC used
 * to determine validity is in the last four bytes.
 *
 * @param offset    the offset from the start of config data to read.
 * @param data      location to store data.
 * @param data_len  the number of bytes to read.
 * @return          non-zero if data could not be loaded.
 */
config_err config_read(uint16_t offset, uint8_t* data, uint8_t data_len);

/**
 * Reads the entire configuration off flash to verify it is valid and updates
 * the internal state tracking information to support subsequent configuration
 * reads.
 *
 * @return          non-zero if the configuration data is invalid.
 */
config_err config_setup(void);

/**
 * Accepts user bytes in the range 0x20-0x2F and enqueues the new config data
 * to write. High nibble is written first. Calling this automatically
 * invalidates the configuration information.
 *
 * Once sufficient data has been written this will automatically check data,
 * and if it is valid, the RTOS will be suspended and a new flash page will
 * be written. This will *not* restore the config, a full restart is required
 * for that to occur. The done flag will be set.
 *
 * Any non-zero response is criticial. Serial data will be reported to the
 * user if errors occur.
 *
 * @param c         data value to enqueue, as above.
 * @param done      set to true on the final nibble
 * @return          device state following action.
 */
config_err config_write_serial_byte(uint8_t c, bool* done);

#endif /* __CONFIG_H__ */
