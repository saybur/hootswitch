/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef __BTSCAN_H__
#define __BTSCAN_H__

#define BTSCAN_DURATION_SECONDS 30

/**
 * Starts the Bluetooth scanning process. This internally times out after
 * (approximately) BTSCAN_DURATION_SECONDS.
 */
void bt_scan(void);

/**
 * Performs initial setup of the scan system. This is only invoked once at
 * startup by bt_init(), do not call from user code.
 */
void bt_scan_init(void);

#endif /* __BTSCAN_H__ */
