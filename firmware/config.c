/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <pico/stdlib.h>

#include "config.h"
#include "debug.h"

#ifdef HOOTSWITCH_WIRELESS
#include <btstack_tlv.h>
#include <btstack_tlv_flash_bank.h>

static const btstack_tlv_t *tlv;
static btstack_tlv_flash_bank_t *tlv_context;

#define TAG_PREFIX   0x48530000L
#endif

config_err config_read(uint16_t tag, uint8_t* data, uint8_t data_len)
{
#ifdef HOOTSWITCH_WIRELESS
	if (!tlv || !tlv_context) return CONFIG_INVALID;

	if (tlv->get_tag(tlv_context, TAG_PREFIX + tag, data, data_len)) {
		return CONFIG_OK;
	} else {
		return CONFIG_INVALID;
	}
#else
	return CONFIG_INVALID;
#endif
}

config_err config_write(uint16_t tag, uint8_t* data, uint8_t data_len)
{
#ifdef HOOTSWITCH_WIRELESS
	if (!tlv || !tlv_context) return CONFIG_INVALID;

	// FIXME this needs an analysis of how core1 locking is performed, along
	// with all the other requirements for writing to flash
	if (tlv->store_tag(tlv_context, TAG_PREFIX + tag, data, data_len)) {
		return CONFIG_OK;
	} else {
		return CONFIG_INVALID;
	}
#else
	return CONFIG_INVALID;
#endif
}

config_err config_setup(void)
{
#ifdef HOOTSWITCH_WIRELESS
	btstack_tlv_get_instance(&tlv, (void**)&tlv_context);
	if (!tlv || !tlv_context) {
		dbg_err("tlv setup failed!");
		return CONFIG_INVALID;
	} else {
		return CONFIG_OK;
	}
#else
	return CONFIG_INVALID;
#endif
}
