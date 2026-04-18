/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdbool.h>
#include <pico/stdlib.h>

#include "config.h"
#include "debug.h"

#ifdef HOOTSWITCH_WIRELESS
#include <btstack_tlv.h>
#include <btstack_tlv_flash_bank.h>

#define TAG_PREFIX   0x48530000L

static volatile bool configured;
static const btstack_tlv_t *tlv;
static btstack_tlv_flash_bank_t *tlv_context;

/*
 * The following implements a wrapper around the default TLV instance. Right
 * now it does nothing. Longer-term purpose is to support SMP by providing a
 * place to signal core1 that a write to flash is imminent and it needs to
 * suspend work.
 *
 * Relevant files in the SDK:
 *
 * src/rp2_common/pico_btstack/btstack_flash_bank.c
 * src/rp2_common/pico_btstack/include/pico/btstack_flash_bank.h
 * src/rp2_common/pico_cyw43_driver/btstack_cyw43.c
 * src/rp2_common/pico_cyw43_arch/cyw43_arch_freertos.c (?)
 * lib/btstack/platform/embedded/btstack_tlv_flash_bank.c
 */
int get_tag_impl(void *context, uint32_t tag, uint8_t * buffer, uint32_t buffer_size)
{
	return tlv->get_tag(context, tag, buffer, buffer_size);
}
int store_tag_impl(void *context, uint32_t tag, const uint8_t *data, uint32_t data_size)
{
	return tlv->store_tag(context, tag, data, data_size);
}
void delete_tag_impl(void *context, uint32_t tag)
{
	return tlv->delete_tag(context, tag);
}
static const btstack_tlv_t config_tlv = {
	.get_tag = &get_tag_impl,
	.store_tag = &store_tag_impl,
	.delete_tag = &delete_tag_impl
};
#endif

config_err config_read(uint16_t tag, uint8_t* data, uint8_t data_len)
{
#ifdef HOOTSWITCH_WIRELESS
	if (!tlv || !tlv_context) return CONFIG_INVALID;

	if (config_tlv.get_tag(tlv_context, TAG_PREFIX + tag, data, data_len)) {
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

	if (config_tlv.store_tag(tlv_context, TAG_PREFIX + tag, data, data_len)) {
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
	if (configured) return CONFIG_OK;

#ifdef HOOTSWITCH_WIRELESS
	/*

	 */
	btstack_tlv_get_instance(&tlv, (void**)&tlv_context);
	if (!tlv || !tlv_context) {
		dbg_err("tlv setup failed!");
		return CONFIG_INVALID;
	}

	// wrap the created instance
	btstack_tlv_set_instance(&config_tlv, tlv_context);
	configured = true;
	return CONFIG_OK;
#else
	return CONFIG_INVALID;
#endif
}
