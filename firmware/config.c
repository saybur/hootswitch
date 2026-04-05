/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <stdio.h>
#include <string.h>
#include "pico/flash.h"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/structs/xip_ctrl.h"

#include "config.h"
#include "debug.h"
#include "hardware.h"

#define CONFIG_SIZE  4096

static volatile bool config_started = false;
static volatile bool config_valid = false;
static uint8_t config_data[CONFIG_SIZE];
static uint16_t config_data_write_pos = 0;

config_err config_read(uint16_t offset, uint8_t* data, uint8_t data_len)
{
	if (! config_valid) return CONFIG_INVALID;
	memcpy(data, config_data + offset, data_len);
	return CONFIG_OK;
}

config_err config_setup(void)
{
	// in general only allow this to be invoked once
	// check is lazy, unlikely to get hit with how firmware starts up
	if (config_started) {
		if (config_valid) {
			return CONFIG_OK;
		} else {
			return CONFIG_INVALID;
		}
	}
	config_started = true;

	// grab channel for our exclusive use, may be used in later steps
	dma_channel_claim(CONFIG_DMA_CHAN);

	// see datasheet 2.6.3.4 for information about what's going on below
	// setup XIP stream peripheral to get block of data from flash
	while (! (xip_ctrl_hw->stat & XIP_STAT_FIFO_EMPTY))
		(void) xip_ctrl_hw->stream_fifo;
	xip_ctrl_hw->stream_addr = (uint32_t) CONFIG_SECTOR;
	xip_ctrl_hw->stream_ctr = CONFIG_SIZE / 4;

	// setup the DMA channel to perform the reading from XIP into RAM
	// perform CRC check against the data at the same time
	dma_sniffer_enable(CONFIG_DMA_CHAN, 1, false);
	dma_sniffer_set_data_accumulator(0xFFFFFFFF);
	dma_channel_config dc = dma_channel_get_default_config(CONFIG_DMA_CHAN);
	channel_config_set_dreq(&dc, DREQ_XIP_STREAM);
	channel_config_set_read_increment(&dc, false);
	channel_config_set_write_increment(&dc, true);
	channel_config_set_sniff_enable(&dc, true);
	dma_channel_configure(CONFIG_DMA_CHAN, &dc,
			(void *) config_data,
			(const void *) XIP_AUX_BASE,
			CONFIG_SIZE / 4,
			true);

	// wait for the read process to complete
	dma_channel_wait_for_finish_blocking(CONFIG_DMA_CHAN);

	// check if accumulator matches data + check value (aka data valid)
	if (dma_sniffer_get_data_accumulator()) {
		dbg("flash config invalid, using defaults");
		return CONFIG_INVALID;
	} else {
		// remainder zero, data + check value good!
		dbg("flash config valid");
		config_valid = true;
		return CONFIG_OK;
	}
}

/*
 * ----------------------------------------------------------------------------
 *   Config Updating / Flashing Routines
 * ----------------------------------------------------------------------------
 */

static config_err config_write_check(uint8_t* data)
{
	// run the data through DMA with the sniffer on to check validity
	uint32_t tmp;
	dma_sniffer_enable(CONFIG_DMA_CHAN, 1, false);
	dma_sniffer_set_data_accumulator(0xFFFFFFFF);
	dma_channel_config dc = dma_channel_get_default_config(CONFIG_DMA_CHAN);
	channel_config_set_read_increment(&dc, true);
	channel_config_set_write_increment(&dc, false);
	channel_config_set_sniff_enable(&dc, true);
	dma_channel_configure(CONFIG_DMA_CHAN, &dc,
			(void *) &tmp,
			(const void *) data,
			CONFIG_SIZE / 4,
			true);

	// wait for the check process to complete
	dma_channel_wait_for_finish_blocking(CONFIG_DMA_CHAN);

	// check if accumulator matches data + check value (aka data valid)
	uint32_t sniffer = dma_sniffer_get_data_accumulator();
	if (sniffer) {
		dbg_err("bad crc: %8X", sniffer);
		return CONFIG_INVALID;
	} else {
		// remainder zero, data + check value good!
		return CONFIG_OK;
	}
}

config_err config_write_serial_byte(uint8_t c, bool* done)
{
	config_err err = CONFIG_OK;

	config_valid = false;
	if (config_data_write_pos & 1) {
		// write low part of byte
		config_data[config_data_write_pos / 2] |= c & 0xF;
	} else {
		// write high part of byte
		config_data[config_data_write_pos / 2] = (c & 0xF) << 4;
	}
	config_data_write_pos++;

	if (config_data_write_pos == CONFIG_SIZE * 2) {
		config_data_write_pos = 0;
		*done = true;

		// sufficient data present, check for consistency
		dbg("config update check...");
		if (err = config_write_check(config_data)) {
			dbg_err("config data check fail, err:%d", err);
			return err;
		}

		// perform erase/program step after disabling scheduler/interrupts
		// WARNING: this is insufficient if core1 becomes used in the future
		// revisit this approach in the future if that happens!
		dbg("config write starting...");
		vTaskSuspendAll();
		uint32_t isr = save_and_disable_interrupts();
		flash_range_erase(CONFIG_SECTOR, FLASH_SECTOR_SIZE);
		flash_range_program(CONFIG_SECTOR, (uint8_t*)config_data, CONFIG_SIZE);
		restore_interrupts_from_disabled(isr);
		xTaskResumeAll();
		dbg("config write done!");
	}

	return CONFIG_OK;
}
