/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

const baud = 115200;
const vid = 0x1209; // pid.codes
const pid = 0x6804; // hootswitch PID from pid.codes

const polynomial = 0x04C11DB7;
const configLength = 4096;
const writeCommand = 0xF3;
const encodeBase = 0x30;

/*
 * ----------------------------------------------------------------------------
 *   Interface Parsing
 * ----------------------------------------------------------------------------
 */

function parseSettings(arr)
{
	// global settings
	if (document.getElementById('cfg-buzzer').checked) {
		arr[4] &= ~0x01;
	}

	// kensington emulation options
	for (let i = 0; i <= 3; i++) {
		let v = document.getElementById('cfg-kens-port' + (i + 1)).value;
		if (v === "tm5") {
			arr[32 + i] = 0x05;
		} else if (v === "tm4") {
			arr[32 + i] = 0x04;
		}
	}
}

/*
 * ----------------------------------------------------------------------------
 *   Flash Commit Logic
 * ----------------------------------------------------------------------------
 */

function insertFrameCheck(arr)
{
	const revPolynomial = crc32_reverse(polynomial);
	const table = crc32_generate(revPolynomial);

	let crc = crc32_initial();
	let i;
	for (i = 0; i < configLength - 4; i++) {
		crc = crc32_add_byte(table, crc, arr[i]);
	}

	arr[i++] = (crc >>> 0) & 0xFF;
	arr[i++] = (crc >>> 8) & 0xFF;
	arr[i++] = (crc >>> 16) & 0xFF;
	arr[i++] = (crc >>> 24) & 0xFF;

	//console.log("crc: " + (crc >>> 0).toString(16));
}

function flashConfig()
{
	if (! port) return;
	if (! window.confirm("This will write a new device configuration with the selected settings."
			+ " Do you want to continue?")) {
		return;
	}

	console.log("config flash requested");

	// make the storage array; 0xFF is default over the valid portion of the
	// array to match the default (unprogrammed) flash value of the device
	const arr = new Uint8Array(configLength * 2);
	for (let i = 0; i < configLength; i++) {
		arr[i] = 0xFF;
	}

	// load settings and then 'sign' them with the frame check
	parseSettings(arr);
	insertFrameCheck(arr);

	// rewrite for wire transmission
	for (let i = configLength - 1; i >= 0; i--) {
		let v = arr[i];
		arr[i * 2] = encodeBase + ((v >>> 4) & 0xF);
		arr[i * 2 + 1] = encodeBase + ((v >>> 0) & 0xF);
	}

	// transmit to the device
	const cmd = new Uint8Array([writeCommand]);
	writer.write(cmd);
	writer.write(arr);
}

/*
 * ----------------------------------------------------------------------------
 *   "About" Text
 * ----------------------------------------------------------------------------
 */

const aboutButton = document.getElementById("about-button");
const aboutArea = document.getElementById("about-area");

aboutButton.addEventListener("click", () => {
	if (aboutArea.classList.contains("hidden")) {
		aboutArea.classList.remove("hidden");
	} else {
		aboutArea.classList.add("hidden");
	}
});
