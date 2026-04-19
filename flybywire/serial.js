/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

/*
 * ----------------------------------------------------------------------------
 *   Serial Port Management and Logging
 * ----------------------------------------------------------------------------
 */

const maxLogLength = 65536;

let port = undefined;
let reader = undefined;
let writer = undefined;
let line = "";
let log = "";

const connectButton = document.getElementById("connect-button");
const controlArea = document.getElementById("control-area");
const disconnectMessage = document.getElementById("disconnect-message");
const textLog = document.querySelector("textarea");
const decoder = new TextDecoder();

async function connect()
{
	if (! ("serial" in navigator)) {
		alert("Web Serial API is not supported by this browser. Try using Chrome/Chromium instead.");
		return;
	}
	if (port) {
		console.log("Ignoring connect(), already connected!");
		return;
	}

	// request port and connect to it
	try {
		port = await navigator.serial.requestPort({
			filters: [{
				usbVendorId: vid,
				usbProductId: pid
				}]
		});
		await port.open({
			baudRate: baud
		});
		reader = port.readable.getReader();
		writer = port.writable.getWriter();
		connectButton.classList.add("hidden");
		log += "[[ Connected! ]]\n";
		textLog.innerHTML = log;
		controlArea.classList.remove("hidden");
	} catch (err) {
		console.log(err);
		if (err.name != "NotFoundError") {
			alert(err);
		}
		return;
	}

	// repeatedly print contents to log until closed
	try {
		while (port) {
			const { value, done } = await reader.read();
			if (done) {
				break;
			}
			// convert chunks to lines prior to sending to terminal
			let chunk = decoder.decode(value);
			for (let i = 0; i < chunk.length; i++) {
				let c = chunk.charAt(i);
				if (c == '\n') {
					readData(line);
					line = "";
				} else if (c == '\r') {
					// ignore
				} else {
					line += c;
				}
			}
		}
	} catch (err) {
		console.log(err);
		alert(err);
	} finally {
		if (reader) {
			reader.releaseLock();
			reader = undefined;
		}
		if (writer) {
			writer.releaseLock();
			writer = undefined;
		}

		port = undefined;
		log += "[[ Disconnected! ]]\n";
		textLog.innerHTML = log;
		controlArea.classList.add("hidden");
		disconnectMessage.classList.remove("hidden");
	}
}

/**
 * Handles both SLIP byte-stuffing needed to transmit a frame and the write()
 * call; use this instead of direct calls to the writer.
 */
function writeData(data)
{
	let stuffedLength = data.length + 1;
	for (let i = 0; i < data.length; i++) {
		if (data[i] == 0xC0) {
			stuffedLength++;
		} else if (data[i] == 0xDB) {
			stuffedLength++;
		}
	}
	let arr = new Uint8Array(stuffedLength);
	let apos = 0;
	for (let i = 0; i < data.length; i++) {
		if (data[i] == 0xC0) {
			arr[apos++] = 0xDB;
			arr[apos++] = 0xDC;
		} else if (data[i] == 0xDB) {
			arr[apos++] = 0xDB;
			arr[apos++] = 0xDD;
		} else {
			arr[apos++] = data[i];
		}
	}
	arr[apos++] = 0xC0;
	writer.write(arr);
}

function readData(data)
{
	if (data.startsWith("[    DATA]")) {
		let token = data.substring(11);
		console.log(`DATA: "${token}"`);
		// TODO implement
	} else {
		// append to log item; if line becomes excessively long trucate
		let newLength = log.length + data.length + 1;
		if (log.length + data.length > maxLogLength) {
			log = log.substring(newLength - maxLogLength) + data + '\n';
		} else {
			log += data + '\n';
		}
		textLog.innerHTML = log;
		textLog.scrollTop = textLog.scrollHeight;
	}
}

async function disconnect()
{
	if (! port) {
		console.log("already disconnected, ignoring disconnected()");
		return;
	}

	// unset, let connect() finish up loop
	port = undefined;
	if (reader) {
		reader.cancel();
	}
}

function restartDebug()
{
	if (! port) return;
	if (window.confirm("Restart device? It will disconnect and wait for you to reconnect before booting.")) {
		console.log("debug restart requested");
		const a = new Uint8Array([0xF2]);
		writeData(a);
	}
}

connectButton.addEventListener("click", async () => {
	if (! port) {
		connect();
	} else {
		disconnect();
	}
});
