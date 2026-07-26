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

let port = undefined;
let reader = undefined;
let writer = undefined;
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
			log += decoder.decode(value);
			textLog.innerHTML = log;
			textLog.scrollTop = textLog.scrollHeight;
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
	if (window.confirm("Do you want to perform a restart to debug? (See the wiki for details)")) {
		console.log("debug restart requested");
		const a = new Uint8Array([0xF2]);
		writer.write(a);
	}
}

connectButton.addEventListener("click", async () => {
	if (! port) {
		connect();
	} else {
		disconnect();
	}
});
