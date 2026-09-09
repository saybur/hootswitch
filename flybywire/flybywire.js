/*
 * Copyright (C) 2024-2026 saybur
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

const baud = 115200;
const movementDivisor = 4;
const computers = 4;
const vid = 0x1209; // pid.codes
const pid = 0x6804; // hootswitch PID from pid.codes

/*
 * Simplistic implementation of a mouse/keyboard capture system for sending
 * user input to a Hootswitch. This relies on the serial driver in the
 * firmware, see that for information on the specific command handling.
 */

/*
 * ----------------------------------------------------------------------------
 *   Mouse Handling
 * ----------------------------------------------------------------------------
 */

const canvas = document.querySelector("canvas");

canvas.addEventListener("click", async () => {
	if (! document.pointerLockElement) {
		try {
			await canvas.requestPointerLock({
				unadjustedMovement: true
			});
		} catch (err) {
			if (err.name == "NotSupportedError") {
				await canvas.requestPointerLock();
			} else {
				throw err;
			}
		}
	}
});

canvas.addEventListener("mousedown", async () => {
	mouseButtons = 0xFE;
	mouseSend(null);
});

canvas.addEventListener("mouseup", async () => {
	mouseButtons = 0xFF;
	mouseSend(null);
});

function lockChangeAlert()
{
	if (document.pointerLockElement === canvas) {
		console.log("pointer locked");
		document.addEventListener("mousemove", mouseSend, false);
	} else {
		console.log("pointer unlocked");
		document.removeEventListener("mousemove", mouseSend, false);
	}
}
document.addEventListener("pointerlockchange", lockChangeAlert, false);

let mouseButtons = 0xFF;
function mouseSend(e)
{
	let x = 0;
	let y = 0;
	if (e !== null) {
		x = e.movementX / movementDivisor;
		x = x >= 0 ? Math.ceil(x) : Math.floor(x);
		y = e.movementY / movementDivisor;
		y = y >= 0 ? Math.ceil(y) : Math.floor(y);
	}
	let a = new Uint8Array([0x03, mouseButtons,
			(x & 0xFF00) >> 8, x & 0xFF,
			(y & 0xFF00) >> 8, y & 0xFF]);
	writeData(a);
}

/*
 * ----------------------------------------------------------------------------
 *   Keyboard Handling
 * ----------------------------------------------------------------------------
 */

/*
 * Convert from KeyboardEvent.code to Mac transition codes. For a full list,
 * see Guide to the Macintosh Family Hardware 2nd Ed, p. 308.
 */
const transitionCodes = {
	"AltLeft": 0x3A,
	"AltRight": 0x3A,
	"ArrowDown": 0x3D,
	"ArrowLeft": 0x3B,
	"ArrowRight": 0x3C,
	"ArrowUp": 0x3E,
	"Backslash": 0x2A,
	"Backspace": 0x33,
	"BracketLeft": 0x21,
	"BracketRight": 0x1E,
	"CapsLock": 0x39,
	"Comma": 0x2B,
	"ControlLeft": 0x36,
	"ControlRight": 0x36,
	"Delete": 0x75,
	"Digit0": 0x1D,
	"Digit1": 0x12,
	"Digit2": 0x13,
	"Digit3": 0x14,
	"Digit4": 0x15,
	"Digit5": 0x17,
	"Digit6": 0x16,
	"Digit7": 0x1A,
	"Digit8": 0x1C,
	"Digit9": 0x19,
	"End": 0x77,
	"Enter": 0x24,
	"Equal": 0x18,
	"Escape": 0x35,
	"F1": 0x7A,
	"F2": 0x78,
	"F3": 0x63,
	"F4": 0x76,
	"F5": 0x60,
	"F6": 0x61,
	"F7": 0x62,
	"F8": 0x64,
	"F9": 0x65,
	"F10": 0x6D,
	"F11": 0x67,
	"F12": 0x6F,
	"Home": 0x73,
	"Insert": 0x72,
	"KeyA": 0x00,
	"KeyB": 0x0B,
	"KeyC": 0x08,
	"KeyD": 0x02,
	"KeyE": 0x0E,
	"KeyF": 0x03,
	"KeyG": 0x05,
	"KeyH": 0x04,
	"KeyI": 0x22,
	"KeyJ": 0x26,
	"KeyK": 0x28,
	"KeyL": 0x25,
	"KeyM": 0x2E,
	"KeyN": 0x2D,
	"KeyO": 0x1F,
	"KeyP": 0x23,
	"KeyQ": 0x0C,
	"KeyR": 0x0F,
	"KeyS": 0x01,
	"KeyT": 0x11,
	"KeyU": 0x20,
	"KeyV": 0x09,
	"KeyW": 0x0D,
	"KeyX": 0x07,
	"KeyY": 0x10,
	"KeyZ": 0x06,
	"MetaLeft": 0x37,
	"MetaRight": 0x37,
	"Minus": 0x1B,
	"NumLock": 0x47,
	"Numpad0": 0x52,
	"Numpad1": 0x53,
	"Numpad2": 0x54,
	"Numpad3": 0x55,
	"Numpad4": 0x56,
	"Numpad5": 0x57,
	"Numpad6": 0x58,
	"Numpad7": 0x59,
	"Numpad8": 0x5B,
	"Numpad9": 0x5C,
	"NumpadAdd": 0x45,
	"NumpadDecimal": 0x41,
	"NumpadDivide": 0x4B,
	"NumpadEnter": 0x4C,
	"NumpadEqual": 0x51,
	"NumpadMultiply": 0x43,
	"NumpadSubtract": 0x4E,
	"PageDown": 0x79,
	"PageUp": 0x74,
	"Pause": 0x71,
	"Period": 0x2F,
	"Power": 0x7F,
	"PrintScreen": 0x69,
	"Quote": 0x27,
	"ScrollLock": 0x6B,
	"Semicolon": 0x29,
	"ShiftLeft": 0x38,
	"ShiftRight": 0x38,
	"Slash": 0x2C,
	"Space": 0x31,
	"Tab": 0x30
};

/*
 * Handle swapping the Meta and Alt key for non-Mac keyboard layouts.
 */
const metaCheckbox = document.getElementById("swap-meta");
function doKeyCodeSwap(code) {
	if (metaCheckbox.checked) {
		if (code === "MetaLeft") {
			return "AltLeft";
		} else if (code === "MetaRight") {
			return "AltRight";
		} else if (code === "AltLeft") {
			return "MetaLeft";
		} else if (code === "AltRight") {
			return "MetaRight";
		}
	}
	return code;
}

/*
 * Actual keycode generation is in keyboard-capture.js
 */
function doKeyDown(code)
{
	if (! port) return;
	console.log("doKeyDown", code);
	code = doKeyCodeSwap(code);
	const tc = transitionCodes[code];
	if (tc != null) {
		const a = new Uint8Array([0x02, tc]);
		writeData(a);
	}
}

function doKeyUp(code)
{
	if (! port) return;
	console.log("doKeyUp", code);
	code = doKeyCodeSwap(code);
	const tc = transitionCodes[code] + 0x80;
	if (tc != null) {
		const a = new Uint8Array([0x02, tc]);
		writeData(a);
	}
}

/*
 * Suspend focus switching via Tab if the mouse is locked. This gets very
 * annoying if you're coding and your style involves tab indentation!
 */
document.addEventListener("keydown", event => {
	if (event.key === "Tab" && document.pointerLockElement === canvas) {
		event.preventDefault();
	}
});

/*
 * ----------------------------------------------------------------------------
 *   Macro Keys and Switcher
 * ----------------------------------------------------------------------------
 */

const macrosArea = document.getElementById("macros");
function switcherSetup()
{
	for (i = 1; i <= computers; i++) {
		const btn = document.createElement("button");
		btn.addEventListener("click", () => {
			if (! port) return;
			const idx = document.activeElement.innerText.slice(-1);
			console.log("switch", idx);
			const a = new Uint8Array([0x01, idx]);
			writeData(a);
		});
		const c = document.createTextNode("Switch Comp " + i);
		btn.appendChild(c);
		macrosArea.appendChild(btn);
	}
}
switcherSetup();

function bluetoothScan()
{
	if (! port) return;
	const a = new Uint8Array([0xE0]);
	writeData(a);
}

function traceLogging()
{
	if (! port) return;
	const a = new Uint8Array([0xE8]);
	writeData(a);
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
