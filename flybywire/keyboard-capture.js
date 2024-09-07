/*
 * This file was originally from the IP-KVM-InterFace project at
 * https://github.com/SterlingButters/ip-kvm-interface
 * and was modified for use with Hootswitch. It remains available under the
 * original license below.
 *
 * Copyright 2018-2020 Sterling Butters and others
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

let Keyboard = window.SimpleKeyboard.default;

let commonKeyboardOptions = {
  onKeyPress: button => onKeyPress(button),
  onKeyReleased: button => onKeyReleased(button),
  theme: "simple-keyboard hg-theme-default hg-layout-default hootswitch",
  physicalKeyboardHighlight: true,
  syncInstanceInputs: true,
  mergeDisplay: true,
  debug: true
};

let keyboard = new Keyboard(".simple-keyboard-main", {
  ...commonKeyboardOptions,
  /**
   * Layout by:
   * Sterling Butters (https://github.com/SterlingButters)
   */
  layout: {
    default: [
      "{escape} {f1} {f2} {f3} {f4} {f5} {f6} {f7} {f8} {f9} {f10} {f11} {f12}",
      "` 1 2 3 4 5 6 7 8 9 0 - = {backspace}",
      "{tab} q w e r t y u i o p [ ] \\",
      "{capslock} a s d f g h j k l ; ' {enter}",
      "{shiftleft} z x c v b n m , . / {shiftright}",
      "{controlleft} {altleft} {metaleft} {space} {metaright} {altright} {controlright}"
    ],
    shift: [
      "{escape} {f1} {f2} {f3} {f4} {f5} {f6} {f7} {f8} {f9} {f10} {f11} {f12}",
      "~ ! @ # $ % ^ & * ( ) _ + {backspace}",
      "{tab} Q W E R T Y U I O P { } |",
      '{capslock} A S D F G H J K L : " {enter}',
      "{shiftleft} Z X C V B N M < > ? {shiftright}",
      "{controlleft} {altleft} {metaleft} {space} {metaright} {altright} {controlright}"
    ]
  },
  display: {
    "{escape}": "esc ⎋",
    "{tab}": "tab ⇥",
    "{backspace}": "delete ⌫",
    "{enter}": "return ↵",
    "{capslock}": "caps ⇪",
    "{shiftleft}": "shift ⇧",
    "{shiftright}": "shift ⇧",
    "{controlleft}": "ctrl ⌃",
    "{controlright}": "ctrl ⌃",
    "{altleft}": "opt ⌥",
    "{altright}": "opt ⌥",
    "{metaleft}": "cmd ⌘",
    "{metaright}": "cmd ⌘"
  }
});

let keyboardControlPad = new Keyboard(".simple-keyboard-control", {
  ...commonKeyboardOptions,
  layout: {
    default: [
      "{prtscr} {scrolllock} {pause}",
      "{insert} {home} {pageup}",
      "{delete} {end} {pagedown}"
    ]
  },
  display: {
    "{insert}": "help",
    "{delete}": "del"
  },
});

let keyboardArrows = new Keyboard(".simple-keyboard-arrows", {
  ...commonKeyboardOptions,
  layout: {
    default: ["{arrowup}", "{arrowleft} {arrowdown} {arrowright}"]
  }
});

let keyboardNumpad = new Keyboard(".simple-keyboard-numpad", {
  ...commonKeyboardOptions,
  layout: {
    default: ["{reset}",
      "{clear} {npeq} {npsl} {npmul}",
      "{np7} {np8} {np9} {npsub}",
      "{np4} {np5} {np6} {npadd}",
      "{np1} {np2} {np3}",
      "{np0} {npdot} {npenter}"
    ]
  },
  display: {
    "{reset}": "◁",
    "{clear}": "clear",
    "{npeq}": "=",
    "{npsl}": "/",
    "{npmul}": "*",
    "{npsub}": "-",
    "{npadd}": "+",
    "{npdot}": ".",
    "{np0}": "0",
    "{np1}": "1",
    "{np2}": "2",
    "{np3}": "3",
    "{np4}": "4",
    "{np5}": "5",
    "{np6}": "6",
    "{np7}": "7",
    "{np8}": "8",
    "{np9}": "9",
    "{npenter}": "↵"
  },
});

// Function to obtain JS code value from virtual keyboard button press; see
// https://developer.mozilla.org/en-US/docs/Web/API/UI_Events/Keyboard_event_code_values
function getKeyCode(layoutKey) {
  const layoutKeyProcessed = layoutKey.replace("{", "").replace("}", "");

  const codeList = {
	"backspace": "Backspace",
    "tab": "Tab",
    "enter": "Enter",
    "shiftleft": "ShiftLeft",
    "shiftright": "ShiftRight",
    "controlleft": "ControlLeft",
    "controlright": "ControlRight",
    "altleft": "AltLeft",
    "altright": "AltRight",
    "pause": "Pause",
    "capslock": "CapsLock",
    "escape": "Escape",
    "space": "Space",
    "pageup": "PageUp",
    "pagedown": "PageDown",
    "end": "End",
    "home": "Home",
    "arrowleft": "ArrowLeft",
    "arrowup": "ArrowUp",
    "arrowright": "ArrowRight",
    "arrowdown": "ArrowDown",
    "insert": "Insert",
    "delete": "Delete",
    "0": "Digit0",
    "1": "Digit1",
    "2": "Digit2",
    "3": "Digit3",
    "4": "Digit4",
    "5": "Digit5",
    "6": "Digit6",
    "7": "Digit7",
    "8": "Digit8",
    "9": "Digit9",
    "a": "KeyA",
    "b": "KeyB",
    "c": "KeyC",
    "d": "KeyD",
    "e": "KeyE",
    "f": "KeyF",
    "g": "KeyG",
    "h": "KeyH",
    "i": "KeyI",
    "j": "KeyJ",
    "k": "KeyK",
    "l": "KeyL",
    "m": "KeyM",
    "n": "KeyN",
    "o": "KeyO",
    "p": "KeyP",
    "q": "KeyQ",
    "r": "KeyR",
    "s": "KeyS",
    "t": "KeyT",
    "u": "KeyU",
    "v": "KeyV",
    "w": "KeyW",
    "x": "KeyX",
    "y": "KeyY",
    "z": "KeyZ",
    "metaleft": "MetaLeft",
    "metaright": "MetaRight",
    "// select": "",
    "f1": "F1",
    "f2": "F2",
    "f3": "F3",
    "f4": "F4",
    "f5": "F5",
    "f6": "F6",
    "f7": "F7",
    "f8": "F8",
    "f9": "F9",
    "f10": "F10",
    "f11": "F11",
    "f12": "F12",
    "numlock": "NumLock",
    "scrolllock": "ScrollLock",
    "prtscr": "PrintScreen",
    ";": "Semicolon",
    "-": "Minus",
    "=": "Equal",
    ",": "Comma",
    ".": "Period",
    "/": "Slash",
    "\\": "Backslash",
    "[": "BracketLeft",
    "]": "BracketRight",
    "'": "Quote",
    "reset": "Power",
    "npeq": "NumpadEqual",
    "npsl": "NumpadDivide",
    "npmul": "NumpadMultiply",
    "npsub": "NumpadSubtract",
    "npadd": "NumpadAdd",
    "npdot": "NumpadDecimal",
    "np0": "Numpad0",
    "np1": "Numpad1",
    "np2": "Numpad2",
    "np3": "Numpad3",
    "np4": "Numpad4",
    "np5": "Numpad5",
    "np6": "Numpad6",
    "np7": "Numpad7",
    "np8": "Numpad8",
    "np9": "Numpad9",
    "npenter": "NumpadEnter"
  };

  const code = codeList[layoutKeyProcessed];
  return code;
}

/**
 * Virtual Keyboard support
 * Using SimpleKeyboard
 */

function onKeyPress(button) {
  button = button.replace('{','').replace('}','');
  const code = getKeyCode(button);
  console.log("Button pressed", button, "was", code);

  if (code) {
    doKeyDown(code);
  }

  if (
    button === "shift" ||
    button === "shiftleft" ||
    button === "shiftright" ||
    button === "capslock"
  ) {
    toggleShiftMode();
  }
}


function onKeyReleased(button) {
  button = button.replace('{','').replace('}','');
  const code = getKeyCode(button);
  console.log("Button released", button, "was", code);

  if (code) {
    doKeyUp(code);
  }

  if (
    button === "shift" ||
    button === "shiftleft" ||
    button === "shiftright"
  ) {
    toggleShiftMode();
  }
}

/**
 * Physical Keyboard support
 * Using document listeners
 */

document.addEventListener("keydown", event => {
  console.log(event.key);

  doKeyDown(event.code);

  // Disabling keyboard input, as some keys (like F5) make the browser lose focus.
  if (event.key === "Alt") event.preventDefault();
  if (event.key === "ArrowUp") event.preventDefault();
  if (event.key === "ArrowDown") event.preventDefault();
  if (event.key === "ArrowLeft") event.preventDefault();
  if (event.key === "ArrowRight") event.preventDefault();
  if (event.key === " ") event.preventDefault();

  if (event.key === "Shift") enableShiftMode(event);
  if (event.key === "CapsLock") {
    toggleShiftMode(event);
    highlightButton(event);
  }
});

var capsTracker = 0;
var isEven = function(x) { return !( x & 1) };

document.addEventListener("keyup", event => {

  doKeyUp(event.code);

  if (event.key === "Shift") disableShiftMode(event);
  if (event.key === "CapsLock") {
    if (isEven(capsTracker)) {
      highlightButton(event);
    } else {unhighlightButton(event)}
    capsTracker += 1;
  }
});

function toggleShiftMode(event) {
  let currentLayout = keyboard.options.layoutName;

  // If currentLayout is default, set to shift, and vice versa
  let shiftToggle = currentLayout === "default" ? "shift" : "default";

  keyboard.setOptions({
    layoutName: shiftToggle
  });
}

function enableShiftMode(event) {
  keyboard.setOptions({
    layoutName: "shift"
  });
  highlightButton(event);
}

function disableShiftMode(event) {
  keyboard.setOptions({
    layoutName: "default"
  });
  unhighlightButton(event);
}

function highlightButton(event) {
  let layoutKeyName = keyboard.physicalKeyboard.getSimpleKeyboardLayoutKey(event);

  let buttonElement =
    keyboard.getButtonElement(layoutKeyName) ||
    keyboard.getButtonElement(`{${layoutKeyName}}`);

  // Highlighting that key manually...
  buttonElement.style.background = "#9ab4d0";
  buttonElement.style.color = "white";
  console.log(buttonElement);
}

function unhighlightButton(event) {
  let layoutKeyName = keyboard.physicalKeyboard.getSimpleKeyboardLayoutKey(event);

  let buttonElement =
    keyboard.getButtonElement(layoutKeyName) ||
    keyboard.getButtonElement(`{${layoutKeyName}}`);

  // Unhighlighting that key manually...
  buttonElement.removeAttribute("style");
  console.log(buttonElement);
}
