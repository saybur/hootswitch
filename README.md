hootswitch
==========

Hootswitch is a prototype [ADB](https://en.wikipedia.org/wiki/Apple_Desktop_Bus)
multiplexer that allows peripherals (inluding keyboards and mice) to be shared
across up to four retro computers.

![Hootswitch 2024a](extras/hootswitch.jpg)

This project has two parts: an example hardware design and the firmware,
located in their respective folders in this repo.

Status
------

Hootswitch is a hobby project and is very much an ongoing work-in-progress.
Bug reports and/or suggestions are welcome.

Devices that follow the standard ADB keyboard/mouse protocol are supported.
This includes most peripherals that do not have a Mac extension. While ADB was
popular many vendors created devices that required special drivers. There is
interface support for some of these:

- Kensington trackballs (Turbo Mice 4.0 and 5.0)
- Gravis joysticks (Firebird / Blackhawk / Mousestick II)

There is also experimental support for Bluetooth peripherals (both classic and
BLE) via the [Bluepad32](https://github.com/ricardoquesada/bluepad32) library.
Supported devices include
[keyboards](https://bluepad32.readthedocs.io/en/latest/supported_keyboards/),
[mice](https://bluepad32.readthedocs.io/en/latest/supported_mice/), and
[gamepads](https://bluepad32.readthedocs.io/en/latest/supported_gamepads/). By
default, gamepads default to emulating an eight-button keyboard; if you have
the Gravis Firebird driver installed on a computer the gamepad will instead
emulate that device as the extension loads.

Finally, a [fly-by-wire](https://saybur.github.io/hootswitch/flybywire/)
interface allows you to drive your retro computers from a Chromium web browser
on a modern system; this requires a direct USB connection to the Hootswitch.

The firmware is still under active development and many bugs remain. Known
issues include:

- Mouse input can be a bit "floaty" especially during extreme movements.
- No support for non-English keyboard layouts (yet).

This section will continue to be updated as progress is (hopefully) made.

Usage
-----

Hootswitch requires power to operate. For almost all users the front USB
connection of the Pico W works best. Place a jumper across (or permanently
solder) the two front-most pins on J10.

> [!CAUTION]
> __ADB does not support hot-plugging!__ Always power down the device and all
> connected computers before changing cables.

Attach ADB peripherals to the front ADB port. The port on the side of the board
(unpopulated in the picture above) may also be soldered down to provide an ADB
pass-through for more peripherals. Attach computers to the back ports; with
the USB port pointing toward you, computer 1 is the left-most port.

Switching between systems works via the button on the front or
<kbd>Control</kbd> / <kbd>Option</kbd> / <kbd>⌘</kbd> / <kbd>Shift</kbd> in
sequence, followed by <kbd>1</kbd> - <kbd>4</kbd> to choose a port.

To pair a Bluetooth device, put it in pairing mode, press and hold the
Hootswitch button for at least three seconds, then let go. The Pico W LED will
flash while in pairing mode.

Hootswitch logs its status over the USB connection, which may help diagnose
issues with device pairing (or other problems). To view the log, use your
favorite terminal emulator or the fly-by-wire interface.

To update the firmware, make sure your power selection is set for USB, then
hold down the BOOTSEL button while you plug in to your computer via USB. Copy
the correct `.uf2` onto the new mass storage device that appears. Once done the
Pico will disconnect automatically.

Licenses
--------

Except where otherwise noted, all software is available under the Mozilla
Public License (MPL) 2.0. The example hardware is available under the CERN Open
Hardware Licence strongly-reciprocal variant, version 2. Refer to the licenses
for specific terms.
