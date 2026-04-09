Debugging
=========

These are notes for a potential way to debug the device during development.
This is how I (@saybur) personally like working on it in a Linux desktop
environment.

Logging
-------

Hootswitch will log some aspects of device detection, host startup, and similar
lifecycle events to the USB CDC device. Connect at 115200 baud, 8-N-1.

Hardware
--------

To capture ADB signals I use an inexpensive logic analyzer compatible with the
`fx2lafw` firmware from the
[sigrok](https://sigrok.org/wiki/Main_Page) project. To sample the device-side
the passthrough ADB connector works great. For one of the hosts you can either
use the second ADB port on a computer that has one (IIci and the like) or a
hacked-together ADB splitter of some kind.

[PulseView](https://sigrok.org/wiki/PulseView) works for reviewing captures. It
works even better with an ADB decoder like
[decoder-adb](https://github.com/tmk/decoder-adb). On my system that had to be
installed in ` ~/.local/share/libsigrokdecode/decoders` in an `adb` subfolder.

Software
--------

You will want to invoke `cmake` with `-DCMAKE_BUILD_TYPE=Debug`.

[The official debugger](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html)
is dirt cheap and works reasonably well if you plan to develop on the firmware.
It can be used as follows to upload without needing to fuss with the BOOTSEL
button. Be in the `dialout` group to avoid needing `sudo` all over the place.
OpenOCD can be started as follows.

```
openocd -c "set USE_CORE 0" -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
    -c "adapter speed 5000" -c "rp2040.core0 configure -rtos FreeRTOS"
```

See <https://github.com/raspberrypi/pico-sdk/issues/1622> for a discussion
about `USE_CORE 0` above. The above is fine for now with the current state of
development, which runs only on a single core anyway.

In another terminal, run `gdb-multiarch hootswitch.elf`. Connect to the device
with `target remote localhost:3333` (or automate this step via `~/.gdbinit`).
See [the documentation](https://openocd.org/doc/html/General-Commands.html) for
specifics; the following commands are likely useful (with the caveat that I am
_most definitely_ not an authority on `gdb`):

* `load` will update programming with the current ELF.
* `monitor reset init` will restart and halt for instructions.
* `print X` prints information about something, including variables.
* `br X` adds a breakpoint at X, `del` removes one or more.
* `continue` will proceed, stopping at breakpoints; Ctrl-C halts.
* `finish` completes a function.
* `bt` prints a backtrace, `bt f` prints a backtrace with symbols.
* `up` and `down` moves around the frames of a call.
* `i r` prints registers.
* `p X` prints variable X.
* `p/x *0x0` prints in hex from the address, useful for peripheral registers.
* `i threads` prints information relevant to FreeRTOS thread execution.

Use `arm-none-eabi-size -A hootswitch.elf` to get an idea about RAM usage.
