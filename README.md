hootswitch
==========

Hootswitch is an [ADB](https://en.wikipedia.org/wiki/Apple_Desktop_Bus)
multiplexer, allowing one keyboard and mouse to be switched between up to
four retro computers.

Building
--------

You will need the Pico SDK and associated tooling. There is a setup script
(here)[https://raw.githubusercontent.com/raspberrypi/pico-setup/master/pico_setup.sh]
that assumes you're using a Raspberry Pi as your development environment if you
want to try that; I personally use Debian _bookworm_ on a regular x86 system
instead, set up as follows.

Dependencies and associated tools:

```
sudo apt install git cmake gcc-arm-none-eabi gcc gdb-multiarch automake \
    autoconf build-essential texinfo libtool libftdi-dev libusb-1.0-0-dev \
    openocd
```

Clone the [Pico SDK](https://github.com/raspberrypi/pico-sdk). Execute
`git submodule update --init` in that project to get sub-libraries added. Set
`PICO_SDK_PATH` to point at where the SDK lives via `~/.bashrc` or equivalent;
something like `export PICO_SDK_PATH=/home/saybur/src/pico-sdk` would be
suitable after being adjusted for your environment.

Once you have everything set up, build as follows.

```
mkdir -p build
cd build
cmake ..
make
```

Among other files, this will produce `hootswitch.uf2`. Make sure your power
selection is correct, then hold down the BOOTSEL button while you plug in
to your computer via USB. Copy the `.uf2` onto the new mass storage device.
Reboot.

Debugging
---------

[The official debugger](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html)
is dirt cheap and works reasonably well if you plan to develop on the firmware.
It can be used as follows to upload without needing to fuss with the BOOTSEL
button. Be in the `dialout` group to avoid needing `sudo` all over the place.

```
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
    -c "adapter speed 5000" -c "program blink.elf verify reset exit"
```

For a proper debugging session, start the OpenOCD server by omitting the last
`-c` argument above. You will want to invoke `cmake` with
`-DCMAKE_BUILD_TYPE=Debug` .

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
* `i r` prints registers.
* `p X` prints variable X.

Licenses
--------

Except where otherwise noted, all software is available under the GNU GPL v3.
The example hardware is available under the CERN Open Hardware Licence
strongly-reciprocal variant, version 2. Refer to the licenses for specific
terms.
