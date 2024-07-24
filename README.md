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
set up as follows.

Dependencies:

```
sudo apt install git cmake gcc-arm-none-eabi gcc gdb-multiarch automake \
    autoconf build-essential texinfo libtool libftdi-dev libusb-1.0-0-dev
```

You may also want `openocd` and `minicom` for debugging.

Clone this repo and the (Pico SDK)[https://github.com/raspberrypi/pico-sdk].
Execute `git submodule update --init` in the project to get sublibraries
added.

Set `PICO_SDK_PATH` to point to where the SDK lives via `~/.bashrc`. Something
like `export PICO_SDK_PATH=/home/saybur/src/pico-sdk` would be suitable.

Once you have everything set up, build as follows.

```
mkdir -p build
cd build
cmake ..
make
```

Licenses
--------

Except where otherwise noted, all software is available under the GNU GPL v3.
The example hardware is available under the CERN Open Hardware Licence
strongly-reciprocal variant, version 2. Refer to the licenses for specific
terms.
