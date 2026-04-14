Building
========

## Setup

You will need the correct tools to build the firmware. There is a setup script
[here](https://raw.githubusercontent.com/raspberrypi/pico-setup/master/pico_setup.sh)
that assumes you're using a Raspberry Pi as your development environment if you
want to try that; I personally use Debian _trixie_ on a regular x86 system
instead, set up as follows.

Dependencies and associated tools:

```
sudo apt install git cmake gcc-arm-none-eabi gcc gdb-multiarch automake \
    autoconf build-essential texinfo libtool libftdi-dev libusb-1.0-0-dev \
    openocd
```

## Clone the Project

Project dependencies are provided as Git submodules. Clone the project, then
execute `git submodule update --init` within the local working tree. If you do
not have a separate Pico SDK installed you will also need to pull its
submodules with `git submodule update --init --recursive`.

### Optional: Separate pico-sdk

If you have multiple Pico projects (or limited storage space) you may want a
separate copy of the SDK. If `PICO_SDK_PATH` is defined that will be used
instead of the submodule. This part can be skipped if you already used the
`--recursive` version above.

Clone the [Pico SDK](https://github.com/raspberrypi/pico-sdk). Execute
`git submodule update --init` in that project to get sub-libraries added. Set
`PICO_SDK_PATH` to point at where the SDK lives via `~/.bashrc` or equivalent;
something like `export PICO_SDK_PATH=/home/saybur/src/pico-sdk` would be
suitable after being adjusted for your environment.

## Build the Firmware

Once you have everything set up, build as follows from the root of the repo.
This builds for the Pico W by default. To build for the standard (non-W) Pico,
pass `-DPICO_BOARD=pico` during the call to `cmake` below.

```
mkdir -p build
cd build
cmake ..
make -j4
```

Among other files, this will produce `hootswitch.uf2`. Flash it to the device
following the instructions in <README.md>.

## Hardware

Both the original Raspberry Pi Pico and Pico W boards are supported. The focus
of development is on the Pico W: in addition to support for Bluetooth, the
current configuration system also requires the _btstack_ library, which is
licensed for use on the Pico W only.

The Pico 2 and Pico 2 W have _not_ been tested and may not work correctly.
