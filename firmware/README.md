# Rust on TM4C129 Launchpad

An example program for the TM4C129-XL Launchpad board, based on the [stellaris-launchpad](https://github.com/thejpster/stellaris-launchpad) crate.

## Requirements

* rustc components (for cross-compiling)
* arm-none-eabi-* (ARM Embedded Toolchain for compiler)
* libusb-1.0-0-dev (for lm4tools)
* lm4tools (serial interface to Launchpad bootloader)
* TI drivers for the in-circuit debugger (to connect to the board's UART via USB)

The process described here works (with some offroading) on Linux and Mac, but has not been tested on Windows.

## Setup

### Rustc components
```bash
rustup component add rust-src
rustup target add thumbv7em-none-eabihf
```

### ARM Embedded Toolchain
Download from [the ARM developer site](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) and install. Unfortunately the install does not add the binaries to path.

Add the binaries to path however you like; on mac, you can add a line to your ~/.bash_profile
```bash
export PATH="$PATH:/Applications/ARM/bin/"
```

### lm4tools
lm4tools provides a serial interface to the Stellaris bootloader on the launchpad board.

First, get libusb.
```bash
# Mac
brew install libusb
```
```bash
# Ubuntu (note the generic libusb-dev package is broken! must install specific version)
sudo apt install libusb-1.0-0-dev
```

Clone https://github.com/uastw-embsys/lm4tools (or one of the 40+ other forks of lm4tools) and run **make** to build the binaries. Then, similar to the ARM toolchain, you can add another line to your bash run command file to get the lm4flash binary into your path, or just reference its path manually with each use.

### TI Drivers
Follow the instructions at https://software-dl.ti.com/ccs/esd/documents/ccs_downloads.html to install the latest version of the USB drivers.


## Compile and flash

```bash
# Compile
cargo build --example blink --release

# Convert binary to a compatible format
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/examples/blink target/thumbv7em-none-eabihf/release/examples/blink.bin

# Flash the board
sudo lm4flash target/thumbv7em-none-eabihf/release/examples/blink.bin
```

## Serial comms

The on-board bootloader acts as a UART-to-USB bridge for UART0. This connection can be monitored using PuTTY on any platform. On Mac, you can also use **screen** (replacing the usb connection name) like:

```bash
# To exit, do
# CTRL-A
# CTRL-     (continue holding down CTRL!)
# CTRL-C
screen /dev/tty.something 115200
```

On Ubuntu,```ls -l /dev/serial/by-id``` will show which device is associated with which port.

Figuring out which serial port it is can be tedious on a Mac. Just watch ```ls /dev/tty*``` while plugging/unplugging the board a few times.

# Future Plans
* Maybe-possibly hardware-out-of-the-loop testing using QEMU
* Definitely hardware-in-the-loop testing using a self-hosted github-actions runner (probably a raspberry pi on a shelf in my apartment) with an unlocking procedure to keep the board from being bricked during testing
