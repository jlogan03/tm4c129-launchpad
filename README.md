# Rust on TM4C129 Launchpad

An example program for the TM4C129-XL Launchpad board, based on the [stellaris-launchpad](https://github.com/thejpster/stellaris-launchpad) crate.

## Requirements

* arm-none-eabi-* (ARM Embedded Toolchain)
* lm4tools

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

Clone https://github.com/utzig/lm4tools and run make to build the binaries. Then, similar to the ARM toolchain, you can add a line like this to your bash run command file to get the lm4flash binary into your path.


## Compile and flash

```bash
# Compile
cargo build --example blink --release

# Convert binary to format compatible with micro
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/examples/blink target/thumbv7em-none-eabihf/release/examples/blink.bin

# Flash the board
sudo lm4flash target/thumbv7em-none-eabihf/release/examples/blink.bin
```

The program can also be debugged while running, using OpenOCD to monitor a GDB debugger server. See OpenOCD quick-start instructions at https://sourceforge.net/p/openocd/code/ci/master/tree/.
