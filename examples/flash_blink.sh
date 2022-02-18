#!/bin/sh

OUTFILE="target/thumbv7em-none-eabihf/release/examples/blink.bin" ; 

# Compile
cargo build --example blink --release
# Convert binary
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/examples/blink target/thumbv7em-none-eabihf/release/examples/blink.bin
# Flash
sudo lm4flash  target/thumbv7em-none-eabihf/release/examples/blink.bin
# Show binary size
ls -lh $OUTFILE
