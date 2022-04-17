#!/bin/sh

INFILE="target/thumbv7em-none-eabihf/release/examples/blink" ;
OUTFILE="target/thumbv7em-none-eabihf/release/examples/blink.bin" ; 

# Compile
cargo build --example blink --release
# Convert binary
arm-none-eabi-objcopy -O binary $INFILE $OUTFILE
# Flash
sudo $LM4FLASH $OUTFILE
# Show binary size
ls -lh $OUTFILE
