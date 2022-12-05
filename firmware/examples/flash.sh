#!/bin/sh

INFILE="target/thumbv7em-none-eabihf/release/examples/$1" ;
OUTFILE="target/thumbv7em-none-eabihf/release/examples/$1.bin" ; 

# Compile
cargo build --example $1 --release  &&
# Convert binary
arm-none-eabi-objcopy -O binary $INFILE $OUTFILE  &&
# Show binary size
ls -lh $OUTFILE &&
# Unlock just in case
# sudo $LM4FLASH -U
# Flash
sudo $LM4FLASH $OUTFILE
