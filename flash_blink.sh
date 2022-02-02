#!/bin/sh

cargo build --example blink --release

arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/examples/blink target/thumbv7em-none-eabihf/release/examples/blink.bin

sudo lm4flash  target/thumbv7em-none-eabihf/release/examples/blink.bin
