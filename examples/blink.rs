//! A blinky-LED example application
//! This example uses launchpad-rs.

#![no_std]
#![no_main]

extern crate embedded_hal;
extern crate tm4c129_launchpad;
extern crate tm4c129x_hal;

use core::fmt::Write;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::serial::Read as ReadHal;
use embedded_hal::digital::v2::*;  // GPIO set high/low
use tm4c129x_hal::gpio::GpioExt;
use tm4c129x_hal::serial;
use tm4c129x_hal::time::Bps;
use tm4c129_launchpad::board;

#[no_mangle]
pub fn stellaris_main(mut board: board::Board) {
    let mut pins_a = board.GPIO_PORTA_AHB.split(&board.power_control);
    let mut uart = serial::Serial::uart0(
        board.UART0,
        pins_a.pa1.into_af_push_pull(&mut pins_a.control),
        pins_a.pa0.into_af_push_pull(&mut pins_a.control),
        (),
        (),
        Bps(115200),
        serial::NewlineMode::SwapLFtoCRLF,
        board::clocks(),
        &board.power_control,
    );
    let mut delay = tm4c129x_hal::delay::Delay::new(
        board.core_peripherals.SYST,
        board::clocks(),
    );

    uart.write_all("Welcome to Launchpad Blink\n");
    let mut loops = 0;
    loop {
        // Spam serial
        writeln!(uart, "Hello, world! Loops = {}", loops).unwrap();
        while let Ok(ch) = uart.read() {
            writeln!(uart, "byte read {}", ch).unwrap();
        }
        loops = loops + 1;

        if *(&(board.button0).is_low().unwrap_or_default()) {
            // Turn all the LEDs on
            let _ = &(board.led0).set_high().unwrap_or_default();
            let _ = &(board.led1).set_high().unwrap_or_default();
            let _ = &(board.led2).set_high().unwrap_or_default();
            let _ = &(board.led3).set_high().unwrap_or_default();
            delay.delay_ms(250u32);
        } else {
            // Cycle through each of the 4 LEDs
            let _ = &(board.led0).set_low().unwrap_or_default();
            let _ = &(board.led1).set_high().unwrap_or_default();
            delay.delay_ms(250u32);
            let _ = &(board.led1).set_low().unwrap_or_default();
            let _ = &(board.led2).set_high().unwrap_or_default();
            delay.delay_ms(250u32);
            let _ = &(board.led2).set_low().unwrap_or_default();
            let _ = &(board.led3).set_high().unwrap_or_default();
            delay.delay_ms(250u32);
            let _ = &(board.led3).set_low().unwrap_or_default();
            let _ = &(board.led0).set_high().unwrap_or_default();
            delay.delay_ms(250u32);
        }

    }
}
