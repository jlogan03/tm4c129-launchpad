//! A blinky-LED example application that uses the SysTick interrupt for static scheduling

#![no_std]
#![no_main]

extern crate embedded_hal;
extern crate tm4c129_launchpad;
extern crate tm4c129x_hal;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{exception, ExceptionFrame};
use static_assertions::const_assert;

use core::fmt::Write;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::*; // GPIO set high/low
use embedded_hal::serial::Read as ReadHal;
use tm4c129_launchpad::board::{clocks, safe, Board};
use tm4c129_launchpad::startup::Interrupt;
use tm4c129x_hal::gpio::GpioExt;
use tm4c129x_hal::serial;
use tm4c129x_hal::time::Bps;

use irq::{handler, scope, scoped_interrupts};

#[no_mangle]
pub fn stellaris_main(mut board: Board) -> ! {
    let mut pins_a = board.GPIO_PORTA_AHB.split(&board.power_control);
    let mut uart = serial::Serial::uart0(
        board.UART0,
        pins_a.pa1.into_af_push_pull(&mut pins_a.control),
        pins_a.pa0.into_af_push_pull(&mut pins_a.control),
        (),
        (),
        Bps(115200),
        serial::NewlineMode::SwapLFtoCRLF,
        clocks(),
        &board.power_control,
    );

    // Set up systick as static scheduling interrupt
    //    Set tick timing
    let us_per_interrupt = 1000; // [us] How many us per interrupt?
    let clock_cycles_per_us = clocks().sysclk.0 / 1_000_000; // [cycles]
    let mut reload = us_per_interrupt * clock_cycles_per_us + 1; //  [cycles/interrupt] Interrupt after this many clock cycles
    
    //    Configure and enable SysTick interrupt
    board.core_peripherals.SYST.set_reload(120_000_000 / 1000);
    board.core_peripherals.SYST.set_clock_source(SystClkSource::Core);  // Use sysclk
    board.core_peripherals.SYST.clear_current();
    board.core_peripherals.SYST.enable_counter();
    board.core_peripherals.SYST.enable_interrupt();

    writeln!(
        uart,
        "SysTick Blink {}",
        board.core_peripherals.SYST.is_interrupt_enabled()
    )
    .unwrap_or_default();

    let mut interrupt_count: u64 = 0;
    let mut loops: u8 = 0;

    handler!(
        systick_handler = || {
            interrupt_count = interrupt_count.wrapping_add(1);

            if interrupt_count % 100 == 0 {
                // Cycle LEDs once every hundred systicks
                loops = loops.wrapping_add(1);

                // Spam serial
                let time_ms = (us_per_interrupt as u64) * interrupt_count / 1000;
                writeln!(uart, "Hello, world! Interrupt Count = {}; Time = {} [ms]", interrupt_count, time_ms)
                    .unwrap_or_default();
                while let Ok(ch) = uart.read() {
                    // Echo
                    writeln!(uart, "byte read {}", ch).unwrap_or_default();
                }

                if *(&(board.button0).is_low().unwrap_or_default()) {
                    // Turn all the LEDs on
                    let _ = &(board.led0).set_high().unwrap_or_default();
                    let _ = &(board.led1).set_high().unwrap_or_default();
                    let _ = &(board.led2).set_high().unwrap_or_default();
                    let _ = &(board.led3).set_high().unwrap_or_default();
                } else {
                    let which_led = loops % 4;

                    // Turn all the LEDs off
                    let _ = &(board.led0).set_low().unwrap_or_default();
                    let _ = &(board.led1).set_low().unwrap_or_default();
                    let _ = &(board.led2).set_low().unwrap_or_default();
                    let _ = &(board.led3).set_low().unwrap_or_default();

                    // Turn on just the selected LED
                    match which_led {
                        0 => {
                            let _ = &(board.led0).set_high();
                        }
                        1 => {
                            let _ = &(board.led1).set_high();
                        }
                        2 => {
                            let _ = &(board.led2).set_high();
                        }
                        3 => {
                            let _ = &(board.led3).set_high();
                        }
                        _ => {}
                    }
                }
            }
        }
    );

    // Create a scope and register the systick interrupt handler.
    // handler!(systick_handler = || systick());
    scope(|s| {
        s.register(Interrupt::SysTick, systick_handler);

        loop {
            // Wait for interrupt
        }
    });

    // Main must not return
    loop {}
}
