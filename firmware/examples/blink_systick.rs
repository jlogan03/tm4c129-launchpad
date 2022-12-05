//! A blinky-LED example application

#![no_std]
#![no_main]

extern crate embedded_hal;
extern crate tm4c129_launchpad;
extern crate tm4c129x_hal;

use cortex_m_rt::{exception, ExceptionFrame};
use static_assertions::const_assert;

use core::fmt::Write;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::*; // GPIO set high/low
use embedded_hal::serial::Read as ReadHal;
use tm4c129_launchpad::board::{clocks, safe, Board};
use tm4c129_launchpad::startup::{Interrupt};
use tm4c129x_hal::gpio::GpioExt;
use tm4c129x_hal::serial;
use tm4c129x_hal::time::Bps;

use irq::{handler, scope, scoped_interrupts};


fn systick() {
    use tm4c129x_hal::{gpio::GpioExt, serial, sysctl::SysctlExt, time::Bps, CorePeripherals};
    let mut core_peripherals = unsafe { CorePeripherals::steal() };

    core_peripherals.SYST.disable_interrupt();
    core_peripherals.SYST.clear_current();

    let p = unsafe { tm4c129x_hal::Peripherals::steal() };
    let pins = p.GPIO_PORTF_AHB.split(&p.SYSCTL.constrain().power_control);

    let mut delay = tm4c129x_hal::delay::Delay::new(core_peripherals.SYST, unsafe { &clocks() });
    let mut led0 = pins.pf0.into_push_pull_output();
    loop {
        let _ = led0.set_high().unwrap_or_default();
        delay.delay_ms(500u32);
        let _ = led0.set_low().unwrap_or_default();
        delay.delay_ms(500u32);
    }
}


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

    // let mut delay = tm4c129x_hal::delay::Delay::new(board.core_peripherals.SYST, clocks());

    // Set up systick interrupt
    //    Set timing duri
    let us = 1000;
    let mut rvr = us * (clocks().sysclk.0 / 1_000_000);
    rvr += (us * ((clocks().sysclk.0 % 1_000_000) / 1_000)) / 1_000;
    rvr += (us * (clocks().sysclk.0 % 1_000)) / 1_000_000;

    &board.core_peripherals.SYST.set_reload(rvr);
    &board.core_peripherals.SYST.clear_current();
    &board.core_peripherals.SYST.enable_counter();
    // let _ = &board.core_peripherals.ITM.
    &board.core_peripherals.SYST.enable_interrupt();

    writeln!(uart, "SysTick Blink {}", &board.core_peripherals.SYST.is_interrupt_enabled()).unwrap_or_default();

    let mut loops = 0;

    // handler!(
    //     systick = || {
    //         // Spam serial
    //         writeln!(uart, "Hello, world! Loops = {}", loops).unwrap_or_default();
    //         while let Ok(ch) = uart.read() {
    //             // Echo
    //             writeln!(uart, "byte read {}", ch).unwrap_or_default();
    //         }

    //         loops = loops + 1;

    //         if *(&(board.button0).is_low().unwrap_or_default()) {
    //             // Turn all the LEDs on
    //             let _ = &(board.led0).set_high().unwrap_or_default();
    //             let _ = &(board.led1).set_high().unwrap_or_default();
    //             let _ = &(board.led2).set_high().unwrap_or_default();
    //             let _ = &(board.led3).set_high().unwrap_or_default();
    //         } else {
    //             let which_led = loops % 4;

    //             // Turn all the LEDs off
    //             let _ = &(board.led0).set_low().unwrap_or_default();
    //             let _ = &(board.led1).set_low().unwrap_or_default();
    //             let _ = &(board.led2).set_low().unwrap_or_default();
    //             let _ = &(board.led3).set_low().unwrap_or_default();

    //             // Turn on just the selected LED
    //             match which_led {
    //                 0 => {
    //                     let _ = &(board.led1).set_high().unwrap_or_default();
    //                 }
    //                 1 => {
    //                     let _ = &(board.led1).set_high().unwrap_or_default();
    //                 }
    //                 2 => {
    //                     let _ = &(board.led2).set_high().unwrap_or_default();
    //                 }
    //                 3 => {
    //                     let _ = &(board.led3).set_high().unwrap_or_default();
    //                 },
    //                 _ => {}
    //             }
    //         }
    //     }
    // );

    // Create a scope and register the systick interrupt handler.
    handler!(systick_handler = || systick());
    scope(|s| {
        s.register(Interrupt::SysTick, systick_handler);


        loop {
            // Wait for interrupt
        }
    });

    // Must not return
    loop {};


}
