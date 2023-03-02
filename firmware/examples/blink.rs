//! A blinky-LED example application

#![no_std]
#![no_main]

extern crate embedded_hal;
extern crate tm4c129_launchpad;
extern crate tm4c129x_hal;

use core::fmt::Write;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::*; // GPIO set high/low
use embedded_hal::serial::Read as ReadHal;
use tm4c129_launchpad::board;
use tm4c129x_hal::gpio::GpioExt;
use tm4c129x_hal::serial;
use tm4c129x_hal::time::Bps;

#[no_mangle]
pub fn stellaris_main(mut board: board::Board) -> ! {
    // Get GPIOs
    //   for LED
    let mut pins_a = board.GPIO_PORTA_AHB.split(&board.power_control);
    let mut pa3 = pins_a.pa3.into_push_pull_output();
    //   for valve driver control
    let mut pins_f = board.GPIO_PORTF_AHB.split(&board.power_control);
    let mut pins_g = board.GPIO_PORTG_AHB.split(&board.power_control);
    let mut pins_k = board.GPIO_PORTK.split(&board.power_control);

    let mut valve_channel_0 = pins_f.pf0.into_push_pull_output();
    let mut valve_channel_1 = pins_f.pf1.into_push_pull_output();
    let mut valve_channel_2 = pins_f.pf2.into_push_pull_output();
    let mut valve_channel_3 = pins_f.pf3.into_push_pull_output();
    let mut valve_channel_4 = pins_g.pg0.into_push_pull_output();
    let mut valve_channel_5 = pins_g.pg1.into_push_pull_output();
    let mut valve_channel_6 = pins_k.pk4.into_push_pull_output();
    let mut valve_channel_7 = pins_k.pk5.into_push_pull_output();

    valve_channel_0.set_low();
    valve_channel_1.set_low();
    valve_channel_2.set_low();
    valve_channel_3.set_low();
    valve_channel_4.set_low();
    valve_channel_5.set_low();
    valve_channel_6.set_low();
    valve_channel_7.set_low();

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
    let mut delay = tm4c129x_hal::delay::Delay::new(board.core_peripherals.SYST, board::clocks());

    let cycle_on_us: u32 = 63;
    let cycle_off_us: u32 = 438;
    let duration_us: u32 = 250_000;
    let cycles: u32 = duration_us / (cycle_on_us + cycle_off_us);

    let mut loops = 0;
    loop {
        // Spam serial
        // uart.write_all(r"Hello, world!\n");
        // // writeln!(uart, , loops).unwrap_or_default();
        // while let Ok(ch) = uart.read() {
        //     // Echo
        //     writeln!(uart, "byte read {}", ch).unwrap_or_default();
        // }

        loops = loops + 1;

        // pa3.set_high();
        // valve_channel_7.set_low();
        // valve_channel_0.set_high();
        // delay.delay_ms(1000u32);

        // pa3.set_low();
        // valve_channel_0.set_low();
        // valve_channel_1.set_high();
        // delay.delay_ms(1000u32);

        pa3.set_high();
        // valve_channel_1.set_low();
        // Janky blocking pwm
        for _ in 0..cycles {
            valve_channel_2.set_high();
            delay.delay_us(cycle_on_us);
            valve_channel_2.set_low();
            delay.delay_us(cycle_off_us);
        }
        pa3.set_low();
        delay.delay_ms(250u32);

        // pa3.set_low();
        // valve_channel_2.set_low();
        // valve_channel_3.set_high();
        // delay.delay_ms(1000u32);

        // pa3.set_high();
        // valve_channel_3.set_low();
        // valve_channel_4.set_high();
        // delay.delay_ms(1000u32);

        // pa3.set_low();
        // valve_channel_4.set_low();
        // valve_channel_5.set_high();
        // delay.delay_ms(1000u32);

        // pa3.set_high();
        // valve_channel_5.set_low();
        // valve_channel_6.set_high();
        // delay.delay_ms(1000u32);

        // pa3.set_low();
        // valve_channel_6.set_low();
        // valve_channel_7.set_high();
        // delay.delay_ms(1000u32);
    }
}
