//! A blinky-LED example application

#![no_std]
#![no_main]

extern crate embedded_hal;
extern crate tm4c129_launchpad;
extern crate tm4c129x_hal;

use core::fmt::Write;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::*; // GPIO set high/low
use embedded_hal::serial::Read as ReadHal;
use tm4c129_launchpad::board;
use tm4c129x_hal::gpio::{GpioExt, AlternateFunctionChoice, AlternateFunction};
use tm4c129x_hal::serial;
use tm4c129x_hal::time::Bps;

use tm4c129x_hal::sysctl::{
    control_power, reset, Clocks, CrystalFrequency, Domain, Oscillator, PllOutputFrequency,
    PowerControl, PowerState, RunMode, SysctlExt, SystemClock,
};

use tm4c129x_hal::gpio::AF1;

#[no_mangle]
pub fn stellaris_main(mut board: board::Board) -> ! {
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
    let mut delay = tm4c129x_hal::delay::Delay::new(board.core_peripherals.SYST, board::clocks());

    // ADC
    // power on (sysctl)
    // enable ADC (ADCACTSS)
    // enable DMA (ADCACTSS)
    // set vref source (ADCCTL)
    // set hardware oversampling (ADCSAC)
    // set sample hold time (ADCSSTSH0)
    // loop
    //    set bank mux (ADCSSEMUX0)
    //    set sample mux (ADCSSMUX0)
    //    start sample (ADCPSSI)
    //    start conversion (automatic??)
    //    wait for sample (not ADCSSFSTAT0.EMPTY)
    //    while not ADCSSFSTAT0.EMPTY
    //        read value (ADCSSFIFO0)
    //    clear fifo overflow status (ADCOSTAT) by writing 1

    // set GPIOE pin modes
    let mut pins_gpioe = board.GPIO_PORTE_AHB.split(&board.power_control);
    pins_gpioe.pe0.into_af_pull_down::<AF1>(&mut pins_gpioe.control);
    pins_gpioe.pe1.into_af_pull_down::<AF1>(&mut pins_gpioe.control);
    pins_gpioe.pe2.into_af_pull_down::<AF1>(&mut pins_gpioe.control);
    pins_gpioe.pe3.into_af_pull_down::<AF1>(&mut pins_gpioe.control);

    // power on
    control_power(&board.power_control, Domain::Adc0, RunMode::Run, PowerState::On);

    // control_power(&board.power_control, Domain::Adc1, RunMode::Run, PowerState::On);
    // leave vref source as internal (default)
    // leave hardware oversampling as default (none)
    // leave sample hold time as default (shortest)
    // use main oscillator
    board.ADC0.cc.write(|w| w.cs().mosc());

    // peripherals.ADC1.actss.write(|w| w.asen0().set_bit().asen1().set_bit().asen2().set_bit().asen3().set_bit());
    // enable dma
    board.ADC0.actss.write(|w| w.aden1().set_bit());
    // enable adc. have to configure before enabling
    board.ADC0.actss.write(|w| w.asen1().set_bit());
    // peripherals.ADC1.actss.write(|w| w.aden0().set_bit().aden1().set_bit().aden2().set_bit().aden3().set_bit());


    uart.write_all("Welcome to Launchpad ADC\n");
    let mut loops = 0;
    loop {
        writeln!(uart, "Loop {}", loops).unwrap_or_default();

        // ADC read
        //    set bank mux (ADCSSEMUX0)
        //    set sample mux (ADCSSMUX0)
        //    start sample (ADCPSSI)
        //    start conversion (automatic??)
        //    wait for sample (not ADCSSFSTAT0.EMPTY)
        //    while not ADCSSFSTAT0.EMPTY
        //        read value (ADCSSFIFO0)
        //    clear fifo overflow status (ADCOSTAT) by writing 1

        // Clear ADC fifo buffer
        while !&board.ADC0.ssfstat3.read().empty().bit_is_set() {
            // Read the value in order to move the buffer along
            // We don't know where this came from, so don't use it
            &board.ADC0.ssfifo3.read();
        }

        // Select first ADC bank (inputs 0..=15)
        &board.ADC0.ssemux1.write_with_zero(|w| w);

        // Select first half of bank
        unsafe {
            &board.ADC0.ssmux1.write(|w| w.mux0().bits(0).mux1().bits(1).mux2().bits(2).mux3().bits(3));
        }

        // Set 4th sample as end of sequence
        &board.ADC0.ssctl1.write(|w| w.end3().set_bit());

        // Start sample sequencer that will capture samples
        &board.ADC0.pssi.write(|w| w.ss1().set_bit());  // sequencer init
        &board.ADC0.pssi.write(|w| w.gsync().set_bit());  // Global sync

        delay.delay_ms(100u32);

        // Wait until the fifo is full
        // while board.ADC0.ssfstat1.read().empty().bit_is_set() {
            // do nothing
            writeln!(uart, "adc busy? {:?}", &board.ADC0.actss.read().busy().bit_is_set());
            writeln!(uart, "init still set? {:?}", &board.ADC0.pssi.read().ss1().bit_is_set());
            writeln!(uart, "sync still set? {:?}", &board.ADC0.pssi.read().gsync().bit_is_set());
            writeln!(uart, "fifo full? {:?}", &board.ADC0.ssfstat1.read().full().bit_is_set());
            writeln!(uart, "fifo empty? {:?}", &board.ADC0.ssfstat1.read().empty().bit_is_set());
            writeln!(uart, "fifo overflow? {:?}", &board.ADC0.ostat.read().ov1().bit_is_set());
            writeln!(uart, "fifo underflow? {:?}", &board.ADC0.ustat.read().uv1().bit_is_set());
            // writeln!(uart, "fifo value? {:?}", &board.ADC0.ssfifo1.read().data().bits());
        // }

        // clear underflow
        board.ADC0.ustat.write(|w| w.uv1().set_bit());



        // Read all 8 values
        let mut adcvals = [0_u16; 20];  // there are 20 total, but we're only reading a few for now
        for i in 0..4_usize {
            adcvals[i] = *(&board.ADC0.ssfifo1.read().data().bits());
        }

        // Spam serial
        // writeln!(uart, "Hello, world! Loops = {}", loops).unwrap_or_default();
        // while let Ok(ch) = uart.read() {
        //     // Echo
        //     writeln!(uart, "byte read {}", ch).unwrap_or_default();
        // }

        for i in 0..4_usize {
            let val = adcvals[i];
            writeln!(uart, "Channel {i}: {val}").unwrap_or_default();
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
