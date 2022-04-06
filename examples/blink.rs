//! A blinky-LED example application
//! This example uses launchpad-rs.

#![no_std]
#![no_main]

extern crate embedded_hal;
extern crate tm4c129_launchpad;
extern crate tm4c129x_hal;

use core::fmt::Write;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::*; // GPIO set high/low
use embedded_hal::serial::Read as ReadHal;
use tm4c129x_hal::gpio::GpioExt;
use tm4c129x_hal::serial;
use tm4c129x_hal::time::Bps;

use catnip::{MACAddr, ip::IPV4Addr};
use tm4c129_launchpad::{board, drivers::ethernet::socket::UDPSocket};

const M: usize = 2;  // Fixed UDP transmission data length in 32-bit words


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
    let mut delay = tm4c129x_hal::delay::Delay::new(board.core_peripherals.SYST, board::clocks());

    let udp: UDPSocket<M> = UDPSocket {
        src_macaddr: MACAddr{value: board.enet.src_macaddr},
        src_ipaddr: IPV4Addr{value: [10, 0, 0, 2]},
        src_port: 8053,
        dst_macaddr: None,
        dst_ipaddr: IPV4Addr{value: [10, 0, 0, 1]},
        dst_port: 8053,
    };

    uart.write_all("Welcome to Launchpad Blink\n");
    let mut loops = 0;
    loop {
        // Spam serial
        writeln!(uart, "Hello, world! Loops = {}", loops).unwrap_or_default();
        while let Ok(ch) = uart.read() {
            // Echo
            writeln!(uart, "byte read {}", ch).unwrap_or_default();

            // Show MAC address
            let addr = udp.src_macaddr.value;
            writeln!(
                uart,
                "MAC Address: {:x}:{:x}:{:x}:{:x}:{:x}:{:x}",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
            )
            .unwrap_or_default();

            // Debugging
            unsafe{
                let descr = board.enet.txdl.get();
                writeln!(uart, "TX Descriptor 0 Word 0: {}", descr.v[0]).unwrap_or_default();
                writeln!(uart, "TX Descriptor 0 Owned: {}", descr.is_owned()).unwrap_or_default();
            }
            // writeln!(uart, "TX Descriptor 0 Word 1: {}", board.emac.tx_descriptors[0].v[1]).unwrap_or_default();
            // writeln!(uart, "TX Descriptor 0 Word 2: {}", board.emac.tx_descriptors[0].v[2]).unwrap_or_default();
            // writeln!(uart, "TX Descriptor 0 Word 3: {}", board.emac.tx_descriptors[0].v[3]).unwrap_or_default();
            // writeln!(uart, "RX Descriptor 0 Word 2: {}", board.emac.rx_descriptors[0].v[2]).unwrap_or_default();
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
