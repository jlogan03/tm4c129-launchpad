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
use tm4c129x_hal::gpio::GpioExt;
use tm4c129x_hal::serial;
use tm4c129x_hal::time::Bps;

use catnip::{IPV4Addr, MACAddr};
use tm4c129_launchpad::{
    board,
    drivers::ethernet::{socket::UDPSocket, RXBUFSIZE},
};

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

    let udp = UDPSocket {
        src_macaddr: MACAddr {
            value: board.enet.src_macaddr,
        },
        src_ipaddr: IPV4Addr {
            value: [10, 0, 0, 252],
        },
        src_port: 8053,
        dst_macaddr: None,
        dst_ipaddr: IPV4Addr {
            value: [0, 0, 0, 0],
        },
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
        }

        // Debugging

        // Test ethernet receive (without UDP socket)

        let mut buf = [0_u8; RXBUFSIZE];
        unsafe {
            match &board.enet.receive(&mut buf) {
                Ok(num_bytes) => {
                    writeln!(uart, "Received {} ethernet bytes", num_bytes).unwrap_or_default()
                }
                Err(x) => writeln!(uart, "Ethernet RX error: {:?}", x).unwrap_or_default(),
            };
        }
        let rxdl = &mut (board.enet.rxdl);
        writeln!(uart, "{:?}", rxdl).unwrap_or_default();

        // Test UDP transmit
        match udp.transmit::<3>(&mut board.enet, *b"hello world!") {
            Ok(_) => writeln!(uart, "UDP transmit started").unwrap_or_default(),
            Err(x) => writeln!(uart, "UDP TX error: {:?}", x).unwrap_or_default(),
        };
        let txdl = &mut (board.enet.txdl);
        writeln!(uart, "{:?}", txdl).unwrap_or_default();

        // Check EMAC status
        let status = board.enet.emac.status.read().txpaused().bit_is_set();
        writeln!(uart, "EMAC TX paused? {status}");
        let status = board.enet.emac.hostxdesc.read().bits();
        writeln!(uart, "EMAC TX descr addr {status}");
        let status = board.enet.emac.hostxba.read().bits();
        writeln!(uart, "EMAC TX buf addr {status}");
        let status = board.enet.emac.txdladdr.read().bits();
        writeln!(uart, "EMAC TXDL addr {status}");
        let status = board.enet.emac.status.read().txfe().bit_is_set();
        writeln!(uart, "EMAC TX FIFO not empty? {status}");
        let status = board.enet.emac.status.read().twc().bit_is_set();
        writeln!(uart, "EMAC TX FIFO write controller active? {status}");
        let status = board.enet.emac.status.read().tpe().bit_is_set();
        writeln!(uart, "EMAC MII transmit protocol engine status? {status}");
        let status = board.enet.emac.status.read().tfc().variant();
        writeln!(uart, "EMAC MII transmit frame controller status {status:?}");
        let status = board.enet.emac.dmaris.read().ts().bits();
        writeln!(uart, "EMAC DMA transmit process state {status:?}");

        let status = board.enet.emac.status.read().rrc().bits();
        writeln!(uart, "EMAC RX FIFO controller state? {status:?}");
        let status = board.enet.emac.hosrxdesc.read().bits();
        writeln!(uart, "EMAC RX descr addr {status}");
        let status = board.enet.emac.hosrxba.read().bits();
        writeln!(uart, "EMAC RX buf addr {status}");
        let status = board.enet.emac.rxdladdr.read().bits();
        writeln!(uart, "EMAC RXDL addr {status}");
        let status = board.enet.emac.status.read().rxf().bits();
        writeln!(uart, "EMAC RX FIFO status? {status:?}");
        let status = board.enet.emac.status.read().rwc().bit_is_set();
        writeln!(uart, "EMAC RX FIFO write controller active? {status}");
        let status = board.enet.emac.status.read().rpe().bit_is_set();
        writeln!(uart, "EMAC MII receive protocol engine status? {status}");
        let status = board.enet.emac.status.read().rfcfc().bits();
        writeln!(uart, "EMAC MII receive frame controller status {status:?}");
        let status = board.enet.emac.dmaris.read().rs().bits();
        writeln!(uart, "EMAC DMA receive process state {status:?}");

        let status = board.enet.emac.dmaris.read().fbi().bit_is_set();
        writeln!(uart, "EMAC DMA bus error? {status:?}");
        let status = board.enet.emac.dmaris.read().ae().bits();
        writeln!(uart, "EMAC DMA access error type {status:?}");

        // board.enet.emacclear(); // Clear clearable interrupts

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
