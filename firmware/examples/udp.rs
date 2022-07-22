//! A blinky-LED example application

#![no_std]
#![no_main]

extern crate embedded_hal;
extern crate tm4c129_launchpad;
extern crate tm4c129x_hal;

use core::convert::Infallible;
use core::fmt::Write;
use ufmt::*;

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::*; // GPIO set high/low
use embedded_hal::prelude::{_embedded_hal_serial_Read, _embedded_hal_serial_Write};

use tm4c129x_hal::gpio::GpioExt;

use tm4c129x_hal::serial;
use tm4c129x_hal::serial::*;
use tm4c129x_hal::time::Bps;

use catnip::{arp::*, enet::*, ip::*, udp::*, *};
use tm4c129_launchpad::{
    board,
    drivers::ethernet::{EthernetDriver, socket::UDPSocket, EthernetError, RXBUFSIZE},
};

struct SerialUWriteable<UART, TX, RX, RTS, CTS>(Serial<UART, TX, RX, RTS, CTS>);

impl<TX, RX, RTS, CTS> uWrite for SerialUWriteable<UART0, TX, RX, RTS, CTS> {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Infallible> {
        write!(self.0, "{}", s);
        Ok(())
    }
}

// IP
const IPSTART: usize = EthernetHeader::BYTE_LEN;
const UDPSTART: usize = IPSTART + IpV4Header::BYTE_LEN;
const ARPSTART: usize = IPSTART;

fn poll_ethernet<TX, RX, RTS, CTS>(
    enet: &mut EthernetDriver,
    uart: &mut SerialUWriteable<UART0, TX, RX, RTS, CTS>,
    udp: &UDPSocket,
    buffer: &mut [u8; RXBUFSIZE]
) {
    // Receive ethernet bytes
    match enet.receive(buffer) {
        Ok(num_bytes) => {
            // We received ethernet data
            let _ = uwriteln!(uart, "\nReceived {} ethernet bytes", num_bytes)
                .unwrap_or_default();

            // Ethernet header
            let ethernet_header = EthernetHeader::read_bytes(buffer);
            uwriteln!(uart, "{:?}", ethernet_header).unwrap_or_default();

            if ethernet_header.ethertype == EtherType::IpV4
                {
                    // IPV4 packet
                    uwriteln!(uart, "IP packet").unwrap_or_default();
                    let ipheader = IpV4Header::read_bytes(&buffer[EthernetHeader::BYTE_LEN..]);
                    uwriteln!(uart, "{:?}", ipheader).unwrap_or_default();

                    match ipheader.protocol {
                        // UDP packet
                        Protocol::Udp => {
                            uwriteln!(uart, "UDP packet").unwrap_or_default();
                            let udpheader = UdpHeader::read_bytes(&buffer[UDPSTART..]);
                            uwriteln!(uart, "{:?}", udpheader).unwrap_or_default();
                        }
                        _ => {}
                    }
                }
                else if ethernet_header.ethertype == EtherType::Arp {
                    // ARP packet
                    uwriteln!(uart, "ARP packet").unwrap_or_default();

                    let mut bytes = [0_u8; 2];
                    bytes.copy_from_slice(&buffer[16..=17]);
                    let ptypeint = u16::from_be_bytes(bytes);
                    let ptype = EtherType::from(ptypeint);
                    
                    // let arp_incoming = ArpPayload::read_bytes(&buffer[ARPSTART..]);
                    // uwriteln!(uart, "{:?}", &arp_incoming).unwrap_or_default();

                    // // Send an ARP response
                    // let arp_response: EthernetFrame<ArpPayload> = EthernetFrame::<ArpPayload> {
                    //     header: EthernetHeader {
                    //         dst_macaddr: MacAddr::ANY, // Ethernet broadcast
                    //         src_macaddr: udp.src_macaddr,
                    //         ethertype: EtherType::ARP,
                    //     },
                    //     data: ArpPayload::new(
                    //         udp.src_macaddr,
                    //         udp.src_ipaddr,
                    //         arp_incoming.src_mac,
                    //         arp_incoming.src_ipaddr,
                    //         ArpOperation::Response,
                    //     ),
                    //     checksum: 0,
                    // };

                    // match enet.transmit(arp_response.to_be_bytes(), None) {
                    //     // Do not insert IP/UDP checksums for ARP packet
                    //     Ok(_) => {} //uwriteln!(uart, "Sent ARP response").unwrap_or_default(),
                    //     Err(x) => {} //uwriteln!(uart, "Ethernet TX error: {:?}", x).unwrap_or_default(),
                    // };
                }
                else {}
            }
        Err(EthernetError::NothingToReceive) => (),
        Err(x) => writeln!(uart.0, "Ethernet RX error: {:?}", x).unwrap_or_default(),
    };

    // Test UDP transmit
    // match udp.transmit(enet, *b"hello world! ... ... ...") {
    //     Ok(_) => (),
    //     Err(x) => writeln!(uart.0, "UDP TX error: {:?}", x).unwrap_or_default(),
    // };
}

#[no_mangle]
pub fn stellaris_main(mut board: board::Board) -> ! {
    // UART
    let mut pins_a = board.GPIO_PORTA_AHB.split(&board.power_control);
    let pa1 = pins_a.pa1.into_af_push_pull(&mut pins_a.control);
    let pa0 = pins_a.pa0.into_af_push_pull(&mut pins_a.control);
    let mut uart = SerialUWriteable(Serial::uart0(
        board.UART0,
        pa1,
        pa0,
        (),
        (),
        Bps(115200),
        serial::NewlineMode::SwapLFtoCRLF,
        board::clocks(),
        &board.power_control,
    ));

    // Delay timer
    let mut delay = tm4c129x_hal::delay::Delay::new(board.core_peripherals.SYST, board::clocks());

    // LEDs
    let mut loops = 0;
    let mut led_state = 0;

    // Ethernet receive buffer
    let mut buffer = [0_u8; RXBUFSIZE];

    // UDP socket
    let udp = UDPSocket {
        src_macaddr: MacAddr::new(board.enet.src_macaddr),
        src_ipaddr: IpV4Addr::new([10, 0, 0, 229]),
        src_port: 8052,
        dst_macaddr: MacAddr::ANY, // Ethernet broadcast required for IP packets
        dst_ipaddr: IpV4Addr::new([10, 0, 0, 127]),
        dst_port: 8053,
    };



    loop {
        delay.delay_ms(10u32);
    
        poll_ethernet(&mut board.enet, &mut uart, &udp, &mut buffer);

        // Spam serial
        if loops % 10000 == 0 {
            let _ = writeln!(uart.0, "\n\nHello, world! Loops = {}", loops).unwrap_or_default();
            while let Ok(_) = uart.0.read() {
                // Echo
                // uwriteln!(uart, "byte read {}", ch).unwrap_or_default();

                // Show MAC address
                // writemac(&mut uart.0, "Our MAC address: ", &udp.src_macaddr.0);
            }

            // Debugging
            // let rxdl = &mut (board.enet.rxdl);
            // writeln!(uart.0, "{:?}", rxdl).unwrap_or_default();

            // let txdl = &mut (board.enet.txdl);
            // writeln!(uart.0, "{:?}", txdl).unwrap_or_default();

            // Check EMAC status
            // let status = board.enet.emac.status.read().txpaused().bit_is_set();
            // writeln!(uart.0, "EMAC TX paused? {status}").unwrap_or_default();
            // let status = board.enet.emac.hostxdesc.read().bits();
            // writeln!(uart.0, "EMAC TX descr addr {status}").unwrap_or_default();
            // let status = board.enet.emac.hostxba.read().bits();
            // writeln!(uart.0, "EMAC TX buf addr {status}").unwrap_or_default();
            // let status = board.enet.emac.txdladdr.read().bits();
            // writeln!(uart.0, "EMAC TXDL addr {status}").unwrap_or_default();
            // let status = board.enet.emac.status.read().txfe().bit_is_set();
            // writeln!(uart.0, "EMAC TX FIFO not empty? {status}").unwrap_or_default();
            // let status = board.enet.emac.status.read().twc().bit_is_set();
            // writeln!(uart.0, "EMAC TX FIFO write controller active? {status}").unwrap_or_default();
            // let status = board.enet.emac.status.read().tpe().bit_is_set();
            // writeln!(uart.0, "EMAC MII transmit protocol engine status? {status}")
            //     .unwrap_or_default();
            // let status = board.enet.emac.status.read().tfc().variant();
            // writeln!(uart.0, "EMAC MII transmit frame controller status {status:?}")
            //     .unwrap_or_default();
            // let status = board.enet.emac.dmaris.read().ts().bits();
            // writeln!(uart.0, "EMAC DMA transmit process state {status:?}").unwrap_or_default();

            // let status = board.enet.emac.status.read().rrc().bits();
            // writeln!(uart.0, "EMAC RX FIFO controller state? {status:?}").unwrap_or_default();
            // let status = board.enet.emac.hosrxdesc.read().bits();
            // writeln!(uart.0, "EMAC RX descr addr {status}").unwrap_or_default();
            // let status = board.enet.emac.hosrxba.read().bits();
            // writeln!(uart.0, "EMAC RX buf addr {status}").unwrap_or_default();
            // let status = board.enet.emac.rxdladdr.read().bits();
            // writeln!(uart.0, "EMAC RXDL addr {status}").unwrap_or_default();
            // let status = board.enet.emac.status.read().rxf().bits();
            // writeln!(uart.0, "EMAC RX FIFO status? {status:?}").unwrap_or_default();
            // let status = board.enet.emac.status.read().rwc().bit_is_set();
            // writeln!(uart.0, "EMAC RX FIFO write controller active? {status}").unwrap_or_default();
            // let status = board.enet.emac.status.read().rpe().bit_is_set();
            // writeln!(uart.0, "EMAC MII receive protocol engine status? {status}").unwrap_or_default();
            // let status = board.enet.emac.status.read().rfcfc().bits();
            // writeln!(uart.0, "EMAC MII receive frame controller status {status:?}")
            //     .unwrap_or_default();
            // let status = board.enet.emac.dmaris.read().rs().bits();
            // writeln!(uart.0, "EMAC DMA receive process state {status:?}").unwrap_or_default();

            // let status = board.enet.emac.dmaris.read().fbi().bit_is_set();
            // writeln!(uart.0, "EMAC DMA bus error? {status:?}").unwrap_or_default();
            // let status = board.enet.emac.dmaris.read().ae().bits();
            // writeln!(uart.0, "EMAC DMA access error type {status:?}\n\n").unwrap_or_default();
        }

        // board.enet.emacclear(); // Clear clearable interrupts
        loops = loops + 1;

        if loops % 1 == 0 {
            led_state = (led_state + 1) % 4;
        }

        if *(&(board.button0).is_low().unwrap_or_default()) {
            // Turn all the LEDs on
            let _ = &(board.led0).set_high();
            let _ = &(board.led1).set_high();
            let _ = &(board.led2).set_high();
            let _ = &(board.led3).set_high();
        } else {
            // Cycle through each of the 4 LEDs
            if led_state == 0 {
                let _ = &(board.led0).set_low();
                let _ = &(board.led1).set_high();
            } else if led_state == 1 {
                let _ = &(board.led1).set_low();
                let _ = &(board.led2).set_high();
            } else if led_state == 2 {
                let _ = &(board.led2).set_low();
                let _ = &(board.led3).set_high();
            } else if led_state == 3 {
                let _ = &(board.led3).set_low();
                let _ = &(board.led0).set_high();
            } else {
                continue;
            }
        }
        delay.delay_ms(1u32);
    }
}
