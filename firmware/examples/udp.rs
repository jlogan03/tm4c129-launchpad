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

use tm4c129x_hal::gpio::GpioExt;

use tm4c129x_hal::serial;
use tm4c129x_hal::serial::*;
use tm4c129x_hal::time::Bps;

use catnip::{arp::*, enet::*, ip::*, udp::*, *};
use tm4c129_launchpad::{
    board,
    drivers::ethernet::{socket::UDPSocket, EthernetDriver, RXBUFSIZE},
};

struct SerialUWriteable<UART, TX, RX, RTS, CTS>(Serial<UART, TX, RX, RTS, CTS>);

impl<TX, RX, RTS, CTS> uWrite for SerialUWriteable<UART0, TX, RX, RTS, CTS> {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Infallible> {
        let _ = write!(self.0, "{}", s).unwrap_or_default();
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
    buffer: &mut [u8; RXBUFSIZE],
) {
    // Receive ethernet bytes
    while let Ok(num_bytes) = enet.receive(buffer) {
        // We received ethernet data
        let _ = uwriteln!(uart, "\nReceived {} ethernet bytes", num_bytes).unwrap_or_default();

        // Ethernet header
        let ethernet_header = EthernetHeader::read_bytes(buffer);
        uwriteln!(uart, "{:?}", ethernet_header).unwrap_or_default();

        match ethernet_header.ethertype {
            EtherType::IpV4 => {
                // IPV4 packet
                let ipheader = IpV4Header::read_bytes(&buffer[EthernetHeader::BYTE_LEN..]);
                uwriteln!(uart, "{:?}", ipheader).unwrap_or_default();

                match ipheader.protocol {
                    // UDP packet
                    Protocol::Udp => {
                        let udpheader = UdpHeader::read_bytes(&buffer[UDPSTART..]);
                        uwriteln!(uart, "{:?}", udpheader).unwrap_or_default();
                    }
                    _ => {}
                }
            }
            EtherType::Arp => {
                // ARP packet
                let arp_incoming = ArpPayload::read_bytes(&buffer[ARPSTART..]);
                uwriteln!(uart, "{:?}", &arp_incoming).unwrap_or_default();

                // Send an ARP response
                if arp_incoming.dst_ipaddr == udp.src_ipaddr {
                    let arp_response: EthernetFrame<ArpPayload> = EthernetFrame::<ArpPayload> {
                        header: EthernetHeader {
                            dst_macaddr: arp_incoming.src_mac, // Ethernet broadcast
                            src_macaddr: udp.src_macaddr,
                            ethertype: EtherType::Arp,
                        },
                        data: ArpPayload::new(
                            udp.src_macaddr,
                            udp.src_ipaddr,
                            arp_incoming.src_mac,
                            arp_incoming.src_ipaddr,
                            ArpOperation::Response,
                        ),
                        checksum: 0,
                    };

                    match enet.transmit(arp_response.to_be_bytes(), None) {
                        // Do not insert IP/UDP checksums for ARP packet
                        Ok(_) => {
                            let _ =
                                uwriteln!(uart, "Sent ARP response: \n{:?}", &arp_response.data)
                                    .unwrap_or_default();
                        }
                        Err(x) => {
                            let _ =
                                uwriteln!(uart, "Ethernet TX error: {:?}", x).unwrap_or_default();
                        }
                    };
                } else {
                    let _ = uwriteln!(uart, "Skipping response to ARP message that is not for us")
                        .unwrap_or_default();
                }
            }
            _ => {}
        };
    }
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
    let mut loops: u64 = 0;
    let mut led_state = 0;

    // Ethernet receive buffer
    let mut buffer = [0_u8; RXBUFSIZE];

    // UDP socket
    let mut udp = UDPSocket {
        src_macaddr: MacAddr::new(board.enet.src_macaddr),
        src_ipaddr: IpV4Addr::new([10, 0, 0, 229]),
        src_port: 8052,
        dst_ipaddr: IpV4Addr::new([10, 0, 0, 127]),
        dst_port: 8052,
        id: 0
    };

    // Send an ARP announcement
    let arp_announcement: EthernetFrame<ArpPayload> = EthernetFrame::<ArpPayload> {
        header: EthernetHeader {
            dst_macaddr: MacAddr::BROADCAST, // Ethernet broadcast
            src_macaddr: udp.src_macaddr,
            ethertype: EtherType::Arp,
        },
        data: ArpPayload::new(
            udp.src_macaddr,
            udp.src_ipaddr,
            MacAddr::ANY,
            udp.src_ipaddr,
            ArpOperation::Request,
        ),
        checksum: 0,
    };

    match &board
        .enet
        .transmit(arp_announcement.to_be_bytes(), None)//Some(TDES0::CicFrameOnly))
    {
        // Do not insert IP/UDP checksums for ARP packet
        Ok(_) => {
            let _ = uwriteln!(
                uart,
                "\nSent ARP announcement: {:?}",
                &arp_announcement.data
            )
            .unwrap_or_default();
        }
        Err(x) => {
            let _ = uwriteln!(uart, "\nEthernet TX error: {:?}", x).unwrap_or_default();
        }
    };

    drop(arp_announcement);

    loop {
        let dt = 10_u32;
        delay.delay_ms(10u32);
        poll_ethernet(&mut board.enet, &mut uart, &udp, &mut buffer);

        // Test UDP transmit once per second
        if loops % (1000 / dt as u64) == 0 {
            match udp.transmit(&mut board.enet, *b"hello world! ... ... ...") {
                Ok(_) => {let _ = uwriteln!(uart, "\nSent UDP frame ID {:?}", &udp.id).unwrap_or_default();},
                Err(x) => {let _ = uwriteln!(uart, "UDP TX error: {:?}", x).unwrap_or_default();},
            };
        }

        // Spam serial
        if loops % 1000 == 0 {
            let _ = writeln!(uart.0, "\n\nLoops = {}", loops).unwrap_or_default();
            // while let Ok(_) = uart.0.read() {
            // writeln!(uart.0, "byte read {}", ch).unwrap_or_default();
            // }

            // Debugging
            let rxdl = &mut (board.enet.rxdl);
            writeln!(uart.0, "{:?}", rxdl).unwrap_or_default();

            let txdl = &mut (board.enet.txdl);
            writeln!(uart.0, "{:?}", txdl).unwrap_or_default();

            // Check EMAC status
            let status = board.enet.emac.status.read().txpaused().bit_is_set();
            writeln!(uart.0, "EMAC TX paused? {status}").unwrap_or_default();
            let status = board.enet.emac.hostxdesc.read().bits();
            writeln!(uart.0, "EMAC TX descr addr {status}").unwrap_or_default();
            let status = board.enet.emac.hostxba.read().bits();
            writeln!(uart.0, "EMAC TX buf addr {status}").unwrap_or_default();
            let status = board.enet.emac.txdladdr.read().bits();
            writeln!(uart.0, "EMAC TXDL addr {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().txfe().bit_is_set();
            writeln!(uart.0, "EMAC TX FIFO not empty? {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().twc().bit_is_set();
            writeln!(uart.0, "EMAC TX FIFO write controller active? {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().tpe().bit_is_set();
            writeln!(uart.0, "EMAC MII transmit protocol engine status? {status}")
                .unwrap_or_default();
            let status = board.enet.emac.status.read().tfc().variant();
            writeln!(uart.0, "EMAC MII transmit frame controller status {status:?}")
                .unwrap_or_default();
            let status = board.enet.emac.dmaris.read().ts().bits();
            writeln!(uart.0, "EMAC DMA transmit process state {status:?}").unwrap_or_default();

            let status = board.enet.emac.status.read().rrc().bits();
            writeln!(uart.0, "EMAC RX FIFO controller state? {status:?}").unwrap_or_default();
            let status = board.enet.emac.hosrxdesc.read().bits();
            writeln!(uart.0, "EMAC RX descr addr {status}").unwrap_or_default();
            let status = board.enet.emac.hosrxba.read().bits();
            writeln!(uart.0, "EMAC RX buf addr {status}").unwrap_or_default();
            let status = board.enet.emac.rxdladdr.read().bits();
            writeln!(uart.0, "EMAC RXDL addr {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().rxf().bits();
            writeln!(uart.0, "EMAC RX FIFO status? {status:?}").unwrap_or_default();
            let status = board.enet.emac.status.read().rwc().bit_is_set();
            writeln!(uart.0, "EMAC RX FIFO write controller active? {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().rpe().bit_is_set();
            writeln!(uart.0, "EMAC MII receive protocol engine status? {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().rfcfc().bits();
            writeln!(uart.0, "EMAC MII receive frame controller status {status:?}")
                .unwrap_or_default();
            let status = board.enet.emac.dmaris.read().rs().bits();
            writeln!(uart.0, "EMAC DMA receive process state {status:?}").unwrap_or_default();

            let status = board.enet.emac.dmaris.read().fbi().bit_is_set();
            writeln!(uart.0, "EMAC DMA bus error? {status:?}").unwrap_or_default();
            let status = board.enet.emac.dmaris.read().ae().bits();
            writeln!(uart.0, "EMAC DMA access error type {status:?}\n\n").unwrap_or_default();
        }

        // board.enet.emacclear(); // Clear clearable interrupts
        loops = loops.wrapping_add(1);

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
        // delay.delay_ms(1u32);
    }
}
