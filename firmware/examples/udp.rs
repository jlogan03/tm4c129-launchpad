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
use tm4c129x_hal::gpio::gpioa::{PA0, PA1};
use tm4c129x_hal::gpio::{AlternateFunction, GpioExt, PushPull, AF1};
use tm4c129x_hal::serial;
use tm4c129x_hal::serial::*;
use tm4c129x_hal::time::Bps;

use catnip::{arp::*, enet::*, ip::*, udp::*, *};
use tm4c129_launchpad::{
    board,
    drivers::ethernet::{socket::UDPSocket, EthernetError, RXBUFSIZE},
};

#[no_mangle]
pub fn stellaris_main(mut board: board::Board) -> ! {
    let mut pins_a = board.GPIO_PORTA_AHB.split(&board.power_control);
    let mut uart = Serial::uart0(
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
        src_macaddr: MacAddr::new(board.enet.src_macaddr),
        src_ipaddr: IpV4Addr::new([10, 0, 0, 229]),
        src_port: 8052,
        dst_macaddr: MacAddr::ANY, // Ethernet broadcast required for IP packets
        dst_ipaddr: IpV4Addr::new([10, 0, 0, 127]),
        dst_port: 8053,
    };

    let mut loops = 0;
    let mut led_state = 0;

    fn writemac<T: Write>(u: &mut T, prefix: &str, v: &[u8; 6]) -> () {
        write!(u, "{prefix}").unwrap_or_default();
        for x in *v {
            write!(u, "{x:x} ").unwrap_or_default();
        }
        writeln!(u, "").unwrap_or_default();
    }

    fn writeip<T: Write>(u: &mut T, prefix: &str, v: &[u8; 4]) -> () {
        write!(u, "{prefix}").unwrap_or_default();
        for x in *v {
            write!(u, "{x:x} ").unwrap_or_default();
        }
        writeln!(u, "").unwrap_or_default();
    }

    let mut b = [0_u8; RXBUFSIZE];

    // Eth
    let mut ethernet_header: EthernetHeader;

    // IP
    const ipstart: usize = EthernetHeader::BYTE_LEN;
    let mut ipheader: IpV4Header;

    // UDP
    const udpstart: usize = ipstart + IpV4Header::BYTE_LEN;
    let mut udpheader: UdpHeader;

    // ARP
    const arpstart: usize = ipstart;
    let mut arp_incoming: ArpPayload;
    let mut arp_response: EthernetFrame<ArpPayload> = EthernetFrame::<ArpPayload> {
        header: EthernetHeader {
            dst_macaddr: MacAddr::ANY, // Ethernet broadcast
            src_macaddr: udp.src_macaddr,
            ethertype: EtherType::ARP,
        },
        data: ArpPayload::new(
            udp.src_macaddr,
            udp.src_ipaddr,
            MacAddr::ANY,
            IpV4Addr::ANY,
            ArpOperation::Response,
        ),
        checksum: 0,
    };

    loop {
        // Test ethernet receive (without UDP socket)
        match &board.enet.receive(&mut b) {
            Ok(num_bytes) => {
                // We received ethernet data
                writeln!(uart, "Received {num_bytes} ethernet bytes").unwrap_or_default();

                // Ethernet header
                ethernet_header = EthernetHeader::read_bytes(&b);

                writemac(&mut uart, "    src macaddr: ", &ethernet_header.src_macaddr);
                writemac(&mut uart, "    dst macaddr: ", &ethernet_header.dst_macaddr);
                writeln!(uart, "Ethertype: {:x}", ethernet_header.ethertype as u16);

                if ethernet_header.ethertype as u16 == 0x800 {
                    // IPV4 packet
                    writeln!(uart, "  IP packet").unwrap_or_default();
                    ipheader = IpV4Header::read_bytes(&b[EthernetHeader::BYTE_LEN..]);
                    // writeln!(uart, "{:?}", ipheader).unwrap_or_default();
                    delay.delay_ms(2000u32);

                    match ipheader.protocol {
                        // UDP packet
                        Protocol::UDP => {
                            udpheader = UdpHeader::read_bytes(&b[udpstart..]);
                            // writeln!(uart, "{:?}", udpheader);
                        }
                        _ => {}
                    }
                } else if ethernet_header.ethertype as u16 == 0x806 {
                    // ARP packet
                    writeln!(uart, "  ARP packet").unwrap_or_default();
                    arp_incoming = ArpPayload::read_bytes(&b[arpstart..]);
                    // writeln!(uart, "{:?}", &arp_incoming).unwrap_or_default();

                    // Send an ARP response
                    arp_response.data.dst_ipaddr = arp_incoming.src_ipaddr;
                    arp_response.data.dst_mac = arp_incoming.src_mac;

                    match board.enet.transmit(arp_response.to_be_bytes(), None) {
                        // Do not insert IP/UDP checksums for ARP packet
                        Ok(_) => writeln!(uart, "Sent ARP response").unwrap_or_default(),
                        Err(x) => writeln!(uart, "Ethernet TX error: {:?}", x).unwrap_or_default(),
                    };
                } else {
                    // Ignore TCP, etc
                    continue;
                }
            }
            Err(EthernetError::NothingToReceive) => (),
            Err(x) => writeln!(uart, "Ethernet RX error: {:?}", x).unwrap_or_default(),
        };

        // Test UDP transmit
        match udp.transmit(&mut board.enet, *b"hello world! ... ... ...") {
            Ok(_) => (), //writeln!(uart, "UDP transmit started").unwrap_or_default(),
            Err(x) => writeln!(uart, "UDP TX error: {:?}", x).unwrap_or_default(),
        };

        // Spam serial
        if loops % 10000 == 0 {
            writeln!(uart, "\n\nHello, world! Loops = {}", loops).unwrap_or_default();
            while let Ok(ch) = uart.read() {
                // Echo
                writeln!(uart, "byte read {}", ch).unwrap_or_default();

                // Show MAC address
                writemac(&mut uart, "Our MAC address: ", &udp.src_macaddr.0);
            }

            // Debugging
            let rxdl = &mut (board.enet.rxdl);
            writeln!(uart, "{:?}", rxdl).unwrap_or_default();

            let txdl = &mut (board.enet.txdl);
            writeln!(uart, "{:?}", txdl).unwrap_or_default();

            // Check EMAC status
            let status = board.enet.emac.status.read().txpaused().bit_is_set();
            writeln!(uart, "EMAC TX paused? {status}").unwrap_or_default();
            let status = board.enet.emac.hostxdesc.read().bits();
            writeln!(uart, "EMAC TX descr addr {status}").unwrap_or_default();
            let status = board.enet.emac.hostxba.read().bits();
            writeln!(uart, "EMAC TX buf addr {status}").unwrap_or_default();
            let status = board.enet.emac.txdladdr.read().bits();
            writeln!(uart, "EMAC TXDL addr {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().txfe().bit_is_set();
            writeln!(uart, "EMAC TX FIFO not empty? {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().twc().bit_is_set();
            writeln!(uart, "EMAC TX FIFO write controller active? {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().tpe().bit_is_set();
            writeln!(uart, "EMAC MII transmit protocol engine status? {status}")
                .unwrap_or_default();
            let status = board.enet.emac.status.read().tfc().variant();
            writeln!(uart, "EMAC MII transmit frame controller status {status:?}")
                .unwrap_or_default();
            let status = board.enet.emac.dmaris.read().ts().bits();
            writeln!(uart, "EMAC DMA transmit process state {status:?}").unwrap_or_default();

            let status = board.enet.emac.status.read().rrc().bits();
            writeln!(uart, "EMAC RX FIFO controller state? {status:?}").unwrap_or_default();
            let status = board.enet.emac.hosrxdesc.read().bits();
            writeln!(uart, "EMAC RX descr addr {status}").unwrap_or_default();
            let status = board.enet.emac.hosrxba.read().bits();
            writeln!(uart, "EMAC RX buf addr {status}").unwrap_or_default();
            let status = board.enet.emac.rxdladdr.read().bits();
            writeln!(uart, "EMAC RXDL addr {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().rxf().bits();
            writeln!(uart, "EMAC RX FIFO status? {status:?}").unwrap_or_default();
            let status = board.enet.emac.status.read().rwc().bit_is_set();
            writeln!(uart, "EMAC RX FIFO write controller active? {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().rpe().bit_is_set();
            writeln!(uart, "EMAC MII receive protocol engine status? {status}").unwrap_or_default();
            let status = board.enet.emac.status.read().rfcfc().bits();
            writeln!(uart, "EMAC MII receive frame controller status {status:?}")
                .unwrap_or_default();
            let status = board.enet.emac.dmaris.read().rs().bits();
            writeln!(uart, "EMAC DMA receive process state {status:?}").unwrap_or_default();

            let status = board.enet.emac.dmaris.read().fbi().bit_is_set();
            writeln!(uart, "EMAC DMA bus error? {status:?}").unwrap_or_default();
            let status = board.enet.emac.dmaris.read().ae().bits();
            writeln!(uart, "EMAC DMA access error type {status:?}\n\n").unwrap_or_default();
        }

        // board.enet.emacclear(); // Clear clearable interrupts
        loops = loops + 1;

        if loops % 50 == 0 {
            led_state = (led_state + 1) % 4;
        }

        if *(&(board.button0).is_low().unwrap_or_default()) {
            // Turn all the LEDs on
            &(board.led0).set_high().unwrap_or_default();
            &(board.led1).set_high().unwrap_or_default();
            &(board.led2).set_high().unwrap_or_default();
            &(board.led3).set_high().unwrap_or_default();
        } else {
            // Cycle through each of the 4 LEDs
            if led_state == 0 {
                &(board.led0).set_low().unwrap_or_default();
                &(board.led1).set_high().unwrap_or_default();
            } else if led_state == 1 {
                &(board.led1).set_low().unwrap_or_default();
                &(board.led2).set_high().unwrap_or_default();
            } else if led_state == 2 {
                &(board.led2).set_low().unwrap_or_default();
                &(board.led3).set_high().unwrap_or_default();
            } else if led_state == 3 {
                &(board.led3).set_low().unwrap_or_default();
                &(board.led0).set_high().unwrap_or_default();
            } else {
                continue;
            }
        }
        delay.delay_ms(1u32);
    }
}
