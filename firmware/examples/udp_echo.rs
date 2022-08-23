//! An example of (fairly manual) UDP comms using a statically self-assigned IP address

#![no_std]
#![no_main]

extern crate embedded_hal;
extern crate tm4c129_launchpad;
extern crate tm4c129x_hal;

use core::convert::Infallible;
use core::fmt::Write;
use ufmt::*;

// use embedded_hal::prelude::*;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::*; // GPIO set high/low
use embedded_hal::prelude::_embedded_hal_serial_Read;

use tm4c129x_hal::gpio::GpioExt;

use tm4c129x_hal::serial;
use tm4c129x_hal::serial::*;
use tm4c129x_hal::time::Bps;

use catnip::{arp::*, dhcp::*, enet::*, ip::*, udp::*, *};
use tm4c129_launchpad::{
    board,
    drivers::ethernet::{socket::UDPSocket, Ethernet, RXBUFSIZE, RXDESCRS, TXDESCRS},
};

/// Wrapper for UART0 to allow us to use ufmt for panic-never compatible comms
struct SerialUWriteable<UART, TX, RX, RTS, CTS>(Serial<UART, TX, RX, RTS, CTS>);

impl<TX, RX, RTS, CTS> uWrite for SerialUWriteable<UART0, TX, RX, RTS, CTS> {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Infallible> {
        let _ = write!(self.0, "{}", s);
        Ok(())
    }
}

// IP
const IPSTART: usize = EthernetHeader::BYTE_LEN;
const UDPSTART: usize = IPSTART + IpV4Header::BYTE_LEN;
const ARPSTART: usize = IPSTART;
const MAX_ECHO: usize = 160; // Maximum number of bytes of UDP data to echo

const IPADDR_LINK_LOCAL_STATIC: IpV4Addr = ByteArray([169, 254, 1, 229]); // An arbitrary IP address in the link-local block
const IPADDR_DHCP_STATIC: IpV4Addr = ByteArray([10, 0, 0, 229]); // An arbitrary IP address in a typical DHCP block
const DST_IPADDR_STATIC: IpV4Addr = ByteArray([10, 0, 0, 127]); // The IP address of the (probably desktop) machine we are talking to

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
    // let mut delay = tm4c129x_hal::delay::Delay::new(board.core_peripherals.SYST, board::clocks());

    // LEDs
    let mut loops: u64 = 0;
    let mut led_state = 0;

    // Ethernet receive buffer
    let mut buffer = [0_u8; RXBUFSIZE];

    // UDP socket
    let mut udp = UDPSocket {
        src_macaddr: MacAddr::new(board.enet.src_macaddr),
        src_ipaddr: IPADDR_LINK_LOCAL_STATIC,
        src_port: 8052,
        dst_macaddr: MacAddr::BROADCAST,
        dst_ipaddr: DST_IPADDR_STATIC,
        dst_port: 8053,
        id: 0,
    };

    // Send an ARP announcement that we are taking our statically assigned IP address
    let arp_announcement = EthernetFrame::<ArpPayload> {
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

    match &board.enet.transmit(arp_announcement.to_be_bytes()) {
        Ok(_) => {
            let _ = uwriteln!(
                uart,
                "\nSent ARP announcement: \n{:?}",
                &arp_announcement.data
            );
        }
        Err(x) => {
            let _ = uwriteln!(uart, "\nEthernet TX error: {:?}", x);
        }
    };

    // UDP socket for DHCP
    let mut dhcp_socket = UDPSocket {
        src_macaddr: udp.src_macaddr,
        src_ipaddr: IpV4Addr::ANY,
        src_port: DHCP_CLIENT_PORT,
        dst_macaddr: MacAddr::BROADCAST,
        dst_ipaddr: IpV4Addr::BROADCAST,
        dst_port: DHCP_SERVER_PORT,
        id: 5147,
    };

    // Send a DHCP INFORM message to tell the network we are taking our statically assigned address
    let dhcp_inform = DhcpFixedPayload::new_inform(udp.src_ipaddr, udp.src_macaddr, 13517);
    match dhcp_socket.transmit(&mut board.enet, dhcp_inform.to_be_bytes(), None) {
        Ok(_) => {
            let _ = uwriteln!(uart, "\nSent DHCP INFORM: \n{:?}", dhcp_inform);
        }
        Err(x) => {
            let _ = uwriteln!(uart, "\nError sending DHCP INFORM: {:?}", x);
        }
    }

    loop {
        // Check ethernet
        poll_ethernet(&mut board.enet, &mut uart, &mut udp, &mut buffer);

        // Check UART
        while let Ok(ch) = uart.0.read() {
            uwriteln!(uart, "byte read {:?}", ch);
        }

        loops = loops.wrapping_add(1);

        if loops % 10000 == 0 {
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
    }
}


fn poll_ethernet<TX, RX, RTS, CTS>(
    enet: &mut Ethernet,
    uart: &mut SerialUWriteable<UART0, TX, RX, RTS, CTS>,
    udp: &mut UDPSocket,
    buffer: &mut [u8; RXBUFSIZE],
) {
    // Flush the EMAC peripheral's RX and TX buffers
    enet.rxpush();

    // Receive all buffered frames
    while let Ok(num_bytes) = enet.receive(buffer) {
        // We received ethernet data
        // let _ = uwriteln!(uart, "\nReceived {} ethernet bytes", num_bytes);

        // Ethernet header
        let ethernet_header = EthernetHeader::read_bytes(buffer);
        // uwriteln!(uart, "{:?}", ethernet_header);

        match ethernet_header.ethertype {
            EtherType::IpV4 => {
                // IPV4 packet
                let ipheader = IpV4Header::read_bytes(&buffer[EthernetHeader::BYTE_LEN..]);
                // uwriteln!(uart, "{:?}", ipheader);

                match ipheader.protocol {
                    // UDP packet
                    Protocol::Udp => {
                        let udpheader = UdpHeader::read_bytes(&buffer[UDPSTART..]);
                        // uwriteln!(uart, "{:?}", udpheader);

                        match udpheader.dst_port {
                            x if (x == DHCP_CLIENT_PORT) || (x == DHCP_SERVER_PORT) => {
                                // let dhcp_bytes = &buffer[UDPSTART + UdpHeader::BYTE_LEN..];
                                // let dhcp_fixed_payload = DhcpFixedPayload::read_bytes(&dhcp_bytes);
                                // let _ = uwriteln!(uart, "{:?}", dhcp_fixed_payload);
                            }
                            x if (x == udp.src_port) && (ipheader.dst_ipaddr == udp.src_ipaddr) => {
                                // If this is a UDP frame for us on our port for testing, echo it back
                                let frame = EthernetFrame::<
                                    IpV4Frame<UdpFrame<ByteArray<{ MAX_ECHO }>>>,
                                >::read_bytes(buffer);
                                let udp_data = &frame.data.data.data;
                                let udp_len =
                                    frame.data.data.header.length as usize - UdpHeader::BYTE_LEN;

                                let mut data = [0_u8; MAX_ECHO];
                                for (i, x) in udp_data.0.iter().enumerate() {
                                    if i == MAX_ECHO.min(udp_len) {
                                        break;
                                    } else {
                                        data[i] = *x;
                                    };
                                }

                                match udp.transmit_to(
                                    ipheader.src_ipaddr,
                                    ethernet_header.src_macaddr,
                                    enet,
                                    data,
                                    Some(udp_len as u16),
                                ) {
                                    Ok(_) => {}
                                    Err(x) => {
                                        let _ =
                                            uwriteln!(uart, "\nError echoing UDP packet: {:?}", x);
                                    }
                                };

                                // enet.txpush();
                            }
                            _ => {}
                        }
                    }
                    _ => {}
                }
            }
            EtherType::Arp => {
                // ARP packet
                let arp_incoming = ArpPayload::read_bytes(&buffer[ARPSTART..]);
                // uwriteln!(uart, "{:?}", &arp_incoming);

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

                    match enet.transmit(arp_response.to_be_bytes()) {
                        Ok(_) => {}
                        Err(x) => {
                            let _ = uwriteln!(uart, "Ethernet TX error: {:?}", x);
                        }
                    };

                    // If this is the control server, register the mac address so that we no longer broadcast
                    if arp_incoming.src_ipaddr == udp.dst_ipaddr {
                        udp.dst_macaddr = arp_incoming.src_mac;
                    };
                } else {
                    // let _ = uwriteln!(uart, "Skipping response to ARP message that is not for us");
                }
            }
            _ => {}
        };
    }

    // Flush EMAC peripheral's TX buffers
    enet.txpush();
}
