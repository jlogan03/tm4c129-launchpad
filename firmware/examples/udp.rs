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

use catnip::{IPV4Addr, MACAddr};
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
        src_macaddr: MACAddr {
            value: board.enet.src_macaddr,
        },
        src_ipaddr: IPV4Addr {
            value: [10, 0, 0, 229],
        },
        src_port: 8052,
        dst_macaddr: Some(MACAddr {
            value: [0xFF_u8; 6], // Ethernet broadcast required for IP packets
        }),
        dst_ipaddr: IPV4Addr {
            value: [10, 0, 0, 127],
        },
        // dst_ipaddr: IPV4Addr {
        //     value: [0, 0, 0, 0],
        // },
        dst_port: 8053,
    };

    let mut loops = 0;
    let mut led_state = 0;

    let mut writemac = |u: &mut Serial<
        UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
                        prefix: &str,
                        v: &[u8; 6]| {
        // let _ = writeln!(uart, "");
        let _ = write!(u, "{prefix}");
        for x in *v {
            let _ = write!(u, "{x:x} ");
        }
        let _ = writeln!(u, "");
    };

    let mut writeip = |u: &mut Serial<
        UART0,
        PA1<AlternateFunction<AF1, PushPull>>,
        PA0<AlternateFunction<AF1, PushPull>>,
        (),
        (),
    >,
                       prefix: &str,
                       v: &[u8; 4]| {
        // let _ = writeln!(uart, "");
        let _ = write!(u, "{prefix}");
        for x in *v {
            let _ = write!(u, "{x} ");
        }
        let _ = writeln!(u, "");
    };

    let mut b = [0_u8; RXBUFSIZE];

    // Eth
    let mut srcmac = [0_u8; 6];
    let mut dstmac = [0_u8; 6];
    let mut srcip = [0_u8; 4];
    let mut dstip = [0_u8; 4];
    let mut ethertype = 0_u16;
    let mut tpi = 0_u16;
    let mut vid = 0_u16;
    let mut tpibytes = [0_u8; 2];
    let mut vidbytes = [0_u8; 2];
    let mut etbytes = [0_u8; 2];

    const untagged: usize = 14;
    const tagged: usize = 18;

    // IP
    let mut protocol = 0_u8;
    let mut ippacketlen = [0_u8; 2];
    let mut version = 0_u8;
    let mut ipheaderlen = 14_u8;

    // ARP
    let mut htypebytes = [0_u8; 2];
    let mut ptypebytes = [0_u8; 2];
    let mut operbytes = [0_u8; 2];
    let mut sha = [0_u8; 6];
    let mut spa = [0_u8; 4];
    let mut tha = [0_u8; 6];
    let mut tpa = [0_u8; 4];
    let mut arppacket = [0_u8; 28];

    loop {
        // Test ethernet receive (without UDP socket)
        match &board.enet.receive(&mut b) {
            Ok(num_bytes) => {
                // We received ethernet data
                let _ = writeln!(uart, "Received {num_bytes} ethernet bytes");

                // Ethernet header
                dstmac.copy_from_slice(&b[0..=5]);
                srcmac.copy_from_slice(&b[6..=11]);

                etbytes.copy_from_slice(&b[12..=13]);
                ethertype = u16::from_be_bytes(etbytes);

                writemac(&mut uart, "    src macaddr: ", &srcmac);
                writemac(&mut uart, "    dst macaddr: ", &dstmac);

                let offs = untagged; // Assume untagged eth header length

                // Handle vlan
                if ethertype == 0x8100 {
                    // VLAN tagged
                    tpibytes.copy_from_slice(&b[12..=13]);
                    tpi = u16::from_be_bytes(etbytes);

                    vidbytes.copy_from_slice(&b[14..=15]);
                    vid = u16::from_be_bytes(vidbytes) & 0b0000_1111_1111_1111;

                    etbytes.copy_from_slice(&b[16..=17]);
                    ethertype = u16::from_be_bytes(etbytes);

                    let _ = writeln!(uart, "    VLAN TPI: 0x{tpi:x}");
                    let _ = writeln!(uart, "    VLAN VID: {vid}");
                }

                let _ = writeln!(uart, "    ethertype: 0x{ethertype:x}");

                if ethertype == 0x800 {
                    // IPV4 packet
                    let _ = writeln!(uart, "  IP packet");
                    srcip.copy_from_slice(&b[offs + 12..=offs + 15]);
                    dstip.copy_from_slice(&b[offs + 16..=offs + 19]);
                    protocol = (&b)[offs + 9];
                    ippacketlen.copy_from_slice(&b[offs + 2..=offs + 3]);
                    let iplen_u16 = u16::from_be_bytes(ippacketlen);
                    version = (b[offs] & 0b1111_0000) >> 4;
                    ipheaderlen = b[offs] & 0b0000_1111_u8;

                    writeip(&mut uart, "    src ipaddr: ", &srcip);
                    writeip(&mut uart, "    dst ipaddr: ", &dstip);
                    let _ = writeln!(uart, "    Protocol: {protocol}");
                    let _ = writeln!(uart, "    IP version: {version}");
                    let _ = writeln!(uart, "    IP header length: {ipheaderlen} words");
                    let _ = writeln!(uart, "    IP packet length: {iplen_u16} bytes");
                    delay.delay_ms(2000u32);
                } else if ethertype == 0x806 {
                    // ARP packet
                    let _ = writeln!(uart, "  ARP packet");
                    arppacket.copy_from_slice(&b[offs..offs + 28]);

                    htypebytes.copy_from_slice(&arppacket[0..=1]);
                    let _ = writeln!(
                        uart,
                        "    ARP hardware type: {}",
                        u16::from_be_bytes(htypebytes)
                    );

                    ptypebytes.copy_from_slice(&arppacket[2..=3]);
                    let _ = writeln!(
                        uart,
                        "    ARP protocol type: 0x{:x}",
                        u16::from_be_bytes(ptypebytes)
                    );

                    let _ = writeln!(uart, "    ARP hardware address length: {}", &arppacket[4]);
                    let _ = writeln!(uart, "    ARP protocol address length: {}", &arppacket[5]);

                    operbytes.copy_from_slice(&arppacket[6..=7]);
                    let _ = writeln!(
                        uart,
                        "    ARP operation type: {}",
                        u16::from_be_bytes(operbytes)
                    );

                    sha.copy_from_slice(&arppacket[8..14]);
                    writemac(&mut uart, "    ARP sender hardware address: ", &sha);

                    spa.copy_from_slice(&arppacket[14..18]);
                    writeip(&mut uart, "    ARP sender protocol address: ", &spa);

                    tha.copy_from_slice(&arppacket[18..24]);
                    writemac(&mut uart, "    ARP target harware address: ", &tha);

                    tpa.copy_from_slice(&arppacket[24..28]);
                    writeip(&mut uart, "    ARP target protocol address: ", &tpa);

                    // Send an ARP response
                    let enetheader = catnip::enet::EthernetHeader::new(
                        MACAddr {
                            value: board.enet.src_macaddr,
                        },
                        Some(MACAddr {
                            value: [0xFF_u8; 6], // Ethernet broadcast required for IP packets
                        }),
                        catnip::EtherType::ARP,
                    );

                    let mut arpresponse = [0_u8; 46];  // 28 bytes padded to ethernet minimum payload size
                    let arpparts = [
                        &1_u16.to_be_bytes()[..],     // hardware type ethernet
                        &0x800_u16.to_be_bytes()[..], // IPV4 protocol
                        &[6_u8][..],                  // 6 byte mac addresses
                        &[4_u8][..],                  // 4 byte ip addresses
                        &2_u16.to_be_bytes()[..],     // Operation type: 1 => request, 2 => response
                        &board.enet.src_macaddr[..],
                        &udp.src_ipaddr.value[..],
                        &sha[..],
                        // &[0_u8; 6],
                        &spa[..],
                    ];
                    let mut k = 0;
                    for i in 0..arpparts.len() {
                        for j in 0..arpparts[i].len() {
                            if k < arpresponse.len() {
                                arpresponse[k] = arpparts[i][j];
                            }
                            k = k + 1;
                        }
                    }
                    // let _ = writeln!(uart, "wrote {k} bytes to ARP packet");

                    let mut eth_arp_frame = [0_u8; 64]; // Minimum ethernet frame length
                    let eth_arp_parts = [
                        &enetheader.to_be_bytes()[..],
                        &arpresponse[..],
                        // &[0_u8; 64 - 14 - 28 - 4], // Padding to reach minimum frame size
                        // &[0_u8; 4][..], // Empty ethernet checksum to be populated by the crc peripheral
                    ];
                    let mut k = 0;
                    for i in 0..eth_arp_parts.len() {
                        for j in 0..eth_arp_parts[i].len() {
                            if k < eth_arp_frame.len() {
                                eth_arp_frame[k] = eth_arp_parts[i][j];
                            }
                            k = k + 1;
                        }
                    }

                    match board.enet.transmit(eth_arp_frame) {
                        Ok(_) => writeln!(uart, "Sent ARP response"),
                        Err(x) => writeln!(uart, "Ethernet TX error: {:?}", x),
                    };
                } else {
                    continue;
                }
            }
            Err(EthernetError::NothingToReceive) => (),
            Err(x) => writeln!(uart, "Ethernet RX error: {:?}", x).unwrap_or_default(),
        };

        // Test UDP transmit
        match udp.transmit::<3>(&mut board.enet, *b"hello world!") {
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
                let addr = udp.src_macaddr.value;
                writeln!(
                    uart,
                    "MAC Address: {:x}:{:x}:{:x}:{:x}:{:x}:{:x}",
                    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]
                )
                .unwrap_or_default();
            }

            // Debugging
            let rxdl = &mut (board.enet.rxdl);
            writeln!(uart, "{:?}", rxdl).unwrap_or_default();

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
            writeln!(uart, "EMAC DMA access error type {status:?}\n\n");
        }

        // board.enet.emacclear(); // Clear clearable interrupts
        loops = loops + 1;

        if loops % 50 == 0 {
            led_state = (led_state + 1) % 4;
        }

        if *(&(board.button0).is_low().unwrap_or_default()) {
            // Turn all the LEDs on
            let _ = &(board.led0).set_high().unwrap_or_default();
            let _ = &(board.led1).set_high().unwrap_or_default();
            let _ = &(board.led2).set_high().unwrap_or_default();
            let _ = &(board.led3).set_high().unwrap_or_default();
        } else {
            // Cycle through each of the 4 LEDs
            if led_state == 0 {
                let _ = &(board.led0).set_low().unwrap_or_default();
                let _ = &(board.led1).set_high().unwrap_or_default();
            } else if led_state == 1 {
                let _ = &(board.led1).set_low().unwrap_or_default();
                let _ = &(board.led2).set_high().unwrap_or_default();
            } else if led_state == 2 {
                let _ = &(board.led2).set_low().unwrap_or_default();
                let _ = &(board.led3).set_high().unwrap_or_default();
            } else if led_state == 3 {
                let _ = &(board.led3).set_low().unwrap_or_default();
                let _ = &(board.led0).set_high().unwrap_or_default();
            } else {
                continue;
            }
        }
        delay.delay_ms(1u32);
    }
}
