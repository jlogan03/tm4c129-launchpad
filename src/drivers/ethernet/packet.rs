//! Build a UDP/IP Ethernet packet and get its representation as network bytes

use catnip::enet::{EtherType, EthernetFrameUDP, EthernetHeader, EthernetPacketUDP};
use catnip::ip::{IPV4Addr, IPV4Header, DSCP};
use catnip::udp::{UDPHeader, UDPPacket};
use catnip::{Data, MACAddr};


/// Build a standard UDP packet, including zeroed-out checksums which will be replaced by the hardware
/// and a 96-bit preable segment which is not expected by the hardware and should be removed
pub fn packet<const N: usize>(enet: EthernetDriver, data: Data<N>) -> EthernetPacketUDP {
    let src_macaddr: MACAddr = MACAddr {
        value: enet.src_macaddr,
    };
    let src_ipaddr: IPV4Addr = IPV4Addr {
        value: enet.src_ipaddr,
    };
    let src_port: u16 = enet.src_port;
    let dst_port: u16 = enet.dst_port;
    let dst_ipaddr: IPV4Addr = IPV4Addr {
        value: enet.dst_ipaddr,
    };

    let ipheader: IPV4Header<0> = IPV4Header::new()  // IP header with no Options segment
        .src_ipaddr(src_ipaddr)
        .src_macaddr(src_macaddr)
        .src_port(src_port)
        .dst_ipaddr(dst_ipaddr)
        .dst_port(dst_port)
        .dst_macaddr(None)
        .dscp(DSCP::Realtime)
        .finalize();
    let udpheader: UDPHeader = UDPHeader::new(src_port, dst_port);
    let udppacket: UDPPacket<0, N> = UDPPacket {  // N words of data and no Options
        ip_header: ipheader,
        udp_header: udpheader,
        udp_data: data,
    }; // Populates packet length fields for both IP and UDP headers
    let enetheader: EthernetHeader = EthernetHeader::new(src_macaddr, dst_macaddr, EtherType::IPV4);
    let enetframe = EthernetFrameUDP::new(enetheader, udppacket);
    let enetpacket = EthernetPacketUDP::new(enetframe);

    return enetpacket
}
