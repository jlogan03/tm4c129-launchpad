//! Build a UDP/IP Ethernet packet and get its representation as network bytes

use catnip::enet::{EtherType, EthernetFrameUDP, EthernetHeader};
use catnip::ip::{IPV4Addr, IPV4Header, DSCP};
use catnip::udp::{UDPHeader, UDPPacket};
use catnip::{Data, MACAddr};

/// Thin adapter layer to generate UDP packets without constantly passing the address info in
pub struct UDPSocket<const M: usize, F>
where
    F: Fn([u8; (4 * 0 + 20) + (4 * M) + 14 + 8]) -> Result<(), ()>,
    [(); 4 * M]:,
{
    tfunc: F,
    src_macaddr: MACAddr,
    src_ipaddr: IPV4Addr,
    src_port: u16,
    dst_macaddr: Option<MACAddr>,
    dst_ipaddr: IPV4Addr,
    dst_port: u16,
}

impl<const M: usize, F> UDPSocket<M, F>
where
    F: Fn([u8; (4 * 0 + 20) + (4 * M) + 14 + 8]) -> Result<(), ()>,
    [u8; 4 * M]:,
    [u8; 4 * 0]:,
    [u8; 4 * 0 + 20]:,
    [u8; 4 * 0 + 20 + 4 * M + 8]:,
    [u8; (4 * 0 + 20) + (4 * M) + 14 + 8]:,
{

    /// Build a UDP packet from data & pass it to the ethernet driver to transmit in hardware
    pub fn transmit(&self, data: [u8; 4 * M]) -> Result<(), ()> {
        let data: Data<M> = Data { value: data };
        let frame: [u8; (4 * 0 + 20) + (4 * M) + 14 + 8] = frame(
            data,
            self.src_macaddr,
            self.src_ipaddr,
            self.src_port,
            self.dst_macaddr,
            self.dst_ipaddr,
            self.dst_port,
        ).to_be_bytes();

        (self.tfunc)(frame)
    }
}

/// Build a standard UDP frame, including zeroed-out checksums which will be replaced by the hardware
/// 
pub fn frame<const M: usize>(
    data: Data<M>,
    src_macaddr: MACAddr,
    src_ipaddr: IPV4Addr,
    src_port: u16,
    dst_macaddr: Option<MACAddr>,
    dst_ipaddr: IPV4Addr,
    dst_port: u16,
) -> EthernetFrameUDP<0, M>
where
    [u8; 4 * 0]:,
    [u8; 4 * 0 + 20]:,
    [u8; 4 * 0 + 20 + 4 * M + 8]:,
    [u8; (4 * 0 + 20) + (4 * M) + 14 + 8]:,
{
    let ipheader: IPV4Header<0> = IPV4Header::new() // IP header with no Options segment
        .src_ipaddr(src_ipaddr)
        .dst_ipaddr(dst_ipaddr)
        .dscp(DSCP::Realtime)
        .finalize();
    let udpheader: UDPHeader = UDPHeader::new(src_port, dst_port);
    let udppacket: UDPPacket<0, M> = UDPPacket {
        // N words of data and no Options
        ip_header: ipheader,
        udp_header: udpheader,
        udp_data: data,
    }; // Populates packet length fields for both IP and UDP headers
    let enetheader: EthernetHeader = EthernetHeader::new(src_macaddr, dst_macaddr, EtherType::IPV4);
    let enetframe: EthernetFrameUDP<0, M> = EthernetFrameUDP::new(enetheader, udppacket);
    // let enetpacket = EthernetPacketUDP::new(enetframe);

    return enetframe;
}
