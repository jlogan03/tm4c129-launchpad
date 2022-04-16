//! Build a UDP/IP Ethernet packet and get its representation as network bytes

use catnip::enet::{EtherType, EthernetFrameUDP, EthernetHeader};
use catnip::ip::{IPV4Header, DSCP};
use catnip::udp::{UDPHeader, UDPPacket};
use catnip::{Data, MACAddr, IPV4Addr};

use super::{EthernetDriver, EthernetError};

/// Thin adapter layer to generate UDP packets without constantly passing the address info around
pub struct UDPSocket {
    /// Source MAC address
    pub src_macaddr: MACAddr,
    /// Source IP address
    pub src_ipaddr: IPV4Addr,
    /// Source port
    pub src_port: u16,
    /// Destination MAC address (can be None, and filled in by the network switch)
    pub dst_macaddr: Option<MACAddr>,
    /// Destination IP address
    pub dst_ipaddr: IPV4Addr,
    /// Destination port
    pub dst_port: u16,
}

impl UDPSocket {
    /// Build a UDP packet from data & pass it to the ethernet driver to transmit in hardware
    pub fn transmit<const M: usize>(
        &self,
        enet: &mut EthernetDriver,
        data: [u8; 4 * M]
    ) -> Result<(), EthernetError>
    where
        [u8; 4 * M]:,
        [u8; 4 * 0]:,
        [u8; 4 * 0 + 20]:,
        [u8; 4 * 0 + 20 + 4 * M + 8]:,
        [u8; (4 * 0 + 20) + (4 * M) + 14 + 8]:,
    {
        let data: Data<M> = Data { value: data };
        let frame: [u8; (4 * 0 + 20) + (4 * M) + 14 + 8] = frame(
            data,
            self.src_macaddr,
            self.src_ipaddr,
            self.src_port,
            self.dst_macaddr,
            self.dst_ipaddr,
            self.dst_port,
        )
        .to_be_bytes();

        unsafe { enet.transmit(frame) }
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
    let mut ipheader: IPV4Header<0> = IPV4Header::new(); // IP header with no Options segment
    ipheader.src_ipaddr(src_ipaddr)
        .dst_ipaddr(dst_ipaddr)
        .ttl(100)
        .dscp(DSCP::Realtime);
    let udpheader: UDPHeader = UDPHeader::new(src_port, dst_port);
    let udppacket: UDPPacket<0, M> = UDPPacket {
        // M words of data and 0 words of Options
        ip_header: ipheader,
        udp_header: udpheader,
        udp_data: data,
    }; // Populates packet length fields for both IP and UDP headers
    let enetheader: EthernetHeader = EthernetHeader::new(src_macaddr, dst_macaddr, EtherType::IPV4);
    let enetframe: EthernetFrameUDP<0, M> = EthernetFrameUDP::new(enetheader, udppacket);

    return enetframe;
}
