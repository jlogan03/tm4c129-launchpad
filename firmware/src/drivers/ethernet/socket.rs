//! Build a UDP/IP Ethernet packet and get its representation as network bytes

use catnip::{enet::*, ip::*, udp::*, *};

use super::tdes::TDES0;
use super::{EthernetDriver, EthernetError};

/// Thin adapter layer to generate UDP packets without constantly passing the address info around
pub struct UDPSocket {
    /// Source MAC address
    pub src_macaddr: MacAddr,
    /// Source IP address
    pub src_ipaddr: IpV4Addr,
    /// Source port
    pub src_port: u16,
    /// Destination MAC address (can be None, and filled in by the network switch)
    pub dst_macaddr: MacAddr,
    /// Destination IP address
    pub dst_ipaddr: IpV4Addr,
    /// Destination port
    pub dst_port: u16,
}

impl UDPSocket {
    /// Build a UDP packet from data & pass it to the ethernet driver to transmit in hardware
    pub fn transmit<const N: usize>(
        &self,
        enet: &mut EthernetDriver,
        data: [u8; N],
    ) -> Result<(), EthernetError>
    where
        [u8; EthernetFrame::<IpV4Frame<UdpFrame<ByteArray<N>>>>::BYTE_LEN]:,
    {
        let frame = build_frame(
            data,
            self.src_macaddr,
            self.src_ipaddr,
            self.src_port,
            self.dst_macaddr,
            self.dst_ipaddr,
            self.dst_port,
        )
        .to_be_bytes();

        enet.transmit(frame, Some(TDES0::CicFull))
    }
}

/// Build a standard UDP frame.
/// Checksums are present but zeroed-out and will be replaced by the hardware.
pub fn build_frame<const N: usize>(
    data: [u8; N],
    src_macaddr: MacAddr,
    src_ipaddr: IpV4Addr,
    src_port: u16,
    dst_macaddr: MacAddr,
    dst_ipaddr: IpV4Addr,
    dst_port: u16,
) -> EthernetFrame<IpV4Frame<UdpFrame<ByteArray<N>>>> {
    let frame = EthernetFrame::<IpV4Frame<UdpFrame<ByteArray<N>>>> {
        header: EthernetHeader {
            dst_macaddr: dst_macaddr,
            src_macaddr: src_macaddr,
            ethertype: EtherType::IPV4,
        },
        data: IpV4Frame::<UdpFrame<ByteArray<N>>> {
            header: IpV4Header {
                version_and_header_length: VersionAndHeaderLength::new()
                    .with_version(4)
                    .with_header_length((IpV4Header::BYTE_LEN / 4) as u8),
                dscp: DSCP::Standard,
                total_length: IpV4Frame::<UdpFrame<ByteArray<N>>>::BYTE_LEN as u16,
                identification: 0,
                fragmentation: Fragmentation::default(),
                time_to_live: 10,
                protocol: Protocol::UDP,
                checksum: 0,
                src_ipaddr: src_ipaddr,
                dst_ipaddr: dst_ipaddr,
            },
            data: UdpFrame::<ByteArray<N>> {
                header: UdpHeader {
                    src_port: src_port,
                    dst_port: dst_port,
                    length: UdpFrame::<ByteArray<N>>::BYTE_LEN as u16,
                    checksum: 0,
                },
                data: ByteArray::<N>(data),
            },
        },
        checksum: 0_u32,
    };

    frame
}
