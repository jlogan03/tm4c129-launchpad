//! Build a UDP/IP Ethernet packet and get its representation as network bytes

use catnip::{enet::*, ip::*, udp::*, *};

use core::ptr::read_volatile;

use super::{EthernetDriver, EthernetError};

/// Thin adapter layer to generate UDP packets without constantly passing the address info around
pub struct UDPSocket {
    /// Source MAC address
    pub src_macaddr: MacAddr,
    /// Source IP address
    pub src_ipaddr: IpV4Addr,
    /// Source port
    pub src_port: u16,
    /// Destination mac address
    pub dst_macaddr: MacAddr,
    /// Destination IP address
    pub dst_ipaddr: IpV4Addr,
    /// Destination port
    pub dst_port: u16,
    /// Incrementing packet identification
    pub id: u16,
}

impl UDPSocket {
    /// Build a UDP packet from data & pass it to the ethernet driver to transmit in hardware
    pub fn transmit<const N: usize>(
        &mut self,
        enet: &mut EthernetDriver,
        data: [u8; N],
    ) -> Result<EthernetFrame::<IpV4Frame<UdpFrame<ByteArray<N>>>>, EthernetError>
    where
        [(); EthernetFrame::<IpV4Frame<UdpFrame<ByteArray<N>>>>::BYTE_LEN]:,
        [(); IpV4Frame::<UdpFrame<ByteArray<N>>>::BYTE_LEN]:,
        [(); UdpFrame::<ByteArray<N>>::BYTE_LEN]:,
    {
        self.id = self.id.wrapping_add(1);
        let frame = build_frame(
            data,
            self.src_macaddr,
            self.src_ipaddr,
            self.src_port,
            self.dst_ipaddr,
            self.dst_macaddr,
            self.dst_port,
            self.id.clone(),
        );

        match unsafe { read_volatile(&enet.transmit(frame.to_be_bytes())) } {
            Ok(_) => Ok(frame),
            Err(x) => Err(x)
        }
    }
}

/// Build a standard UDP frame.
/// Checksums are present but zeroed-out and will be replaced by the hardware.
pub fn build_frame<const N: usize>(
    data: [u8; N],
    src_macaddr: MacAddr,
    src_ipaddr: IpV4Addr,
    src_port: u16,
    dst_ipaddr: IpV4Addr,
    dst_macaddr: MacAddr,
    dst_port: u16,
    id: u16,
) -> EthernetFrame<IpV4Frame<UdpFrame<ByteArray<N>>>>
where
    [(); IpV4Frame::<UdpFrame<ByteArray<N>>>::BYTE_LEN]:,
    [(); UdpFrame::<ByteArray<N>>::BYTE_LEN]:,
{
    // Standard method for calculating UDP checksum using pseudoheader
    let mut frame = EthernetFrame::<IpV4Frame<UdpFrame<ByteArray<N>>>> {
        header: EthernetHeader {
            dst_macaddr: dst_macaddr, // Always broadcast for IP frames
            src_macaddr: src_macaddr,
            ethertype: EtherType::IpV4,
        },
        data: IpV4Frame::<UdpFrame<ByteArray<N>>> {
            header: IpV4Header {
                version_and_header_length: VersionAndHeaderLength::new()
                    .with_version(4)
                    .with_header_length((IpV4Header::BYTE_LEN / 4) as u8),
                dscp: DSCP::Standard,
                total_length: IpV4Frame::<UdpFrame<ByteArray<N>>>::BYTE_LEN as u16,
                identification: id,
                fragmentation: Fragmentation::default(),
                time_to_live: 64,
                protocol: Protocol::Udp,
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
        checksum: 0_u32, // This one will be replaced by the CRC engine
    };

    // Write IP and UDP checksums

    // Standard method for calculating IP checksum
    // Ideally, this would be done by the checksum offload engine, but so far
    // the checksum engine neither runs nor gives any specific error, so we're
    // doing it in software for now. The UDP checksum is optional and involves
    // a pseudoheader structure, so we skip it for now.
    frame.data.data.header.checksum = calc_udp_checksum(&frame.data);
    frame.data.header.checksum = calc_ip_checksum(&frame.data.header.to_be_bytes());

    frame
}
