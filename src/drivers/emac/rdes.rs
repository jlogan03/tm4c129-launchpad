use volatile::Volatile;

/// RX buffer descriptor field definitions.
///
/// "Descriptor" structure is the software interface with the direct memory access controller.
/// Hardware interprets Descriptors as members of a linked list with a particular format.
/// Each Descriptor contains a pointer to its buffer segment, a pointer to the next descriptor,
/// and information about how the content of the buffer should be interpreted
///
/// Assumes we are using 8-word descriptors ("alternate descriptor size" peripheral config).
/// 
/// Note the DMA controller requires the descriptors to be aligned on 32-bit words instead of bytes,
/// hence the repr(align(4)). We also need safely-made pointers to address the actual location of the
/// values within the struct, hence the repr(C).
/// 
/// See datasheet Figure 23-4 for layout.
#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct RDES {
    pub v: [u32; 8],
}

impl RDES {
    /// New blank descriptor
    pub fn new() -> RDES {
        RDES { v: [0_u32; 8] }
    }

    /// Check if software owns this descriptor, or the DMA
    pub fn is_owned(&self) -> bool {
        let vv = Volatile::new(&(self.v[0]));
        if vv.read() & RDES0::OWN as u32 != 0 {
            return false;
        } else {
            return true;
        }
    }

    /// Give ownership of this descriptor to the DMA by setting the OWN bit
    pub fn give(&mut self) {
        let mut vv = Volatile::new(&mut (self.v[0])); // Volatile representation of RDES0
        vv.update(|val| *val |= RDES0::OWN as u32); // Set the OWN bit
    }

    /// Get collision count encountered during send
    pub fn get_cc(&self) -> u8 {
        let vv = Volatile::new(&(self.v[0]));
        ((vv.read() & RDES0::CC as u32) >> 3) as u8
    }

    /// Get pointer to this RDES as u32
    pub fn get_pointer(&self) -> u32 {
        (self as *const _) as u32
    }

    /// Set the pointer to the next descriptor in the ring
    pub fn set_next_pointer(&mut self, ptr: u32) {
        self.v[3] = ptr;
    }

    /// Set the pointer to the buffer segment associated with this
    pub fn set_buffer_pointer(&mut self, ptr: u32) {
        self.v[2] = ptr;
    }

    /// Get number of bytes to receive from this buffer, in bytes.
    pub fn get_buffer_size(&self, n: u16) -> u16 {
        let v = Volatile::new(&(self.v[1])).read(); // Volatile read of RDES1
        (v | (RDES1::RBS1 as u32)) as u16
    }


}

/// TX descriptor field masks for the first word (RDES0)
/// See datasheet Table 23-8
#[repr(u32)]
pub enum RDES0 {
    /// Status flag set by the DMA or software to transfer ownership
    /// 0 => owned by software; 1 => owned by DMA
    OWN = 1 << 31,
    /// AFM: Destination Address Filter Fail
    /// When set, this bit indicates a frame failed in the DA filter in the MAC.
    AFM = 1 << 30,
    /// FL: Frame Length
    /// These bits indicate the byte length of the received frame that was transferred to the system memory. This field
    /// is valid when the Last Descriptor (RDES0[8]) is set and either the Descriptor Error (RDES0[14]) or the Overflow
    /// Error bit (RDES0[11]) is clear.
    /// When the Last Descriptor bit is not set, this field indicates the accumulated number of bytes that have been
    /// transferred for the current frame. The inclusion of CRC length in the frame length depends on the settings of
    /// CRC configuration bits, ACS and CST in the EMACCFG register.
    FL = 0b1111_1111_1111_1 << 16,
    /// ES: Error Summary
    ES = 1 << 15,
    /// DE: Descriptor Error
    /// When set, this bit indicates a frame truncation caused by a frame that does not fit within the current descriptor
    /// buffers, and that the DMA does not own the Next Descriptor. The frame is truncated. This field is valid only when
    /// the Last Descriptor (RDES0[8]) is set.
    DE = 1 << 14,
    /// SAF: Source Address Filter Fail
    /// When set, this bit indicates that the SA field of frame failed the SA Filter in the MAC.
    SAF = 1 << 13,
    /// LE: Length Error
    /// When set, this bit indicates that the actual length of the frame received and the Length/Type field do not match.
    /// This bit is valid only when the Frame Type (RDES0[5]) bit is reset. Length error status is not valid when CRC
    /// error is present.
    LE = 1 << 12,
    /// OE: Overflow Error
    /// When set, this bit indicates that the received frame is damaged because of buffer overflow in RX FIFO.
    /// Note: This bit is set only when the DMA transfers a partial frame to the application. This happens only when
    /// the RX FIFO is operating in the threshold mode. In the store-and-forward mode, all partial frames are
    /// dropped completely in RX FIFO.
    OE = 1 << 11,
    /// VLAN: VLAN Tag
    /// When set, this bit indicates that the frame pointed to by this descriptor is a VLAN frame tagged by the MAC. The
    /// VLAN tagging depends on checking VLAN fields of the received frame configured in the Ethernet MAC VLAN
    /// Tag (EMACVLANTG) register, offset 0x01C
    VLAN = 1 << 10,
    /// FS: First Descriptor
    /// When set, this bit indicates that this descriptor contains the first buffer of the frame. If the size of the first buffer
    /// is 0, the second buffer contains the beginning of the frame. If the size of the second buffer is also 0, the next
    /// Descriptor contains the beginning of the frame.
    FS = 1 << 9,
    /// LS: Last Descriptor
    /// When set, this bit indicates that the buffers pointed to by this descriptor are the last buffers of the frame.
    LS = 1 << 8,
    /// Timestamp Available or Giant Frame
    /// When Advanced Timestamp feature is enabled, this bit indicates that a snapshot of the Timestamp is written in
    /// descriptor words 6 (RDES6) and 7 (RDES7). This is valid only when the Last Descriptor bit (RDES0[8]) is set.
    /// Otherwise, this bit, when set, indicates the Giant Frame Status. Giant frames are larger than 1,518-byte (or
    /// 1,522-byte for VLAN or 2,000-byte when Bit 27 of MAC Configuration register is set) normal frames and larger
    /// than 9,018-byte (9,022-byte for VLAN) frame when Jumbo Frame processing is enabled.
    TA = 1 << 7,
    /// LC: Late Collision
    /// When set, this bit indicates that a late collision has occurred while receiving the frame in half-duplex mode.
    LC = 1 << 6,
    /// FT: Frame Type
    /// When set, this bit indicates that the Receive Frame is an Ethernet-type frame (the LT field is greater than or equal
    /// to 1,536). When this bit is reset, it indicates that the received frame is an IEEE 802.3 frame. This bit is not valid
    /// for Runt frames less than 14 bytes. In addition when the IPC bit is set in the EMACCFG register, this bit conveys
    /// different information. See Table 23-9 on page 1545.
    FT = 1 << 5,
    /// RWT: Receive Watchdog Timeout
    /// When set, this bit indicates that the Receive Watchdog Timer has expired while receiving the current frame and
    /// the current frame is truncated after the Watchdog Timeout.
    RWT = 1 << 4,
    /// RE: Receive Error
    /// When set, this bit indicates that an error occurred during frame reception.
    RE = 1 << 3,
    /// DE: Dribble Bit Error
    /// When set, this bit indicates that the received frame has a non-integer multiple of bytes (odd nibbles).
    DE = 1 << 2,
    /// CE: CRC Error
    /// When set, this bit indicates that a Cyclic Redundancy Check (CRC) Error occurred on the received frame. This
    /// field is valid only when the Last Descriptor bit (RDES0[8]) is set.
    CE = 1 << 1,
    /// Extended Status Available/RX MAC Address
    /// When set, this bit indicates that the extended status is available in descriptor word 4 (RDES4). This is valid only
    /// when the Last Descriptor bit (RDES0[8]) is set. This bit is invalid when Bit 30 is set.
    ESA = 1,
}

/// TX descriptor field masks for second word (RDES1)
/// See datasheet table 23-9
#[repr(u32)]
pub enum RDES1 {
    /// Disable Interrupt on Completion
    /// When set, this bit prevents the setting of the Receive Interrupt (RI) bit in the EMACDMARIS register and
    /// prevents the receive interrupt from being asserted.
    DI = 1 << 31,
    /// Size of valid data in second buffer
    RBS2 = 0b1111_1111_1111 << 16,
    /// RER: Receive End of Ring
    /// When set, this bit indicates that the descriptor list reached its final descriptor. The DMA returns to the base
    /// address of the list, creating a Descriptor Ring.
    RER = 1 << 15,
    /// RCH: Second Address Chained
    /// When set, this bit indicates that the second address in the descriptor is the Next Descriptor address rather
    /// than the second buffer address. When this bit is set, RBS2 (RDES1[28:16]) is a "don’t care" value. RDES1[15]
    /// takes precedence over RDES1[14].
    RCH = 1 << 14,
    /// Size of valid data in first buffer
    RBS1 = 0b1111_1111_1111,
}
