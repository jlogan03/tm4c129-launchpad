use volatile::Volatile;

/// TX buffer descriptor field definitions
///
/// "Descriptor" structure is the software interface with the direct memory access controller.
/// Hardware interprets Descriptors as members of a linked list with a particular format.
/// Each Descriptor contains a pointer to its buffer segment, a pointer to the next descriptor,
/// and information about how the content of the buffer should be interpreted
///
/// Assumes we are using 8-word descriptors ("alternate descriptor size" peripheral config).
#[derive(Clone, Copy)]
#[repr(transparent)]
pub(crate) struct TDES {
    v: [u32; 8],
}

impl TDES {
    /// Check if software owns this descriptor, or the DMA
    pub fn is_owned(&self) -> bool {
        let vv = Volatile::new(&(self.v[0]));
        if vv.read() & TDES0::OWN as u32 != 0 {
            return false;
        } else {
            return true;
        }
    }

    /// Give ownership of this descriptor to the DMA by setting the OWN bit
    pub fn give(&mut self) {
        let mut vv = Volatile::new(&mut (self.v[0])); // Volatile representation of TDES0
        vv.update(|val| *val |= TDES0::OWN as u32);  // Set the OWN bit
    }

    /// Get collision count encountered during send
    pub fn get_cc(&self) -> u8 {
        let vv = Volatile::new(&(self.v[0]));
        ((vv.read() & TDES0::CC as u32) >> 3) as u8
    }

    /// Set the pointer to the next descriptor in the ring
    pub fn set_next_pointer(&mut self, nextdes: &TDES) {
        let next_descr_addr: u32 = (nextdes as *const _) as u32; // Memory address of next descriptor
        self.v[3] = next_descr_addr;
    }

    /// Set the pointer to the buffer segment associated with this
    pub fn set_buffer_pointer<const M: usize>(&mut self, buffer: &[u8; M]) {
        let buffer_addr: u32 = (buffer as *const _) as u32; // Memory address of buffer segment
        self.v[2] = buffer_addr;
    }

    /// Set number of bytes to send from this buffer, in bytes, truncated to 12 bits.
    /// Clears existing value.
    pub fn set_buffer_size(&mut self, n: u16) {
        let mut vv = Volatile::new(&mut (self.v[1])); // Volatile representation of TDES1
        vv.update(|val| *val &= !(TDES1::TBS1 as u32)); // Clear field via read-modify-write

        let m = (n & 0b0000_1111_1111_1111) as u32; // Truncate to 12 bits
        vv.update(|val| *val |= m); // Set new value via read-modify-write
    }

    /// Set a flag field in TDES0 by OR-ing in the new value via volatile read-modify-write.
    /// Does not check if an overlapping value is already set!
    pub fn set_tdes0(&mut self, field: TDES0) {
        let mut vv = Volatile::new(&mut (self.v[0])); // Volatile representation of TDES0
        vv.update(|val| *val |= field as u32); // Volatile read-modify-write
    }

    /// Set a flag field in TDES1 by OR-ing in the new value via volatile read-modify-write.
    /// Does not check if an overlapping value is already set!
    pub fn set_tdes1(&mut self, field: TDES1) {
        let mut vv = Volatile::new(&mut (self.v[1])); // Volatile representation of TDES1
        vv.update(|val| *val |= field as u32); // Volatile read-modify-write
    }
}

/// TX descriptor field masks for the first word (TDES0)
/// See datasheet Table 23-2
pub(crate) enum TDES0 {
    // Status flag set by the DMA or the user to transfer ownership
    OWN = 1 << 31, // Flag that DMA owns this descriptor
    // To be set by the user
    TI = 1 << 30,   // Interrupt at end of transmission
    LS = 1 << 29,   // Last segment of the frame
    FS = 1 << 28,   // First segment of the frame
    DC = 1 << 27,   // Disable CRC check
    DP = 1 << 26,   // Disable zero-padding
    TTSE = 1 << 25, // Transmit Timestamp Enable (for IEEE-1588 PTP timestamping)
    CRCR = 1 << 24, // Enable Ethernet checksum replacement
    // CicDisable = 0, // Do not insert IPV4 or UDP/TCP checksums
    CicIPV4 = 1 << 22,      // Insert IPV4 checksum only
    CicFrameOnly = 2 << 22, // Insert IPV4 checksum & TCP/UDP checksum without pseudoheader
    CicFull = 3 << 22, // Insert IPV4 checksum & TCP/UDP checksum including pseudoheader (per the actual standard)
    TER = 1 << 21,     // Transmit End of Ring: this descriptor is the last one in the ring
    TCH = 1 << 20, // Transmit Chain: the second pointer field is a pointer to the next descriptor, not a buffer
    // VlanDisable = 0, // No VLAN tagging
    VlanRemove = 1 << 18,  // Strip VLAN tag before transmission
    VlanInsert = 2 << 18,  // Insert VLAN tag into frame (copied from register EMACVLNINCREP)
    VlanReplace = 3 << 18, // Replace existing VLAN tag in frame (copied from register EMACVLNINCREP)

    // Status flags set by the DMA
    TTSS = 1 << 17, // Timestamp was captured for this frame
    IHE = 1 << 16,  // IP header error occurred
    ES = 1 << 15,   // An error of any kind occurred
    JT = 1 << 14,   // Jabber timeout error
    FF = 1 << 13,   // Frame flushed by software
    IPE = 1 << 12,  // IP payload error
    LOC = 1 << 11,  // Loss of Carrier error
    NC = 1 << 10,   // No Carrier error
    LC = 1 << 9,    // Late Collision error
    EC = 1 << 8,    // Excessive Collision error

    VF = 1 << 7,     // Transmitted frame was a VLAN frame
    CC = 0b111 << 3, // Collision count

    ED = 1 << 2, // Excessive Deferral error
    UF = 1 << 1, // Underflow error
    DB = 1,      // Deferral bit (only relevant to half-duplex mode)
}

// TX descriptor field masks for second word (TDES1)
// See datasheet table 23-3
pub(crate) enum TDES1 {
    SA1 = 1 << 31,                 // Use MAC address register 1 instead of 0
    SaiInsert = 1 << 29,           // Insert source address into frame
    SaiReplace = 2 << 29,          // Replace existing source address in frame
    TBS2 = 0b1111_1111_1111 << 16, // Size of valid data in second buffer
    TBS1 = 0b1111_1111_1111,       // Size of valid data in first buffer
}
