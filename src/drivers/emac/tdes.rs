use volatile::Volatile;


/// TX Descriptor List ring using descriptors initialized by the microcontroller in SRAM
///
/// We don't know where the descriptors are, so we have to chase the buffer around
/// and hope for the best
#[repr(C, align(4))]
pub struct TXDL {
    /// Address of start of descriptor list
    pub txdladdr: *mut TDES,
    /// Address of current descriptor
    pub tdesref: *mut TDES,
}

impl TXDL {
    /// Initialize with the current descriptor pointed at the start of the list.
    /// 
    /// The configuration of each descriptor must be updated by the driver.
    pub fn new(txdladdr: *mut TDES) -> TXDL {
        TXDL {
            txdladdr: txdladdr,
            tdesref: txdladdr,
        }
    }

    /// Move the address of the current descriptor to the next one in the chain
    /// or loop back to the start if this is the last one
    pub unsafe fn next(&mut self){
        let tdes: TDES = *self.tdesref;
        if tdes.get_tdes0(TDES0::TCH) != 0 {  // We are chaining to the next descriptor in the list
            self.tdesref = (*self.tdesref).get_next_pointer() as *mut TDES;
        } else {  // We are looping back to the start of the list
            self.tdesref = self.txdladdr;
        }
    }

    /// Dereference the current descriptor
    pub unsafe fn get(&self) -> TDES {
        *self.tdesref
    }
}

/// TX buffer descriptor field definitions.
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
/// See datasheet Figure 23-3 for layout.
#[derive(Clone, Copy)]
#[repr(C, align(4))]
pub struct TDES {
    pub v: [u32; 8],
}

impl TDES {
    /// New blank descriptor
    pub fn new() -> TDES {
        TDES { v: [0_u32; 8] }
    }

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
        vv.update(|val| *val |= TDES0::OWN as u32); // Set the OWN bit
    }

    /// Get collision count encountered during send
    pub fn get_cc(&self) -> u8 {
        let vv = Volatile::new(&(self.v[0]));
        ((vv.read() & TDES0::CC as u32) >> 3) as u8
    }

    /// Get pointer to this TDES as u32
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

    /// Get the raw pointer to the next descriptor in the ring
    pub fn get_next_pointer(&self) -> u32 {
        let vv = Volatile::new(&(self.v[3]));
        vv.read()
    }

    /// Get the raw pointer to the buffer segment associated with this
    pub fn get_buffer_pointer(&self) -> u32 {
        let vv = Volatile::new(&(self.v[2]));
        vv.read()
    }

    /// Set number of bytes to send from this buffer, in bytes, truncated to 12 bits.
    /// Clears existing value.
    pub fn set_buffer_size(&mut self, n: u16) {
        let mut vv = Volatile::new(&mut (self.v[1])); // Volatile representation of TDES1
        vv.update(|val| *val &= !(TDES1::TBS1 as u32)); // Clear field via read-modify-write

        let m = (n & 0b0000_1111_1111_1111) as u32; // Truncate to 12 bits and expand to u32
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

    /// Get an arbitrary field from TDES0
    pub fn get_tdes0(&self, field: TDES0) -> u32 {
        use TDES0::*;
        let v = Volatile::new(&(self.v[0])).read(); // Volatile read of TDES0
        let masked = v & (field as u32);
        match field {
            // Handle numeric values
            CC => masked >> 3,
            // Handle all flag fields as integer representation of bool
            _ => match masked {
                0 => 0,
                _ => 1,
            },
        }
    }

    /// Get an arbitrary field from TDES1
    pub fn get_tdes1(&self, field: TDES1) -> u32 {
        use TDES1::*;
        let v = Volatile::new(&(self.v[1])).read(); // Volatile read of TDES1
        let masked = v & (field as u32);
        match field {
            // Handle numeric values
            TBS1 => masked,
            TBS2 => masked >> 16,
            // Handle all flag fields as integer representation of bool
            _ => match masked {
                0 => 0,
                _ => 1,
            },
        }
    }
}

/// TX descriptor field masks for the first word (TDES0)
/// See datasheet Table 23-2
#[derive(Clone, Copy)]
#[repr(u32)]
pub enum TDES0 {
    // Status flag set by the DMA or the user to transfer ownership
    /// Flag that DMA owns this descriptor
    OWN = 1 << 31,
    // To be set by the user
    /// Interrupt at end of transmission
    TI = 1 << 30,
    /// Last segment of the frame
    LS = 1 << 29,
    /// First segment of the frame
    FS = 1 << 28,
    /// Disable CRC check
    DC = 1 << 27,
    /// Disable zero-padding
    DP = 1 << 26,
    /// Transmit Timestamp Enable (for IEEE-1588 PTP timestamping)
    TTSE = 1 << 25,
    /// Enable Ethernet checksum replacement
    CRCR = 1 << 24,
    // CicDisable = 0, // Do not insert IPV4 or UDP/TCP checksums
    /// Insert IPV4 checksum only
    CicIPV4 = 1 << 22,
    /// Insert IPV4 checksum & TCP/UDP checksum without pseudoheader
    CicFrameOnly = 2 << 22,
    /// Insert IPV4 checksum & TCP/UDP checksum including pseudoheader (per the actual standard)
    CicFull = 3 << 22,
    /// Transmit End of Ring: this descriptor is the last one in the ring
    TER = 1 << 21,
    /// Transmit Chain: the second pointer field is a pointer to the next descriptor, not a buffer
    TCH = 1 << 20,
    // VlanDisable = 0, // No VLAN tagging
    /// Strip VLAN tag before transmission
    VlanRemove = 1 << 18,
    /// Insert VLAN tag into frame (copied from register EMACVLNINCREP)
    VlanInsert = 2 << 18,
    /// Replace existing VLAN tag in frame (copied from register EMACVLNINCREP)
    VlanReplace = 3 << 18,

    // Status flags set by the DMA
    /// Timestamp was captured for this frame
    TTSS = 1 << 17,
    /// IP header error occurred
    IHE = 1 << 16,
    /// An error of any kind occurred
    ES = 1 << 15,
    /// Jabber timeout error
    JT = 1 << 14,
    /// Frame flushed by software
    FF = 1 << 13,
    /// IP payload error
    IPE = 1 << 12,
    /// Loss of Carrier error
    LOC = 1 << 11,
    /// No Carrier error
    NC = 1 << 10,
    /// Late Collision error
    LC = 1 << 9,
    /// Excessive Collision error
    EC = 1 << 8,
    /// Transmitted frame was a VLAN frame
    VF = 1 << 7,
    /// Collision count
    CC = 0b111 << 3,

    /// Excessive Deferral error
    ED = 1 << 2,
    /// Underflow error
    UF = 1 << 1,
    /// Deferral bit (only relevant to half-duplex mode)
    DB = 1,
}

/// TX descriptor field masks for second word (TDES1)
/// See datasheet table 23-3
#[derive(Clone, Copy)]
#[repr(u32)]
pub enum TDES1 {
    /// Use MAC address register 1 instead of 0
    SA1 = 1 << 31,
    /// Insert source address into frame            
    SaiInsert = 1 << 29,
    /// Replace existing source address in frame
    SaiReplace = 2 << 29,
    /// Size of valid data in second buffer
    TBS2 = 0b1111_1111_1111 << 16,
    /// Size of valid data in first buffer
    TBS1 = 0b1111_1111_1111,
}
