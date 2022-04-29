//! TX buffer descriptor field definitions and volatile access

use core::fmt;

/// Number of descriptors/buffer segments
pub const TXDESCRS: usize = 4;

/// Number of bytes per buffer segment
pub const TXBUFSIZE: usize = 1522;  // Maximum size of standard frame

/// TX Descriptor List ring buffer
#[repr(C, align(4))]
pub struct TXDL {
    /// Address of start of descriptor list
    pub txdladdr: *mut TDES,
    /// Address of current descriptor
    pub tdesref: *mut TDES,
    /// Descriptor data
    pub descriptors: [TDES; TXDESCRS],
    /// Buffers sized for non-jumbo frames
    pub buffers: [[u8; TXBUFSIZE]; TXDESCRS],
}

impl TXDL {
    /// Initialize with the current descriptor pointed at the start of the list.
    ///
    /// The configuration of each descriptor must be updated by the driver.
    pub fn new() -> TXDL {
        let mut txdl = TXDL {
            txdladdr: 0 as *mut TDES,
            tdesref: 0 as *mut TDES,
            descriptors: [TDES { v: [0_u32; 8] }; TXDESCRS],
            buffers: [[0_u8; TXBUFSIZE]; TXDESCRS],
        };
        // Set descriptor list start pointer
        txdl.txdladdr = &mut (txdl.descriptors[0]) as *mut TDES;

        // Configure descriptors
        unsafe {
            for i in 0..TXDESCRS {
                // Populate pointers
                txdl.tdesref = &mut (txdl.descriptors[i]) as *mut TDES;
                let buffer_ptr = &mut (txdl.buffers[i]) as *mut [u8; TXBUFSIZE];
                txdl.set_buffer_pointer(buffer_ptr as u32);
                if i < TXDESCRS - 1 {
                    txdl.set_next_pointer(&(txdl.descriptors[i + 1]) as *const TDES as u32);
                }
                else {
                    // This is the end of the ring. Point back toward the start and set the end-of-ring flag
                    txdl.set_next_pointer(txdl.txdladdr as u32);
                    txdl.set_tdes0(TDES0::TER);  // End-of-ring
                }
                // We are not using multi-buffer frames; set both start of frame and end of frame flags
                txdl.set_tdes0(TDES0::FS);
                // txdl.set_tdes0(TDES0::LS);

                txdl.set_tdes0(TDES0::TCH);  // Next descriptor is chained
            }
        }

        txdl.tdesref = txdl.txdladdr;  // Reset current descriptor to the start of the ring
        txdl
    }

    /// Move the address of the current descriptor to the next one in the chain
    /// or loop back to the start if this is the last one
    pub unsafe fn next(&mut self) -> &mut TXDL {
        if self.get_tdes0(TDES0::TCH) != 0 {
            // We are chaining to the next descriptor in the list
            self.tdesref = self.get_next_pointer() as *mut TDES;
        } else {
            // We are looping back to the start of the list
            self.tdesref = self.txdladdr;
        }
        self
    }

    /// Volatile read of the current descriptor
    pub unsafe fn get(&self) -> TDES {
        self.tdesref.read_volatile()
    }

    /// Check if software owns this descriptor
    pub unsafe fn is_owned(&self) -> bool {
        let v = self.tdesref.read_volatile().v[0];
        if v & TDES0::OWN as u32 != 0 {
            return false;
        } else {
            return true;
        }
    }

    /// Give ownership of this descriptor to the DMA by setting the OWN bit
    pub unsafe fn give(&mut self) {
        let mut tdes = self.tdesref.read_volatile();
        tdes.v[0] |= TDES0::OWN as u32;
        self.tdesref.write_volatile(tdes);
    }

    /// Take ownership of this descriptor back from the DMA by clearing the OWN bit
    /// This may cause unusual behavior and should only be done as a last resort
    pub unsafe fn take(&mut self) {
        let mut tdes = self.tdesref.read_volatile();
        tdes.v[0] |= !(TDES0::OWN as u32);
        self.tdesref.write_volatile(tdes);
    }

    /// Set the pointer to the next descriptor in the ring
    pub unsafe fn set_next_pointer(&mut self, ptr: u32) {
        let mut tdes = self.tdesref.read_volatile(); // Volatile read via copy
        tdes.v[3] = ptr;
        self.tdesref.write_volatile(tdes);
    }

    /// Set the pointer to the buffer segment associated with this
    pub unsafe fn set_buffer_pointer(&mut self, ptr: u32) {
        let mut tdes = self.tdesref.read_volatile(); // Volatile read via copy
        tdes.v[2] = ptr;
        self.tdesref.write_volatile(tdes);
    }

    /// Get the raw pointer to the next descriptor in the ring
    pub unsafe fn get_next_pointer(&self) -> u32 {
        self.tdesref.read_volatile().v[3]
    }

    /// Get the raw pointer to the buffer segment associated with this
    pub unsafe fn get_buffer_pointer(&self) -> u32 {
        self.tdesref.read_volatile().v[2]
    }

    /// Set number of bytes to send from this buffer, in bytes, truncated to 12 bits.
    /// Clears existing value.
    pub unsafe fn set_buffer_size(&mut self, n: u16) {
        let mut tdes = self.tdesref.read_volatile();
        tdes.v[1] &= !(TDES1::TBS1 as u32);
        let m = (n & 0b0000_1111_1111_1111) as u32; // Truncate to 12 bits and expand to u32
        tdes.v[1] |= m;
        self.tdesref.write_volatile(tdes);
    }

    /// Set a flag field in TDES0 by OR-ing in the new value via volatile read-modify-write.
    /// Does not check if an overlapping value is already set!
    pub unsafe fn set_tdes0(&mut self, field: TDES0) {
        let mut tdes = self.tdesref.read_volatile();
        tdes.v[0] |= field as u32;
        self.tdesref.write_volatile(tdes);
    }

    /// Set a flag field in TDES1 by OR-ing in the new value via volatile read-modify-write.
    /// Does not check if an overlapping value is already set!
    pub unsafe fn set_tdes1(&mut self, field: TDES1) {
        let mut tdes = self.tdesref.read_volatile();
        tdes.v[1] |= field as u32;
        self.tdesref.write_volatile(tdes);
    }

    /// Get an arbitrary field from TDES0
    pub unsafe fn get_tdes0(&self, field: TDES0) -> u32 {
        use TDES0::*;
        let v = self.tdesref.read_volatile().v[0]; // Volatile read of TDES0
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
    pub unsafe fn get_tdes1(&self, field: TDES1) -> u32 {
        use TDES1::*;
        let v = self.tdesref.read_volatile().v[1]; // Volatile read of TDES1
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

impl fmt::Debug for TXDL {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // f.debug_struct("TX Descriptor List").field("\nStart Address", &self.txdladdr).finish()
        unsafe {
            write!(
                f,
                "
            TX Descriptor List
            ------------------
            Root Descriptor Address: {}
            Current Descriptor Address: {}
            Current Descriptor
              Owned by DMA: {}
              Error Status
                ES: {}
                IHE: {}
                JT: {}
                FF: {}
                IPE: {}
                LOC: {}
                NC: {}
                LC: {}
                EC: {}
                ED: {}
                UF: {}
                Collision Count: {}
              Configuration
                Next Descriptor Address: {}
                Buffer Address: {}
                Buffer 1 Size: {}
                Buffer 2 Size: {}
                IC: {}
                LS: {}
                FS: {}
                DC: {}
                DP: {}
                TTSE: {}
                CRCR: {}
                CicIPV4: {}
                CicFrameOnly: {}
                CicFull: {}
                TER: {}
                TCH: {}
                VlanRemove: {}
                VLanInsert: {}
                VlanReplace: {}
                TTSS: {}
                SA1: {}
                SaiInsert: {}
                SaiReplace: {}
                ",
                self.txdladdr as usize,
                self.tdesref as usize,
                self.get_tdes0(TDES0::OWN),
                self.get_tdes0(TDES0::ES),
                self.get_tdes0(TDES0::IHE),
                self.get_tdes0(TDES0::JT),
                self.get_tdes0(TDES0::FF),
                self.get_tdes0(TDES0::IPE),
                self.get_tdes0(TDES0::LOC),
                self.get_tdes0(TDES0::NC),
                self.get_tdes0(TDES0::LC),
                self.get_tdes0(TDES0::EC),
                self.get_tdes0(TDES0::ED),
                self.get_tdes0(TDES0::UF),
                self.get_tdes0(TDES0::CC),
                self.get_next_pointer(),
                self.get_buffer_pointer(),
                self.get_tdes1(TDES1::TBS1),
                self.get_tdes1(TDES1::TBS2),
                self.get_tdes0(TDES0::IC),
                self.get_tdes0(TDES0::LS),
                self.get_tdes0(TDES0::FS),
                self.get_tdes0(TDES0::DC),
                self.get_tdes0(TDES0::DP),
                self.get_tdes0(TDES0::TTSE),
                self.get_tdes0(TDES0::CRCR),
                self.get_tdes0(TDES0::CicIPV4),
                self.get_tdes0(TDES0::CicFrameOnly),
                self.get_tdes0(TDES0::CicFull),
                self.get_tdes0(TDES0::TER),
                self.get_tdes0(TDES0::TCH),
                self.get_tdes0(TDES0::VlanRemove),
                self.get_tdes0(TDES0::VlanInsert),
                self.get_tdes0(TDES0::VlanReplace),
                self.get_tdes0(TDES0::TTSS),
                self.get_tdes1(TDES1::SA1),
                self.get_tdes1(TDES1::SaiInsert),
                self.get_tdes1(TDES1::SaiReplace),
            )
        }
    }
}

/// TX buffer descriptor layout.
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
#[repr(transparent)]
pub struct TDES {
    /// Content
    pub v: [u32; 8],
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
    IC = 1 << 30,
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
