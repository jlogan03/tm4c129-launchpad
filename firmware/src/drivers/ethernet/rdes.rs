//! RX buffer descriptor field definitions and volatile access

use core::fmt;

use static_assertions::const_assert;
use ufmt::derive::uDebug;

/// Number of descriptors/buffer segments
pub const RXDESCRS: usize = 30;

/// Number of bytes per buffer segment
/// Length MUST be a multiple of 4, or we end up with misalignment at the boundaries
pub const RXBUFSIZE: usize = 1524; // 1522 is maximum size of standard frame with vlan tag

const_assert!(RXBUFSIZE % 4 == 0);
const_assert!(RXDESCRS > 3);

/// Word-aligned buffers
#[repr(C, align(4))]
pub struct RxBuf([[u8; RXBUFSIZE]; RXDESCRS]);

/// RX descriptor ring
#[repr(C, align(4))]
pub struct RXDL {
    /// Descriptor data
    pub descriptors: [RDES; RXDESCRS],
    /// Buffers sized for non-jumbo frames
    pub buffers: RxBuf,
    /// Address of start of descriptor list
    pub rxdladdr: *mut RDES,
    /// Address of current descriptor
    pub rdesref: *mut RDES,
}

impl RXDL {
    /// Initialize with the current descriptor pointed at the start of the list.
    ///
    /// The configuration of each descriptor must be updated by the driver.
    pub fn new() -> RXDL {
        let mut rxdl = RXDL {
            rxdladdr: 0 as *mut RDES,
            rdesref: 0 as *mut RDES,
            descriptors: [RDES { v: [0_u32; 8] }; RXDESCRS],
            buffers: RxBuf([[0_u8; RXBUFSIZE]; RXDESCRS]),
        };
        // Set descriptor list start pointer
        rxdl.rxdladdr = &mut (rxdl.descriptors[0]) as *mut RDES;

        for i in 0..RXDESCRS {
            rxdl.rdesref = &mut (rxdl.descriptors[i]) as *mut RDES;
            let buffer_ptr = &mut (rxdl.buffers.0[i]) as *mut [u8; RXBUFSIZE];
            rxdl.set_buffer_pointer(buffer_ptr as u32);

            if i < RXDESCRS - 1 {
                rxdl.set_next_pointer(&(rxdl.descriptors[i + 1]) as *const RDES as u32);
            } else {
                // This is the end of the ring. Point back toward the start.
                rxdl.set_next_pointer(rxdl.rxdladdr as u32);
                rxdl.set_rdes1(RDES1::RER, None);
            }
            // Indicate descriptors are chained
            rxdl.set_rdes1(RDES1::RCH, None);
            // Disable interrupt-on-completion
            rxdl.set_rdes1(RDES1::DI, None);
            // Tell the DMA how large the buffers are
            rxdl.set_rdes1(RDES1::RBS1, Some(RXBUFSIZE as u16));
            // DMA initially owns all the descriptors
            rxdl.give();
        }

        rxdl.rdesref = rxdl.rxdladdr; // Reset current descriptor to the start of the ring
        rxdl
    }

    /// Move the address of the current descriptor to the next one in the chain
    /// or loop back to the start if this is the last one
    pub fn next(&mut self) -> &mut RXDL {
        if self.get_rdes1(RDES1::RCH) != 0 {
            // We are chaining to the next descriptor in the list
            self.rdesref = self.get_next_pointer() as *mut RDES;
        } else {
            // We are looping back to the start of the list
            self.rdesref = self.rxdladdr;
        }
        self
    }

    /// Check if software owns this descriptor
    pub fn is_owned(&self) -> bool {
        let v = unsafe { self.rdesref.read_volatile().v[0] };
        if v & RDES0::OWN as u32 != 0 {
            return false;
        } else {
            return true;
        }
    }

    /// Give ownership of this descriptor to the DMA by setting the OWN bit
    pub fn give(&mut self) {
        self.set_rdes0(RDES0::OWN)
    }

    /// Set the pointer to the next descriptor in the ring
    pub fn set_next_pointer(&mut self, ptr: u32) {
        let mut rdes = unsafe { self.rdesref.read_volatile() };
        rdes.v[3] = ptr;
        unsafe { self.rdesref.write_volatile(rdes) };
    }

    /// Set the pointer to the buffer segment associated with this
    pub fn set_buffer_pointer(&mut self, ptr: u32) {
        let mut rdes = unsafe { self.rdesref.read_volatile() };
        rdes.v[2] = ptr;
        unsafe { self.rdesref.write_volatile(rdes) };
    }

    /// Get the pointer to the next descriptor in the ring
    pub fn get_next_pointer(&self) -> u32 {
        unsafe { self.rdesref.read_volatile().v[3] }
    }

    /// Get the pointer to the buffer segment
    pub fn get_buffer_pointer(&self) -> u32 {
        unsafe { self.rdesref.read_volatile().v[2] }
    }

    /// Get number of bytes to receive from this buffer
    pub fn get_buffer_size(&self) -> u16 {
        self.get_rdes1(RDES1::RBS1) as u16
    }

    /// Get an arbitrary field from RDES0
    pub fn get_rdes0(&self, field: RDES0) -> u32 {
        use RDES0::*;
        let rdes = unsafe { self.rdesref.read_volatile() };
        let masked = rdes.v[0] & (field as u32);
        match field {
            // Get the count of field length and align as u32
            FL => masked >> 16,
            // Handle all flag fields as integer representation of bool
            _ => match masked {
                0 => 0,
                _ => 1,
            },
        }
    }

    /// Set an arbitrary field in RDES1
    pub fn set_rdes0(&mut self, field: RDES0) {
        use RDES0::*;
        let mut rdes = unsafe { self.rdesref.read_volatile() };

        match field {
            // Handle numeric values
            FL => {} // Frame length only set by DMA
            // Handle all flag fields
            _ => rdes.v[0] |= field as u32,
        }
        // Write the modified descriptor
        unsafe { self.rdesref.write_volatile(rdes) };
    }

    /// Get an arbitrary field from RDES1
    pub fn get_rdes1(&self, field: RDES1) -> u32 {
        use RDES1::*;
        let rdes = unsafe { self.rdesref.read_volatile() };
        let masked = rdes.v[1] & (field as u32);
        match field {
            // Handle numeric values
            RBS2 => masked >> 16,
            RBS1 => masked,
            // Handle all flag fields as integer representation of bool
            _ => match masked {
                0 => 0,
                _ => 1,
            },
        }
    }

    /// Set an arbitrary field in RDES1
    pub fn set_rdes1(&mut self, field: RDES1, value: Option<u16>) {
        use RDES1::*;
        let mut rdes = unsafe { self.rdesref.read_volatile() };
        let x: u16 = match value {
            Some(x) => x,
            None => 0_u16,
        };
        let masked = (x & (0b0000_1111_1111_1111 as u16)) as u32;
        match field {
            // Handle numeric values
            RBS1 => {
                rdes.v[1] &= !(RBS1 as u32); // Clear field
                rdes.v[1] |= masked; // Set new value
            }
            RBS2 => {
                rdes.v[1] &= !(RBS2 as u32); // Clear field via read-modify-write
                rdes.v[1] |= masked << 16; // Set new value
            }
            // Handle all flag fields
            _ => rdes.v[1] |= field as u32,
        }
        // Write the modified descriptor
        unsafe { self.rdesref.write_volatile(rdes) };
    }
}

impl fmt::Debug for RXDL {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // f.debug_struct("TX Descriptor List").field("\nStart Address", &self.txdladdr).finish()
        write!(
            f,
            "
            RX Descriptor List
            ------------------
            Root Descriptor Address: {}
            Current Descriptor Address: {}
            Current Descriptor
              Owned by DMA: {}
              Error Status
                ES: {}
                AFM: {}
                DE: {}
                SAF: {}
                LE: {}
                OE: {}
                LC: {}
                RWT: {}
                RE: {}
                DBE: {}
                CE: {}
              Configuration
                Next Descriptor Address: {}
                Buffer Address: {}
                Buffer 1 Size: {}
                Buffer 2 Size: {}
                FL: {}
                VLAN: {}
                FS: {}
                LS: {}
                TA: {}
                FT: {}
                ESA: {}
                DI: {}
                RER: {}
                RCH: {}
                ",
            self.rxdladdr as usize,
            self.rdesref as usize,
            self.get_rdes0(RDES0::OWN),
            // Errors
            self.get_rdes0(RDES0::ES),
            self.get_rdes0(RDES0::AFM),
            self.get_rdes0(RDES0::DE),
            self.get_rdes0(RDES0::SAF),
            self.get_rdes0(RDES0::LE),
            self.get_rdes0(RDES0::OE),
            self.get_rdes0(RDES0::LC),
            self.get_rdes0(RDES0::RWT),
            self.get_rdes0(RDES0::RE),
            self.get_rdes0(RDES0::DBE),
            self.get_rdes0(RDES0::CE),
            // Cfg
            self.get_next_pointer(),
            self.get_buffer_pointer(),
            self.get_rdes1(RDES1::RBS1),
            self.get_rdes1(RDES1::RBS2),
            self.get_rdes0(RDES0::FL),
            self.get_rdes0(RDES0::VLAN),
            self.get_rdes0(RDES0::FS),
            self.get_rdes0(RDES0::LS),
            self.get_rdes0(RDES0::TA),
            self.get_rdes0(RDES0::FT),
            self.get_rdes0(RDES0::ESA),
            self.get_rdes1(RDES1::DI),
            self.get_rdes1(RDES1::RER),
            self.get_rdes1(RDES1::RCH),
        )
    }
}

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
#[derive(Clone, Copy, uDebug)]
#[repr(C, align(4))]
pub struct RDES {
    /// Content
    pub v: [u32; 8],
}

impl RDES {
    /// New blank descriptor
    pub fn new() -> RDES {
        RDES { v: [0_u32; 8] }
    }
}

/// TX descriptor field masks for the first word (RDES0)
/// See datasheet Table 23-8
#[derive(Clone, Copy, uDebug)]
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
    DBE = 1 << 2,
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
#[derive(Clone, Copy, uDebug)]
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
