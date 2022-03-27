//! Drivers for TM4C129's EMAC/PHY media access control peripherals
use tm4c129x_hal::{
    sysctl::{PllOutputFrequency, PowerControl},
    tm4c129x::{EMAC0, FLASH_CTRL},
};

use volatile::Volatile;

/// Empty type to guarantee that the emac_reset closure passed to EMACDriver::init has the correct effects
pub(crate) struct EmacR;

/// Empty type to guarantee that the ephy_reset closure passed to EMACDriver::init has the correct effects
pub(crate) struct EphyR;

/// Get preprogrammed MAC address from ROM
pub fn get_rom_macaddr(flash: &FLASH_CTRL) -> [u8; 6] {
    // Unpack address values from register structure
    // They're stored as u32 in read-only flash memory programmed by the mfr
    // Only the least-significant 3 bytes of each register are parts of the address,
    // and they're stored in reverse order
    let addr0: [u8; 4] = flash.userreg0.read().bits().to_be_bytes();
    let addr1: [u8; 4] = flash.userreg1.read().bits().to_be_bytes();
    let addr: [u8; 6] = [addr0[3], addr0[2], addr0[1], addr1[3], addr1[2], addr1[1]];
    addr
}

/// Configuration & TX/RX for EMAC0 peripheral
///
/// Assumes speed is 100 base T in full duplex mode, using internal PHY, PHY uses MDIX and autonegotiation,
/// 8-word descriptor size, MMC interrupts all masked, using source address from descriptor (populated by software)
pub struct EMACDriver<const M: usize, const N: usize, const P: usize, const Q: usize>
{
    // EMAC
    /// EMAC peripheral registers
    pub emac: EMAC0,
    /// System clock frequency
    pub system_clk_freq: PllOutputFrequency,
    /// Source mac address
    pub src_macaddr: [u8; 6],
    /// Use processor-offloaded checksum calc peripheral
    pub checksum_offload: bool,
    /// Clock-sync preamble length
    pub preamble_length: PreambleLength,
    /// Inter-frame silence duration in bits
    pub interframe_gap: InterFrameGap,
    /// Exponential backoff saturation limit
    pub backoff_limit: BackOffLimit,
    /// RX store-and-forward
    pub rx_store_fwd: bool,
    /// TX store-and-forward
    pub tx_store_fwd: bool,

    // Direct Memory Access controller
    /// TX DMA transfer threshold
    pub tx_thresh: TXThresholdDMA,
    /// RX DMA transfer threshold
    pub rx_thresh: RXThresholdDMA,
    /// RX DMA burst size
    pub rx_burst_size: BurstSizeDMA,
    /// TX DMA burst size
    pub tx_burst_size: BurstSizeDMA,

    // RX/TX structures
    /// Volatile access to TX buffer descriptors
    pub tx_descriptors: [[u32; 8]; N],
    /// Volatile access to TX buffer data
    pub tx_buffers: [[u8; M];  N],
    /// Volatile access to RX buffer descriptors
    pub rx_descriptors: [[u32; 8]; Q],
    /// Volatile access to RX buffer data
    pub rx_buffers: [[u8; P]; Q],
}

impl<const M: usize, const N: usize, const P: usize, const Q: usize> EMACDriver<M, N, P, Q>
{
    /// Send raw ethernet frame that includes destination address, etc.
    pub async fn transmit(data: &[u8]) {}

    /// Receive raw data
    pub async fn receive(data: &mut [u8]) {}

    /// Build and initialize
    pub(crate) fn new<F, G>(
        pc: &PowerControl,
        ephy_reset: F,
        emac_reset: G,
        emac: EMAC0,
        system_clk_freq: PllOutputFrequency,
        src_macaddr: [u8; 6],
        checksum_offload: bool,
        preamble_length: PreambleLength,
        interframe_gap: InterFrameGap,
        backoff_limit: BackOffLimit,
        rx_store_fwd: bool,
        tx_store_fwd: bool,
        tx_thresh: TXThresholdDMA,
        rx_thresh: RXThresholdDMA,
        rx_burst_size: BurstSizeDMA,
        tx_burst_size: BurstSizeDMA,
    ) -> EMACDriver<M, N, P, Q>
    where
        F: Fn(&PowerControl) -> EphyR,
        G: Fn(&PowerControl) -> EmacR,
    {
        let mut emacdriver: EMACDriver<M, N, P, Q> = EMACDriver {
            emac: emac,
            system_clk_freq: system_clk_freq,
            src_macaddr: src_macaddr,
            checksum_offload: checksum_offload,
            preamble_length: preamble_length,
            interframe_gap: interframe_gap,
            backoff_limit: backoff_limit,
            rx_store_fwd: rx_store_fwd,
            tx_store_fwd: tx_store_fwd,
            rx_burst_size: rx_burst_size,
            tx_burst_size: tx_burst_size,
            rx_thresh: rx_thresh,
            tx_thresh: tx_thresh,

            tx_descriptors: [[0_u32; 8]; N],
            tx_buffers: [[0_u8; M]; N],
            rx_descriptors: [[0_u32; 8]; Q],
            rx_buffers: [[0_u8; P]; Q],
        };
        // Write registers and populate buffers
        emacdriver.init(pc, |pc| ephy_reset(pc), |pc| emac_reset(pc));

        emacdriver
    }

    /// 1. Reset to clear configuration
    ///
    /// 2. Set latching configuration & reset so that it takes effect
    ///
    /// 3. Apply non-latching configuration
    /// 
    /// 4. Populate buffers
    pub(crate) fn init<F, G>(&mut self, pc: &PowerControl, ephy_reset: F, emac_reset: G)
    where
        F: Fn(&PowerControl) -> EphyR,
        G: Fn(&PowerControl) -> EmacR,
    {
        // -------------------- LATCHING CONFIGURATION ------------------------
        // Some of this configuration survives peripheral reset & takes effect after reset

        // Reset PHY then MAC to clear configuration
        ephy_reset(pc);
        emac_reset(pc);

        // Assumptions
        self.emac.pc.modify(|_, w| w.phyext().clear_bit()); // Use internal PHY (disable external)
        self.emac.pc.modify(|_, w| w.mdixen().set_bit()); // Enable MDIX
        self.emac.pc.modify(|_, w| w.anen().set_bit()); // Enable autonegotiation
        self.emac.cfg.modify(|_, w| w.fes().set_bit()); // Speed 100 base T
        self.emac.cfg.modify(|_, w| w.dupm().set_bit()); // Full duplex mode

        // Reset PHY then MAC to latch configuration
        ephy_reset(pc);
        emac_reset(pc);

        // -------------------- NON-LATCHING CONFIGURATION --------------------
        // This configuration is cleared on peripheral reset and takes effect more-or-less immediately during operation

        // Assumptions
        self.emac.cfg.modify(|_, w| w.ps().set_bit()); // Set port select (req. for MII/RMII PHY interface)
        self.emac.cfg.modify(|_, w| w.dupm().set_bit()); // Full duplex mode (reassert after reset)
        self.emac.dmabusmod.modify(|_, w| w.atds().set_bit()); // 8-word descriptor size

        // Set MAC address
        // Per mfr, addr0l must be set last because writing that register is the trigger
        // to latch the new address in hardware.
        unsafe {
            let addr = self.src_macaddr;
            self.emac
                .addr2h
                .modify(|_, w| w.addrhi().bits(addr[4] as u16));
            self.emac
                .addr2l
                .modify(|_, w| w.addrlo().bits(addr[5] as u32));
            self.emac
                .addr1h
                .modify(|_, w| w.addrhi().bits(addr[2] as u16));
            self.emac
                .addr1l
                .modify(|_, w| w.addrlo().bits(addr[3] as u32));
            self.emac
                .addr0h
                .modify(|_, w| w.addrhi().bits(addr[1] as u16));
            self.emac
                .addr0l
                .modify(|_, w| w.addrlo().bits(addr[0] as u32));
        }

        // Burst transfer limits
        // Not sure what the tradeoffs are
        // Pretty sure this is only unsafe due to old-style conversion
        // to 5-bit integer; should be able to update upstream calc to be safe
        let rxburst = match self.rx_burst_size {
            // No 8x multiplier needed
            BurstSizeDMA::_1 => 1,
            BurstSizeDMA::_2 => 2,
            BurstSizeDMA::_4 => 4,
            BurstSizeDMA::_8 => 8,
            BurstSizeDMA::_16 => 16,
            BurstSizeDMA::_32 => 32,
        }; // RX DMA controller max memory transfer size in words
        let txburst = match self.rx_burst_size {
            // No 8x multiplier needed
            BurstSizeDMA::_1 => 1,
            BurstSizeDMA::_2 => 2,
            BurstSizeDMA::_4 => 4,
            BurstSizeDMA::_8 => 8,
            BurstSizeDMA::_16 => 16,
            BurstSizeDMA::_32 => 32,
        }; // If these are different sizes, unset fixed burst
           // Set flag for whether programmable burst limits are the same for RX and TX
        match rxburst == txburst {
            true => self.emac.dmabusmod.modify(|_, w| w.usp().clear_bit()), // RX and TX burst limits are the same
            false => self.emac.dmabusmod.modify(|_, w| w.usp().set_bit()), // RX and TX burst limits are different
        }
        // Set actual programmable burst limits
        unsafe {
            self.emac.dmabusmod.modify(|_, w| w.rpbl().bits(rxburst));
            self.emac.dmabusmod.modify(|_, w| w.pbl().bits(txburst));
        };

        // Disable interrupts (set interrupt masks)
        self.emac.mmcrxim.modify(|_, w| {
            w.algnerr()
                .set_bit()
                .crcerr()
                .set_bit()
                .gbf()
                .set_bit()
                .ucgf()
                .set_bit()
        }); // Mask all rx interrupts
        self.emac.mmctxim.modify(|_, w| {
            w.gbf()
                .set_bit()
                .mcollgf()
                .set_bit()
                .octcnt()
                .set_bit()
                .scollgf()
                .set_bit()
        }); // Mask all tx interrupts

        // MII (communication between EMAC and EPHY)
        match self.system_clk_freq {
            // These are below the minimum and may cause hardware UB
            PllOutputFrequency::_6mhz => self.emac.miiaddr.modify(|_, w| w.cr()._20_35()),
            PllOutputFrequency::_12mhz => self.emac.miiaddr.modify(|_, w| w.cr()._20_35()),
            // These are nominal
            PllOutputFrequency::_24mhz => self.emac.miiaddr.modify(|_, w| w.cr()._20_35()),
            PllOutputFrequency::_30mhz => self.emac.miiaddr.modify(|_, w| w.cr()._20_35()),
            PllOutputFrequency::_48mhz => self.emac.miiaddr.modify(|_, w| w.cr()._35_60()),
            PllOutputFrequency::_60mhz => self.emac.miiaddr.modify(|_, w| w.cr()._35_60()),
            PllOutputFrequency::_120mhz => self.emac.miiaddr.modify(|_, w| w.cr()._100_150()),
        }

        // Setup up EMAC transmission behavior
        match self.checksum_offload {
            true => self.emac.cfg.modify(|_, w| w.ipc().set_bit()), // Checksum offload
            false => self.emac.cfg.modify(|_, w| w.ipc().clear_bit()), // Use software checksum
        }

        match self.preamble_length {
            PreambleLength::_3 => self.emac.cfg.modify(|_, w| w.prelen()._3()),
            PreambleLength::_5 => self.emac.cfg.modify(|_, w| w.prelen()._5()),
            PreambleLength::_7 => self.emac.cfg.modify(|_, w| w.prelen()._7()),
        }

        match self.interframe_gap {
            InterFrameGap::_40 => self.emac.cfg.modify(|_, w| w.ifg()._40()),
            InterFrameGap::_48 => self.emac.cfg.modify(|_, w| w.ifg()._48()),
            InterFrameGap::_56 => self.emac.cfg.modify(|_, w| w.ifg()._56()),
            InterFrameGap::_64 => self.emac.cfg.modify(|_, w| w.ifg()._64()),
            InterFrameGap::_72 => self.emac.cfg.modify(|_, w| w.ifg()._72()),
            InterFrameGap::_80 => self.emac.cfg.modify(|_, w| w.ifg()._80()),
            InterFrameGap::_88 => self.emac.cfg.modify(|_, w| w.ifg()._88()),
            InterFrameGap::_96 => self.emac.cfg.modify(|_, w| w.ifg()._96()),
        }

        match self.backoff_limit {
            BackOffLimit::_2 => self.emac.cfg.modify(|_, w| w.bl()._2()),
            BackOffLimit::_8 => self.emac.cfg.modify(|_, w| w.bl()._8()),
            BackOffLimit::_256 => self.emac.cfg.modify(|_, w| w.bl()._256()),
            BackOffLimit::_1024 => self.emac.cfg.modify(|_, w| w.bl()._1024()),
        }

        // Set memory controller op mode
        match self.rx_store_fwd {
            true => self.emac.dmaopmode.modify(|_, w| w.rsf().set_bit()),
            false => self.emac.dmaopmode.modify(|_, w| w.rsf().clear_bit()),
        }
        match self.tx_store_fwd {
            true => self.emac.dmaopmode.modify(|_, w| w.tsf().set_bit()),
            false => self.emac.dmaopmode.modify(|_, w| w.tsf().clear_bit()),
        }
        match self.tx_thresh {
            TXThresholdDMA::_16 => self.emac.dmaopmode.modify(|_, w| w.ttc()._16()),
            TXThresholdDMA::_24 => self.emac.dmaopmode.modify(|_, w| w.ttc()._24()),
            TXThresholdDMA::_32 => self.emac.dmaopmode.modify(|_, w| w.ttc()._32()),
            TXThresholdDMA::_40 => self.emac.dmaopmode.modify(|_, w| w.ttc()._40()),
            TXThresholdDMA::_64 => self.emac.dmaopmode.modify(|_, w| w.ttc()._64()),
            TXThresholdDMA::_128 => self.emac.dmaopmode.modify(|_, w| w.ttc()._128()),
            TXThresholdDMA::_192 => self.emac.dmaopmode.modify(|_, w| w.ttc()._192()),
            TXThresholdDMA::_256 => self.emac.dmaopmode.modify(|_, w| w.ttc()._256()),
        }
        match self.rx_thresh {
            RXThresholdDMA::_32 => self.emac.dmaopmode.modify(|_, w| w.rtc()._32()),
            RXThresholdDMA::_64 => self.emac.dmaopmode.modify(|_, w| w.rtc()._64()),
            RXThresholdDMA::_96 => self.emac.dmaopmode.modify(|_, w| w.rtc()._96()),
            RXThresholdDMA::_128 => self.emac.dmaopmode.modify(|_, w| w.rtc()._128()),
        }

        // Set up buffers per datasheet section 23.3.2.5
        // "Descriptor" structure is the software interface with the direct memory access controller.
        // Hardware interprets Descriptors as members of a linked list with a particular format.
        // Each Descriptor contains a pointer to its buffer segment, a pointer to the next descriptor,
        // and information about how the content of the buffer should be interpreted
        //
        // Assumes we are using 8-word descriptors ("alternate descriptor size" peripheral config).

        //    Populate TX descriptor pointers
        self.tx_descriptors[N-1][3] = (&self.tx_descriptors[0] as *const _) as u32;  // Point last descriptor back at the first
        self.tx_descriptors[N-1][2] = (&self.tx_buffers[N-1] as *const _) as u32;  // Populate last descriptor's buffer pointer
        for i in 0..N - 1 {
            let next_descr_addr = (&self.tx_descriptors[i + 1] as *const _) as u32;  // Memory address of next descriptor
            self.tx_descriptors[i][3] = next_descr_addr;
            let buffer_addr: u32 = (&self.tx_buffers[i] as *const _) as u32;  // Memory address of buffer segment
            self.tx_descriptors[i][2] = buffer_addr;
        }
        //    Populate RX descriptor pointers
        self.rx_descriptors[Q-1][3] = (&self.rx_descriptors[0] as *const _) as u32;  // Point last descriptor back at the first
        self.rx_descriptors[Q-1][2] = (&self.rx_buffers[Q-1] as *const _) as u32;  // Populate last descriptor's buffer pointer
        for i in 0..Q - 1 {
            let next_descr_addr = (&self.rx_descriptors[i + 1] as *const _) as u32;  // Memory address of next descriptor
            self.rx_descriptors[i][3] = next_descr_addr;
            let buffer_addr: u32 = (&self.rx_buffers[i] as *const _) as u32;  // Memory address of buffer segment
            self.rx_descriptors[i][2] = buffer_addr;
        }
        
        // Placeholder volatile access for testing
        let mut dv = Volatile::new(&mut self.tx_descriptors[0]);
        let val = dv.read();
        dv.write(val);

    }
}

/// Choices of preamble length in bytes.
///
/// This is the number of alternating 0-1 bits transmitted at the start of each frame
/// in order to synchronize clocks between the transmitter and receiver.
#[allow(missing_docs)]
pub enum PreambleLength {
    _3,
    _5,
    _7,
}

/// Choices of interframe gap length in bits.
///
/// This is the duration of radio-silence used to signal the end of a transmission frame.
#[allow(missing_docs)]
pub enum InterFrameGap {
    _40,
    _48,
    _56,
    _64,
    _72,
    _80,
    _88,
    _96,
}

/// Choices of back-off limit in bits.
///
/// This is a cap on the rescheduling duration given by the exponential back-off algorithm
/// in the event of repeated collisions during transmission.
#[allow(missing_docs)]
pub enum BackOffLimit {
    _2,
    _8,
    _256,
    _1024,
}

/// TX memory transfer threshold for memory controller
///
/// This is the number of bytes in the buffer required to trigger a transfer.
#[allow(missing_docs)]
pub enum TXThresholdDMA {
    _16,
    _24,
    _32,
    _40,
    _64,
    _128,
    _192,
    _256,
}

/// RX memory transfer threshold for memory controller
///
/// This is the number of bytes in the buffer required to trigger a transfer.
#[allow(missing_docs)]
pub enum RXThresholdDMA {
    _32,
    _64,
    _96,
    _128,
}

/// RX memory transfer burst size in 32-bit words
#[allow(missing_docs)]
pub enum BurstSizeDMA {
    // Settings that do not require 8x flag
    _1,
    _2,
    _4,
    _8,
    _16,
    _32,
    // Settings that do require 8x flag
    // The 8x flag must match both RX and TX burst size, which introduces complicated logic
    // that is difficult to implement without introducing panic branches or unexpected behavior
    // to resolve incompatible input combinations

    // _64,
    // _128,
    // _256,
}

/// TX descriptor fields for the first word (TDES0)
/// See datasheet Table 23-2
pub(crate) enum TXDescriptor0 {
    // Status flag set by the DMA or the user to transfer ownership
    OWN = 1 << 31,  // Flag that DMA owns this descriptor
    // To be set by the user
    TI = 1 << 30,  // Interrupt at end of transmission
    LS = 1 << 29,  // Last segment of the frame
    FS = 1 << 28,  // First segment of the frame
    DC = 1 << 27,  // Disable CRC check
    DP = 1 << 26,  // Disable zero-padding
    TTSE = 1 << 25,  // Transmit Timestamp Enable (for IEEE-1588 PTP timestamping)
    CRCR = 1 << 24,  // Enable Ethernet checksum replacement
    // CicDisable = 0, // Do not insert IPV4 or UDP/TCP checksums
    CicIPV4 = 1 << 22, // Insert IPV4 checksum only
    CicFrameOnly = 2 << 22, // Insert IPV4 checksum & TCP/UDP checksum without pseudoheader
    CicFull = 3 << 22, // Insert IPV4 checksum & TCP/UDP checksum including pseudoheader (per the actual standard)
    TER = 1 << 21, // Transmit End of Ring: this descriptor is the last one in the ring
    TCH = 1 << 20, // Transmit Chain: the second pointer field is a pointer to the next descriptor, not a buffer
    // VlanDisable = 0, // No VLAN tagging
    VlanRemove = 1 << 18, // Strip VLAN tag before transmission
    VlanInsert = 2 << 18, // Insert VLAN tag into frame (copied from register EMACVLNINCREP)
    VlanReplace = 3 << 18, // Replace existing VLAN tag in frame (copied from register EMACVLNINCREP)

    // Status flags set by the DMA
    TTSS = 1 << 17, // Timestamp was captured for this frame
    IHE = 1 << 16, // IP header error occurred
    ES = 1 << 15, // An error of any kind occurred
    JT = 1 << 14, // Jabber timeout error
    FF = 1 << 13, // Frame flushed by software
    IPE = 1 << 12, // IP payload error
    LOC = 1 << 11, // Loss of Carrier error
    NC = 1 << 10, // No Carrier error
    LC = 1 << 9, // Late Collision error
    EC = 1 << 8, // Excessive Collision error

    VF = 1 << 7, // Transmitted frame was a VLAN frame
    CC = 0b111 << 3, // Collision count
    
    ED = 1 << 2, // Excessive Deferral error
    UF = 1 << 1, // Underflow error
    DB = 1, // Deferral bit (only relevant to half-duplex mode)
}

// TX descriptor fields for second word (TDES1)
// See datasheet table 23-3
pub(crate) enum TXDescriptor1 {
    SA1 = 1 << 31,
    SaiInsert = 1 << 29,
    SaiReplace = 2 << 29,

}