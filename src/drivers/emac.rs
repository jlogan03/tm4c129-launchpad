//! Drivers for TM4C129's EMAC/PHY media access control peripherals
use tm4c129x_hal::{tm4c129x::{EMAC0, FLASH_CTRL}, sysctl::PllOutputFrequency};

/// Get preprogrammed MAC address from ROM
pub fn get_rom_macaddr(flash: &FLASH_CTRL) -> [u8; 6] {
    // Unpack address values from register structure
    // They're stored as u32 in read-only flash memory programmed by the mfr
    let addr0: [u8; 4] = flash.userreg0.read().bits().to_be_bytes();
    let addr1: [u8; 4] = flash.userreg1.read().bits().to_be_bytes();

    // Only the least-significant 3 bytes of each register are parts of the address,
    // and they're stored in reverse order
    let addr: [u8; 6] = [
        addr0[3],
        addr0[2],
        addr0[1],
        addr1[3],
        addr1[2],
        addr1[1], 
    ];

    addr
}

/// Configure EMAC to use internal PHY & configure internal PHY
pub fn phy_cfg(emac: &EMAC0) {
    emac.pc.modify(|_, w| w.phyext().clear_bit()); // Use internal PHY (disable external)
    emac.pc.modify(|_, w| w.mdixen().set_bit()); // Enable MDIX
    emac.pc.modify(|_, w| w.anen().set_bit()); // Enable autonegotiation
    emac.cfg.modify(|_, w| w.fes().set_bit()); // Speed 100 base T
    emac.cfg.modify(|_, w| w.dupm().set_bit()); // Duplex mode full
}

/// Configure EMAC memory controller and clock
/// Assumes system clock is set to 120MHz
pub fn emac_init(emac: &EMAC0) {
    // Set up direct memory access
    let rxburst = 4_u8; // RX DMA controller max memory transfer size in words
    let txburst = 4_u8; // If these are different sizes, unset fixed burst
    emac.dmabusmod.modify(|_, w| w.usp().clear_bit()); // RX and TX burst limits are the same
    emac.dmabusmod.modify(|_, w| w.atds().set_bit()); // Alternate descriptor size

    // Burst transfer limits
    // Just using the values the mfr uses here - not sure what the tradeoffs are
    // Pretty sure this is only unsafe due to old-style conversion
    // to 5-bit integer; should be able to update upstream calc to be safe
    unsafe {
        emac.dmabusmod.modify(|_, w| w.rpbl().bits(rxburst));
        emac.dmabusmod.modify(|_, w| w.pbl().bits(txburst));
    };

    // MII (communication between EMAC and EPHY)
    emac.miiaddr.modify(|_, w| w.cr()._100_150()); // Set clock divider to match system

    // Disable interrupts (set interrupt masks)
    emac.mmcrxim.modify(|_, w| {
        w.algnerr()
            .set_bit()
            .crcerr()
            .set_bit()
            .gbf()
            .set_bit()
            .ucgf()
            .set_bit()
    });  // Mask all rx interrupts
    emac.mmctxim.modify(|_, w| {
        w.gbf()
            .set_bit()
            .mcollgf()
            .set_bit()
            .octcnt()
            .set_bit()
            .scollgf()
            .set_bit()
    });  // Mask all tx interrupts
}

/// Configure EMAC (must run phy_cfg, reset, then emac_init, then reset, then run emac_cfg)
/// Note this configures to use the processor-offloaded ethernet checksum generator and IP/UDP
/// checksum validator - ethernet frames should be given with the checksum zeroed-out
pub fn emac_cfg(emac: &EMAC0) {
    // Setup up EMAC transmission behavior
    emac.cfg.modify(|_, w| w.dupm().set_bit()); // Full duplex mode
    emac.cfg.modify(|_, w| w.ipc().set_bit()); // Checksum offload
    emac.cfg.modify(|_, w| w.prelen()._7()); // 7-byte preamble
    emac.cfg.modify(|_, w| w.ifg()._96()); // 96-bit iter-frame gap
    emac.cfg.modify(|_, w| w.bl()._1024());  // 1024-bit back-off limit
    emac.cfg.modify(|_, w| w.ps().set_bit());  // Set port select (req. for MII/RMII PHY interface)

    // Set memory controller op mode
    emac.dmaopmode.modify(|_, w| w.rsf().set_bit());  // RX store-and-forward
    emac.dmaopmode.modify(|_, w| w.tsf().set_bit());  // TX store-and-forward 
    emac.dmaopmode.modify(|_, w| w.ttc()._32());  // TX threshold
    emac.dmaopmode.modify(|_, w| w.rtc()._32());  // RX threshold
}


/// Configuration & TX/RX for EMAC0 peripheral using internal PHY
pub struct EMACDriver {
    // EMAC
    emac: EMAC0,
    system_clk_freq: PllOutputFrequency,
    src_macaddr: [u8; 6],
    speed: Speed,
    checksum_offload: bool,
    preamble_length: PreambleLength,
    interframe_gap: InterFrameGap,
    backoff_limit: BackOffLimit,
    rx_store_fwd: bool,
    tx_store_fwd: bool,
    full_duplex: bool,
    // EPHY
    phy_mdix: bool,
    phy_autonegotiate: bool,
    // Direct Memory Access controller
    tx_thresh: TXThresholdDMA,
    rx_thresh: RXThresholdDMA,
    rx_burst_size: BurstSizeDMA,
    tx_burst_size: BurstSizeDMA,
    // RX/TX structures
    
}

impl EMACDriver {
    /// Configure PHY
    /// 
    /// This must be run before resetting phy, then resetting emac, then doing cfg_emac!!
    /// 
    /// The settings done here are not cleared by reset & must be latched with resets in the proper order
    fn cfg_phy(&self) {
        self.emac.pc.modify(|_, w| w.phyext().clear_bit()); // Use internal PHY (disable external)

        match self.phy_mdix {
            true => self.emac.pc.modify(|_, w| w.mdixen().set_bit()), // Enable MDIX
            false => self.emac.pc.modify(|_, w| w.mdixen().clear_bit()), // Disable MDIX
        }
        
        self.emac.pc.modify(|_, w| w.anen().set_bit()); // Enable autonegotiation
        self.emac.cfg.modify(|_, w| w.fes().set_bit()); // Speed 100 base T
        self.emac.cfg.modify(|_, w| w.dupm().set_bit()); // Duplex mode full
    }

    async fn transmit(data: &[u8]) {

    }

    async fn receive(data: &mut [u8]) {

    }
}

/// Choices of speed standard.
#[allow(missing_docs)]
pub enum Speed {
    _10,
    _100
}

/// Choices of preamble length in bytes.
/// 
/// This is the number of alternating 0-1 bits transmitted at the start of each frame
/// in order to synchronize clocks between the transmitter and receiver.
#[allow(missing_docs)]
pub enum PreambleLength {
    _3,
    _5,
    _7
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
    _96
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
    _1024
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
    _256
}

/// RX memory transfer threshold for memory controller
/// 
/// This is the number of bytes in the buffer required to trigger a transfer.
#[allow(missing_docs)]
pub enum RXThresholdDMA {
    _32,
    _64,
    _96,
    _128
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
    _64,
    _128,
    _256
}
