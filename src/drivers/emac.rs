//! Drivers for TM4C129's EMAC/PHY media access control peripherals
use tm4c129x_hal::{
    sysctl::PllOutputFrequency,
    tm4c129x::{EMAC0, FLASH_CTRL},
};

/// Get preprogrammed MAC address from ROM
pub fn get_rom_macaddr(flash: &FLASH_CTRL) -> [u8; 6] {
    // Unpack address values from register structure
    // They're stored as u32 in read-only flash memory programmed by the mfr
    let addr0: [u8; 4] = flash.userreg0.read().bits().to_be_bytes();
    let addr1: [u8; 4] = flash.userreg1.read().bits().to_be_bytes();

    // Only the least-significant 3 bytes of each register are parts of the address,
    // and they're stored in reverse order
    let addr: [u8; 6] = [addr0[3], addr0[2], addr0[1], addr1[3], addr1[2], addr1[1]];

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
    }); // Mask all rx interrupts
    emac.mmctxim.modify(|_, w| {
        w.gbf()
            .set_bit()
            .mcollgf()
            .set_bit()
            .octcnt()
            .set_bit()
            .scollgf()
            .set_bit()
    }); // Mask all tx interrupts
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
    emac.cfg.modify(|_, w| w.bl()._1024()); // 1024-bit back-off limit
    emac.cfg.modify(|_, w| w.ps().set_bit()); // Set port select (req. for MII/RMII PHY interface)

    // Set memory controller op mode
    emac.dmaopmode.modify(|_, w| w.rsf().set_bit()); // RX store-and-forward
    emac.dmaopmode.modify(|_, w| w.tsf().set_bit()); // TX store-and-forward
    emac.dmaopmode.modify(|_, w| w.ttc()._32()); // TX threshold
    emac.dmaopmode.modify(|_, w| w.rtc()._32()); // RX threshold
}

/// Configuration & TX/RX for EMAC0 peripheral using internal PHY
/// 
/// Assumes speed is 100 base T, using internal PHY, PHY uses MDIX and autonegotiation, 
/// 8-word descriptor size, mmc interrupts all masked, full duplex mode
pub struct EMACDriver {
    // EMAC
    emac: EMAC0,
    system_clk_freq: PllOutputFrequency,
    src_macaddr: [u8; 6],
    checksum_offload: bool,
    preamble_length: PreambleLength,
    interframe_gap: InterFrameGap,
    backoff_limit: BackOffLimit,
    rx_store_fwd: bool,
    tx_store_fwd: bool,
    // EPHY
    // phy_mdix: bool,
    // phy_autonegotiate: bool,
    // Direct Memory Access controller
    tx_thresh: TXThresholdDMA,
    rx_thresh: RXThresholdDMA,
    rx_burst_size: BurstSizeDMA,
    tx_burst_size: BurstSizeDMA,
    // RX/TX structures
    // rx_descriptor
}

impl EMACDriver {
    /// Configure PHY
    ///
    /// This must be run before resetting phy, then resetting emac, then doing cfg_emac!!
    ///
    /// The settings done here are not cleared by reset & must be latched with resets in the proper order
    fn cfg_ephy(&self) {
        self.emac.pc.modify(|_, w| w.phyext().clear_bit()); // Use internal PHY (disable external)
        self.emac.pc.modify(|_, w| w.mdixen().set_bit()); // Enable MDIX
        self.emac.pc.modify(|_, w| w.anen().set_bit()); // Enable autonegotiation
        self.emac.cfg.modify(|_, w| w.fes().set_bit()); // Speed 100 base T
        self.emac.cfg.modify(|_, w| w.dupm().set_bit()); // Duplex mode full
    }

    /// Configure EMAC
    ///
    /// This must be done in a specific order - see docs for cfg_ephy
    fn cfg_emac(&self) {
        // Assumptions
        self.emac.cfg.modify(|_, w| w.ps().set_bit()); // Set port select (req. for MII/RMII PHY interface)
        self.emac.cfg.modify(|_, w| w.dupm().set_bit()); // Full duplex mode (reassert after reset)
        self.emac.dmabusmod.modify(|_, w| w.atds().set_bit()); // 8-word descriptor size

        // Burst transfer limits
        // Just using the values the mfr uses here - not sure what the tradeoffs are
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
            false => self.emac.dmabusmod.modify(|_, w| w.usp().set_bit()) // RX and TX burst limits are different
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

    }

    async fn transmit(data: &[u8]) {}

    async fn receive(data: &mut [u8]) {}
}

/// Choices of speed standard.
// #[allow(missing_docs)]
// pub enum Speed {
//     _10,
//     _100,
// }

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
