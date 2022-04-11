//! Drivers for TM4C129's EMAC/PHY media access control peripherals

pub mod rdes;
pub mod socket;
pub mod tdes; // TX descriptor ring definitions // RX ...

use tm4c129x_hal::{
    sysctl::{PllOutputFrequency, PowerControl},
    tm4c129x::{EMAC0, FLASH_CTRL},
};

use self::rdes::*;
use self::tdes::*;

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

/// Configuration of EMAC0 peripheral as a UDP socket for a single connection
///
/// Assumes speed is 100 base T in full duplex mode, using internal PHY, PHY uses MDIX and autonegotiation,
/// 8-word descriptor size, MMC interrupts all masked, using source address from descriptor (populated by software)
///
/// Note the DMA controller requires the descriptors to be aligned on 32-bit words instead of bytes,
/// hence the repr(align(4)). We also need safely-made pointers to address the actual location of the
/// values within the struct, hence the repr(C).
#[repr(C, align(4))]
pub struct EthernetDriver {
    // EMAC
    /// EMAC peripheral registers
    pub emac: EMAC0,
    /// System clock frequency
    pub system_clk_freq: PllOutputFrequency,

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

    // Addressing
    /// Source mac address
    pub src_macaddr: [u8; 6],
    /// Source IP address
    pub src_ipaddr: [u8; 4],
    /// Source port
    pub src_port: u16,
    /// Destination IP address
    pub dst_ipaddr: [u8; 4],
    /// Destination port
    pub dst_port: u16,

    // RX/TX structures
    /// Volatile access to TX descriptor list
    pub txdl: TXDL,
    /// Volatile access to RX descriptor list
    pub rxdl: RXDL,
}

impl EthernetDriver {
    // Send raw ethernet frame that includes destination address, etc.
    // pub async fn transmit(data: &[u8]) {}

    // Receive raw data
    // pub async fn receive(data: &mut [u8]) {}

    /// Build and initialize
    pub(crate) fn new<F, G>(
        pc: &PowerControl,
        ephy_reset: F,
        emac_reset: G,
        emac: EMAC0,
        system_clk_freq: PllOutputFrequency,
        src_macaddr: [u8; 6],
        src_ipaddr: [u8; 4],
        src_port: u16,
        dst_ipaddr: [u8; 4],
        dst_port: u16,
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
    ) -> EthernetDriver
    where
        F: Fn(&PowerControl) -> EphyR,
        G: Fn(&PowerControl) -> EmacR,
    {
        // Get raw pointers to the first descriptors in each ring
        // let txdladdr: *mut TDES = emac.txdladdr.read().bits() as *mut TDES;
        let rxdladdr: *mut RDES = emac.rxdladdr.read().bits() as *mut RDES;

        // Build driver struct & initialize descriptor lists from SRAM
        let mut enet: EthernetDriver = EthernetDriver {
            emac: emac,
            system_clk_freq: system_clk_freq,
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

            src_macaddr: src_macaddr,
            src_ipaddr: src_ipaddr,
            src_port: src_port,
            dst_ipaddr: dst_ipaddr,
            dst_port: dst_port,

            txdl: TXDL::new(),
            rxdl: RXDL::new(),
        };
        // Write registers and populate buffers
        enet.init(pc, |pc| ephy_reset(pc), |pc| emac_reset(pc));

        enet
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

        // Initialize descriptor lists and buffers

        // Make sure DMA and EMAC are stopped in order to set new descriptor list pointers
        self.stop();

        // Set descriptor list pointers
        self.emac.txdladdr.write(|w| unsafe{w.bits(self.txdl.txdladdr as u32)});

        // Start DMA and EMAC transmit/receive
        self.start();
    }

    /// Stop transmit and receive DMA then EMAC
    pub fn stop(&mut self) {
        // DMA transmit/receive
        self.emac.dmaopmode.modify(|_, w| w.st().clear_bit());
        self.emac.dmaopmode.modify(|_, w| w.sr().clear_bit());

        // EMAC transmit/receive
        self.emac.cfg.modify(|_, w| w.te().clear_bit());
        self.emac.cfg.modify(|_, w| w.re().clear_bit());
    }

    /// Start transmit and receive DMA then EMAC
    pub fn start(&mut self) {
        // DMA transmit/receive
        self.emac.dmaopmode.modify(|_, w| w.st().set_bit());
        self.emac.dmaopmode.modify(|_, w| w.sr().set_bit());

        // EMAC transmit/receive
        self.emac.cfg.modify(|_, w| w.te().set_bit());
        self.emac.cfg.modify(|_, w| w.re().set_bit());
    }

    /// Attempt to send an ethernet frame that has been reduced to bytes
    pub unsafe fn transmit<const N: usize>(
        &mut self,
        data: [u8; N],
        attempts: usize,
    ) -> Result<usize, ()> {
        for i in 0..attempts {
            if self.txdl.is_owned() {
                // We own the current descriptor; load our data into the buffer and tell the DMA to send it
                //    Load data into buffer
                let mut _buffer: [u8; N] = *(self.txdl.get_buffer_pointer() as *mut [u8; N]);
                _buffer = data;
                //    Set buffer length
                self.txdl.set_buffer_size(N as u16);
                //    Set common settings
                self.txdl.set_tdes0(TDES0::CRCR); // Enable ethernet checksum replacement
                self.txdl.set_tdes0(TDES0::CicFull); // Full calculation of IPV4 and TCP/UDP checksums using pseudoheader
                // self.txdl.set_tdes0(TDES0::TTSE); // Transmit IEEE-1588 64-bit timestamp
                self.txdl.set_tdes1(TDES1::SaiReplace); // Replace source MAC address in frame with value programmed into peripheral

                self.txdl.give(); // Give this descriptor & buffer back to the DMA

                return Ok(i);
            } else {
                // We do not own the current descriptor and can't use it to send data
                // Go to the next descriptor
                self.txdl.next();
            }
        }
        return Err(());
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

///
pub enum EthernetError {
    /// The current descriptor is owned by the DMA and can't be used to transmit or receive
    DescriptorOwnedError,
}
