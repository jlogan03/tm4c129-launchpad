//! Drivers for TM4C129's EMAC/PHY media access control peripherals

pub mod rdes; // RX ...
pub mod socket; // UDP socket
pub mod status; // Status register parsers
pub mod tdes; // TX descriptor ring definitions

use tm4c129x_hal::{
    sysctl::{PllOutputFrequency, PowerControl},
    tm4c129x::{EMAC0, FLASH_CTRL},
};

use ufmt::derive::uDebug;

use self::rdes::*;
pub use self::rdes::{RXBUFSIZE, RXDESCRS};

use self::tdes::*;
pub use self::tdes::{TDES0, TXBUFSIZE, TXDESCRS};

use self::status::{DmaStatus, EmacStatus};

/// Empty type to guarantee that the emac_reset closure passed to EMACDriver::init has the correct effects
pub(crate) struct EmacR;

/// Empty type to guarantee that the ephy_reset closure passed to EMACDriver::init has the correct effects
pub(crate) struct EphyR;

/// PHY MII Interrupt Status 1 address
const EPHYMISR1: u8 = 0x12;

/// PHY MII Interrupt Status 2 address
const EPHYMISR2: u8 = 0x13;

/// PHY Basic Mode Control address
const EPHYBMCR: u8 = 0x0;

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

/// Configuration of EMAC0 peripheral as a full-duplex 100-baseT ethernet controller.
///
/// Assumes speed is 100 base T in full duplex mode, using internal PHY, PHY uses MDIX and autonegotiation,
/// 8-word descriptor size, MMC interrupts all masked, using source address from descriptor (populated by software)
///
/// Note the DMA controller requires the descriptors to be aligned on 32-bit words instead of bytes,
/// hence the repr(align(4)). We also need safely-made pointers to address the actual location of the
/// values within the struct, hence the repr(C).
#[repr(C, align(4))]
pub struct Ethernet {
    // EMAC
    /// EMAC peripheral registers
    pub emac: EMAC0,
    /// System clock frequency
    pub system_clk_freq: PllOutputFrequency,

    /// Clock-sync preamble length
    pub preamble_length: PreambleLength,
    /// Inter-frame silence duration in bits
    pub interframe_gap: InterFrameGap,

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

    // RX/TX structures
    /// Volatile access to TX descriptor list
    pub txdl: TXDL,
    /// Volatile access to RX descriptor list
    pub rxdl: RXDL,
}

impl Ethernet {
    /// Build and initialize
    pub(crate) fn new<F>(
        pc: &PowerControl,
        ephy_reset: F,
        emac: EMAC0,
        system_clk_freq: PllOutputFrequency,
        src_macaddr: [u8; 6],
        preamble_length: PreambleLength,
        interframe_gap: InterFrameGap,
        tx_thresh: TXThresholdDMA,
        rx_thresh: RXThresholdDMA,
        rx_burst_size: BurstSizeDMA,
        tx_burst_size: BurstSizeDMA,
    ) -> Ethernet
    where
        F: Fn(&PowerControl) -> EphyR,
    {
        // Build driver struct & initialize descriptor lists and buffers
        let mut enet: Ethernet = Ethernet {
            emac: emac,
            system_clk_freq: system_clk_freq,
            preamble_length: preamble_length,
            interframe_gap: interframe_gap,
            rx_burst_size: rx_burst_size,
            tx_burst_size: tx_burst_size,
            rx_thresh: rx_thresh,
            tx_thresh: tx_thresh,

            src_macaddr: src_macaddr,

            txdl: TXDL::new(),
            rxdl: RXDL::new(),
        };

        // Write registers and populate buffers
        enet.init(pc, |pc| ephy_reset(pc));

        enet
    }

    /// Attempt to send an ethernet frame that has been reduced to bytes
    pub fn transmit<const N: usize>(&mut self, data: [u8; N]) -> Result<(), EthernetError> {
        // Check if data fits in buffer
        if N > TXBUFSIZE {
            return Err(EthernetError::BufferOverflow);
        }
        // Check if data meets minimum length required for ethernet frame
        if N < 64 {
            return Err(EthernetError::FrameTooShort);
        }

        // Attempt send
        unsafe {
            // Start from wherever the hardware is now
            self.txdl.tdesref = self.emac.hostxdesc.read().bits() as *mut TDES;

            for _ in 0..TXDESCRS {
                if self.txdl.is_owned() {
                    // We own the current descriptor; load our data into the buffer and tell the DMA to send it
                    //    Load data into buffer
                    let mut _buffer: *mut [u8; N] = self.txdl.get_buffer_pointer() as *mut [u8; N];
                    _buffer.write_volatile(data);
                    //    Set buffer length
                    self.txdl.set_buffer_size(N as u16);

                    // Transmit IEEE-1588 64-bit timestamp
                    // This should only be enabled for specific packets that are expecting it,
                    // so it's staying disabled for now until that functionality is added
                    // self.txdl.set_tdes0(TDES0::TTSE);

                    // Give ownership of this descriptor & buffer back to the DMA
                    self.txdl.give();

                    // Tell the DMA that there is demand for transmission by writing any value to the register
                    self.emac.txpolld.write(|w| w.tpd().bits(0));

                    // Make sure the transmitter is enabled
                    self.txstart();

                    return Ok(());
                } else {
                    // We do not own the current descriptor and can't use it to send data
                    // Go to the next descriptor and try that one
                    self.txstart();
                    self.txdl.next();
                }
            }
        }
        // We checked every descriptor and none of them are available
        return Err(EthernetError::DescriptorUnavailable);
    }

    /// Receive a frame of unknown size that may be up to 1522 bytes
    pub fn receive(&mut self, buf: &mut [u8; RXBUFSIZE]) -> Result<usize, EthernetError> {
        // Poll the DMA in case it has suspended
        self.emac.rxpolld.write(|w| unsafe { w.rpd().bits(0) });
        // Make sure the receive engine is enabled
        self.rxstart();

        // Start from wherever the hardware is now
        let mut bytes_received: usize = 0;

        unsafe {
            // Walk through descriptor list until we find one that is the start of a received frame
            // and is owned by software, or we have checked all the descriptors
            for _ in 0..RXDESCRS {
                let owned = self.rxdl.is_owned();

                // Poll the DMA again in case it was stalled due to lack of free descriptors
                self.emac.rxpolld.write(|w| w.rpd().bits(0));
                // Make sure the receive engine is enabled
                self.rxstart();

                if owned {
                    break;
                } else {
                    self.rxdl.next();
                }
            }

            // Did we actually find a frame to receive, or did we run out of descriptors?
            if !self.rxdl.is_owned() {
                // There is no valid data to receive from the buffer
                return Err(EthernetError::NothingToReceive);
            } else {
                // There is a frame to receive
                // We do not have jumbo frames enabled and are using store-and-forward,
                // so the entire frame should be stored in a single buffer.
                // Otherwise, we would have to handle frames spread across multiple descriptors/buffers.
                let descr_buf = self.rxdl.get_buffer_pointer() as *mut [u8; RXBUFSIZE];

                // Figure out how many bytes of data are valid
                bytes_received = self.rxdl.get_rdes0(RDES0::FL) as usize;

                // Copy valid part of buffer into target buffer, making sure we don't run off the end anywhere
                // We have to use a for-loop here instead of copying from slice to avoid a panic branch.
                // This costs about 10us of round-trip latency.
                let descr_buf_vals = descr_buf.read_volatile();
                let ix = descr_buf_vals.len().min(bytes_received).min(buf.len()).max(1);
                for i in 0..ix {
                    buf[i] = descr_buf_vals[i];
                }
            }

            // Give this descriptor back to the DMA
            self.rxdl.give();

            
            Ok(bytes_received)
        }
    }

    /// 1. Reset to clear configuration
    ///
    /// 2. Set latching configuration & reset so that it takes effect
    ///
    /// 3. Apply non-latching configuration
    ///
    /// 4. Populate buffers
    pub(crate) fn init<F>(&mut self, pc: &PowerControl, ephy_reset: F)
    where
        F: Fn(&PowerControl) -> EphyR,
    {
        // -------------------- LATCHING CONFIGURATION ------------------------
        // Some of this configuration survives peripheral reset & takes effect after reset

        // Make sure DMA and EMAC are stopped in order to allow configuration
        self.rxstop();
        self.txstop();

        // Reset PHY then MAC to clear configuration
        self.emac_reset();
        ephy_reset(pc);

        // Assumptions
        self.emac.pc.write(|w| w.phyext().clear_bit()); // Use internal PHY (disable external)
        self.emac.pc.write(|w| w.mdixen().set_bit()); // Enable MDIX
        self.emac.pc.write(|w| w.anen().set_bit()); // Enable autonegotiation
        self.emac.cfg.write(|w| w.fes().set_bit()); // Speed 100 base T
        self.emac.cfg.write(|w| w.dupm().set_bit()); // Full duplex mode

        // Reset MAC again to latch configuration
        self.emac_reset();

        // Set config-register configuration again just to be sure
        // Touching the PC register again at this point will semi-break
        // the peripheral (98%+ packet loss) until next reset
        self.emac.cfg.write(|w| w.fes().set_bit()); // Speed 100 base T
        self.emac.cfg.write(|w| w.dupm().set_bit()); // Full duplex mode

        // -------------------- NON-LATCHING CONFIGURATION --------------------
        // This configuration is cleared on peripheral reset and takes effect more-or-less immediately during operation

        // Make sure DMA and EMAC are stopped in order to allow configuration
        self.rxstop();
        self.txstop();

        // Assumptions
        self.emac.dmabusmod.write(|w| w.atds().set_bit()); // 8-word descriptor size

        // Set MAC address
        // Per mfr, addr0l must be set last because writing that register is the trigger
        // to latch the new address in hardware.
        let mut hi_bytes = [0_u8; 2];
        hi_bytes.copy_from_slice(&self.src_macaddr[4..=5]);
        let hi = u16::from_le_bytes(hi_bytes);

        let mut lo_bytes = [0_u8; 4];
        lo_bytes.copy_from_slice(&self.src_macaddr[0..=3]);
        let lo = u32::from_le_bytes(lo_bytes);

        self.emac.addr0h.write(|w| unsafe { w.addrhi().bits(hi) });
        self.emac.addr0l.write(|w| unsafe { w.addrlo().bits(lo) });

        // Burst transfer limits
        let rxburst = (&self).rx_burst_size as u8;
        let txburst = (&self).tx_burst_size as u8;
        // Set actual programmable burst limits
        self.emac
            .dmabusmod
            .write(|w| unsafe { w.rpbl().bits(rxburst) });
        self.emac
            .dmabusmod
            .write(|w| unsafe { w.pbl().bits(txburst) });
        // Set flag for whether programmable burst limits are the same for RX and TX
        match rxburst == txburst {
            true => self.emac.dmabusmod.write(|w| w.usp().clear_bit()), // RX and TX burst limits are the same
            false => self.emac.dmabusmod.write(|w| w.usp().set_bit()), // RX and TX burst limits are different
        }

        // Disable (mask) all RX/TX interrupts (set interrupt masks)
        // Doing this using unsafe instead of field-by-field because there's a bit in each register
        // that needs to be cleared in the same write operation as all the others,
        // and both registers are safe to write to every field.
        self.emac
            .mmcrxim
            .write(|w| unsafe { w.bits(u32::MAX) });
        self.emac
            .mmctxim
            .write(|w| unsafe { w.bits(u32::MAX) });

        // MII (communication between EMAC and EPHY)
        match self.system_clk_freq {
            // These are below the minimum and may cause hardware UB
            PllOutputFrequency::_6mhz => self.emac.miiaddr.write(|w| w.cr()._20_35()),
            PllOutputFrequency::_12mhz => self.emac.miiaddr.write(|w| w.cr()._20_35()),
            // These are nominal
            PllOutputFrequency::_24mhz => self.emac.miiaddr.write(|w| w.cr()._20_35()),
            PllOutputFrequency::_30mhz => self.emac.miiaddr.write(|w| w.cr()._20_35()),
            PllOutputFrequency::_48mhz => self.emac.miiaddr.write(|w| w.cr()._35_60()),
            PllOutputFrequency::_60mhz => self.emac.miiaddr.write(|w| w.cr()._35_60()),
            PllOutputFrequency::_120mhz => self.emac.miiaddr.write(|w| w.cr()._100_150()),
        }

        // Setup up EMAC transmission behavior

        // No reason not to offload checksums; always faster than software
        // However, this setting does not appear to work, despite adding the corresponding
        // settings in the descriptor and producing verifiably well-formed packets, so
        // we have to do our checksums in software instead.
        // self.emac.cfg.write(|w| w.ipc().set_bit());

        match self.preamble_length {
            PreambleLength::_3 => self.emac.cfg.write(|w| w.prelen()._3()),
            PreambleLength::_5 => self.emac.cfg.write(|w| w.prelen()._5()),
            PreambleLength::_7 => self.emac.cfg.write(|w| w.prelen()._7()),
        }

        match self.interframe_gap {
            InterFrameGap::_40 => self.emac.cfg.write(|w| w.ifg()._40()),
            InterFrameGap::_48 => self.emac.cfg.write(|w| w.ifg()._48()),
            InterFrameGap::_56 => self.emac.cfg.write(|w| w.ifg()._56()),
            InterFrameGap::_64 => self.emac.cfg.write(|w| w.ifg()._64()),
            InterFrameGap::_72 => self.emac.cfg.write(|w| w.ifg()._72()),
            InterFrameGap::_80 => self.emac.cfg.write(|w| w.ifg()._80()),
            InterFrameGap::_88 => self.emac.cfg.write(|w| w.ifg()._88()),
            InterFrameGap::_96 => self.emac.cfg.write(|w| w.ifg()._96()),
        }

        // The exponential backoff limit setting has no effect on full-duplex,
        // but we bookkeep it here in case this structure is ever used for half-duplex
        // self.emac.cfg.write(|w| w.bl()._2());

        match self.tx_thresh {
            TXThresholdDMA::_16 => self.emac.dmaopmode.write(|w| w.ttc()._16()),
            TXThresholdDMA::_24 => self.emac.dmaopmode.write(|w| w.ttc()._24()),
            TXThresholdDMA::_32 => self.emac.dmaopmode.write(|w| w.ttc()._32()),
            TXThresholdDMA::_40 => self.emac.dmaopmode.write(|w| w.ttc()._40()),
            TXThresholdDMA::_64 => self.emac.dmaopmode.write(|w| w.ttc()._64()),
            TXThresholdDMA::_128 => self.emac.dmaopmode.write(|w| w.ttc()._128()),
            TXThresholdDMA::_192 => self.emac.dmaopmode.write(|w| w.ttc()._192()),
            TXThresholdDMA::_256 => self.emac.dmaopmode.write(|w| w.ttc()._256()),
        }
        match self.rx_thresh {
            RXThresholdDMA::_32 => self.emac.dmaopmode.write(|w| w.rtc()._32()),
            RXThresholdDMA::_64 => self.emac.dmaopmode.write(|w| w.rtc()._64()),
            RXThresholdDMA::_96 => self.emac.dmaopmode.write(|w| w.rtc()._96()),
            RXThresholdDMA::_128 => self.emac.dmaopmode.write(|w| w.rtc()._128()),
        }

        // Set descriptor list pointers
        self.emac
            .txdladdr
            .write(|w| unsafe { w.bits(self.txdl.txdladdr as u32) });
        self.emac
            .rxdladdr
            .write(|w| unsafe { w.bits(self.rxdl.rxdladdr as u32) });

        // Clear PHY and MAC interrupts
        self.phyclear();
        self.emacclear();

        // Start DMA and EMAC transmit/receive
        self.rxstart();
        self.txstart();

        // Clear interrupts again
        self.phyclear();
        self.emacclear();

        // Start PHY autonegotiation
        let anen_enable: u16 = 0x1000;
        let anen_restart: u16 = 0x200;
        self.phywrite(EPHYBMCR, anen_enable | anen_restart);
    }

    /// Stop transmit EMAC then DMA (order is important)
    pub fn txstop(&mut self) {
        self.emac.cfg.write(|w| w.te().clear_bit());
        self.emac.dmaopmode.write(|w| w.st().clear_bit());
    }

    /// Stop receive EMAC then DMA (order is important)
    pub fn rxstop(&mut self) {
        self.emac.cfg.write(|w| w.re().clear_bit());
        self.emac.dmaopmode.write(|w| w.sr().clear_bit());
    }

    /// Start transmit DMA then EMAC (order is important)
    #[inline(always)]
    pub fn txstart(&mut self) {
        self.emac.dmaopmode.write(|w| w.st().set_bit());
        self.emac.cfg.write(|w| w.te().set_bit());
    }

    /// Start receive DMA then EMAC (order is important)
    #[inline(always)]
    pub fn rxstart(&mut self) {
        self.emac.dmaopmode.write(|w| w.sr().set_bit());
        self.emac.cfg.write(|w| w.re().set_bit());
    }

    /// Flush receive descriptors by restarting the engine
    /// the maximum number of times that it can get stuck given our configuration
    #[inline(always)]
    pub fn rxflush(&mut self) {
        self.emacclear();
        for _ in 0..RXDESCRS {
            self.rxstart();
        }
    }

    /// Flush transmit descriptors by restarting the engine
    /// the maximum number of times that it can get stuck given our configuration
    #[inline(always)]
    pub fn txflush(&mut self) {
        self.emacclear();
        for _ in 0..TXDESCRS {
            self.txstart();
        }
    }

    /// Read a register from the _internal_ PHY via MII.
    ///
    /// This is kept private as it can cause a permanent freeze if the MII link to the PHY
    /// is busy, so it should only be used during init.
    ///
    /// Requires that system clock frequency has already been set during init.
    fn phyread(&mut self, reg_addr: u8) -> u16 {
        // Wait for MII link to be idle
        while self.emac.miiaddr.read().miib().bit_is_set() {}

        // Tell the EMAC to read the register from the PHY
        let phy_addr: u8 = 0; // Use internal PHY explicitly
        unsafe {
            self.emac.miiaddr.write(|w| {
                w.mii()
                    .bits(reg_addr) // PHY register to read from
                    .pla()
                    .bits(phy_addr) // PHY to read that register from
                    .miib()
                    .set_bit() // Set MII Busy flag so that we can wait for it to be cleared
            });
        }

        // Wait for MII link to go idle again (indicating read operation is complete)
        while self.emac.miiaddr.read().miib().bit_is_set() {}

        // Get data that was read from PHY
        let mii_data: u16 = self.emac.miidata.read().data().bits();
        return mii_data;
    }

    /// Write a register on the _internal_ PHY via MII
    ///
    /// This is kept private as it can cause a permanent freeze if the MII link to the PHY
    /// is busy, so it should only be used during init.
    ///
    /// Requires that system clock frequency has already been set during init.
    fn phywrite(&mut self, reg_addr: u8, value: u16) {
        // Wait for MII link to be idle
        while self.emac.miiaddr.read().miib().bit_is_set() {}

        unsafe {
            // Set the data to send
            self.emac.miidata.write(|w| w.data().bits(value));

            // Tell the EMAC to write the register on the PHY
            let phy_addr: u8 = 0; // Use internal PHY explicitly
            self.emac.miiaddr.write(|w| {
                w.mii()
                    .bits(reg_addr) // PHY register to read from
                    .pla()
                    .bits(phy_addr) // PHY to read that register from
                    .miiw()
                    .set_bit() // This is a write operation
                    .miib()
                    .set_bit() // Set MII Busy flag so that we can wait for it to be cleared
            });
        }

        // Wait for MII link to be idle again (indicating write operation is complete)
        while self.emac.miiaddr.read().miib().bit_is_set() {}
    }

    /// Clear PHY interrupts by reading their status
    fn phyclear(&mut self) {
        self.phyread(EPHYMISR1);
        self.phyread(EPHYMISR2);
    }

    /// Clear EMAC interrupts by setting their bits
    pub fn emacclear(&mut self) {
        // These have to be done all-at-once, because the summary bits are sticky and will reset otherwise
        self.emac.dmaris.write(|w| {
            unsafe{w.bits(u32::MAX)}
        }); // This interrupt is cleared by setting the bit, not by clearing it
    }

    /// Do a soft reset of the EMAC and DMA.
    ///
    /// This may loop indefinitely if the EMAC fails to come out of reset.
    pub fn emac_reset(&mut self) {
        self.emac.dmabusmod.write(|w| w.swr().set_bit());
        while self.emac.dmabusmod.read().swr().bit_is_set() {}
    }

    /// Get formattable representation of EMACSTATUS register
    pub fn emac_status(&self) -> EmacStatus {
        EmacStatus::new(self.emac.status.read().bits())
    }

    /// Get formattable representation of EMACDMARIS register
    pub fn dma_status(&self) -> DmaStatus {
        DmaStatus::new(self.emac.dmaris.read().bits())
    }
}

/// Choices of preamble length in bytes.
///
/// This is the number of alternating 0-1 bits transmitted at the start of each frame
/// in order to synchronize clocks between the transmitter and receiver.
#[derive(uDebug, Debug)]
#[allow(missing_docs)]
pub enum PreambleLength {
    _3,
    _5,
    _7,
}

/// Choices of interframe gap length in bits.
///
/// This is the duration of radio-silence used to signal the end of a transmission frame.
#[derive(uDebug, Debug)]
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
#[derive(uDebug, Debug)]
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
#[derive(uDebug, Debug)]
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
#[derive(uDebug, Debug)]
#[allow(missing_docs)]
pub enum RXThresholdDMA {
    _32,
    _64,
    _96,
    _128,
}

/// RX memory transfer burst size in 32-bit words
#[derive(uDebug, Debug, Eq, PartialEq)]
#[allow(missing_docs)]
#[repr(u8)]
pub enum BurstSizeDMA {
    // Settings that do not require 8x flag
    _1 = 1,
    _2 = 2,
    _4 = 4,
    _8 = 8,
    _16 = 16,
    _32 = 32,
    // Settings that do require 8x flag
    // The 8x flag must match both RX and TX burst size, which introduces complicated logic
    // that is difficult to implement without introducing panic branches or unexpected behavior
    // to resolve incompatible input combinations

    // _64,
    // _128,
    // _256,
}

/// Ethernet driver errors
#[derive(Debug, uDebug)]
pub enum EthernetError {
    /// Data too large for buffer
    BufferOverflow,
    /// No descriptor available for transmission
    DescriptorUnavailable,
    /// Frame is less than 64 bytes and needs padding
    FrameTooShort,
    /// Nothing to receive from RX descriptor buffers
    NothingToReceive,
}
