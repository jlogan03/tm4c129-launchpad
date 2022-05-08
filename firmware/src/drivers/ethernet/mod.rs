//! Drivers for TM4C129's EMAC/PHY media access control peripherals

pub mod rdes;
pub mod socket;
pub mod tdes; // TX descriptor ring definitions // RX ...

use tm4c129x_hal::{
    sysctl::{PllOutputFrequency, PowerControl},
    tm4c129x::{EMAC0, FLASH_CTRL},
};

use self::rdes::*;
pub use self::rdes::{RXBUFSIZE, RXDESCRS};

use self::tdes::*;
pub use self::tdes::{TXBUFSIZE, TXDESCRS};

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
    pub(crate) fn new<F>(
        pc: &PowerControl,
        ephy_reset: F,
        // emac_reset: G,
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
    ) -> EthernetDriver
    where
        F: Fn(&PowerControl) -> EphyR,
        // G: Fn(&PowerControl) -> EmacR,
    {
        // Build driver struct & initialize descriptor lists and buffers
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

            txdl: TXDL::new(),
            rxdl: RXDL::new(),
        };

        // Write registers and populate buffers
        enet.init(pc, |pc| ephy_reset(pc));

        enet
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
        self.emac.pc.modify(|_, w| w.phyext().clear_bit()); // Use internal PHY (disable external)
                                                            // self.emac.cc.modify(|_, w| ); // Use internal PHY (disable external)
        self.emac.pc.modify(|_, w| w.mdixen().set_bit()); // Enable MDIX
        self.emac.pc.modify(|_, w| w.anen().set_bit()); // Enable autonegotiation
        self.emac.cfg.modify(|_, w| w.fes().set_bit()); // Speed 100 base T
        self.emac.cfg.modify(|_, w| w.dupm().set_bit()); // Full duplex mode

        // Reset MAC again to latch configuration
        self.emac_reset();
        // ephy_reset(pc);

        // -------------------- NON-LATCHING CONFIGURATION --------------------
        // This configuration is cleared on peripheral reset and takes effect more-or-less immediately during operation

        // Make sure DMA and EMAC are stopped in order to allow configuration
        self.rxstop();
        self.txstop();

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

        // Set source-address replacement using mac address 0 (the source address field exists in provided frames, but will be overwritten by the MAC)
        // This has to be done manually because svd2rust doesn't break out the SADDR field for some reason
        unsafe {
            self.emac.cfg.modify(|r, w| w.bits(r.bits() | 0x03 << 28));
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

        // Set descriptor list pointers
        self.emac
            .txdladdr
            .write(|w| unsafe { w.bits(self.txdl.txdladdr as u32) }); // List start address
        self.emac
            .rxdladdr
            .write(|w| unsafe { w.bits(self.rxdl.rxdladdr as u32) }); // List start address

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
        self.emac.cfg.modify(|_, w| w.te().clear_bit());
        self.emac.dmaopmode.modify(|_, w| w.st().clear_bit());
    }

    /// Stop receive EMAC then DMA (order is important)
    pub fn rxstop(&mut self) {
        self.emac.cfg.modify(|_, w| w.re().clear_bit());
        self.emac.dmaopmode.modify(|_, w| w.sr().clear_bit());
    }

    /// Start transmit DMA then EMAC (order is important)
    pub fn txstart(&mut self) {
        self.emac.dmaopmode.modify(|_, w| w.st().set_bit());
        self.emac.cfg.modify(|_, w| w.te().set_bit());
    }

    /// Start receive DMA then EMAC (order is important)
    pub fn rxstart(&mut self) {
        self.emac.dmaopmode.modify(|_, w| w.sr().set_bit());
        self.emac.cfg.modify(|_, w| w.re().set_bit());
    }

    /// Flush receive descriptors
    pub unsafe fn rxflush(&mut self) {
        for _ in 0..RXDESCRS {
            self.rxdl.give();
            self.rxdl.next();
        }
    }

    /// Flush transmit descriptors
    pub unsafe fn txflush(&mut self) {
        for _ in 0..TXDESCRS {
            self.txdl.take();
            self.txdl.next();
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
            self.emac.miiaddr.modify(|_, w| {
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
            self.emac.miidata.modify(|_, w| w.data().bits(value));

            // Tell the EMAC to write the register on the PHY
            let phy_addr: u8 = 0; // Use internal PHY explicitly
            self.emac.miiaddr.modify(|_, w| {
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
        self.emac.dmaris.modify(|_, w| {
            w.nis()
                .set_bit()
                .ais()
                .set_bit()
                .eri()
                .set_bit()
                // .fbi()
                // .set_bit()  // Must only be cleared by reset
                .eti()
                .set_bit()
                .rwt()
                .set_bit()
                .rps()
                .set_bit()
                .ru()
                .set_bit()
                .ri()
                .set_bit()
                .unf()
                .set_bit()
                .ovf()
                .set_bit()
                .tjt()
                .set_bit()
                .tu()
                .set_bit()
                .ti()
                .set_bit()
        }); // This interrupt is cleared by setting the bit, not by clearing it
    }

    /// Do a soft reset of the EMAC and DMA.
    ///
    /// This may loop indefinitely if the EMAC fails to come out of reset.
    pub fn emac_reset(&mut self) {
        self.emac.dmabusmod.modify(|_, w| w.swr().set_bit());
        while self.emac.dmabusmod.read().swr().bit_is_set() {}
    }

    /// Attempt to send an ethernet frame that has been reduced to bytes
    pub fn transmit<const N: usize>(&mut self, data: [u8; N]) -> Result<(), EthernetError> {
        // Check data length
        if N > TXBUFSIZE / 4 {
            return Err(EthernetError::BufferOverflow);
        }

        // Attempt send
        unsafe {
            for _ in 0..TXDESCRS {
                if self.txdl.is_owned() {
                    // We own the current descriptor; load our data into the buffer and tell the DMA to send it
                    //    Load data into buffer
                    let mut _buffer: *mut [u8; N] = self.txdl.get_buffer_pointer() as *mut [u8; N];
                    _buffer.write_volatile(data);
                    //    Set buffer length
                    self.txdl.set_buffer_size(N as u16);
                    //    Set common settings
                    self.txdl.set_tdes0(TDES0::CRCR); // Enable ethernet checksum replacement
                    self.txdl.set_tdes0(TDES0::CicFull); // Full calculation of IPV4 and TCP/UDP checksums using pseudoheader
                                                         // self.txdl.set_tdes0(TDES0::TTSE); // Transmit IEEE-1588 64-bit timestamp
                    self.txdl.set_tdes1(TDES1::SaiReplace); // Replace source MAC address in frame with value programmed into peripheral
                    self.txdl.give(); // Give this descriptor & buffer back to the DMA

                    // Make sure the transmitter is enabled, since it may have been disabled by errors
                    self.txstart();

                    // Tell the DMA that there is demand for transmission by writing any value to the register
                    self.emac.txpolld.write(|w| w.tpd().bits(0));

                    return Ok(());
                } else {
                    // We do not own the current descriptor and can't use it to send data
                    // Go to the next descriptor and try that one
                    self.txdl.next();
                }
            }
        }
        // We checked every descriptor and none of them are available
        return Err(EthernetError::DescriptorUnavailable);
    }

    /// Receive a frame of unknown size that may be up to 1522 bytes
    pub fn receive(&mut self, buf: &mut [u8; RXBUFSIZE]) -> Result<usize, EthernetError> {
        // Make sure the receive engine is enabled
        self.rxstart();

        unsafe {
            // Walk through descriptor list until we find one that is the start of a received frame
            // and is owned by software, or we have checked all the descriptors
            let mut num_owned: usize = 0;
            for _ in 0..RXDESCRS {
                let owned = self.rxdl.is_owned();
                num_owned += owned as usize;
                if owned && (self.rxdl.get_rdes0(RDES0::FS) != 0) {
                    break;
                } else {
                    self.rxdl.next();
                }
            }

            // Did we actually find a frame to receive, or did we run out of descriptors?
            if !(self.rxdl.is_owned() && (self.rxdl.get_rdes0(RDES0::FS) != 0)) {
                // If all the descriptors are owned by software and yet there is no frame to receive, something
                // has gone wrong. Flush the buffer by returning all descriptors to the DMA.
                if num_owned == RXDESCRS {
                    self.rxflush();
                }
                // Either way, there is no valid data to receive from the buffer
                return Err(EthernetError::NothingToReceive);
            } else {
                // There is a frame to receive
                // We do not have jumbo frames enabled and are using store-and-forward,
                // so the entire frame should be stored in a single buffer.
                // Otherwise, we would have to handle frames spread across multiple descriptors/buffers.
                let descr_buf = self.rxdl.get_buffer_pointer() as *mut [u8; RXBUFSIZE];
                buf.copy_from_slice(&(descr_buf.read_volatile()[..RXBUFSIZE])); // Slice to guarantee length does not exceed buffer size
                                                                                // Clear the descriptor buffer so that it does not accumulate stray data on the next received frame
                descr_buf.write_volatile([0_u8; RXBUFSIZE]);
            }

            // Give this descriptor back to the DMA
            self.rxdl.give();

            let bytes_received = self.rxdl.get_rdes0(RDES0::FL) as usize;
            Ok(bytes_received)
        }
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

/// Ethernet driver errors
#[derive(Debug)]
pub enum EthernetError {
    /// Data too large for buffer
    BufferOverflow,
    /// No descriptor available for transmission
    DescriptorUnavailable,
    /// Nothing to receive from RX descriptor buffers
    NothingToReceive,
}

// impl fmt::Debug for EthernetDriver {
//     fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
//         // f.debug_struct("TX Descriptor List").field("\nStart Address", &self.txdladdr).finish()
//         unsafe {
//             write!(
//                 f,
//                 "
//             Ethernet Controller
//             ------------------
//             Root Descriptor Address: {}
//                 ",
//                 self.txdladdr as usize,

//             )
//         }
//     }
// }
