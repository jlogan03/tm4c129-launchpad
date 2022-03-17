//! Drivers for TM4C129's EMAC/PHY media access control peripherals
use tm4c129x_hal::tm4c129x::EMAC0;

/// Get preprogrammed MAC address from ROM
pub fn get_rom_macaddr(emac: &EMAC0) -> [u8; 6] {
    // Unpack address values from register structure
    // They're stored as u16 (populating half a 32-bit register) and needed as u8
    let addr: [u8; 6] = [
        (&emac.addr0h).read().addrhi().bits() as u8,
        (&emac.addr0l).read().addrlo().bits() as u8,
        (&emac.addr1h).read().addrhi().bits() as u8,
        (&emac.addr1l).read().addrlo().bits() as u8,
        (&emac.addr2h).read().addrhi().bits() as u8,
        (&emac.addr2l).read().addrlo().bits() as u8,
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

///
pub fn emac_init(emac: &EMAC0) {
    // Set up direct memory access
    let rxburst = 4_u8; // RX DMA controller max memory transfer size in words
    let txburst = 4_u8;

    emac.dmabusmod.modify(|_, w| w.fb().set_bit()); // Fixed burst mode (RX and TX are the same)
    // emac.dmabusmod.modify(|_, w| w.txpr().set_bit());  // Fixed priority
    emac.dmabusmod.modify(|_, w| w.atds().set_bit()); // Alternate descriptor size

    unsafe {  // Only unsafe due to old-style conversion to 5-bit integer; could update upstream calc to be safe
        // Burst transfer limits
        // Just using the values the mfr uses here - not sure what the tradeoffs are
        emac.dmabusmod.modify(|_, w| w.rpbl().bits(rxburst));
        emac.dmabusmod.modify(|_, w| w.pbl().bits(txburst));
    };

    // MII (communication between EMAC and EPHY)
    emac.miiaddr.modify(|_, w| w.cr()._100_150()); // System clock speed 


}

/// Configure EMAC (must run phy_cfg first, then reset before configuring EMAC!)
pub fn emac_cfg(emac: &EMAC0) {}

// pub struct EMACDriver {
//     descripto
// }
