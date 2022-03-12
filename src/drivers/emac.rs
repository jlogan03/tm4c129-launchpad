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
    emac.pc.modify(|_, w| w.phyext().clear_bit());  // Use internal PHY (disable external)
    emac.pc.modify(|_, w| w.mdixen().set_bit());    // Enable MDIX
    emac.pc.modify(|_, w| w.anen().set_bit());    // Enable autonegotiation
    emac.cfg.modify(|_, w| w.fes().set_bit());    // Speed 100 base T
    emac.cfg.modify(|_, w| w.dupm().set_bit());    // Duplex mode full
}

/// Configure EMAC (must run phy_cfg first, then reset before configuring EMAC!)
pub fn emac_cfg(emac: &EMAC0) {

}