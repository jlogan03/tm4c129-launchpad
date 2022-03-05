//! Drivers for TM4C129's EMAC/PHY peripheral
use tm4c129x_hal::tm4c129x::EMAC0;


/// Get preprogrammed MAC address from ROM
pub fn get_rom_macaddr(emac: &EMAC0) -> [u8; 6] {
    let mut addr = [0_u8; 6];
    let ah = emac.addr0h.read().bits();

    addr
}

// pub fn cfg() -> () {
    
// }