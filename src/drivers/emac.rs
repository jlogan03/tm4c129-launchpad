//! Drivers for TM4C129's EMAC/PHY peripheral
use tm4c129x_hal::tm4c129x::EMAC0;


/// Get preprogrammed MAC address from ROM
pub fn get_rom_macaddr(emac: &EMAC0) -> [u8; 6] {
    let mut addr = [0_u8; 6];

    // Unpack address values from register structure
    // They're stored as u16, padded up to u32 by the HAL, and needed as u8
    let a0h = (&emac.addr0h).read();
    // addr[0] = a0h[0];
    // addr[1] = a0h[1];
    // let a1h: [u8; 2] = (emac.addr1h.read().bits() as u16).to_be_bytes();
    // addr[2] = a1h[0];
    // addr[3] = a1h[1];
    // let a2h: [u8; 2] = (emac.addr2h.read().bits() as u16).to_be_bytes();
    // addr[4] = a2h[0];
    // addr[5] = a2h[1];

    addr
}

// pub fn cfg() -> () {
    
// }