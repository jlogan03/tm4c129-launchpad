//! Drivers for TM4C129's EMAC/PHY media access control peripherals
// use tm4c129x_hal::sysctl::{control_power, reset, Domain, PowerControl, PowerState, RunMode};
use tm4c129x_hal::{tm4c129x::EMAC0, sysctl::{PowerControl, Domain, reset}};

/// Get preprogrammed MAC address from ROM
pub fn get_rom_macaddr(emac: &EMAC0) -> [u8; 6] {
    // Unpack address values from register structure
    // They're stored as u16, padded up to u32 by the HAL, and needed as u8
    let addr: [u8; 6] = [
        (&emac.addr0h).read().bits() as u8,
        (&emac.addr0l).read().bits() as u8,
        (&emac.addr1h).read().bits() as u8,
        (&emac.addr1l).read().bits() as u8,
        (&emac.addr2h).read().bits() as u8,
        (&emac.addr2l).read().bits() as u8,
    ];

    addr
}

/// Test function to probe elusive panic branch
pub fn dummy(power_control: &PowerControl) {
    // reset(power_control, Domain::Emac0);
}
