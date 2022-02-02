//! Crate for operating TM4C129-XL Launchpad

#![no_std]
#![warn(dead_code)]
#![deny(missing_docs)]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate embedded_hal;
pub extern crate tm4c129x_hal;
extern crate volatile_register;

pub mod board;
pub mod drivers;
pub mod builtins;