//! Crate for operating TM4C129-XL Launchpad

#![no_std]
#![warn(dead_code)]
#![deny(missing_docs)]
#![feature(generic_const_exprs)]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate embedded_hal;
extern crate tm4c129x_hal;

pub mod board;
pub mod startup;
pub mod drivers;
pub mod builtins;
