//! Crate for operating TM4C129-XL Launchpad

#![no_std]
// #![warn(dead_code)] // NOTE: turn this back on once dead code in macros is no longer a lint
#![allow(dead_code)]
#![allow(non_snake_case)] // Macros produce a lot of private non-snake-case
#![deny(missing_docs)]
#![feature(generic_const_exprs)]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate embedded_hal;
extern crate tm4c129x_hal;

pub mod board;
pub mod builtins;
pub mod drivers;
pub mod startup;
