#![no_std]
// The flight core contains no unsafe code; the opt-in diagnostics feature exports
// counters with `#[unsafe(no_mangle)]`, so it is exempt.
#![cfg_attr(not(feature = "runtime-diagnostics"), forbid(unsafe_code))]
pub mod battery;
pub mod board;
pub mod comm;
pub mod command;
pub mod companion;
pub mod control;
pub mod controller;
pub mod errors;
pub mod estimator;
pub mod events;
pub mod log;
pub mod math;
pub mod mixer;
pub mod packets;
pub mod params;
pub mod ports;
pub mod pwm;
pub mod rc;
pub mod sensors;
pub mod state_machine;
pub mod vehicle;
pub mod world;

#[cfg(test)]
pub(crate) mod test_support;
