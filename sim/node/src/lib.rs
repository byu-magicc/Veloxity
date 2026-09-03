//! Support library for the `veloxity_sil_board` binary.
//!
//! The rclcpp-shaped bootstrap [`shim`] is a library rather than a module of
//! the binary so that its `pub` surface — the parts the sibling
//! `rosplane_rs`/`roscopter_rs` copies also expose, and their tests — stays
//! reachable instead of reading as dead code.

pub mod shim;
