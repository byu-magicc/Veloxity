//! Support library for the `veloxity_sil_board` binary.
//!
//! Two things live here. The rclcpp-shaped bootstrap [`shim`] is a library
//! rather than a module of the binary so that its `pub` surface — the parts
//! the sibling `rosplane_rs`/`roscopter_rs` copies also expose, and their
//! tests — stays reachable instead of reading as dead code. [`compat_msgs`]
//! carries the message types `hiroz-msgs` gets wrong for ROS 2 Humble.

pub mod compat_msgs;
pub mod shim;
