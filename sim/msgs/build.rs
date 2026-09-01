//! Generates Rust types for the vendored `rosflight_msgs` ROS 2 package
//! (`../msg_vendor/rosflight_msgs`) via `hiroz-codegen`.
//!
//! `HIROZ_MSG_PATH` must point at exactly this one package directory. Never
//! add a bundled/standard package (e.g. `std_msgs`, `sensor_msgs`) here —
//! `hiroz-codegen` already loads those from its own bundled assets to
//! resolve type hashes for fields like `std_msgs/Header`; adding one to
//! `HIROZ_MSG_PATH` would generate a second, shadowing copy of that type
//! that silently breaks wire interop with the rest of the Hiroz stack.

use std::{env, path::PathBuf};

fn main() -> anyhow::Result<()> {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR")?);
    let vendor_path = manifest_dir.join("../msg_vendor/rosflight_msgs");

    println!("cargo:rerun-if-changed=../msg_vendor/rosflight_msgs");
    println!("cargo:rerun-if-env-changed=HIROZ_MSG_PATH");

    // hiroz-codegen unconditionally emits `#[cfg_attr(feature = "python_registry", ...)]`
    // attributes in generated.rs regardless of whether the consuming crate has a
    // `python_registry` feature. We don't enable that feature (no `hiroz-derive`
    // dependency here), so without this declaration rustc's unexpected_cfgs lint
    // would flag every occurrence.
    println!(r#"cargo::rustc-check-cfg=cfg(feature, values("python_registry"))"#);

    // SAFETY: build scripts run single-threaded before any other code in
    // this process reads or writes environment variables, so this set_var
    // cannot race with a concurrent reader.
    unsafe {
        std::env::set_var("HIROZ_MSG_PATH", &vendor_path);
    }

    let out_dir = PathBuf::from(env::var("OUT_DIR")?);
    hiroz_codegen::generate_user_messages(&out_dir, true)?;

    Ok(())
}
