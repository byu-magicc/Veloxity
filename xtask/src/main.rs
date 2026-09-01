use std::env;
use std::ffi::OsStr;
use std::fs;
use std::path::Path;
use std::process::{Command, ExitCode};

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let Some(command) = args.next() else {
        print_usage();
        return ExitCode::from(2);
    };

    let status = match command.as_str() {
        "check-host" => cargo(["check"]),
        "test-host" => cargo(["test", "-p", "veloxity_core"])
            .and_then(|_| cargo(["test", "-p", "sim", "--lib"])),
        "check-board" => {
            let Some(board) = args.next() else {
                eprintln!("missing board name: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            let Some(target) = board_target(&board) else {
                eprintln!("unknown board `{board}`: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            cargo([
                "check",
                "-p",
                board.as_str(),
                "--target",
                target,
                "--bin",
                "veloxity",
            ])
        }
        "build-board" => {
            let Some(board) = args.next() else {
                eprintln!("missing board name: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            let Some(target) = board_target(&board) else {
                eprintln!("unknown board `{board}`: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            cargo([
                "build",
                "-p",
                board.as_str(),
                "--target",
                target,
                "--bin",
                "veloxity",
            ])
        }
        "flash-board" => {
            let Some(board) = args.next() else {
                eprintln!("missing board name: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            let Some(target) = board_target(&board) else {
                eprintln!("unknown board `{board}`: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            let extra_args = args.collect::<Vec<_>>();
            match flash_args(&board, target, &extra_args) {
                Ok(cargo_args) if board == "pixracerpro" => cargo(cargo_args)
                    .and_then(|_| flash_pixracerpro_detached(target))
                    .and_then(|_| reset_pixracerpro()),
                Ok(cargo_args) => cargo(cargo_args),
                Err(message) => {
                    eprintln!("{message}");
                    return ExitCode::from(2);
                }
            }
        }
        "build-sil-node" => cargo(["build", "-p", "veloxity_sil_node", "--release"]),
        "clean-generated" => clean_generated(),
        _ => {
            eprintln!("unknown command `{command}`");
            print_usage();
            return ExitCode::from(2);
        }
    };

    match status {
        Ok(()) => ExitCode::SUCCESS,
        Err(code) => ExitCode::from(code),
    }
}

fn flash_args(board: &str, target: &str, options: &[String]) -> Result<Vec<String>, String> {
    let mut args = vec![
        "run".to_owned(),
        "-p".to_owned(),
        board.to_owned(),
        "--target".to_owned(),
        target.to_owned(),
        "--bin".to_owned(),
        "veloxity".to_owned(),
    ];

    if board != "pixracerpro" {
        if let Some(option) = options.first() {
            return Err(format!(
                "option `{option}` is only supported when flashing `pixracerpro`"
            ));
        }
        return Ok(args);
    }

    // The flight firmware baseline is optimized and contains no diagnostic or
    // alternate-transport features. With no optional feature selected, Pixracer
    // Pro's MAVLink transport is the companion-computer UART. The required MCU
    // feature is added explicitly below because defaults are disabled here.
    args.extend(["--release".to_owned(), "--no-default-features".to_owned()]);
    // Pixracer flashing is completed below with `probe-rs download` followed
    // by a detached reset. `cargo run` would invoke the configured probe-rs
    // runner and leave a live debug session attached to the flight controller.
    args[0] = "build".to_owned();

    let mut features = vec!["mcu-h743ii"];
    for option in options {
        let feature = match option.as_str() {
            "--vcp" => "usb-vcp-serial",
            "--scope-timing-pins" => "scope-timing-pins",
            "--sensor-poll-diagnostics" => "sensor-poll-diagnostics",
            "--runtime-diagnostics" => "runtime-diagnostics",
            _ => {
                return Err(format!(
                    "unknown Pixracer Pro flash option `{option}`; expected `--vcp`, \
                     `--scope-timing-pins`, `--sensor-poll-diagnostics`, or \
                     `--runtime-diagnostics`"
                ));
            }
        };
        if !features.contains(&feature) {
            features.push(feature);
        }
    }

    args.push("--features".to_owned());
    args.push(features.join(","));

    Ok(args)
}

fn flash_pixracerpro_detached(target: &str) -> Result<(), u8> {
    let elf = format!("target/{target}/release/veloxity");
    command_status(
        "probe-rs",
        [
            "download",
            "--chip",
            "STM32H743IIKx",
            "--protocol",
            "swd",
            "--speed",
            "4000",
            "--verify",
            elf.as_str(),
        ],
    )
}

fn reset_pixracerpro() -> Result<(), u8> {
    command_status("probe-rs", ["reset", "--chip", "STM32H743IIKx"])
}

fn command_status<I, S>(program: &str, args: I) -> Result<(), u8>
where
    I: IntoIterator<Item = S>,
    S: AsRef<OsStr>,
{
    let status = Command::new(program).args(args).status().map_err(|err| {
        eprintln!("failed to run {program}: {err}");
        1
    })?;

    if status.success() {
        Ok(())
    } else {
        Err(status.code().unwrap_or(1).try_into().unwrap_or(1))
    }
}

fn clean_generated() -> Result<(), u8> {
    for path in [
        "target",
        "workspace",
        "rosflight_memory",
        "tools/__pycache__",
    ] {
        remove_dir_if_exists(path)?;
    }

    let espnow_dir = Path::new("tools/espnow_uart_bridge");
    if espnow_dir.exists() {
        for entry in fs::read_dir(espnow_dir).map_err(|err| {
            eprintln!("failed to read {}: {err}", espnow_dir.display());
            1
        })? {
            let entry = entry.map_err(|err| {
                eprintln!("failed to read ESP-NOW bridge directory entry: {err}");
                1
            })?;
            let file_name = entry.file_name();
            let file_name = file_name.to_string_lossy();
            if file_name.starts_with("build") && entry.path().is_dir() {
                remove_dir_if_exists(entry.path())?;
            }
        }

        remove_file_if_exists(espnow_dir.join("sdkconfig"))?;
        remove_file_if_exists(espnow_dir.join("dependencies.lock"))?;
    }

    Ok(())
}

fn board_target(board: &str) -> Option<&'static str> {
    match board {
        "nucleo" | "pixracerpro" => Some("thumbv7em-none-eabihf"),
        "pico2w" => Some("thumbv8m.main-none-eabihf"),
        _ => None,
    }
}

fn remove_dir_if_exists<P>(path: P) -> Result<(), u8>
where
    P: AsRef<Path>,
{
    let path = path.as_ref();
    if !path.exists() {
        return Ok(());
    }
    if !path.is_dir() {
        eprintln!("refusing to remove non-directory {}", path.display());
        return Err(1);
    }

    println!("removing {}", path.display());
    fs::remove_dir_all(path).map_err(|err| {
        eprintln!("failed to remove {}: {err}", path.display());
        1
    })
}

fn remove_file_if_exists<P>(path: P) -> Result<(), u8>
where
    P: AsRef<Path>,
{
    let path = path.as_ref();
    if !path.exists() {
        return Ok(());
    }
    if !path.is_file() {
        eprintln!("refusing to remove non-file {}", path.display());
        return Err(1);
    }

    println!("removing {}", path.display());
    fs::remove_file(path).map_err(|err| {
        eprintln!("failed to remove {}: {err}", path.display());
        1
    })
}

fn cargo<I, S>(args: I) -> Result<(), u8>
where
    I: IntoIterator<Item = S>,
    S: AsRef<OsStr>,
{
    let status = Command::new("cargo").args(args).status().map_err(|err| {
        eprintln!("failed to run cargo: {err}");
        1
    })?;

    if status.success() {
        Ok(())
    } else {
        Err(status.code().unwrap_or(1).try_into().unwrap_or(1))
    }
}

fn print_usage() {
    eprintln!(
        "usage: cargo xtask <command>\n\
         \n\
         commands:\n\
           check-host       check host-compatible workspace packages\n\
           test-host        run host-side Rust tests\n\
           check-board      check embedded firmware: nucleo | pixracerpro | pico2w\n\
           build-board      build embedded firmware: nucleo | pixracerpro | pico2w\n\
           flash-board      build and flash embedded firmware with probe-rs\n\
                            Pixracer Pro: release UART by default; opt in with\n\
                            --vcp, --scope-timing-pins, --sensor-poll-diagnostics,\n\
                            or --runtime-diagnostics\n\
           build-sil-node   build the veloxity_sil_board ROS 2 node\n\
           clean-generated  remove ignored local build/runtime artifacts"
    );
}

#[cfg(test)]
mod tests {
    use super::flash_args;

    const TARGET: &str = "thumbv7em-none-eabihf";

    #[test]
    fn pixracerpro_flash_keeps_required_mcu_feature() {
        let args = flash_args("pixracerpro", TARGET, &[]).unwrap();

        assert!(args.iter().any(|arg| arg == "--release"));
        assert_eq!(args.first().map(String::as_str), Some("build"));
        assert!(args.iter().any(|arg| arg == "--no-default-features"));
        assert_eq!(args.last().map(String::as_str), Some("mcu-h743ii"));
    }

    #[test]
    fn pixracerpro_flash_maps_explicit_options_to_features() {
        let options = vec![
            "--vcp".to_owned(),
            "--scope-timing-pins".to_owned(),
            "--sensor-poll-diagnostics".to_owned(),
            "--runtime-diagnostics".to_owned(),
        ];
        let args = flash_args("pixracerpro", TARGET, &options).unwrap();

        assert_eq!(
            args.last().unwrap(),
            "mcu-h743ii,usb-vcp-serial,scope-timing-pins,sensor-poll-diagnostics,runtime-diagnostics"
        );
    }

    #[test]
    fn pixracerpro_flash_rejects_unknown_options() {
        let error = flash_args("pixracerpro", TARGET, &["--debug".to_owned()]).unwrap_err();

        assert!(error.contains("unknown Pixracer Pro flash option `--debug`"));
    }
}
