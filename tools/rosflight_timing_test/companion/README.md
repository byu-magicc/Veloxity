# ROSflight timing companion bundle

This directory builds and uses a private ROSflight timing overlay. It does not require or modify a
normal ROSflight workspace. For Veloxity it automatically uses
`~/.config/veloxity/airframes/3dquad/firmware/snapshots/quad_x_slim.yaml`; set
`RF_TIMING_VELOXITY_PARAM_FILE` before sourcing `setup.zsh` to select another file.

It does not contain or flash C or Veloxity firmware. Flash the board from the separate flashing
computer, then connect the board directly to this companion over USB.

## First use on Ubuntu 24.04 / ROS 2 Jazzy

The setup auto-detects Jazzy or Humble. It also expects:

- parameter loader: the existing `~/.config/.../verified_param_loader.py`, or the bundled fallback
- parameter YAMLs: `~/.config/veloxity/airframes/3dquad/firmware/`

Override the ROS path before sourcing `setup.zsh` only if it is nonstandard, for example:

```zsh
export RF_TIMING_ROS_SETUP="/opt/ros/jazzy/setup.bash"
source setup.zsh
```

Build the ARM64 overlay once:

```zsh
source setup.zsh
timing_build
```

The first build needs network access. With an existing local `rosflight_ros_pkgs` source checkout:

```zsh
timing_build "$HOME/rosflight_ws/src/rosflight_ros_pkgs"
```

## Run

After flashing the selected firmware and reconnecting USB:

```zsh
timing_smoke c
timing_run c
```

After flashing Veloxity:

```zsh
timing_smoke veloxity
timing_run veloxity
```

The Veloxity image must be built from current `main` plus the timing echo, using the
`timing-test-current-main` branch prepared on the flashing computer. Flash it with:

```zsh
cargo xtask flash-board pixracerpro --vcp
```

Each command verifies the USB firmware identity, starts the private overlay's `rosflight_io`, loads
and reads back the firmware-specific parameters, rejects the known excessive-mixer warning, stops
that process, and runs the isolated timing overlay. Results stay inside this directory.

Package the results with `timing_package`.

Each command carries a 1-based sequence ID and a per-run token in the existing `u[2]` and `u[3]`
float fields. The firmware-facing MAVLink OFFBOARD_CONTROL frame remains 43 bytes. Results include:

- `rtt_samples.csv`: only the first valid response for each command; use this for unique-response
  latency summaries
- `all_echoes.csv`: every response, including its sequence, run token, and classification
- `summary.txt` and `summary.json`: unique, duplicate, missing, foreign-run, invalid-sequence, and
  out-of-order counts

Use `all_echoes.csv` for raw outlier plots. A plot based only on `rtt_samples.csv` intentionally
omits duplicate arrivals.

## Leave the timing environment

The Zsh setup file never sources ROS or either overlay into the parent shell. Its commands invoke
Bash child processes for the ROS build and test executables, so your interactive Zsh environment
remains unchanged. Remove its helper functions with:

```zsh
timing_deactivate
```

No normal ROS workspace is required or modified. To remove the timing installation completely, delete
this extracted directory after copying its results.
