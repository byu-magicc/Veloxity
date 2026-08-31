# Canonical ROS 2 serial RTT timing test (Pixracer Pro + Raspberry Pi)

This is the **current, self-contained timing-test harness** for measuring ROSflight command
round-trip time (RTT) through a Raspberry Pi companion and a Pixracer Pro. If you are searching
this repository for the ROS 2 timing test, Raspberry Pi timing harness, serial delay test,
Pixracer Pro latency test, 400 Hz timing test, C-versus-Veloxity comparison, bell-curve plot, or
3 ms outlier plot, use this directory.

The harness was last validated on 2026-08-31 against Veloxity commit `d353e3f` on branch
`timing-test-current-main`. That commit is the then-current upstream `main` commit `eea98c9` plus
the test-only OFFBOARD_CONTROL echo. Older ad hoc `timing_testing_for_real` directories and
`rosflight_timing_companion_pi_zsh_v1` through `v4` archives are superseded by this copy.

## What the test measures

```text
ROS 2 timing driver publishes /command at 400 Hz
  -> timing-instrumented rosflight_io timestamps and sends OFFBOARD_CONTROL
  -> Pixracer Pro receives and immediately echoes the MAVLink frame
  -> rosflight_io decodes the echo and publishes /serial_time_delay_ns
  -> timing driver records the response and its classification
```

The timestamp begins inside `rosflight_io::commandCallback()` and ends after the echoed frame is
decoded. DDS delivery into `commandCallback()` and delivery of `/serial_time_delay_ns` to the
collector are outside the measured RTT. MAVLink packing, companion TX queuing, USB transport,
firmware echo, return transport, and companion decoding are inside it.

Each command carries a sequence ID and per-run token in `u[2]` and `u[3]`. The companion records
both the first response for every command and every raw response, so delayed duplicate echoes are
not lost during outlier analysis.

## Safety first

This test continuously publishes pass-through commands. Before every smoke or full run:

- remove all propellers;
- keep the controller disarmed;
- disconnect motor power and use safe bench power;
- restrain the airframe;
- stop every other `/command` publisher and `rosflight_io` process.

The runner asks for confirmation and refuses to proceed unless the answer is exactly `yes`.

## Directory map

| Path | Purpose |
| --- | --- |
| `companion/` | Complete, reviewable source installed on the Raspberry Pi/Distrobox. |
| `dist/rosflight_timing_companion_pi_zsh_v5_current_main_20260831.zip` | Exact Pi ZIP validated in the 2026-08-31 run. |
| `analysis/plot_raw_timing_comparison.py` | Produces two separate C-versus-Veloxity plots: raw RTTs at/below 3 ms and raw outliers above 3 ms. Duplicate arrivals are `x` markers. |
| `c_firmware/` | Optional patches for reproducing the fair ROSflight C firmware comparison. |
| `reference/latest_180s_400hz.md` | Known-good C and Veloxity results from the latest comparison. |
| `build_companion_bundle.sh` | Rebuilds a Pi ZIP from the checked-in `companion/` source. |

## Requirements

Flashing computer:

- this branch of Veloxity;
- Rust/embedded toolchain used by the normal Pixracer Pro workflow;
- a connected Pixracer Pro and supported debug probe.

Raspberry Pi companion:

- 64-bit Linux (validated on Raspberry Pi 5);
- ROS 2 Jazzy or Humble;
- Bash and/or Zsh, Git, CMake, a C++ compiler, `colcon`, and Python 3;
- network access for the first private-overlay build;
- `dialout` access to the Pixracer Pro USB serial device;
- a validated firmware parameter YAML.

Plotting computer:

```bash
python3 -m pip install numpy matplotlib
```

## 1. Check out, build, and flash the Veloxity timing firmware

Run these commands from the Veloxity repository on the flashing computer:

```bash
git fetch origin
git switch timing-test-current-main
git status --short --branch
cargo test -p veloxity_mavlink --lib
cargo xtask flash-board pixracerpro --vcp
```

The expected branch contains the test-only raw OFFBOARD_CONTROL echo in
`comms/veloxity_mavlink/src/link.rs`. The `--vcp` option is required because the companion
identifies Veloxity through its USB VCP device.

After flashing, unplug and reconnect the Pixracer Pro USB cable before running the companion test.

## 2. Copy the tested companion ZIP to the Raspberry Pi

Run `scp` as one command; do not put a newline between the source and destination unless the
previous line ends in `\`.

```bash
scp tools/rosflight_timing_test/dist/rosflight_timing_companion_pi_zsh_v5_current_main_20260831.zip ferris@192.168.1.107:~/projects/foo/.distrobox-home/foo/
```

Replace the user, host, and destination when using another Pi. The destination above places the
archive directly in the tested Distrobox home.

SSH to the Pi, enter the Distrobox, and extract it:

```zsh
ssh ferris@192.168.1.107
dist
cd ~
unzip -q rosflight_timing_companion_pi_zsh_v5_current_main_20260831.zip
cd rosflight_timing_companion
```

Do not record passwords in this repository or in copied command examples.

## 3. Select validated parameters

For Veloxity, the setup automatically checks these files in order:

1. `$RF_TIMING_VELOXITY_PARAM_FILE`, when explicitly set;
2. `~/.config/veloxity/airframes/3dquad/firmware/snapshots/quad_x_slim.yaml`;
3. `~/.config/veloxity/airframes/3dquad/firmware/quad_x_slim.yaml`;
4. the legacy `firmware-startup-veloxity.yaml` path.

Verify the tested default:

```zsh
ls -l ~/.config/veloxity/airframes/3dquad/firmware/snapshots/quad_x_slim.yaml
```

To use another validated snapshot, set it before sourcing the setup file:

```zsh
export RF_TIMING_VELOXITY_PARAM_FILE="$HOME/path/to/validated-veloxity.yaml"
```

The optional C run expects
`~/.config/veloxity/airframes/3dquad/firmware/firmware-startup-c.yaml`. These files are deliberately
not bundled: parameter sets are airframe-specific and loading an unreviewed set is unsafe.

## 4. Build the isolated ROS 2 companion overlay

Inside the Distrobox:

```zsh
source setup.zsh
timing_build
```

This creates a private overlay under the extracted directory. It checks out pinned
`rosflight_ros_pkgs` commit `d7d8cbdb7c318fa0583deb7002217af2196a0977`, applies only the timing
instrumentation to that private copy, and builds the timing driver. It does not modify a normal
ROSflight workspace.

If the Pi already has a local `rosflight_ros_pkgs` checkout and network access is unavailable:

```zsh
timing_build "$HOME/path/to/rosflight_ros_pkgs"
```

`timing_build` is needed once per extracted bundle. If an old bundle left environment overrides
behind, reset them and source this bundle again:

```zsh
timing_deactivate 2>/dev/null || true
unset RF_TIMING_PARAM_DIR RF_TIMING_VELOXITY_PARAM_FILE
source setup.zsh
```

## 5. Run Veloxity

First run the five-second smoke test:

```zsh
timing_smoke veloxity
```

A healthy result has a near-400 Hz achieved rate, approximately 2,000 commands, 100% unique
responses, and no duplicate echoes. `raw echoes received: 0` means the firmware lacks the timing
echo or the wrong firmware was flashed.

Then run the standard five-second warmup and 180-second measurement:

```zsh
timing_run veloxity
```

After the line below, silence for roughly three minutes is normal:

```text
warming up serial link and allowing parameter synchronization for 5 seconds
```

Do not interrupt it unless a safety problem occurs. The summary and result-directory path print
when collection finishes.

## 6. Run the optional C comparison

To reproduce the reference C firmware rather than reusing an existing C result, clone current
ROSflight firmware at the recorded baseline, apply both supplied patches, build for Pixracer Pro,
and flash using the normal ROSflight board procedure:

```bash
git clone --recursive https://github.com/rosflight/rosflight_firmware.git
cd rosflight_firmware
git checkout cd787430a960aadbb59cef07ad1f2abc0e8cc0ae
git submodule update --init --recursive
/path/to/Veloxity/tools/rosflight_timing_test/c_firmware/apply_patches.sh "$PWD"
cmake -S . -B build-pixracerpro -DBOARD_TO_BUILD=pixracer_pro -DCMAKE_BUILD_TYPE=Release
cmake --build build-pixracerpro -j
```

The echo patch is required for RTT measurements. The pseudoinverse patch prevents unsafe mixer
gains from rank-deficient canned mixers. Flash the resulting Pixracer Pro image with the normal
ROSflight debug-probe procedure, reconnect USB to the Pi, then run:

```zsh
timing_smoke c
timing_run c
```

Use the same Pi, cable, USB port, power configuration, parameters, ROS domain, and background load
for C and Veloxity. Run both for 180 seconds at 400 Hz.

## 7. Package and copy results back

On the Pi, from the extracted companion directory:

```zsh
timing_package
```

The command prints the full path to `rosflight_timing_results_<UTC>.tar.gz`. Copy that path back to
the analysis computer, for example:

```bash
scp ferris@192.168.1.107:~/projects/foo/.distrobox-home/foo/rosflight_timing_results_<UTC>.tar.gz .
mkdir imported_timing_results
tar -xzf rosflight_timing_results_<UTC>.tar.gz -C imported_timing_results
```

Each run directory contains:

- `rtt_samples.csv`: first valid response for each unique command sequence;
- `all_echoes.csv`: every raw arrival and its `first`, `duplicate`, foreign-run, invalid, or
  out-of-order classification;
- `publish_intervals.csv`: actual publisher timing;
- `summary.txt` and `summary.json`: response counts and latency statistics;
- `run_metadata.txt`, `rosflight_io.log`, and `driver.log`.

## 8. Create the two standalone comparison plots

Use `all_echoes.csv`, not only `rtt_samples.csv`, when investigating outliers. The latter is
deduplicated by design and will hide delayed duplicate C responses.

```bash
python3 tools/rosflight_timing_test/analysis/plot_raw_timing_comparison.py \
  /path/to/C_RUN_DIRECTORY \
  /path/to/VELOXITY_RUN_DIRECTORY \
  --output-dir /path/to/comparison_plots
```

This writes two independent images:

- `raw_rtt_at_or_below_3ms.png`: overlaid histogram of every raw arrival at or below 3 ms;
- `raw_rtt_outliers_above_3ms.png`: every raw arrival above 3 ms over elapsed time, with duplicate
  echoes shown as `x` markers.

Without `--output-dir`, the script opens both figures interactively. Change the split with
`--threshold-ms`, or pass `--central-output` and `--output` for explicit filenames.

## Troubleshooting

### The parameter YAML cannot be read

Confirm the path under [Select validated parameters](#3-select-validated-parameters). If a stale
environment variable points to an old location, deactivate, unset it, and source `setup.zsh` again.

### The smoke test reports zero echoes

The Pixracer Pro is not running an echo-enabled firmware, the wrong USB device is connected, or the
private timing overlay was not built/selected. Reflash this branch with `--vcp`, reconnect USB, and
rerun `timing_smoke veloxity`.

### The run appears stuck after warmup

The full runner is quiet during its 180-second collection. Wait until it prints the summary. Use
`timing_smoke` first when debugging.

### The script finds zero or multiple USB devices

Inspect `/dev/serial/by-id/`, disconnect unrelated boards, and verify `dialout` permissions. The
wrapper intentionally requires exactly one matching firmware identity.

### C outliers seem to be missing

Plot `all_echoes.csv`. In the latest reference run, 417 delayed C duplicates were present in
addition to 16 non-duplicate C outliers above 3 ms. A plot based only on `rtt_samples.csv` omits
those duplicates.

## Maintaining this canonical test

When rebasing the branch onto a newer Veloxity `main`:

1. preserve and retest the OFFBOARD_CONTROL echo;
2. run the Rust unit test and Pixracer Pro VCP release build;
3. validate a smoke and full Pi run;
4. update the validation date/commit and reference results here;
5. rebuild the distributable with `./tools/rosflight_timing_test/build_companion_bundle.sh`;
6. keep this documentation indexed from `docs/README.md`, `docs/tutorials/README.md`, and
   `tools/README.md`.

Do not add generated overlay workspaces or complete result archives to Git. Keep the reviewable
source, small tested bundle, instructions, and compact reference summaries here.
