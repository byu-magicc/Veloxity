# Current ROS 2 Pixracer Pro serial RTT timing test

The canonical Raspberry Pi companion timing test for current Veloxity firmware lives in
[`tools/rosflight_timing_test/README.md`](https://github.com/magicc-safety/Veloxity/tree/timing-test-current-main/tools/rosflight_timing_test).

Use that harness for searches involving the ROS 2 timing test, Raspberry Pi timing test,
Pixracer Pro round-trip latency, serial RTT, `/serial_time_delay_ns`, C-versus-Veloxity timing,
400 Hz timing, the timing bell curve, or raw outliers above 3 ms. It contains the tested companion
ZIP, its reviewable source, firmware/ROS patches, analysis scripts, known-good results, safety
requirements, and complete first-time instructions.

Older local archives named `rosflight_timing_companion_pi_zsh_v1` through `v4` and ad hoc
`timing_testing_for_real` directories are superseded.
