# Reference result: Raspberry Pi 5, 400 Hz, 180 seconds

These results are a known-good sanity reference, not a universal performance guarantee. Both runs
used the same Raspberry Pi 5, cable, Pixracer Pro, private ROS 2 overlay, 921600 baud, five-second
warmup, 400 Hz requested rate, and 180-second measurement.

| Metric | ROSflight C | Veloxity current-main timing branch |
| --- | ---: | ---: |
| Published commands | 71,999 | 71,999 |
| Raw echoes | 72,411 | 71,999 |
| Unique matched echoes | 71,994 | 71,999 |
| Duplicate echoes | 417 | 0 |
| Missing echoes | 5 | 0 |
| Unique response rate | 99.993% | 100.000% |
| Achieved publish rate | 399.995 Hz | 399.995 Hz |
| Mean RTT | 0.974429 ms | 0.737531 ms |
| Median RTT | 0.947036 ms | 0.704369 ms |
| p90 | 1.600494 ms | 1.345986 ms |
| p95 | 1.713469 ms | 1.646230 ms |
| p99 | 1.814337 ms | 1.887292 ms |
| p99.9 | 1.920054 ms | 1.970572 ms |
| Maximum unique RTT | 5.733385 ms | 3.415438 ms |
| Raw non-duplicate observations above 3 ms | 16 | 5 |
| Raw duplicate observations above 3 ms | 417 | 0 |

Recorded firmware and companion revisions:

- Veloxity timing commit: `d353e3f6c904bb8709d3b0db95ad4d103ea665bd`
- Veloxity parent `main`: `eea98c98519529af05d2ffbdde31f7623cb6d410`
- C firmware baseline: `cd787430a960aadbb59cef07ad1f2abc0e8cc0ae` plus the two patches under
  `c_firmware/`
- `rosflight_ros_pkgs` overlay baseline: `d7d8cbdb7c318fa0583deb7002217af2196a0977`
- Tested Pi ZIP SHA-256:
  `2ef3769959c329d55c880079319983f06b0a55c369413b4ce6891a6ab06d5171`

The raw-outlier count must come from `all_echoes.csv`. The unique-latency summary comes from
`rtt_samples.csv`; it intentionally excludes duplicate arrivals.
