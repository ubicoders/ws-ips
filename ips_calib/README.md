# ips_calib

Subscribes to every detector's marker stream while the cameras look at the IPS calib board, accumulates the marker centres per camera, and solves each camera's pose in the world frame by a rigid least-squares (Kabsch) fit against the board geometry in `calibration/ips_calib.yaml`. Writes one `sbNN_full.yaml` per camera. Run once per camera placement, not part of the live pipeline. Needs neither OpenCV nor iceoryx2: the detectors did the image work.

## Inputs

- `<device>/ws_ips/ips_detect-$*/zenoh/markers` with the detectors set to `family: april`, `marker_size: 0.185`.
- `calibration/ips_calib.yaml` (board and placement), `calibration/intrinsics/sbNN-camchain.yaml` (K, copied into the output).
- `parameters/calib_params.yaml`: `cameras`, `frames`, `timeout_s`, `min_markers`.

## Outputs

- `calibration/extrinsics/sbNN_full.yaml`: `K`, distortion, `R`, `t` (`world = R * cam + t`).
- The reprojection residual per camera, printed. Under a centimetre is good.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| markers (all cameras, subscribed) | zenoh | `<device>/ws_ips/ips_detect-$*/zenoh/markers` |

## Layout

```
src/main.rs          subscribe, accumulate until frames or timeout, fit, write, exit
src/board.rs         ips_calib.yaml to world-frame marker centres
src/accumulate.rs    per camera, per marker id: median of the observed centres
src/fit.rs           fit_extrinsics, reprojection_residual                      <- stubs
src/writer.rs        sbNN_full.yaml
tests/fit.rs
parameters/calib_params.yaml
sb.dev.yml.in        subscriber `markers`
src/lib.rs           crate root: the swarmbotix_io include, the prost types, the modules above
build.rs             prost on ../messages/custom/*.proto
```

Depends on zenoh 1.9, prost, nalgebra, serde_yaml.

## State at handout, and what you build

Scaffold. The subscriber, the accumulation, and the yaml writer are in place. You fill, in module 2:

| Stub | Where | Test |
|---|---|---|
| `fit_extrinsics`: paired world and camera-frame marker centres to `R`, `t` (Kabsch / Umeyama, SVD) | `src/fit.rs` | `tests/fit.rs`: synthetic pairs with a known transform, a reflected set, too few points; the board model on `calibration/ips_calib.yaml` |
| `reprojection_residual`: RMS of the fitted centres against the measured ones | `src/fit.rs` | same |

A camera needs at least `min_markers` distinct board ids in view or it is skipped.

## Run

```bash
bash ips_calib/runscript.bash   # one shot, exits when done; own tmux session ips_calib-dev
                                # (not in workspace/flow.yaml, so `sb up` never starts it)
cargo test                  # inside this folder; the acceptance check
```

Parameters live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml.
