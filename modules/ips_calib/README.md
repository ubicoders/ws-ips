# ips_calib

Subscribes to every detector's marker stream while the cameras look at the IPS calib board, accumulates the marker centres per camera, and solves each camera's pose in the world frame by a rigid least-squares (Kabsch) fit against the board geometry in `parameters/ips_calib.yaml`. Writes one `sbNN_full.yaml` per camera. Run once per camera placement, not part of the live pipeline. By default it runs in a window that shows every camera's live image, and the operator starts the collection, the fit and the write by key; `ui: false` runs the same steps headless, in one shot. The detectors still do all the marker detection: the window uses OpenCV and iceoryx2 only to show the images.

## Inputs

- `<device>/ws_ips/ips_detect-$*/zenoh/markers` with the detectors set to `family: april`, `marker_size: 0.185`.
- `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image` (MONO8 1280x1024), window mode only: one iceoryx2 subscriber per camera, opened by the calibrator beside the detector's own subscription to the same stream. iceoryx2 has no wildcard, so this input is not in the `wires:` of `calib.flow.yaml`. No image is saved to disk.
- `calibration/intrinsics/sbNN-camchain.yaml`: K, copied into the output and used to place the tag labels in the window.
- `parameters/ips_calib.yaml`: the run settings `cameras`, `frames`, `timeout_s`, `min_markers`, `ui`, `rms_warn_m`, then the board (`world_frame`, `board`, `placement`). The only file that describes the board; nothing else reads it.

## Outputs

- `calibration/extrinsics/sbNN_full.yaml`: `K`, distortion, `R`, `t` (`world = R * cam + t`). In the window, written only on W, then Y.
- The reprojection residual per camera: printed in the headless mode, shown on each camera's tile in the window. Under a centimetre is good.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| markers (all cameras, subscribed) | zenoh | `<device>/ws_ips/ips_detect-$*/zenoh/markers` |
| image (each camera, subscribed, window only) | iceoryx2 | `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image` |

## Two modes

`ui:` in `parameters/ips_calib.yaml` chooses the mode.

`ui: true`, the default, opens one OpenCV window titled `ips_calib`: the cameras in a grid (2x2 for four, `ceil(sqrt(N))` columns in general) and a text panel under it. Each tile is the camera's live frame at half size, with the board tags the detector reports labelled `[id]`, a counter at the top right and a progress bar while collecting (`142/300`), and a status line at the bottom left. The labels are placed by projecting each tag centre through `K` (pinhole, distortion ignored), so a label may sit a few pixels off its tag. The panel gives the state, one instruction, and only the keys valid in that state:

| Key | Effect |
|---|---|
| S | start collecting; S again stops early |
| C | calibrate (fit) from what was collected |
| W | write the files: opens a confirmation box that lists each `sbNN_full.yaml` with its old and new residual |
| Y, N | in a confirmation box: confirm, cancel |
| R | reset to idle |
| Q, Esc | quit, with a confirmation when a fit would be lost; closing the window quits too |

Nothing is written to disk until Y. A skipped camera (fewer than `min_markers` board ids, no frames, or a detector on another family) keeps its existing file. A residual above `rms_warn_m` shows amber. `frames` is the collection goal; `timeout_s` does not apply. The window needs a display: `sb up calib` starts the calibrator inside tmux, which inherits `DISPLAY` from the terminal that ran `sb up`, and `sb run --fg ips_calib` runs it in the current terminal instead.

`ui: false` is the headless one-shot: accumulate until every camera has `frames` marker frames or `timeout_s` passes, fit, write, print the residuals, exit. It needs no display, and it is the only mode of a build without the `ui` feature.

## Layout

```
src/main.rs          load the setup, then the window or the headless run, by `ui`
src/setup.rs         parameters (run settings and board), paths, camchains, the marker stream
src/board.rs         the board section of ips_calib.yaml to world-frame marker centres
src/accumulate.rs    per camera, per marker id: median of the observed centres
src/fit.rs           fit_extrinsics, reprojection_residual                      <- stubs
src/calibrate.rs     fit and write one camera
src/writer.rs        sbNN_full.yaml
src/headless.rs      the one-shot (ui: false): accumulate until frames or timeout, fit, write, exit
src/images.rs        one iceoryx2 image subscriber per camera; a copy of ips_detect/src/camera.rs by design, since modules share no code
src/ui.rs            the window's state machine and text, pure (no OpenCV, no wire, no files)
src/mosaic.rs        OpenCV drawing: the camera grid, the panel, the confirmation box (feature `ui`)
src/session.rs       the interactive run, ui: true (feature `ui`)
tests/fit.rs
tests/ui.rs
parameters/ips_calib.yaml
sb.module.yaml       subscribers `markers`, `image`; LIBCLANG_PATH under platforms.<os>.env
src/lib.rs           crate root: the swarmbotix_io includes, the prost types, the modules above
build.rs             prost on ../../messages/custom/*.proto
```

Depends on zenoh 1.9, prost, nalgebra, serde_yaml, iceoryx2 `=0.9.3` (the image streams), and, for the window, opencv (`highgui`, `imgproc`) behind the cargo feature `ui`, which is on by default. The opencv crate needs `LIBCLANG_PATH` at build time only; sb sets it from `platforms.linux.env` in `sb.module.yaml`, default `/usr/lib/llvm-18/lib`, as for `ips_detect`. Export it yourself for a `cargo build` or `cargo test` by hand. A machine without OpenCV builds with `cargo build --no-default-features` and can run only the headless mode.

Tests: `tests/fit.rs` (the two fit tests and the board model), `tests/ui.rs` (the window's happy path, S to Y), and unit tests in `src/ui.rs` (the label projection) and `src/writer.rs` (the file format). `cargo test --no-default-features` runs them all without OpenCV.

## State at handout, and what you build

Scaffold. The subscribers, the accumulation, the window, and the yaml writer are in place. You fill, in module 2:

| Stub | Where | Test |
|---|---|---|
| `fit_extrinsics`: paired world and camera-frame marker centres to `R`, `t` (Kabsch / Umeyama, SVD) | `src/fit.rs` | `tests/fit.rs`: synthetic pairs with a known transform, a reflected set, too few points; the board model on `parameters/ips_calib.yaml` |
| `reprojection_residual`: RMS of the fitted centres against the measured ones | `src/fit.rs` | same |

A camera needs at least `min_markers` distinct board ids in view or it is skipped.

## Run

```bash
sb down && sb up calib      # the calibration flow (calib.flow.yaml): cameras, detectors and this module together;
                            # ui: true opens the window (S, C, W, Y); ui: false runs one shot and exits, and its
                            # tmux window ips_calib keeps the printed results until `sb stop ips_calib`
sb run ips_calib            # run it again, while calib is the active flow
sb run --fg ips_calib       # the same, in this terminal
cargo test                  # inside this folder; the acceptance check
cargo test --no-default-features   # the same on a machine without OpenCV
```

This module is registered in `sb.workspace.yaml` but is a node of `calib.flow.yaml` only, so `sb up` (the default flow) never builds or starts it, and `sb run ips_calib` is refused while the default flow is active (`module ips_calib is not a node of flow default`). Use plain `sb up calib` when no session is running; `sb down && sb up` returns to the live pipeline.

Parameters live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml.
