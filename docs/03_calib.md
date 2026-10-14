# 03 Calibrate

Module 2, second half. Intrinsics with Kalibr, then extrinsics with `ips_calib`.

1. Kalibr on your own AprilGrid images; the camchain goes to `calibration/intrinsics/sbNN-camchain.yaml`.
2. Measure the calib board tags with the caliper and put the size in `calibration/ips_calib.yaml`.
3. Set the detectors to `family: april`, `marker_size: 0.185` (or your measured value) in `modules/ips_detect/parameters/detector_params.yaml`. The detectors read it at startup, and the calibration flow of step 4 starts them afresh.
4. Fill `fit_extrinsics` and `reprojection_residual` in `modules/ips_calib/src/fit.rs`, then start the calibration flow, `calib.flow.yaml`: `sb down && sb up calib`, or plain `sb up calib` when no session is running. It builds and starts the cameras, the detectors and `ips_calib` together; the calibrator accumulates, fits, writes `calibration/extrinsics/sbNN_full.yaml` and exits, and its window keeps the results until `sb stop ips_calib`. Calibration is not part of `sb up`: `ips_calib` is not a node of the default flow, so `sb run ips_calib` is refused while the live pipeline is the active flow (`module ips_calib is not a node of flow default`), and `sb up` refuses a session that already exists, hence the `sb down`. Once `calib` is the active flow, `sb run ips_calib` runs the calibrator again (for example after a timeout), and `sb run --fg ips_calib` runs it in the current terminal.

Acceptance: `cargo test` inside `modules/ips_calib/` passes, a residual under 1 cm per camera, `sbNN_full.yaml` written for every camera. Then set the detectors back to `aruco`, `0.087` and return to the live pipeline with `sb down && sb up`; `ips_track` loads the new extrinsics at startup.
