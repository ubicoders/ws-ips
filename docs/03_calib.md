# 03 Calibrate

Module 2, second half. Intrinsics with Kalibr, then extrinsics with `ips_calib`.

1. Kalibr on your own AprilGrid images; the camchain goes to `calibration/intrinsics/sbNN-camchain.yaml`.
2. Measure the calib board tags with the caliper and put the size in `calibration/ips_calib.yaml`.
3. Set the detectors to `family: april`, `marker_size: 0.185` (or your measured value), restart `ips_detect`.
4. Fill `fit_extrinsics` and `reprojection_residual` in `ips_calib/src/fit.rs`, then `bash ips_calib/runscript.bash` (calibration is not part of `sb up`).

Acceptance: `cargo test` inside `ips_calib/` passes, a residual under 1 cm per camera, `sbNN_full.yaml` written for every camera. Then set the detectors back to `aruco`, `0.087`.
