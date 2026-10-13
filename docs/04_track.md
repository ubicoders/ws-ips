# 04 Track

Module 3. Fill `lift_to_world` and `fit_cube` in `modules/ips_track/src/cube.rs`, then the Kalman filter in `modules/ips_track/src/filter_kf.rs`: `KalmanFilter::update` and the `predict` and `update` steps of `Cv1` (constant-velocity position, one axis) and `Mekf` (error-state orientation). Then run the live pipeline.

Acceptance: `cargo test` inside `modules/ips_track/` passes; `ips_track` holds 90 Hz for 60 s with all four cameras; the residual RMS stays under the tolerance while the cube moves; the velocity plot in `ips_viz` is smooth and finite.
