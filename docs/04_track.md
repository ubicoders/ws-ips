# 04 Track

Module 3. Fill `lift_to_world`, `fit_cube`, and `KalmanFilter::update` in `ips_common`, then run the live pipeline.

Acceptance: `cargo test -p ips_common` passes; `ips_track` holds 90 Hz for 60 s with all four cameras; the residual RMS stays under the tolerance while the cube moves; the velocity plot in `ips_viz` is smooth and finite.
