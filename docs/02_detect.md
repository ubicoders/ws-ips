# 02 Detect

Module 2. Fill `pose_from_corners` in `ips_detect/src/markers.rs`, then run `ips_detect` on the recorded calib board frames and on your cameras.

Acceptance: `cargo test` inside `ips_detect/` passes, and `sb topic listen` on a markers key shows marker poses whose distances match the tape measure within a few centimetres.
