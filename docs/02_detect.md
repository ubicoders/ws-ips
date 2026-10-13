# 02 Detect

Module 2. Fill `pose_from_corners` in `modules/ips_detect/src/markers.rs`, then run `ips_detect` on the recorded calib board frames and on your cameras.

Acceptance: `cargo test` inside `modules/ips_detect/` passes, and the detector window (`ips_detect-sbNN` under `sb attach`) shows marker poses whose distances match the tape measure within a few centimetres.
