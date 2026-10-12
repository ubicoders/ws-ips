# ips_common

Rust library shared by `ips_detect`, `ips_calib`, and `ips_track`, so no module copies another. Not an sb module: no `sb.dev.yml`, no run script.

| File | Holds | State at handout |
|---|---|---|
| `src/camera.rs` | camchain loading, the per-instance image subscription | full |
| `src/markers.rs` | dictionary selection, the OpenCV detector call, `pose_from_corners` | `pose_from_corners` stubbed (module 2) |
| `src/fit.rs` | `fit_extrinsics` (Kabsch / Umeyama), `reprojection_residual` | both stubbed (module 2) |
| `src/cube.rs` | the cube model (five faces, body frame), `lift_to_world`, `fit_cube` | model full, both functions stubbed (module 3) |
| `src/filter_lp.rs`, `src/filter_one_euro.rs` | the low-pass and 1 Euro filters behind the `Filter` trait | full |
| `src/filter_kf.rs` | the constant-velocity Kalman filter with error-state orientation | `update` stubbed (module 3) |
| `tests/` | one test per stub, on synthetic data and on the recorded captures in `docs/data/` | full |

A stub compiles and returns `todo!()`. Its test fails until you fill it. `cargo test -p ips_common` is the acceptance check for modules 2 and 3.
