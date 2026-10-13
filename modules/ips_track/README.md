# ips_track

Subscribes to every detector's marker stream, lifts each marker into the world frame with the camera's extrinsics, and fits one rigid pose per marker cube from the known five-marker layout. Ticks at a fixed 90 Hz: each tick takes the latest frame from every live camera, drops stale cameras, fits, filters, and publishes. Also serves each camera's `K`, `R`, `t` over a zenoh queryable for `ips_viz`.

## Inputs

- `<device>/ws_ips/ips_detect-$*/zenoh/markers` with the detectors on the cube ArUco (0.087 m).
- `calibration/extrinsics/sbNN_full.yaml` for every camera in `parameters/tracker_params.yaml`.
- `parameters/tracker_params.yaml`: cube geometry, `cube_ids`, cameras, tolerances, `rate_hz`, `stale_ms`, the filter and its tuning.

## Outputs

- `custom/CubePoses` (protobuf) over zenoh at `rate_hz`: per cube `R`, `t`, euler, quaternion, velocity, marker ids, camera count, residual RMS, liveness. The stamp is the newest contributing frame's capture stamp.
- Zenoh service `cam_param/sbNN` answering with that camera's `K`, `R`, `t` as JSON.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| markers (all cameras, subscribed) | zenoh | `<device>/ws_ips/ips_detect-$*/zenoh/markers` |
| cube_poses | zenoh | `<device>/ws_ips/ips_track/zenoh/cube_poses` |
| cam_param | zenoh queryable | `<device>/ws_ips/ips_track/zenoh/service/cam_param/sbNN` |

## Layout

```
src/main.rs            parameters, extrinsics, the 90 Hz tick loop, encoding, publish
src/cam_param.rs       the cam_param routes on the sb service router (swarmbotix_io/services/cam_param.rs)
src/cube.rs            cube model, lift_to_world, fit_cube                      <- stubs
src/filter.rs          the Filter enum and the none | lp | oneeuro | kf selection
src/filter_lp.rs       exponential low-pass
src/filter_one_euro.rs 1 Euro filter
src/filter_kf.rs       constant-velocity Kalman, KalmanFilter::update            <- stub
src/fps.rs             rate meter
tests/
parameters/tracker_params.yaml
sb.dev.yml             subscriber `markers`, publisher `cube_poses`, service `cam_param`
src/lib.rs             crate root: the swarmbotix_io includes, the prost types, the modules above
build.rs               prost on ../../messages/custom/*.proto
```

Depends on zenoh 1.9, prost, nalgebra, serde. No OpenCV, no iceoryx2.

## State at handout, and what you build

Scaffold. The subscribers, the 90 Hz tick loop, the stale-camera drop, the publisher, the cam_param service, and the low-pass and 1 Euro filters are in place. You fill, in module 3:

| Stub | Where | Test |
|---|---|---|
| `lift_to_world`: a marker pose in camera frame plus the camera's `R`, `t` to a pose in the world frame | `src/cube.rs` | synthetic |
| `fit_cube`: observed world-frame marker poses plus the body-frame layout to one cube pose, with the reject pass | `src/cube.rs` | `tests/fit_cube.rs`: a known pose from all five faces, from one face, and with a gross outlier; a recorded capture once `docs/data` has one |
| `KalmanFilter::update` and the `Cv1`, `Mekf` predict and update steps: constant-velocity position plus error-state orientation | `src/filter_kf.rs` | `tests/kalman.rs`: a static pose is held, constant velocity and constant angular rate are tracked with low lag |

Markers pack 5 per cube: cube `k` owns ids `5k` to `5k+4`, faces TOP, BACK, RIGHT, FRONT, LEFT, cubes 0-based. The body frame is the cube centre, RDF: +X toward RIGHT, +Y toward BACK, +Z toward the blank BOTTOM. The TOP marker's X is body +X; its Y and Z are body -Y and -Z.

## Run

```bash
sb run ips_track
sb stop ips_track
cargo test                  # inside this folder; the acceptance check
```

Parameters live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml.
