# ips_track

Subscribes to every detector's marker stream, lifts each marker into the world frame with the camera's extrinsics, and fits one rigid pose per marker cube from the known five-marker layout. Ticks at a fixed 90 Hz: each tick takes the latest frame from every live camera, drops stale cameras, fits, filters, and publishes. Also serves each camera's `K`, `R`, `t` over a zenoh queryable for `ips_viz`.

## Inputs

- `<device>/ws_ips/ips_detect-*/zenoh/markers` with the detectors on the cube ArUco (0.087 m).
- `calibration/extrinsics/sbNN_full.yaml` for every camera in `parameters/tracker_params.yaml`.
- `parameters/tracker_params.yaml`: cube geometry, `cube_ids`, cameras, tolerances, `rate_hz`, `stale_ms`, the filter and its tuning.

## Outputs

- `custom/CubePoses` (protobuf) over zenoh at `rate_hz`: per cube `R`, `t`, euler, quaternion, velocity, marker ids, camera count, residual RMS, liveness.
- Zenoh service `cam_param/sbNN` answering with that camera's `K`, `R`, `t` as JSON.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| markers (all cameras) | zenoh | `<device>/ws_ips/ips_detect-*/zenoh/markers` |
| cube_poses | zenoh | `<device>/ws_ips/ips_track/zenoh/cube_poses` |
| cam_param | zenoh queryable | `<device>/ws_ips/ips_track/zenoh/service/cam_param/sbNN` |

## State at handout, and what you build

Scaffold. The subscribers, the 90 Hz tick loop, the stale-camera drop, the publisher, the cam_param service, and the low-pass filter are in place. You fill, in module 3:

| Stub | Where | Test |
|---|---|---|
| `lift_to_world`: a marker pose in camera frame plus the camera's `R`, `t` to a pose in the world frame | `ips_common/src/cube.rs` | synthetic |
| `fit_cube`: observed world-frame marker poses plus the body-frame layout to one cube pose, with the reject pass | `ips_common/src/cube.rs` | the recorded zenoh capture of a moving cube |
| `KalmanFilter::update`: constant-velocity position plus error-state orientation | `ips_common/src/filter_kf.rs` | the same capture; velocity must come out finite and smooth |

Markers pack 5 per cube: cube `k` owns ids `5k` to `5k+4`, faces TOP, BACK, RIGHT, FRONT, LEFT. The body frame is the cube centre with axes of the TOP marker.

## Run

```bash
sb run ips_track            # this module alone, in its own tmux session
sb stop ips_track
```

Parameters, when the module has them, live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml.
