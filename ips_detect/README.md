# ips_detect

Subscribes to one camera's images, detects ArUco (or, for calibration, AprilTag) markers with OpenCV, estimates each marker's pose in the camera's optical frame, and publishes the detections for that frame over zenoh. One instance per camera, same id as the camera it reads.

## Inputs

- `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image` (iceoryx2).
- `calibration/intrinsics/sbNN-camchain.yaml` (Kalibr).
- `parameters/detector_params.yaml`: `cameras` (indices into cam_params.yaml, 0 -> sb00), `family` (aruco | april), `marker_size` (m), `debug`, `save`.

## Outputs

- `custom/MarkerDetections` (protobuf) over zenoh, every frame, empty when nothing is seen (heartbeat). The stamp is the image's capture stamp, carried through.
- Optional annotated colour frame over iceoryx2 when `debug: true`, for `ips_viz`.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| image (subscribed) | iceoryx2 | `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image` |
| markers | zenoh | `<device>/ws_ips/ips_detect-sbNN/zenoh/markers` |
| debug_image | iceoryx2 | `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image` |

## Layout

```
src/main.rs          --id parsing, one thread per camera, the frame loop, encoding, publish
src/camera.rs        the iceoryx2 image subscription, wait-and-retry until the camera is up
src/intrinsics.rs    camchain yaml to the OpenCV camera matrix and distortion
src/markers.rs       dictionary selection, the detector call, pose_from_corners   <- stub
src/view.rs          the annotated debug frame
src/fps.rs           rate meter
tests/pose_from_corners.rs
parameters/detector_params.yaml
sb.dev.yml.in        subscriber `image`, publishers `markers`, `debug_image`
build.rs             prost on messages/custom/*.proto out of the sb vault
```

Depends on opencv (4.6 contrib `aruco`), iceoryx2 `=0.9.3`, zenoh 1.9, prost. Needs `LIBCLANG_PATH` to build; the run script sets it.

## State at handout, and what you build

Scaffold. The subscriber, the detector call, the message encoding, and the publisher are in place. You fill, in module 2:

| Stub | Where | Test |
|---|---|---|
| `pose_from_corners`: four image corners plus `K`, distortion, and the marker side to a pose in the camera frame | `src/markers.rs` | `tests/pose_from_corners.rs`: recorded frames of the calib board, known distances |

Rule that bites: `marker_size` must match the printed tag. Cube ArUco 0.087 m, calib board 0.185 m, measured with the caliper. A wrong value scales every distance along the camera's view ray.

## Run

```bash
sb run ips_detect              # every camera in detector_params.yaml, one thread each
sb run ips_detect --id sb00    # one camera
sb stop ips_detect
cargo test                     # inside this folder; the acceptance check
```

Parameters live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml; `--id` only narrows the list.
