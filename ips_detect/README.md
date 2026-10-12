# ips_detect

Subscribes to one camera's images, detects ArUco (or, for calibration, AprilTag) markers with OpenCV, estimates each marker's pose in the camera's optical frame, and publishes the detections for that frame over zenoh. One process per camera, pinned with `--id sbNN`.

## Inputs

- `<device>/ws_ips/cam_gige_ht-sbNN/iox2/image` (iceoryx2).
- `calibration/intrinsics/sbNN-camchain.yaml` (Kalibr).
- `parameters/detector_params.yaml`: `family` (aruco | april), `marker_size` (m), `debug`, `save`.

## Outputs

- `custom/MarkerDetections` (protobuf) over zenoh, every frame, empty when nothing is seen (heartbeat).
- Optional annotated colour frame over iceoryx2 when `debug: true`, for `ips_viz`.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| markers | zenoh | `<device>/ws_ips/ips_detect-sbNN/zenoh/markers` |
| debug_image | iceoryx2 | `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image` |

## State at handout, and what you build

Scaffold. The subscriber, the detector call, the message encoding, and the publisher are in place. You fill, in module 2:

| Stub | Where | Test |
|---|---|---|
| `pose_from_corners`: four image corners plus `K`, distortion, and the marker side to a pose in the camera frame | `ips_common/src/markers.rs` | recorded frames of the calib board, known distances |

Rule that bites: `marker_size` must match the printed tag. Cube ArUco 0.087 m, calib board 0.185 m, measured with the caliper. A wrong value scales every distance along the camera's view ray.

## Run

```bash
sb run ips_detect            # this module alone, in its own tmux session
sb stop ips_detect
```

Parameters, when the module has them, live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml.
