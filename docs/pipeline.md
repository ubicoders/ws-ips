# Pipeline

Four GigE cameras, one detector per camera, one tracker, one viewer. Images stay on the host in shared memory (iceoryx2); detections and the cube pose cross the network (zenoh). `<device>` is the name set with `sb config set device`; `ws_ips` is the workspace. Every key is `<device>/ws_ips/<module>[-<id>]/<transport>/<topic>`.

```
 CAMERAS (GigE, HuaTeng)                       ips_cam_gcam_pub  (Rust on drivers/gcam-mvsdk-rs, x4, full at handout)
 ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐           ┌────────────────────────────────┐
 │ sb00 │ │ sb01 │ │ sb02 │ │ sb03 │ ───────▶  │ capture MONO8 1280x1024        │
 └──────┘ └──────┘ └──────┘ └──────┘           │ publish over iceoryx2 (shmem)  │
                                               └───────────────┬────────────────┘
                                                               │ iox2  sensor_msgs/Image1280x1024
                                                               │ <device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image
                                                               ▼
                                               ips_detect  (Rust, one per camera, x4)
                                               ┌────────────────────────────────┐
                                               │ OpenCV aruco DICT_4X4_250      │
                                               │ marker side 0.087 m            │
                                               │ pose per marker, camera frame  │  <- stub: pose_from_corners
                                               └───────┬────────────────┬───────┘
                                                       │ zenoh  custom/MarkerDetections
                                                       │ <device>/ws_ips/ips_detect-sbNN/zenoh/markers
                                 ┌─────────────────────┤                │ iox2  debug_image (debug: true)
                                 ▼                     ▼                │
   ips_calib  (one-shot, x1)                   ips_track  (Rust, x1)    │
   ┌──────────────────────────┐  calibration/  ┌────────────────────────┴───────┐
   │ AprilTag board 0.185 m   │  extrinsics/   │ subscribe ips_detect-$*/markers│
   │ Kabsch fit per camera    │ ──sbNN_full──▶ │ lift markers to world frame    │  <- stub: lift_to_world
   │ writes K + R|t           │  .yaml (K,R|t) │ 5 markers per cube: TOP BACK   │
   └──────────────────────────┘                │   RIGHT FRONT LEFT (ids 5k..)  │
     <- stub: fit_extrinsics                   │ rigid fit per cube             │  <- stub: fit_cube
                                               │ filter (lp | oneeuro | kf)     │  <- stub: KalmanFilter::update
                                               │ fixed 90 Hz tick, stale drop   │
                                               └──────┬──────────────────┬──────┘
                                                      │ zenoh            │ zenoh queryable
                                                      │ custom/CubePoses │ cam_param/sbNN
                                                      │ <device>/ws_ips/ips_track/zenoh/cube_poses
                                                      ▼                  ▼
                                               ips_viz  (Rust, x1, full at handout)
                                               ┌────────────────────────────────┐
                                               │ cube boxes + camera axes       │
                                               │ position / velocity plots      │
                                               │ optional 2x2 debug images      │
                                               └───────────────┬────────────────┘
                                                               │ gRPC
                                                               │ rerun+http://127.0.0.1:9876/proxy
                                                               ▼
                                               ┌────────────────────────────────┐
                                               │ rerun viewer (3D world view)   │
                                               └────────────────────────────────┘

   Also subscribing to cube_poses over the zenoh router: a phone, a second machine,
   and the onboard computer of the ws-px4 repository. This repository ends at that key.
```

## Transports

| Hop | Transport | Message | Key |
|---|---|---|---|
| camera to detector | iceoryx2 (shared memory, zero copy) | `sensor_msgs/Image1280x1024` | `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image` |
| detector to tracker and calib | zenoh (protobuf) | `custom/MarkerDetections` | `<device>/ws_ips/ips_detect-sbNN/zenoh/markers`, subscribed as `ips_detect-$*` |
| tracker to viewer and network | zenoh (protobuf) | `custom/CubePoses` | `<device>/ws_ips/ips_track/zenoh/cube_poses` |
| tracker to viewer (startup) | zenoh queryable | JSON (K, R, t) | `<device>/ws_ips/ips_track/zenoh/service/cam_param/sbNN` |
| detector to viewer (debug) | iceoryx2 | `sensor_msgs/Image1280x1024Color` | `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image` |
| viewer to rerun | gRPC | rerun log stream | `rerun+http://127.0.0.1:9876/proxy` |

The wildcard is `$*`, not `*`: zenoh allows `*` only as a whole segment, and the instance is a suffix of the module segment.

## Instances

A camera and the detector reading it share an id, `sb00` to `sb03`. The id is the camera's user-set name, written once with `gcam-mvsdk-rs rename`, and becomes the module suffix on the wire. `workspace/flow.yaml` lists the instances; each module's `parameters/` yaml lists the cameras it handles.

`sb up` starts every module with no arguments. A module started without `--id` runs every camera in its parameters yaml in one process, one thread per camera. `sb run <module> --id sbNN` runs one camera. Each module is one tmux window in the session `sb-ws_ips`.

## Run order

1. `sb up`: cameras, detectors, tracker, viewer. The detectors wait for their camera, the tracker publishes empty heartbeats until markers arrive.
2. Once per camera placement: set the detectors to `family: april`, `marker_size: 0.185`, restart them, then `sb run ips_calib`. It writes `calibration/extrinsics/sbNN_full.yaml` and exits. Restart `ips_track` so it loads them.
3. Detectors back on `aruco`, `0.087`.
4. `rerun` for the viewer.

## Frames

- Camera optical frame RDF (+x right, +y down, +z forward), metres.
- Extrinsics: `world = R * cam + t`, camera centre at `t`.
- Cube body frame: origin at the cube centre, RDF: +X toward the RIGHT face, +Y toward the BACK face, +Z toward the blank BOTTOM face. The TOP marker's X axis is body +X; its Y and Z axes are body -Y and -Z. `cube = id / 5`, `face = id % 5`, faces TOP, BACK, RIGHT, FRONT, LEFT. Cube ids are 0-based.
- Cube pose: `world = R * body + t`, published with euler angles, quaternion, velocity, contributing marker ids, camera count, residual RMS, and a liveness flag.
