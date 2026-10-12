# Pipeline

Four GigE cameras, one detector per camera, one tracker, one viewer. Images stay on the host in shared memory (iceoryx2); detections and the cube pose cross the network (zenoh). `<device>` is the name set with `sb config`; `ws_ips` is the workspace.

```
 CAMERAS (GigE, HuaTeng MVSDK)                 cam_gige_ht  (C++, Docker, x4, full at handout)
 +------+ +------+ +------+ +------+           +--------------------------------+
 | sb00 | | sb01 | | sb02 | | sb03 | --------> | capture MONO8 1280x1024        |
 +------+ +------+ +------+ +------+           | publish over iceoryx2 (shmem)  |
                                               +---------------+----------------+
                                                               | iox2
                                                               | <device>/ws_ips/cam_gige_ht-sbNN/iox2/image
                                                               v
                                               ips_detect  (Rust, one process per camera, x4)
                                               +--------------------------------+
                                               | OpenCV aruco DICT_4X4          |
                                               | marker side 0.087 m            |
                                               | pose per marker, camera frame  |  <- stub: pose_from_corners
                                               +---------------+----------------+
                                                               | zenoh  custom/MarkerDetections
                                                               | <device>/ws_ips/ips_detect-sbNN/zenoh/markers
                                                               v
   ips_calib (one-shot)                        ips_track  (Rust, x1)
   +--------------------------+  extrinsics/   +--------------------------------+
   | AprilTag board 0.185 m   | --sbNN_full--> | subscribe ips_detect-*/markers |
   | Kabsch fit per camera    |   .yaml        | lift markers to world frame    |  <- stub: lift_to_world
   | writes K + R|t           |  (K, R|t)      | 5 markers per cube: TOP BACK   |
   +--------------------------+                |   RIGHT FRONT LEFT (ids 5k..)  |
     <- stub: fit_extrinsics                   | rigid fit per cube             |  <- stub: fit_cube
                                               | filter (lp | oneeuro | kf)     |  <- stub: KalmanFilter::update
                                               | fixed 90 Hz tick, stale drop   |
                                               +------+------------------+------+
                                                      | zenoh            | zenoh queryable
                                                      | custom/CubePoses | cam_param/sbNN
                                                      | .../ips_track/zenoh/cube_poses
                                                      v                  v
                                               ips_viz  (Rust, x1, full at handout)
                                               +--------------------------------+
                                               | cube boxes + camera axes       |
                                               | position / velocity plots      |
                                               | optional 2x2 debug images      |
                                               +---------------+----------------+
                                                               | gRPC
                                                               | rerun+http://127.0.0.1:9876/proxy
                                                               v
                                               +--------------------------------+
                                               | rerun viewer (3D world view)   |
                                               +--------------------------------+

   Also subscribing to cube_poses over the zenoh router: a phone, a second machine,
   and the onboard computer of the ws-px4 repository. This repository ends at that key.
```

## Transports

| Hop | Transport | Message | Key |
|---|---|---|---|
| camera to detector | iceoryx2 (shared memory, zero copy) | `sensor_msgs/Image1280x1024` | `<device>/ws_ips/cam_gige_ht-sbNN/iox2/image` |
| detector to tracker and calib | zenoh (protobuf) | `custom/MarkerDetections` | `<device>/ws_ips/ips_detect-sbNN/zenoh/markers` |
| tracker to viewer and network | zenoh (protobuf) | `custom/CubePoses` | `<device>/ws_ips/ips_track/zenoh/cube_poses` |
| tracker to viewer (startup) | zenoh queryable | JSON (K, R, t) | `<device>/ws_ips/ips_track/zenoh/service/cam_param/sbNN` |
| detector to viewer (debug) | iceoryx2 | `sensor_msgs/Image1280x1024Color` | `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image` |
| viewer to rerun | gRPC | rerun log stream | `rerun+http://127.0.0.1:9876/proxy` |

## Run order

1. `sb run cam_gige_ht --id sbNN`, one per camera (or all through `sb up`).
2. Once per camera placement: detectors on `family: april`, `marker_size: 0.185`, then `sb run ips_calib`. Writes `calibration/extrinsics/sbNN_full.yaml`.
3. Detectors back on `aruco`, `0.087`. `sb up` runs cameras, detectors, tracker, viewer.
4. `rerun` for the viewer.

## Frames

- Camera optical frame RDF (+x right, +y down, +z forward), metres.
- Extrinsics: `world = R * cam + t`, camera centre at `t`.
- Cube body frame: origin at the cube centre, axes of the TOP marker (id 5k). `cube = id / 5`, `face = id % 5`.
- Cube pose: `world = R * body + t`, published with euler angles, quaternion, velocity, contributing marker ids, camera count, residual RMS, and a liveness flag.
