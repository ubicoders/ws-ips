# Pipeline

Four GigE cameras, one detector per camera, one tracker, one viewer. Images stay on the host in shared memory (iceoryx2); detections and the cube pose cross the network (zenoh). `<device>` is the name set with `sb config set device`; `ws_ips` is the workspace. Every key is `<device>/ws_ips/<module>[-<id>]/<transport>/<topic>`.

The full diagram is in the [root README](../README.md#pipeline).

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

A camera and the detector reading it share an id, `sb00` to `sb03`. The id is the camera's user-set name, written once with `gcam-mvsdk-rs rename`, and becomes the module suffix on the wire. Each flow file lists the ids once, as the `instances:` of `ips_cam_gcam_pub` and `ips_detect`: `sb.flow.yaml` (the flow `default`, the live pipeline) and `calib.flow.yaml` (the flow `calib`, the calibration) carry the same four ids. Each module's `parameters/` yaml lists the cameras it handles, and the lists must agree. The modules themselves are registered in `sb.workspace.yaml`. Each flow's `wires:` records which publisher feeds which subscriber; `sb ws check` validates them (row `ws flows`, one line per flow), while each module's `main()` still sets its subscriber keys at startup.

`sb up` starts the default flow: it builds the four live modules (`cargo build --release`), then opens the tmux session `sb-ws_ips` with one window per instance: `ips_cam_gcam_pub-sb00` to `-sb03` and `ips_detect-sb00` to `-sb03`, each a separate process started with `--id sbNN`, plus `ips_track` and `ips_viz`. `ips_track` merges the four detector streams. `sb stop <module>` closes every window of that module; `sb run <module>:sbNN` starts or restarts a single camera or detector, and `sb run --fg <module>:sbNN` runs it in the current terminal instead. `ips_calib` is registered but is a node of the `calib` flow only, so `sb up` skips it and `sb run ips_calib` is refused while the default flow is active; calibration switches flows with `sb down && sb up calib` (see Run order). Switch windows with `Ctrl+b w`.

## Run order

1. `sb up`: cameras, detectors, tracker, viewer. The detectors wait for their camera, the tracker publishes empty heartbeats until markers arrive.
2. Once per camera placement: set the detectors to `family: april`, `marker_size: 0.185`, then switch to the calibration flow with `sb down && sb up calib`. It starts the cameras, the detectors (which read the new values at startup) and `ips_calib`, which writes `calibration/extrinsics/sbNN_full.yaml` and exits. While `calib` is the active flow, `sb run ips_calib` runs it again.
3. Detectors back on `aruco`, `0.087`, then `sb down && sb up` to return to the live pipeline; `ips_track` loads the new extrinsics at startup.
4. `rerun` for the viewer.

## Frames

- Camera optical frame RDF (+x right, +y down, +z forward), metres.
- Extrinsics: `world = R * cam + t`, camera centre at `t`.
- Cube body frame: origin at the cube centre, RDF: +X toward the RIGHT face, +Y toward the BACK face, +Z toward the blank BOTTOM face. The TOP marker's X axis is body +X; its Y and Z axes are body -Y and -Z. `cube = id / 5`, `face = id % 5`, faces TOP, BACK, RIGHT, FRONT, LEFT. Cube ids are 0-based.
- Cube pose: `world = R * body + t`, published with euler angles, quaternion, velocity, contributing marker ids, camera count, residual RMS, and a liveness flag.
