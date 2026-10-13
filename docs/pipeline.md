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

A camera and the detector reading it share an id, `sb00` to `sb03`. The id is the camera's user-set name, written once with `gcam-mvsdk-rs rename`, and becomes the module suffix on the wire. `workspace/flow.yaml` lists the instances; each module's `parameters/` yaml lists the cameras it handles.

`sb up` starts every module in `flow.yaml` with no arguments, one tmux window each in the session `sb-ws_ips`. The run scripts of `ips_cam_gcam_pub` and `ips_detect` open one window per camera in their parameters yaml, named `ips_cam_gcam_pub-sbNN` and `ips_detect-sbNN`, each a separate process started with `--id sbNN`, and exit, so their own window closes. The session then holds four camera windows, four detector windows, `ips_track` and `ips_viz`. `ips_track` merges the four detector streams. `sb stop <module>` closes that module's camera windows; `sb run <module> --id sbNN` runs a single camera by hand. `ips_calib` is not in `flow.yaml`: start it with `bash ips_calib/runscript.bash` when calibrating. Switch windows with `Ctrl+b w`.

## Run order

1. `sb up`: cameras, detectors, tracker, viewer. The detectors wait for their camera, the tracker publishes empty heartbeats until markers arrive.
2. Once per camera placement: set the detectors to `family: april`, `marker_size: 0.185`, restart them, then `bash ips_calib/runscript.bash` (not part of `sb up`). It writes `calibration/extrinsics/sbNN_full.yaml` and exits. Restart `ips_track` so it loads them.
3. Detectors back on `aruco`, `0.087`.
4. `rerun` for the viewer.

## Frames

- Camera optical frame RDF (+x right, +y down, +z forward), metres.
- Extrinsics: `world = R * cam + t`, camera centre at `t`.
- Cube body frame: origin at the cube centre, RDF: +X toward the RIGHT face, +Y toward the BACK face, +Z toward the blank BOTTOM face. The TOP marker's X axis is body +X; its Y and Z axes are body -Y and -Z. `cube = id / 5`, `face = id % 5`, faces TOP, BACK, RIGHT, FRONT, LEFT. Cube ids are 0-based.
- Cube pose: `world = R * body + t`, published with euler angles, quaternion, velocity, contributing marker ids, camera count, residual RMS, and a liveness flag.
