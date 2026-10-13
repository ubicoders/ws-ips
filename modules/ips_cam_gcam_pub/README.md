# ips_cam_gcam_pub

Reads one HuaTeng GigE camera (MONO8, 1280x1024) through `drivers/gcam-mvsdk-rs` and publishes every frame over iceoryx2 shared memory. One instance per camera, selected by the camera's user-set name. Full at handout: camera capture is not a lesson of this workshop.

Rust, native, no Docker. The driver loads the vendor library from `~/.gcam-mvsdk/` at run time (installed by `drivers/gcam-mvsdk-rs/install.bash`); nothing from the vendor is in this repository. No privileges are needed: discovery and streaming work from a plain user process (if `gcam-mvsdk-rs list` finds nothing, `gcam-mvsdk-rs doctor` names the host setting to fix).

## Inputs

- The camera on the LAN, named `sb00` to `sb03` with `gcam-mvsdk-rs rename` (docs/01_cameras.md).
- `parameters/cam_params.yaml`: `cameras`, `exposure_us`, `frame_rate_hz`, `packet_bytes`, `num_buffers`.

## Outputs

- `ros2/sensor_msgs/Image1280x1024`, one iceoryx2 service per camera, about 90 Hz. The header stamp is the capture time; `frame_id` is the camera name.

The camera must be on its full 1280x1024 preset (the factory default). A frame of any other size is not resized, because a resized image no longer matches the intrinsics: the error is printed once and that camera's thread stops while the others keep publishing.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| image | iceoryx2 | `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image` |

## Layout

```
src/main.rs        the camera selected by --id (all of them, one thread each, only without tmux), retry until the camera answers, the publish loop
runscript.bash     no --id: opens one tmux window per camera running `--id sbNN`, then exits
src/capture.rs     cam_params.yaml and --id, open by name, exposure, packet size, MONO8, grab into the payload
src/lib.rs         crate root: the swarmbotix_io include and capture
parameters/cam_params.yaml
sb.dev.yml         publisher `image`
```

Depends on `drivers/gcam-mvsdk-rs` by path (`default-features = false`) and on iceoryx2 `=0.9.3`.

## State at handout, and what you build

Full. Nothing to build.

## Run

```bash
sb run ips_cam_gcam_pub              # one tmux window and process per camera in cam_params.yaml (ips_cam_gcam_pub-sbNN)
sb run ips_cam_gcam_pub --id sb00    # one camera
sb stop ips_cam_gcam_pub
```

Parameters live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml; `--id` only narrows the list.
