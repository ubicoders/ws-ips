# ips_cam_gcam_pub

Reads one HuaTeng GigE camera (MONO8, 1280x1024) through `drivers/gcam-mvsdk-rs` and publishes every frame over iceoryx2 shared memory. One instance per camera, selected by the camera's user-set name. Full at handout: camera capture is not a lesson of this workshop.

Rust, native, no Docker. The driver loads the vendor library from `~/.gcam-mvsdk/` at run time (installed by `drivers/gcam-mvsdk-rs/install.bash`); nothing from the vendor is in this repository. Camera discovery needs raw sockets, so the run script sets `cap_net_raw` on the built binary.

## Inputs

- The camera on the LAN, named `sb00` to `sb03` with `gcam-mvsdk-rs rename` (docs/01_cameras.md).
- `parameters/cam_params.yaml`: `cameras`, `exposure_us`, `frame_rate_hz`, `packet_bytes`, `num_buffers`.

## Outputs

- `ros2/sensor_msgs/Image1280x1024`, one iceoryx2 service per camera, about 90 Hz. The header stamp is the capture time; `frame_id` is the camera name.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| image | iceoryx2 | `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image` |

## Layout

```
src/main.rs        --id parsing, one thread per camera, the publish loop
src/capture.rs     open by name, exposure, packet size, MONO8, continuous grab
parameters/cam_params.yaml
sb.dev.yml.in      publisher `image`
```

Depends on `drivers/gcam-mvsdk-rs` by path (`default-features = false`) and on iceoryx2 `=0.9.3`.

## State at handout, and what you build

Full. Nothing to build.

## Run

```bash
sb run ips_cam_gcam_pub              # every camera in cam_params.yaml, one thread each
sb run ips_cam_gcam_pub --id sb00    # one camera
sb stop ips_cam_gcam_pub
```

Parameters live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml; `--id` only narrows the list.
