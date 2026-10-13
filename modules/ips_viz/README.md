# ips_viz

Subscribes to the cube poses from `ips_track`, fetches each camera's extrinsics once from the cam_param service, and streams the scene (camera axes, oriented cube boxes, position and velocity plots, optional 2x2 debug images) to a running rerun viewer over gRPC. Full at handout; the tracker never talks to rerun itself, so a viewer hiccup cannot stall tracking.

## Inputs

- `<device>/ws_ips/ips_track/zenoh/cube_poses`.
- `<device>/ws_ips/ips_track/zenoh/service/cam_param/sbNN` at startup, retried until the tracker answers.
- `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image` when `debug_show_images: true`.
- `parameters/viz_params.yaml`: cube box size, cameras to draw, `rerun_hz`, which panels to show.

## Outputs

- A rerun log stream to `rerun+http://127.0.0.1:9876/proxy`. Start `rerun` (0.32) first; with no viewer up it buffers and reconnects.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| cube_poses (subscribed) | zenoh | `<device>/ws_ips/ips_track/zenoh/cube_poses` |
| cam_param (queried) | zenoh queryable | `<device>/ws_ips/ips_track/zenoh/service/cam_param/sbNN` |
| debug_image (subscribed) | iceoryx2 | `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image` |

## Layout

```
src/main.rs         the cam_param fetch, the subscribe loop
src/visualize.rs    parameters, cube box and camera extrinsics, the rerun blueprint, cameras, cubes, plots, debug images
parameters/viz_params.yaml
sb.dev.yml          subscribers `cube_poses`, `debug_image`
src/lib.rs          crate root: the swarmbotix_io includes, the prost types, visualize
build.rs            prost on ../../messages/custom/*.proto
```

Depends on zenoh 1.9, iceoryx2 `=0.9.3`, prost, nalgebra, rerun 0.32 (sdk only, no bundled viewer; the crate's minor version must match the installed `rerun` viewer).

## State at handout, and what you build

Full. Nothing to build. Use the position plot to read jitter and the velocity plot to judge the Kalman stub in module 3.

## Run

```bash
rerun                       # the viewer, once
sb run ips_viz
sb stop ips_viz
```

Parameters live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml.
