# workspace/

The sb workspace `ws_ips`: `flow.yaml` and the script that installs it.

| File | What |
|---|---|
| `flow.yaml` | the workspace definition sb reads: which modules exist and which instances they have |
| `setup.sh` | one-time install of this clone for sb; safe to re-run |

## How sb finds a module

sb follows three files. `setup.sh` writes all of them.

1. `~/.swarmbotix/active` holds the active workspace name, `ws_ips` (written by `sb ws set`).
2. `~/.swarmbotix/workspaces/ws_ips/flow.yaml` is the module registry. Its `modules:` map goes from the name you type after `sb run` to the absolute path of that module's `sb.dev.yml`. A module not in this map does not exist to sb.
3. That `sb.dev.yml` names the module folder (`root:`) and its publishers and subscribers. sb runs `runscript.bash` and `stopscript.bash` from that root.

The folder name itself is never used. Only the key in `flow.yaml` and the `root:` path matter.

## flow.yaml

sb reads two things from it: `modules:` and `instances:`. It does not list edges. An edge exists when one module subscribes to the key another one publishes; the drawing is in the [root README](../README.md).

Every key is `<device>/ws_ips/<module>[-<id>]/<transport>/<topic>`. `<device>` comes from `sb config get device`, `ws_ips` is this workspace, and `-<id>` is the instance.

### modules

`ips_cam_gcam_pub`, `ips_detect`, `ips_track`, `ips_viz`. `sb up` starts every module listed here, one tmux window each in the session `sb-ws_ips`.

`ips_calib` is deliberately absent. Calibration is a one-shot step, not part of the live pipeline, so `sb run ips_calib` fails on purpose. Start it with `bash modules/ips_calib/runscript.bash`, which opens its own tmux session `ips_calib-dev`. `setup.sh` still generates its `swarmbotix_io/`.

### instances

Only the modules that run several copies side by side: `ips_cam_gcam_pub` and `ips_detect`, one instance per camera. An instance id becomes the module suffix on the wire (`ips_detect-sb00`) and is the camera's user-set name, written into the camera once with `gcam-mvsdk-rs rename <serial> sbNN`. A detector and the camera it reads share the id.

The four cameras are predefined, not discovered. The ids here must agree with the camera list in every module's parameters yaml (`cam_params`, `detector_params`, `calib_params`, `tracker_params`, `viz_params`). Adding a camera means adding it in all six places.

`ips_track` and `ips_viz` are not listed. There is a single copy of each (one tracker fuses all cameras, one viewer draws it), so they carry no suffix and use plain keys such as `ips_track/zenoh/cube_poses`. Listing one here would give it an id and turn its key into `ips_track-<id>/...`, which nothing expects. Being under `modules:` is what makes them part of the workspace.

### Launch and fan-out

sb does not read `instances:` at launch. `sb up` starts each module with no arguments. The run scripts of `ips_cam_gcam_pub` and `ips_detect` do the fan-out: started without `--id` they open one window per camera in their parameters yaml (`ips_detect-sb00`, ...), each its own process running `--id sbNN`, then exit so the module's own window closes.

| Command | Effect |
|---|---|
| `sb up` | every module in `flow.yaml`; cameras and detectors fan out to one window each |
| `sb run ips_detect --id sb00` | one detector, in the current window |
| `sb run ips_track` | the singleton tracker (likewise `ips_viz`) |
| `sb stop ips_detect` | kills the module and its camera windows (`stopscript.bash`) |
| `bash modules/ips_calib/runscript.bash` | calibration, not part of `sb up` |

## setup.sh

Run once after cloning. Re-run after editing `flow.yaml`, any `sb.dev.yml.in`, or `messages/custom/*.proto`, and after moving the clone or changing the device name, because the generated files hold absolute paths and the device name. It does not build any module; each `runscript.bash` runs `cargo build --release` on launch.

1. Pulls the driver submodules.
2. Creates and activates the sb workspace `ws_ips`.
3. Installs `flow.yaml` to `~/.swarmbotix/workspaces/ws_ips/` with `<REPO>` replaced.
4. Writes every module's `sb.dev.yml` from its `sb.dev.yml.in`, same replacement.
5. Copies `messages/custom/*.proto` into the sb message vault and compiles them for the iox2 and proto backends.
6. Generates every module's `swarmbotix_io/` from its `sb.dev.yml`, plus `service.rs` for the `ips_track` cam_param queryable.
