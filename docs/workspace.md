# The sb workspace

`flow.yaml` at the repository root defines the sb workspace `ws_ips` (sb 0.5.0 and later). It is
committed, its paths are relative to the file, and `sb install .` turns it into the installed copy
that every `sb` verb reads.

## How sb finds a module

1. `~/.swarmbotix/active` holds the active workspace name, `ws_ips`. `sb install .` writes it.
2. `~/.swarmbotix/workspaces/ws_ips/flow.yaml` is the installed copy: the same file with absolute
   paths and a `source:` block. Never edit it; re-run `sb install .` instead. `sb up` and `sb run`
   warn when the repo copy is newer than the installed one.
3. Each module's `sb.dev.yml` names its publishers, subscribers and services. It has no `root:`
   line, so the module root is the folder holding it, and the file is committed. sb runs
   `runscript.bash` and `stopscript.bash` from that root.

The folder name itself is never used. Only the key in `flow.yaml` and the module's own
`module:` field matter.

## flow.yaml

| Key | Here |
|---|---|
| `name` | `ws_ips`, the second segment of every wire key `<device>/ws_ips/<module>[-<id>]/<transport>/<topic>` |
| `modules` | the five modules under `modules/`; `ips_calib` is `{ path: ..., up: false }` |
| `messages` | `messages/custom`, imported into the vault and compiled for `proto` and `iox2` on install |
| `instances` | one id per camera for `ips_cam_gcam_pub` and `ips_detect` |

### up: false

Calibration is a one-shot step, not part of the live pipeline. `ips_calib` is registered, so
`sb install .` renders its `swarmbotix_io/` and `sb run ips_calib` works, but `sb up` skips it.
`bash modules/ips_calib/runscript.bash` starts it in its own tmux session as well.

### instances

Only the modules that run several copies side by side: `ips_cam_gcam_pub` and `ips_detect`, one
instance per camera. An instance id becomes the module suffix on the wire (`ips_detect-sb00`)
and is the camera's user-set name, written into the camera once with
`gcam-mvsdk-rs rename <serial> sbNN`. A detector and the camera it reads share the id.

The four cameras are predefined, not discovered. The ids here must agree with the camera list in
every module's parameters yaml (`cam_params`, `detector_params`, `calib_params`,
`tracker_params`, `viz_params`). Adding a camera means adding it in all six places.

`ips_track` and `ips_viz` are not listed. There is a single copy of each, so they carry no suffix
and use plain keys such as `ips_track/zenoh/cube_poses`.

### Launch and fan-out

sb does not read `instances:` at launch. `sb up` starts each module with no arguments. The run
scripts of `ips_cam_gcam_pub` and `ips_detect` do the fan-out: started without `--id` they open
one window per camera in their parameters yaml, each its own process running `--id sbNN`, then
exit so the module's own window closes.

| Command | Effect |
|---|---|
| `sb install .` | install or refresh the workspace: flow, messages, generated io |
| `sb ws check` | report stale installs, missing generated files, type and instance problems |
| `sb up` | every `up: true` module; cameras and detectors fan out to one window each |
| `sb run ips_detect --id sb00` | one detector, in the current window |
| `sb run ips_track` | the singleton tracker (likewise `ips_viz`) |
| `sb run ips_calib` | calibration, skipped by `sb up` |
| `sb stop ips_detect` | kills the module and its camera windows (`stopscript.bash`) |

## What sb install does not do

- It does not pull the driver submodules: clone with `--recurse-submodules` or run
  `git submodule update --init --recursive`.
- It does not build: each `runscript.bash` runs `cargo build --release` on launch.
- It does not render or refresh service routers (`services:` in `sb.dev.yml`), because sb treats
  them as user-owned files. The tracker's run script renders `swarmbotix_io/services/cam_param.rs`
  when it is missing; after changing the device name run `sb io update --services ips_track`.
