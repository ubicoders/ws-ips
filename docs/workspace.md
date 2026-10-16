# The sb workspace

Four committed files at the repository root define the sb workspace `ws_ips` (sb 0.8.0 and
later). `sb.workspace.yaml` is the inventory: the workspace name, the six modules and the
message definitions. `sb.flow.yaml`, `relay.flow.yaml` and `calib.flow.yaml` are the three flows,
the three ways to run the workspace: which modules run (nodes), how many processes each gets
(instances), and which publisher feeds which subscriber (wires). Paths are relative to
`sb.workspace.yaml`, and `sb install .` turns the four files into the installed copy that every
`sb` verb reads.

## How sb finds a module

1. `~/.swarmbotix/active` holds the active workspace name, `ws_ips`. `sb install .` writes it.
2. `~/.swarmbotix/workspaces/ws_ips/` holds the installed copy: `sb.workspace.yaml`, the same
   inventory with absolute paths (each naming the module's `sb.module.yaml`) and a `source:`
   block, and `flows/`, which holds the three flow files as committed. Once
   `sb up` has run, `active_flow` beside them names the flow it launched. Never edit the installed
   copy; re-run `sb install .` instead. `sb up` and `sb run` warn when a repository file is newer
   than the installed copy.
3. Each module's `sb.module.yaml` is its manifest: language, supported platforms, publishers,
   subscribers and services. It has no `root:` line, so the module root is the folder holding it,
   and the file is committed. No module declares `scripts:`, so sb applies the Rust defaults: it
   builds with `cargo build --release` in the module root and starts `target/release/<module>`.

The folder name itself is never used. Only the key under `modules:` in `sb.workspace.yaml` and
the module's own `module:` field matter. A flow names modules by that key and never holds a path.

## sb.workspace.yaml and the flows

`sb.workspace.yaml`:

| Key | Here |
|---|---|
| `name` | `ws_ips`, the second segment of every wire key `<device>/ws_ips/<module>[-<id>]/<transport>/<topic>`; spelled out because the folder name `ws-ips` is not an identifier |
| `sb` | `">=0.8"`, the sb version the workspace needs (informative, not enforced) |
| `modules` | the six modules under `modules/`, each a name and a path, nothing else |
| `messages` | one entry, `custom`: `messages/custom`, imported into the vault as style `ros2` and compiled for `proto` and `iox2` on install |

The flow files, one per way to run the workspace:

| File | Flow | Started by | `nodes` |
|---|---|---|---|
| `sb.flow.yaml` | `default` | `sb up` | `ips_cam_gcam_pub`, `ips_detect`, `ips_track`, `ips_viz` |
| `relay.flow.yaml` | `relay` | `sb up relay` | `ips_cam_gcam_pub`, `ips_detect`, `ips_track`, `ips_viz`, `ips_image_relay` |
| `calib.flow.yaml` | `calib` | `sb up calib` | `ips_cam_gcam_pub`, `ips_detect`, `ips_calib` |

Each flow holds `name:` (which must equal the name its file gives it: `default` for
`sb.flow.yaml`, `relay` for `relay.flow.yaml`, `calib` for `calib.flow.yaml`), `nodes:`,
`instances:` and `wires:`. A module listed in `sb.workspace.yaml` is registered: `sb install .`
renders its `swarmbotix_io/` and `sb ws check` checks it, whether or not a flow runs it. It runs
only in a flow that lists it under `nodes:`. sb has no optional node: `sb up` starts every node
of its flow, and `sb run` refuses a module that is not a node of the active flow. A module that
runs only sometimes therefore gets a flow of its own, as `ips_calib` and `ips_image_relay` do.

### The calib flow

Calibration is a separate step, run once per camera placement, not part of the live pipeline.
`ips_calib` is registered, so `sb install .` renders its `swarmbotix_io/`, but it is a node of
the `calib` flow only: `sb up` neither builds nor starts it, and `sb run ips_calib` is refused
while the default flow is active (`module ips_calib is not a node of flow default`). The tmux
session `sb-ws_ips` belongs to the workspace, not to a flow, and `sb up` refuses a session that
already exists, so switching to calibration is `sb down && sb up calib` (plain `sb up calib` when
nothing is running). It builds and starts the cameras, the detectors and `ips_calib` together,
and records `calib` as the active flow. With `ui: true` in its `ips_calib.yaml`, the default,
the calibrator opens an OpenCV window, `ips_calib`, with every camera's live image, and collects,
fits and writes on keys (see `modules/ips_calib/README.md`). The window needs a display: the
calibrator runs inside tmux and inherits `DISPLAY` from the terminal that ran `sb up`. Besides
the wired marker stream, it opens one iceoryx2 image subscriber per camera itself, which no wire
records because iceoryx2 has no wildcard. `sb run` follows the active flow: while `calib` is
active, `sb run ips_calib` runs the calibrator again in its tmux window, and
`sb run --fg ips_calib` runs it in the current terminal instead, which is where the headless mode
(`ui: false`) belongs: it prints the results and asks `[y/N]` before writing. The tmux window stays
open after the calibrator exits, so the printed results remain readable until `sb stop ips_calib`
closes it.
`sb down && sb up` returns to the live pipeline. `sb down` leaves the active flow recorded, so
until the next `sb up`, `sb run` still reads the `calib` flow and refuses `ips_track` and
`ips_viz`.

### The relay flow

`ips_image_relay` halves each camera's two iceoryx2 image streams and republishes them over zenoh
for network peers that cannot read shared memory (see [06_relay.md](06_relay.md)). It is
optional, so it is not a node of the default flow: `sb up` never starts it, and
`sb run ips_image_relay` is refused while the default flow is active
(`module ips_image_relay is not a node of flow default`). `relay.flow.yaml` is the default flow
plus the relay: five nodes, twelve instances (cameras, detectors and relays, four each) and four
wires. `sb up relay` builds and starts all of it (`sb down && sb up relay` when a session is
running) and records `relay` as the active flow. Then `sb run ips_image_relay` and
`sb run ips_image_relay:sb00` start or restart the relays, and `sb stop ips_image_relay` closes
their windows while the pipeline keeps running. `sb down && sb up` returns to the pipeline
without the relay. Beside the default flow, the relay can still be started by hand from its
module folder, outside sb: `target/release/ips_image_relay` runs every camera of
`relay_params.yaml` in one process, one thread each, and `--id sbNN` narrows it to one.

### instances

Only the modules that run several copies side by side: `ips_cam_gcam_pub`, `ips_detect` and
`ips_image_relay`, one instance per camera, listed under `instances:` in each flow that runs them
(`ips_detect: [sb00, sb01, sb02, sb03]`); only `relay.flow.yaml` runs the relay. Instances
belong to the flow, not to the module; all three flows here list the same four ids.
An instance id becomes the module suffix on the wire (`ips_detect-sb00`) and is the camera's
user-set name, written into the camera once with `gcam-mvsdk-rs rename <serial> sbNN`. A camera,
the detector reading it and the relay halving its images share the id.

The four cameras are predefined, not discovered. The id list lives once per flow, in each of the
three flow files, and decides how many processes sb starts. The programs still read their own
camera list from their parameters yaml (`cam_params`, `detector_params`, `relay_params`,
`ips_calib`, `tracker_params`, `viz_params`), and the two must agree: an instance whose id the
parameters do not list stops at startup with an error. Adding a camera means adding it to every
`instances:` list in the three flow files (two each in `sb.flow.yaml` and `calib.flow.yaml`,
three in `relay.flow.yaml`) and to all six parameters files.

`ips_track`, `ips_viz` and `ips_calib` have no instances. There is a single copy of each, so they
carry no suffix and use plain keys such as `ips_track/zenoh/cube_poses`.

### wires

`wires:` records, per publisher, the subscribers it feeds, publisher on the left. In
`sb.flow.yaml` the camera with a detector's own id feeds that detector
(`ips_cam_gcam_pub[{id}].image: ips_detect.image`), every detector feeds the tracker
(`ips_detect[*].markers: ips_track.markers`), and the tracker feeds the viewer
(`ips_track.cube_poses: ips_viz.cube_poses`). `relay.flow.yaml` has the same three wires, except
that the camera also feeds the relay of its id
(`ips_cam_gcam_pub[{id}].image: [ips_detect.image, ips_image_relay.image]`), plus a fourth: each
detector feeds its annotated frame to the relay of its id
(`ips_detect[{id}].debug_image: ips_image_relay.debug_image`). `calib.flow.yaml` has the default
camera wire and feeds every detector to the calibrator instead
(`ips_detect[*].markers: ips_calib.markers`). Both modules of a wire must be nodes of its flow,
which is why the `ips_calib` wire lives in `calib.flow.yaml` and the relay wires in
`relay.flow.yaml`. `sb ws check` validates each wire against the manifests in its `ws flows` row,
one line per flow (modules and entries must exist, transports and types must agree, `[*]` must
not be on iceoryx2), but code generation does not read them yet: each module's `main()` still
sets its subscriber's full key at startup, as before. `ips_viz.debug_image` is not wired, because
iceoryx2 has no wildcard; `main()` opens one subscriber per camera
(`ips_detect-sbNN/iox2/debug_image`) itself. The relay's `debug_image` is wired, because each
relay reads the one detector of its own id (`[{id}]`, not `[*]`).

### Launch and fan-out

sb does the fan-out itself. `sb up` launches the default flow: it builds every node of
`sb.flow.yaml` (`cargo build --release` in each module root, inline in your terminal; the first
failed build stops it before any window opens), then opens the tmux session `sb-ws_ips` with one
window per instance: `ips_cam_gcam_pub-sb00` .. `-sb03`, `ips_detect-sb00` .. `-sb03`,
`ips_track` and `ips_viz`. `sb up relay` and `sb up calib` do the same with the nodes of
`relay.flow.yaml` (adding `ips_image_relay-sb00` .. `-sb03`) and of `calib.flow.yaml`. An
instance's process is started with `--id <id>`; every process runs in its module root with
`SB_WORKSPACE`, `SB_MODULE` and `SB_MODULE_ROOT` set, plus `SB_INSTANCE_ID` for an instance. A
window stays open after its process exits, so a crash leaves its output readable.

| Command | Effect |
|---|---|
| `sb install .` | install or refresh the workspace: inventory, the three flows, messages, generated io, missing service routers |
| `sb ws check` | report stale installs, missing generated files, type and flow problems (row `ws flows`, one line per flow), deprecated leftovers |
| `sb up` | build, then start every node of the default flow, one window per instance |
| `sb up relay` | build, then start the nodes of the relay flow: the pipeline plus one `ips_image_relay` per camera; `sb down` first when a session is running |
| `sb up calib` | build, then start the nodes of the calibration flow; `sb down` first when a session is running |
| `sb run ips_detect` | build, then start (or restart) all four detectors of the active flow |
| `sb run ips_detect:sb00` | one detector, in window `ips_detect-sb00` |
| `sb run --fg ips_viz` | build, then run the viewer in the current terminal, no tmux; an instanced module needs its id (`ips_detect:sb00`) |
| `sb run ips_track` | the singleton tracker (likewise `ips_viz`), while the default or the relay flow is active |
| `sb run ips_calib` | the calibrator again, while `calib` is the active flow |
| `sb run ips_image_relay` | build, then start (or restart) the four relays, while `relay` is the active flow |
| `sb stop ips_detect` | close every `ips_detect-sbNN` window |
| `sb down` | kill the session `sb-ws_ips`; the active flow stays recorded |

Up to sb 0.6, this repository started each module through a per-module run script; sb 0.7.0 runs
none, so a leftover one is ignored and `sb ws check` lists it as deprecated. Up to sb 0.7, one
`sb.flow.yaml` held the modules, the messages, the instances and the wires; sb 0.8.0 splits it
into `sb.workspace.yaml` and the flow files. sb 0.8.0 still reads the old single file with a
deprecation warning, and sb 0.8.1 will not.

## What sb install does not do

- It does not pull the driver submodules: clone with `--recurse-submodules` or run
  `git submodule update --init --recursive`.
- It does not build: `sb up` and `sb run` build each module (`cargo build --release`) before
  starting it. `LIBCLANG_PATH`, which the opencv crate in `ips_detect` and `ips_image_relay`
  needs at build time only, comes from `platforms.linux.env` in each of the two modules'
  `sb.module.yaml` (default `/usr/lib/llvm-18/lib`); export the variable before `sb up` to point
  it elsewhere.
- It does not overwrite service routers (`services:` in `sb.module.yaml`), because sb treats them
  as user-owned files. It renders the tracker's `swarmbotix_io/services/cam_param.rs` only when the
  file is missing; after changing the device name, run `sb io update --services ips_track` to
  re-render it.
