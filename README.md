# ws-ips

Indoor positioning system (IPS) built in the Ubicoders IPS workshop. Four ceiling cameras watch a room, each frame is scanned for ArUco markers, the markers from all cameras are fused into one rigid pose per marker cube, and the pose is published over zenoh at 90 Hz. The repository ends at that pose on the wire. Feeding it into PX4 is the separate `ws-px4` repository.

Every `ips_*` folder under `modules/` is one swarmbotix (`sb`) module: one standalone Rust crate, one `sb.module.yaml`, one process per instance (a camera publisher and a detector run as four processes each, one per camera, in their own tmux windows, and so does the optional image relay). sb builds and starts every module itself from that manifest. A module depends on nothing in another module's folder. The pipeline, in run order:

```
ips_cam_gcam_pub (x4)  ->  ips_detect (x4)  ->  ips_track  ->  ips_viz  ->  rerun
     iceoryx2          |       zenoh              zenoh         gRPC
                       |          \-> ips_calib (window, writes calibration/extrinsics/) -> ips_track
                       \-> ips_image_relay (x4, optional, plus ips_detect's debug_image)  ->  network peers
                                zenoh, half size; only in the relay flow (sb up relay)
```

The wire keys and transports are tabulated in [docs/pipeline.md](docs/pipeline.md); the workspace and flow files are explained in [docs/workspace.md](docs/workspace.md).

| Module | Copies | Process and tmux window | Wire segment |
|---|---|---|---|
| `ips_cam_gcam_pub` | one per camera | `ips_cam_gcam_pub-sb00` .. `-sb03` | `ips_cam_gcam_pub-sbNN` |
| `ips_detect` | one per camera, same id as its camera | `ips_detect-sb00` .. `-sb03` | `ips_detect-sbNN` |
| `ips_image_relay` | one per camera, same id as its camera; optional, halves the camera and debug images onto zenoh, not part of `sb up` | `ips_image_relay-sb00` .. `-sb03`, started with the relay flow, `sb up relay` | `ips_image_relay-sbNN` |
| `ips_track` | singleton, merges the four detector streams | `ips_track` | `ips_track` |
| `ips_viz` | singleton | `ips_viz` | none (subscribes only) |
| `ips_calib` | singleton, calibration window, not part of `sb up` | `ips_calib`, started with the calibration flow, `sb up calib` | none (subscribes only) |

`sb.workspace.yaml` registers all six modules and the message definitions. Which of them run is said by the three flow files beside it: `sb.flow.yaml`, the flow `default` that `sb up` starts, runs the four live modules; `relay.flow.yaml`, the flow `relay` that `sb up relay` starts, runs the same four plus the optional `ips_image_relay`; and `calib.flow.yaml`, the flow `calib` that `sb up calib` starts, runs the cameras, the detectors and `ips_calib`. The camera ids live once per flow, as the `instances:` of the per-camera modules (two in `sb.flow.yaml` and `calib.flow.yaml`, three in `relay.flow.yaml`), from which sb starts one process per id, and each flow's `wires:` records which publisher feeds which subscriber. Inside the programs, the camera list comes from `modules/ips_cam_gcam_pub/parameters/cam_params.yaml`, and the other modules refer to a camera by its index there. The ids in all three flow files and the camera lists in the parameters yaml files must agree.

## Pipeline

Images stay on the host in shared memory (iceoryx2); detections and the cube pose cross the network (zenoh), and so do half-size copies of the images while the optional `ips_image_relay` runs (the relay flow, `sb up relay`). `<device>` is the name set with `sb config set device`; every key is `<device>/ws_ips/<module>[-<id>]/<transport>/<topic>`.

```
 CAMERAS (GigE, HuaTeng)                       ips_cam_gcam_pub  (Rust on drivers/gcam-mvsdk-rs, x4, full at handout)
 ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐           ┌────────────────────────────────┐
 │ sb00 │ │ sb01 │ │ sb02 │ │ sb03 │ ───────▶  │ capture MONO8 1280x1024        │
 └──────┘ └──────┘ └──────┘ └──────┘           │ publish over iceoryx2 (shmem)  │
                                               └───────────────┬────────────────┘
                                                               │ iox2  sensor_msgs/Image1280x1024
                                                               │ <device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image
                  ┌────────────────────────────────────────────┤
                  ▼                                            ▼
   ips_image_relay  (optional, x4)             ips_detect  (Rust, one per camera, x4)
   ┌─────────────────────────────┐             ┌────────────────────────────────┐
   │ halve image and debug_image │ iox2        │ OpenCV aruco DICT_4X4_250      │
   │ 1280x1024 -> 640x512        │◀────────────┤ marker side 0.087 m            │
   │ INTER_AREA, header copied   │ debug_image │ pose per marker, camera frame  │  <- stub: pose_from_corners
   └────┬────────────────────────┘             └───────┬────────────────┬───────┘
        │ zenoh  sensor_msgs/Image                     │ zenoh  custom/MarkerDetections
        │ .../ips_image_relay-sbNN/zenoh/image         │ <device>/ws_ips/ips_detect-sbNN/zenoh/markers
        │ .../ips_image_relay-sbNN/zenoh/debug_image   │                │
        ▼                        ┌─────────────────────┤                │ iox2  debug_image (debug: true)
   network peers (phone app)     │                     │                │
                                 ▼                     ▼                │
   ips_calib  (window, x1)                     ips_track  (Rust, x1)    │
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
   ips_image_relay is optional and not part of sb up: the relay flow (sb up relay) runs
   the same pipeline with it.
```

## Layout

| Folder | What | Shared how |
|---|---|---|
| `modules/` | `ips_cam_gcam_pub`, `ips_detect`, `ips_image_relay`, `ips_calib`, `ips_track`, `ips_viz`: the six modules | not shared; each builds alone |
| `drivers/` | `gcam-mvsdk-rs`, `gcam-mvsdk-py`: the camera driver, git submodules, pinned | path dependency of `ips_cam_gcam_pub` |
| `messages/` | the two `.proto` payloads | every module compiles them in its own `build.rs`; `sb install .` also imports them into the sb vault so the io generators know the types |
| `calibration/` | intrinsics, the calib board, extrinsics | files read and written at runtime |
| `sb.workspace.yaml` | the sb workspace inventory: the six modules and the messages | read by `sb install .` |
| `sb.flow.yaml`, `relay.flow.yaml`, `calib.flow.yaml` | the flows `default` (live pipeline), `relay` (live pipeline plus the image relay) and `calib` (calibration): `nodes:`, the camera `instances:`, `wires:` | read by `sb install .`; started by `sb up`, `sb up relay` and `sb up calib` |
| `docs/` | the modules' lessons, `pipeline.md`, recorded captures in `data/` | reading |

Every module folder has the same skeleton:

```
modules/ips_<name>/
├── Cargo.toml          crate name = folder name = sb module name = wire segment; lib plus one bin
├── build.rs            prost on ../../messages/custom/*.proto (the four modules that speak them; ips_image_relay compiles the vault's sensor_msgs/Image instead)
├── sb.module.yaml      sb module manifest (language, platforms, publishers, subscribers, services), portable, committed; sb builds and starts the module from it
├── parameters/         <name>_params.yaml, read at startup
├── swarmbotix_io/      sb-generated pub/sub/service code, written by `sb install .`, gitignored
├── src/                lib.rs (includes swarmbotix_io and the prost types), main.rs, this module's own code and stubs
├── tests/              one test per stub, on synthetic data and docs/data captures
└── README.md           inputs, outputs, wire keys, stubs
```

`sb up` and `sb run` build each module themselves (`cargo build --release`, the Rust default; no module declares `scripts:`). To build and test by hand, work inside the module folder (`cargo build --release`, `cargo test`). The OpenCV crate in `ips_detect` and `ips_image_relay` needs `LIBCLANG_PATH` at build time: sb sets it from `platforms.linux.env` in each of the two modules' `sb.module.yaml` (default `/usr/lib/llvm-18/lib`; export the variable before `sb up` to point it elsewhere), and a hand-run `cargo` there needs it exported in your shell. Do not run `cargo fmt` at a module root: it follows the `#[path]` includes and rewrites the generated `swarmbotix_io/` files.

## What arrives full and what you build

| Folder | At handout | Filled by |
|---|---|---|
| `drivers/` | full, git submodules, pinned | nobody, read only |
| `messages/`, `sb.workspace.yaml`, `sb.flow.yaml`, `relay.flow.yaml`, `calib.flow.yaml`, `docs/`, `calibration/intrinsics/` | full | nobody |
| `modules/ips_cam_gcam_pub/` | full | nobody |
| `modules/ips_detect/` | module, `pose_from_corners` stubbed | you, module 2 |
| `modules/ips_calib/` | module, `fit_extrinsics` and `reprojection_residual` stubbed | you, module 2 |
| `modules/ips_track/` | module, `lift_to_world`, `fit_cube`, and the Kalman filter (`KalmanFilter::update` plus the `Cv1` and `Mekf` predict and update steps) stubbed | you, module 3 |
| `modules/ips_viz/` | full | nobody |
| `modules/ips_image_relay/` | full | nobody |
| `calibration/extrinsics/` | empty | `ips_calib`, at runtime |

Each module's README states its inputs, outputs, wire keys, and the stubs to fill. A stub compiles and returns `todo!()`; its test in the module's `tests/` fails until you fill it, and `cargo test` inside the module folder is the acceptance check. A `solution/module<N>` tag is pushed after each module's deadline.

## Setup

```bash
git clone --recurse-submodules https://github.com/ubicoders/ws-ips
cd ws-ips
sb config set device <yourname>   # first segment of every wire key; the generated code bakes it in
sb install .                      # sb.workspace.yaml and the flow files -> sb workspace ws_ips, messages into the vault, swarmbotix_io per module
sb up                             # builds, then starts the default flow (sb.flow.yaml): every module except ips_calib and ips_image_relay
```

Details in [docs/00_setup.md](docs/00_setup.md).

## Run

Three levels. Every command works from any directory: sb finds each module through the installed workspace. `sb up` and `sb run` first build the module (`cargo build --release`, in your terminal; a failed build starts nothing), then start one process per instance in the module folder. An instance's process gets `--id <id>`; every process gets `SB_WORKSPACE`, `SB_MODULE` and `SB_MODULE_ROOT`, and an instance also `SB_INSTANCE_ID`. All windows land in the tmux session `sb-ws_ips` (`sb attach`, switch with `Ctrl+b w`). A window stays open after its process exits, so a crash leaves its output on screen. `sb up` starts one flow and records it as the active flow; `sb run` takes a module's instances from the active flow and refuses a module that is not a node of it.

### 1. Everything

```bash
sb up        # build, then start every node of sb.flow.yaml: 4 camera windows, 4 detector windows, ips_track, ips_viz
sb up relay  # the same plus 4 image relay windows (relay.flow.yaml); `sb down` first when a session runs
sb down      # kill the whole tmux session sb-ws_ips
```

### 2. Per-camera modules: `ips_cam_gcam_pub`, `ips_detect`, `ips_image_relay`

One process per camera. sb opens one window per id in the module's `instances:` in the active flow (`sb.flow.yaml` after `sb up`), named `<module>-<id>`, and starts the binary there with `--id <id>`. `<module>:<id>` targets one instance; `sb run` on a window that is already open restarts it in place.

```bash
sb run ips_cam_gcam_pub              # all four cameras, one window each (ips_cam_gcam_pub-sb00 .. -sb03)
sb run ips_cam_gcam_pub:sb00         # one camera, window ips_cam_gcam_pub-sb00
sb stop ips_cam_gcam_pub             # closes every ips_cam_gcam_pub-sbNN window

sb run ips_detect                    # all four detectors, one window each (ips_detect-sb00 .. -sb03)
sb run ips_detect:sb00               # one detector, window ips_detect-sb00
sb stop ips_detect                   # closes every ips_detect-sbNN window

sb run --fg ips_detect:sb00          # no tmux: one detector in this terminal, Ctrl+C stops it (CI, IDE)
```

`--fg` runs exactly one process, so an instanced module needs its `:<id>`. Started by hand without `--id` (`cargo run --release` inside the module folder), each of the three binaries instead runs every camera of its parameters yaml in one process, one thread each.

`ips_image_relay` is optional and a node of the relay flow only, so while `sb up` runs the default flow, `sb run ips_image_relay` is refused (`module ips_image_relay is not a node of flow default`). In the relay flow:

```bash
sb down && sb up relay               # the pipeline plus the relays; plain `sb up relay` when nothing runs
sb run ips_image_relay               # all four relays, one window each (ips_image_relay-sb00 .. -sb03)
sb run ips_image_relay:sb00          # one relay, window ips_image_relay-sb00
sb stop ips_image_relay              # closes every ips_image_relay-sbNN window; the pipeline keeps running
sb down && sb up                     # back to the pipeline without the relay
```

Beside the default flow, start the relay by hand from `modules/ips_image_relay/` once it is built: `target/release/ips_image_relay` (every camera, one process, one thread each) or `target/release/ips_image_relay --id sb00`. Details in [docs/06_relay.md](docs/06_relay.md).

Debug options, all in `modules/ips_detect/parameters/detector_params.yaml` (restart the detectors with `sb run ips_detect` after editing):

| Key | Effect |
|---|---|
| `debug: true` | publish an annotated colour frame on `.../ips_detect-sbNN/iox2/debug_image`; set `debug_show_images: true` in `modules/ips_viz/parameters/viz_params.yaml` to see the 2x2 grid in rerun; a running `ips_image_relay` republishes it at 640x512 on `.../ips_image_relay-sbNN/zenoh/debug_image` |
| `save: <dir>` | write raw frames to `<dir>/sbNN/<seq>.png`; `""` is off |
| `save_frames: N` | stop saving after N frames; `0` is unlimited |
| `family`, `marker_size` | `aruco` / `0.087` for the cubes, `april` / `0.185` for calibration |

The relay's own settings are in `modules/ips_image_relay/parameters/relay_params.yaml`: `scale` (integer divisor of both sides, `2` gives 640x512; it must divide 1280 and 1024) and `every` (relay every Nth received frame per stream, `1` relays all), the knob for a Wi-Fi peer that needs less bandwidth. Restart the relays after editing (`sb run ips_image_relay` in the relay flow).

Camera check before anything else: `gcam-mvsdk-rs measure sb00` must report about 91 fps with 0 lost, `gcam-mvsdk-rs grab sb00` saves one test frame, `sb topic list` shows the live image keys. The vendor SDK keeps the cameras' parameter groups (group A, loaded on every open, holds the IPS values: High frame-speed tier, 5 ms exposure) in `drivers/gcam-mvsdk-rs/data`: `.vscode/settings.json` sets `GCAM_MVSDK_DATA_DIR` for terminals opened here and the publisher's `sb.module.yaml` for `sb up`; export it yourself in other shells, so that no camera configuration lives outside the repository.

### 3. Singletons: `ips_track`, `ips_viz`, `ips_calib`

```bash
sb run ips_track                     # the tracker; it loads calibration/extrinsics/ at startup
sb stop ips_track

rerun                                # the viewer app, start it first
sb run ips_viz                       # streams to rerun+http://127.0.0.1:9876/proxy
sb run --fg ips_viz                  # the same, in this terminal instead of a tmux window
sb stop ips_viz                      # restarted rerun? restart ips_viz too, the stream does not reconnect

sb down && sb up calib               # calibration flow: cameras, detectors, ips_calib (window: S collect, C calibrate, W write); plain `sb up calib` when nothing runs
sb run ips_calib                     # the calibrator again, while calib is the active flow
sb run --fg ips_calib                # the same, in this terminal; exits with the calibrator's exit code
sb down && sb up                     # back to the live pipeline; ips_track loads the new extrinsics at startup
```

`ips_calib` is not a node of the default flow, so while `sb up` runs the live pipeline, `sb run ips_calib` is refused (`module ips_calib is not a node of flow default`). The session belongs to the workspace and `sb up` refuses one that exists, hence the `sb down` before switching flows. Details in [docs/03_calib.md](docs/03_calib.md).

### Peek at a topic

Fully-qualified keys start with `/`; without it sb looks for a bare topic in the module of the current directory. Shared-memory topics need `--transport iceoryx2`. The zenoh publishers tag every message with its type (`application/protobuf;ros2/custom/<Leaf>`, and `application/protobuf;ros2/sensor_msgs/Image` for the relay's images), so no `--type` is needed; `--show` opens the image in a window.

```bash
sb topic list
sb topic listen --transport iceoryx2 /<device>/ws_ips/ips_detect-sb00/iox2/debug_image -n 1 --no-data
sb topic listen --transport iceoryx2 /<device>/ws_ips/ips_detect-sb00/iox2/debug_image --show --no-data
sb topic listen /<device>/ws_ips/ips_detect-sb00/zenoh/markers -n 1
sb topic listen /<device>/ws_ips/ips_track/zenoh/cube_poses -n 1
sb topic listen /<device>/ws_ips/ips_image_relay-sb00/zenoh/image -n 1
```

If a zenoh listen prints `schema: null` and hex, that module's publisher predates the type tag: re-render it with `sb io update <module>` (or `sb install .` at the repository root, which re-renders every module), rebuild, restart.

`ips_viz` debug plots are toggled in `modules/ips_viz/parameters/viz_params.yaml`: `debug_show_pos_xyz`, `debug_show_vel_xyz` (filled only by the `kf` filter), `debug_show_ang_xyz`, `debug_show_images`. The tracker's filter is `filter:` in `modules/ips_track/parameters/tracker_params.yaml` (`lp`, `oneeuro`, `kf`).
