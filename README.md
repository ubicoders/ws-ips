# ws-ips

Indoor positioning system (IPS) built in the Ubicoders IPS workshop. Four ceiling cameras watch a room, each frame is scanned for ArUco markers, the markers from all cameras are fused into one rigid pose per marker cube, and the pose is published over zenoh at 90 Hz. The repository ends at that pose on the wire. Feeding it into PX4 is the separate `ws-px4` repository.

Every `ips_*` folder under `modules/` is one swarmbotix (`sb`) module: one standalone Rust crate, one `sb.dev.yml`, one run script, one process per instance (a camera publisher and a detector run as four processes each, one per camera, in their own tmux windows). A module depends on nothing in another module's folder. The pipeline, in run order:

```
ips_cam_gcam_pub (x4)  ->  ips_detect (x4)  ->  ips_track  ->  ips_viz  ->  rerun
     iceoryx2                  zenoh              zenoh         gRPC
                                  \-> ips_calib (one-shot, writes calibration/extrinsics/) -> ips_track
```

The wire keys and transports are tabulated in [docs/pipeline.md](docs/pipeline.md); the workspace file is explained in [docs/workspace.md](docs/workspace.md).

| Module | Copies | Process and tmux window | Wire segment |
|---|---|---|---|
| `ips_cam_gcam_pub` | one per camera | `ips_cam_gcam_pub-sb00` .. `-sb03` | `ips_cam_gcam_pub-sbNN` |
| `ips_detect` | one per camera, same id as its camera | `ips_detect-sb00` .. `-sb03` | `ips_detect-sbNN` |
| `ips_track` | singleton, merges the four detector streams | `ips_track` | `ips_track` |
| `ips_viz` | singleton | `ips_viz` | none (subscribes only) |
| `ips_calib` | singleton, one-shot, not part of `sb up` | `bash modules/ips_calib/runscript.bash`, own session `ips_calib-dev` | none (subscribes only) |

The camera ids come from `modules/ips_cam_gcam_pub/parameters/cam_params.yaml`; the other modules refer to a camera by its index there. `flow.yaml` lists all five modules (`ips_calib` with `up: false`, so `sb up` skips it) and the two per-camera ones under `instances:`.

## Pipeline

Images stay on the host in shared memory (iceoryx2); detections and the cube pose cross the network (zenoh). `<device>` is the name set with `sb config set device`; every key is `<device>/ws_ips/<module>[-<id>]/<transport>/<topic>`.

```
 CAMERAS (GigE, HuaTeng)                       ips_cam_gcam_pub  (Rust on drivers/gcam-mvsdk-rs, x4, full at handout)
 ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐           ┌────────────────────────────────┐
 │ sb00 │ │ sb01 │ │ sb02 │ │ sb03 │ ───────▶  │ capture MONO8 1280x1024        │
 └──────┘ └──────┘ └──────┘ └──────┘           │ publish over iceoryx2 (shmem)  │
                                               └───────────────┬────────────────┘
                                                               │ iox2  sensor_msgs/Image1280x1024
                                                               │ <device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image
                                                               ▼
                                               ips_detect  (Rust, one per camera, x4)
                                               ┌────────────────────────────────┐
                                               │ OpenCV aruco DICT_4X4_250      │
                                               │ marker side 0.087 m            │
                                               │ pose per marker, camera frame  │  <- stub: pose_from_corners
                                               └───────┬────────────────┬───────┘
                                                       │ zenoh  custom/MarkerDetections
                                                       │ <device>/ws_ips/ips_detect-sbNN/zenoh/markers
                                 ┌─────────────────────┤                │ iox2  debug_image (debug: true)
                                 ▼                     ▼                │
   ips_calib  (one-shot, x1)                   ips_track  (Rust, x1)    │
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
```

## Layout

| Folder | What | Shared how |
|---|---|---|
| `modules/` | `ips_cam_gcam_pub`, `ips_detect`, `ips_calib`, `ips_track`, `ips_viz`: the five modules | not shared; each builds alone |
| `drivers/` | `gcam-mvsdk-rs`, `gcam-mvsdk-py`: the camera driver, git submodules, pinned | path dependency of `ips_cam_gcam_pub` |
| `messages/` | the two `.proto` payloads | every module compiles them in its own `build.rs`; `sb install .` also imports them into the sb vault so the io generators know the types |
| `calibration/` | intrinsics, the calib board, extrinsics | files read and written at runtime |
| `flow.yaml` | the sb workspace: modules (`ips_calib` with `up: false`), messages, instances | read by `sb install .` |
| `docs/` | the modules' lessons, `pipeline.md`, recorded captures in `data/` | reading |

Every module folder has the same skeleton:

```
modules/ips_<name>/
├── Cargo.toml          crate name = folder name = sb module name = wire segment; lib plus one bin
├── build.rs            prost on ../../messages/custom/*.proto (the four modules that speak protobuf)
├── sb.dev.yml          sb module definition (publishers, subscribers, services), portable, committed
├── runscript.bash      what `sb run` / `sb up` execute; forwards all args
├── stopscript.bash     what `sb stop` runs after killing the process
├── parameters/         <name>_params.yaml, read at startup
├── swarmbotix_io/      sb-generated pub/sub/service code, written by `sb install .`, gitignored
├── src/                lib.rs (includes swarmbotix_io and the prost types), main.rs, this module's own code and stubs
├── tests/              one test per stub, on synthetic data and docs/data captures
└── README.md           inputs, outputs, wire keys, stubs
```

Build and test inside the module folder (`cargo build --release`, `cargo test`); the OpenCV crate needs `LIBCLANG_PATH=/usr/lib/llvm-18/lib`, which the run scripts export. Do not run `cargo fmt` at a module root: it follows the `#[path]` includes and rewrites the generated `swarmbotix_io/` files.

## What arrives full and what you build

| Folder | At handout | Filled by |
|---|---|---|
| `drivers/` | full, git submodules, pinned | nobody, read only |
| `messages/`, `flow.yaml`, `docs/`, `calibration/intrinsics/`, `calibration/ips_calib.yaml` | full | nobody |
| `modules/ips_cam_gcam_pub/` | full | nobody |
| `modules/ips_detect/` | module, `pose_from_corners` stubbed | you, module 2 |
| `modules/ips_calib/` | module, `fit_extrinsics` and `reprojection_residual` stubbed | you, module 2 |
| `modules/ips_track/` | module, `lift_to_world`, `fit_cube`, and the Kalman filter (`KalmanFilter::update` plus the `Cv1` and `Mekf` predict and update steps) stubbed | you, module 3 |
| `modules/ips_viz/` | full | nobody |
| `calibration/extrinsics/` | empty | `ips_calib`, at runtime |

Each module's README states its inputs, outputs, wire keys, and the stubs to fill. A stub compiles and returns `todo!()`; its test in the module's `tests/` fails until you fill it, and `cargo test` inside the module folder is the acceptance check. A `solution/module<N>` tag is pushed after each module's deadline.

## Setup

```bash
git clone --recurse-submodules https://github.com/ubicoders/ws-ips
cd ws-ips
sb config set device <yourname>   # first segment of every wire key; the generated code bakes it in
sb install .                      # flow.yaml -> sb workspace ws_ips, messages into the vault, swarmbotix_io per module
sb up                             # every module in flow.yaml
```

Details in [docs/00_setup.md](docs/00_setup.md).

## Run

Three levels. Every command works from any directory; the run scripts `cd` to their module. All windows land in the tmux session `sb-ws_ips` (`sb attach`, switch with `Ctrl+b w`).

### 1. Everything

```bash
sb up        # every module in flow.yaml: 4 camera windows, 4 detector windows, ips_track, ips_viz
sb down      # kill the whole tmux session
```

### 2. Per-camera modules: `ips_cam_gcam_pub`, `ips_detect`

One process per camera. Without `--id` the run script opens one window per camera listed in the module's `parameters/` yaml; with `--id` it runs that one camera in the current window.

```bash
sb run ips_cam_gcam_pub              # all cameras in cam_params.yaml, one window each
sb run ips_cam_gcam_pub --id sb00    # one camera, this window
sb stop ips_cam_gcam_pub             # closes every ips_cam_gcam_pub-sbNN window

sb run ips_detect                    # all cameras in detector_params.yaml, one window each
sb run ips_detect --id sb00          # one detector, this window
sb stop ips_detect                   # closes every ips_detect-sbNN window

SB_NO_TMUX=1 bash modules/ips_detect/runscript.bash   # no tmux: one process, one thread per camera (CI, IDE)
```

Debug options, all in `modules/ips_detect/parameters/detector_params.yaml` (restart the detectors after editing):

| Key | Effect |
|---|---|
| `debug: true` | publish an annotated colour frame on `.../ips_detect-sbNN/iox2/debug_image`; set `debug_show_images: true` in `modules/ips_viz/parameters/viz_params.yaml` to see the 2x2 grid in rerun |
| `save: <dir>` | write raw frames to `<dir>/sbNN/<seq>.png`; `""` is off |
| `save_frames: N` | stop saving after N frames; `0` is unlimited |
| `family`, `marker_size` | `aruco` / `0.087` for the cubes, `april` / `0.185` for calibration |

Camera check before anything else: `gcam-mvsdk-rs grab sb00` saves one test frame, `sb topic list` shows the live image keys.

### 3. Singletons: `ips_track`, `ips_viz`, `ips_calib`

```bash
sb run ips_track                     # the tracker; restart it after a new calibration
sb stop ips_track

rerun                                # the viewer app, start it first
sb run ips_viz                       # streams to rerun+http://127.0.0.1:9876/proxy
sb stop ips_viz                      # restarted rerun? restart ips_viz too, the stream does not reconnect

bash modules/ips_calib/runscript.bash        # calibration, one-shot, own tmux session ips_calib-dev; not in flow.yaml
```

### Peek at a topic

Fully-qualified keys start with `/`; without it sb looks for a bare topic in the module of the current directory. Shared-memory topics need `--transport iceoryx2`. The zenoh publishers tag every message with its type (`application/protobuf;ros2/custom/<Leaf>`), so no `--type` is needed; `--show` opens the image in a window.

```bash
sb topic list
sb topic listen --transport iceoryx2 /<device>/ws_ips/ips_detect-sb00/iox2/debug_image -n 1 --no-data
sb topic listen --transport iceoryx2 /<device>/ws_ips/ips_detect-sb00/iox2/debug_image --show --no-data
sb topic listen /<device>/ws_ips/ips_detect-sb00/zenoh/markers -n 1
sb topic listen /<device>/ws_ips/ips_track/zenoh/cube_poses -n 1
```

If a zenoh listen prints `schema: null` and hex, that module's publisher predates the type tag: re-render it with `sb io update <module>` (or `sb install .` at the repository root, which re-renders every module), rebuild, restart.

`ips_viz` debug plots are toggled in `modules/ips_viz/parameters/viz_params.yaml`: `debug_show_pos_xyz`, `debug_show_vel_xyz` (filled only by the `kf` filter), `debug_show_ang_xyz`, `debug_show_images`. The tracker's filter is `filter:` in `modules/ips_track/parameters/tracker_params.yaml` (`lp`, `oneeuro`, `kf`).
