# ws-ips

Indoor positioning system (IPS) built in the Ubicoders IPS workshop. Four ceiling cameras watch a room, each frame is scanned for ArUco markers, the markers from all cameras are fused into one rigid pose per marker cube, and the pose is published over zenoh at 90 Hz. The repository ends at that pose on the wire. Feeding it into PX4 is the separate `ws-px4` repository.

Every `ips_*` folder is one swarmbotix (`sb`) module: one standalone Rust crate, one process, one `sb.dev.yml`, one run script. A module depends on nothing in another module's folder. The pipeline, in run order:

```
ips_cam_gcam_pub (x4)  ->  ips_detect (x4)  ->  ips_track  ->  ips_viz  ->  rerun
     iceoryx2                  zenoh              zenoh         gRPC
                                  \-> ips_calib (one-shot, writes calibration/extrinsics/) -> ips_track
```

The full diagram, the wire keys, and the transports are in [docs/pipeline.md](docs/pipeline.md).

## Layout

| Folder | What | Shared how |
|---|---|---|
| `ips_cam_gcam_pub/`, `ips_detect/`, `ips_calib/`, `ips_track/`, `ips_viz/` | the five modules | not shared; each builds alone |
| `drivers/` | `gcam-mvsdk-rs`, `gcam-mvsdk-py`: the camera driver, git submodules, pinned | path dependency of `ips_cam_gcam_pub` |
| `messages/` | the two `.proto` payloads | copied into the sb vault by setup; every module compiles them in its own `build.rs` |
| `calibration/` | intrinsics, the calib board, extrinsics | files read and written at runtime |
| `workspace/` | `flow.yaml` (modules, instances, the pipeline drawn), `setup.sh` | sb workspace |
| `docs/` | the modules' lessons, `pipeline.md`, recorded captures in `data/` | reading |

Every module folder has the same skeleton:

```
ips_<name>/
├── Cargo.toml          crate name = folder name = sb module name = wire segment
├── sb.dev.yml.in       sb module definition; setup.sh writes sb.dev.yml from it
├── runscript.bash      what `sb run` / `sb up` execute; forwards all args
├── stopscript.bash     what `sb stop` runs after killing the process
├── parameters/         <name>_params.yaml, read at startup
├── swarmbotix_io/      sb-generated pub/sub code, per clone, gitignored
├── src/                this module's own code, including its stubs
├── tests/              one test per stub, on synthetic data and docs/data captures
└── README.md           inputs, outputs, wire keys, stubs
```

## What arrives full and what you build

| Folder | At handout | Filled by |
|---|---|---|
| `drivers/` | full, git submodules, pinned | nobody, read only |
| `messages/`, `workspace/`, `docs/`, `calibration/intrinsics/`, `calibration/ips_calib.yaml` | full | nobody |
| `ips_cam_gcam_pub/` | full | nobody |
| `ips_detect/` | module, `pose_from_corners` stubbed | you, module 2 |
| `ips_calib/` | module, `fit_extrinsics` and `reprojection_residual` stubbed | you, module 2 |
| `ips_track/` | module, `lift_to_world`, `fit_cube`, `KalmanFilter::update` stubbed | you, module 3 |
| `ips_viz/` | full | nobody |
| `calibration/extrinsics/` | empty | `ips_calib`, at runtime |

Each module's README states its inputs, outputs, wire keys, and the stubs to fill. A stub compiles and returns `todo!()`; its test in the module's `tests/` fails until you fill it, and `cargo test` inside the module folder is the acceptance check. A `solution/module<N>` tag is pushed after each module's deadline.

## Setup

```bash
git clone --recurse-submodules https://github.com/ubicoders/ws-ips
cd ws-ips
sb config set device <yourname>   # first segment of every wire key; setup bakes it in
bash workspace/setup.sh           # sb workspace, sb.dev.yml, message vault, swarmbotix_io, submodules
sb up                             # every module in workspace/flow.yaml
```

Details in [docs/00_setup.md](docs/00_setup.md).
