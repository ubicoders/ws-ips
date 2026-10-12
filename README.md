# ws-ips

Indoor positioning system (IPS) built in the Ubicoders IPS workshop. Four ceiling cameras watch a room, each frame is scanned for ArUco markers, the markers from all cameras are fused into one rigid pose per marker cube, and the pose is published over zenoh at 90 Hz. The repository ends at that pose on the wire. Feeding it into PX4 is the separate `ws-px4` repository.

Every folder at the root except `drivers/`, `messages/`, `calibration/`, `workspace/`, and `docs/` is one swarmbotix (`sb`) module: one process, one `sb.dev.yml`, one run script. The pipeline, in run order:

```
cam_gige_ht (x4)  ->  ips_detect (x4)  ->  ips_track  ->  ips_viz
   iceoryx2             zenoh              zenoh         rerun
                            \-> ips_calib (one-shot, writes calibration/extrinsics/)
```

The full diagram, the wire keys, and the transports are in [docs/pipeline.md](docs/pipeline.md).

## What arrives full and what you build

| Folder | At handout | Filled by |
|---|---|---|
| `drivers/` | full, git submodules, pinned | nobody, read only |
| `messages/`, `workspace/`, `docs/`, `calibration/intrinsics/`, `calibration/ips_calib.yaml` | full | nobody |
| `cam_gige_ht/` | full | nobody |
| `ips_common/` | types and traits, functions stubbed | you, modules 2 and 3 |
| `ips_detect/`, `ips_calib/` | module scaffold, core stubbed | you, module 2 |
| `ips_track/` | module scaffold, core stubbed | you, module 3 |
| `ips_viz/` | full | nobody |
| `calibration/extrinsics/` | empty | `ips_calib`, at runtime |

Each module's README states its inputs, outputs, wire keys, and the stubs to fill. A `solution/module<N>` tag is pushed after each module's deadline.

## Setup

```bash
git clone --recurse-submodules https://github.com/ubicoders/ws-ips
cd ws-ips
bash workspace/setup.sh      # sb workspace, message vault, submodules
sb up                        # every module in workspace/flow.yaml
```

Details in [docs/00_setup.md](docs/00_setup.md).
