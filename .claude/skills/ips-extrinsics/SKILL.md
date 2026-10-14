---
name: ips-extrinsics
description: Run the extrinsic calibration of the ws-ips cameras with ips_calib and the calibration flow (sb up calib). The static AprilTag calib board described in modules/ips_calib/parameters/ips_calib.yaml defines the world frame; the detectors, switched to family april, report its tags; ips_calib accumulates the tag centres per camera, fits each camera's pose R, t by a rigid Kabsch fit, and writes calibration/extrinsics/sbNN_full.yaml, which ips_track loads at startup. Use when the user wants to calibrate, recalibrate after moving a camera, check a residual, read or regenerate an sbNN_full.yaml, run the headless one-shot, scale the setup to more or fewer cameras, or when ips_track or ips_viz complain about missing or stale extrinsics. Intrinsics come from Kalibr and are not produced here.
---

# ips-extrinsics

Extrinsics only. Each camera's intrinsics `K` and distortion come from Kalibr and live in
`calibration/intrinsics/sbNN-camchain.yaml`; `ips_calib` copies `K` into its output so one file per
camera holds everything. The chain:

```
calib board (AprilTag 36h11, static)        modules/ips_calib/parameters/ips_calib.yaml: ids, tag size, gap, layout, placement
        |  seen by every camera              world frame = the board placement (RDF, metres)
        v
ips_cam_gcam_pub-sbNN  (image, iox2)  ->  ips_detect-sbNN  (family: april)  ->  markers (zenoh)
                                                                                   |
        v  (window mode also shows the images)                                     v
ips_calib: median tag centre per camera and id  ->  fit_extrinsics (Kabsch)  ->  calibration/extrinsics/sbNN_full.yaml
                                                                                   |
                                                       ips_track loads at startup  <-  (world = R * cam + t)
```

Run once per camera placement. Moving a camera, changing its lens focus, or swapping a unit into a
position invalidates that position's `sbNN_full.yaml`.

## Prerequisites

- Cameras named and streaming: `gcam-list`, `gcam-live-view`. Every camera used must see at least
  `min_markers` (default 3) distinct board tags, sharp, at the IPS exposure of 5 ms.
- `calibration/intrinsics/sbNN-camchain.yaml` for every camera in `ips_calib.yaml`. A camera
  without a camchain is skipped at startup and not waited for.
- The board tags measured with a caliper: `marker_size_mm` under `board:` in `modules/ips_calib/parameters/ips_calib.yaml` (black
  square side) and `marker_gap_mm` (edge to edge). The same size, in metres, goes into the detectors.
- The two stubs filled (module 2): `fit_extrinsics` and `reprojection_residual` in
  `modules/ips_calib/src/fit.rs`; `cargo test` inside `modules/ips_calib/` passes. An unfilled stub
  panics with `todo!()` at the first fit.
- A display for the window mode. `sb up calib` starts the calibrator inside tmux, which inherits
  `DISPLAY` from the terminal that ran `sb up`; run `sb up calib` from a desktop terminal, not from
  a bare SSH session, or use `ui: false`.

## Files

| File | Role | Edit? |
|---|---|---|
| `modules/ips_detect/parameters/detector_params.yaml` | `family: april`, `marker_size: 0.185` (metres, your measured value) for calibration; `aruco`, `0.087` for the live pipeline | yes, before and after |
| `modules/ips_calib/parameters/ips_calib.yaml` | run settings: `cameras` (indices into `cam_params.yaml`), `frames` (300), `timeout_s` (30, headless only), `min_markers` (3), `ui` (true), `rms_warn_m` (0.01); and the board: `world_frame`, `board` (`family`, `marker_size_mm`, `marker_gap_mm`, `layout` as id per row and column), `placement` (`origin_id`, `position_mm`, `orientation_deg`), which defines the world frame | run settings per run; the board once, after measuring |
| `calib.flow.yaml` | flow `calib`: nodes `ips_cam_gcam_pub`, `ips_detect`, `ips_calib`; `instances:` per camera, same list as `sb.flow.yaml` | only when adding or removing cameras |
| `calibration/extrinsics/sbNN_full.yaml` | output: `K`, `dist`, `R`, `t`, `euler_xyz_deg` (for humans), `residual_rms_m`, `markers_used`, `frames` | never by hand |

## Steps: window mode (default)

1. Set the detectors to the board:

   ```yaml
   # modules/ips_detect/parameters/detector_params.yaml
   family: april
   marker_size: 0.185      # metres, = marker_size_mm / 1000 from modules/ips_calib/parameters/ips_calib.yaml
   ```

2. Place the board where every camera sees it, flat and still. `placement:` in `ips_calib.yaml` must
   describe where it lies: `origin_id`'s centre is the world origin, `position_mm` offsets it,
   `orientation_deg` is the board's XYZ Euler in the world frame. The board defines the world, so
   all cameras calibrated against one placement share one frame; do not move the board between
   cameras within a run.

3. Start the calibration flow:

   ```bash
   sb down && sb up calib        # or plain `sb up calib` when no session runs
   sb attach                      # windows ips_cam_gcam_pub-sbNN, ips_detect-sbNN, ips_calib
   ```

   `ips_calib` is a node of `calib.flow.yaml` only, so `sb run ips_calib` is refused while the
   default flow is active (`module ips_calib is not a node of flow default`), and `sb up` refuses a
   session that already exists, hence the `sb down`.

4. In the `ips_calib` window, one tile per camera with the live image, the detected tags labelled
   `[id]`, a counter and a status line, and a panel listing the keys valid now:

   | Key | Effect |
   |---|---|
   | S | start collecting; S again stops early. Keep the board still while the counters run (`142/300`). |
   | C | fit from what was collected; each tile shows its residual: green at or under `rms_warn_m`, amber above |
   | W | confirmation box listing each `sbNN_full.yaml` with old and new residual |
   | Y / N | write the files / cancel. Nothing touches disk before Y. |
   | R | back to idle, discard the collection |
   | Q, Esc | quit (asks when a fit would be lost); closing the window quits too |

   Wait until every tile shows the board before S. A tile that reads `skipped, kept` after C saw
   fewer than `min_markers` ids, got no frames, or its detector is on the wrong family; its existing
   file stays.

5. Check the result per camera: residual under 0.01 m is good, 0.02 m acceptable for a first pass,
   above that see "If the residual is high". Then press W, Y.

6. Return to the live pipeline:

   ```yaml
   # detector_params.yaml
   family: aruco
   marker_size: 0.087
   ```

   ```bash
   sb down && sb up               # ips_track loads calibration/extrinsics/ at startup
   ```

   Re-run `ips_calib` later while `calib` is still the active flow with `sb run ips_calib`, or in
   the current terminal with `sb run --fg ips_calib`.

## Steps: headless one-shot

For a machine without a display, or for scripted recalibration. Set `ui: false` in
`ips_calib.yaml`, then `sb up calib` (or `sb run ips_calib` while `calib` is active). The
calibrator accumulates until every camera has `frames` marker frames or `timeout_s` passes, fits,
writes every camera that qualifies, prints one result line per camera under `=== results ===`
(`<inst>: <reason>, skipped` for the others), and exits. Its tmux window keeps the output until
`sb stop ips_calib`. There is no confirmation: files are overwritten as soon as the fit succeeds,
so keep a copy of `calibration/extrinsics/` before a headless run on a working system. A build
without OpenCV (`cargo build --no-default-features`) has only this mode.

## Reading sbNN_full.yaml

```yaml
camera: sb00
world_frame: RDF
units: m
K: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
dist: [k1, k2, p1, p2]              # radtan, copied from the camchain
R: [[...], [...], [...]]            # world = R * cam + t; camera optical frame RDF (+x right, +y down, +z forward)
euler_xyz_deg: [...]                # the same rotation for humans; R is authoritative
t: [x, y, z]                        # the camera centre in the world frame, metres
residual_rms_m: 0.0387
markers_used: [0, 1, 2, 3, 4, 5]
frames: 116
```

Sanity checks: `t` is where the camera physically is relative to the board origin (a ceiling camera
2.7 m above a board at `position_mm: [0, 0, 100]` in an RDF world has a `t` whose magnitude is about
that distance), `markers_used` lists most board ids, `frames` is near `frames` in `ips_calib.yaml`.
`ips_track` reads only `R` and `t` from this file; `ips_viz` gets `K`, `R`, `t` through the tracker's
`cam_param` service and draws the camera axes, so a camera drawn in the wrong place in rerun is the
quickest visual check of an extrinsics file.

## If the residual is high

| Symptom | Cause and fix |
|---|---|
| All cameras amber, similar residual | `marker_size_mm` or `marker_gap_mm` wrong: re-measure; the detectors' `marker_size` must equal `marker_size_mm / 1000`. |
| One camera amber | Blur or glancing view: refocus, lower exposure to 5 ms, move the board nearer the image centre of that camera, re-collect. A wrong or stale camchain for that camera gives the same picture. |
| Residual fine, cubes drift between cameras in the tracker | Board moved during collection, or cameras calibrated against different placements. Redo all cameras in one run with the board still. |
| `skipped` with fewer ids than `min_markers` | Camera sees fewer than three board tags: move the board or lower `min_markers` only if the tags still span the view (three points is the minimum for a rigid fit, four or more is wanted). |

## Scaling to more or fewer cameras

The camera count is not hard-coded anywhere, but it is written in five places that must agree:

1. `modules/ips_cam_gcam_pub/parameters/cam_params.yaml`: `cameras: [sb00, ..., sbNN]`, and the
   physical cameras named accordingly (`gcam-rename`).
2. `sb.flow.yaml` and `calib.flow.yaml`: `instances:` of `ips_cam_gcam_pub` and `ips_detect`, the
   same list in both files. Then `sb install .` so the installed flows follow; `sb ws check` reports
   `ws flows` with the new instance count.
3. `modules/ips_detect/parameters/detector_params.yaml`: `cameras: [0, ..., N-1]` (indices).
4. `modules/ips_calib/parameters/ips_calib.yaml`: `cameras: [0, ..., N-1]`.
5. `calibration/intrinsics/sbNN-camchain.yaml` for each new camera (Kalibr).

The calibrator window lays the tiles out as `ceil(sqrt(N))` columns, so five to nine cameras give a
3x3 grid. `ips_track` tracks whatever cameras publish and serves every `*_full.yaml` it finds in
`calibration/extrinsics/`, so a removed camera's file should be deleted or it keeps appearing in
`ips_viz`. A larger room may need a larger board or several placements; with one board per run,
every camera in the run must see that board, so calibrate cameras in overlapping groups against the
same placement only if the board does not move between groups.

## If it fails

| Output | Cause and fix |
|---|---|
| `module ips_calib is not a node of flow default` | `sb down && sb up calib`. |
| `sb up` refuses, a session exists | `sb down` first. |
| Build: `couldn't find any valid shared libraries matching: ['libclang.so', ...]` | `bash rust-opencv-dependency.bash`; sb sets `LIBCLANG_PATH` from `sb.module.yaml` (default `/usr/lib/llvm-18/lib`); export it for a hand-run `cargo`. |
| Window never opens, OpenCV display error in the `ips_calib` tmux window | No `DISPLAY` in the shell that ran `sb up calib`. Run it from a desktop terminal, or `sb run --fg ips_calib` there, or `ui: false`. |
| Tiles show images but no `[id]` labels | Detectors still on `aruco`: set `family: april`, `marker_size`, then `sb run ips_detect`. Confirm with `sb topic listen /<device>/ws_ips/ips_detect-sb00/zenoh/markers -n 1`. |
| Tile black, counter stays 0 | That camera's publisher is retrying (name, held camera): `gcam-list`, the `ips_cam_gcam_pub-sbNN` window. |
| `<inst>: intrinsics load failed (...), skipped` at startup | Missing or malformed `calibration/intrinsics/sbNN-camchain.yaml`. |
| Panic `not yet implemented` at C | `fit_extrinsics` or `reprojection_residual` still `todo!()`; `cargo test` in `modules/ips_calib/`. |
| `ips_track` prints `cannot read extrinsics '...'` | File missing for a camera it tracks: calibrate that camera, or remove it from the lists above. |
| Labels sit a few pixels off the tags | Expected: labels are projected through `K` with distortion ignored. Not an error. |

## Safety

- Only W then Y (window) or a completed headless fit writes `calibration/extrinsics/`. Back the
  folder up before a headless run; the window shows old and new residual before overwriting.
- Switching the detectors to `april` stops marker tracking of the cubes; switch back and `sb down &&
  sb up` after calibrating. The tracker loads extrinsics at startup only.
- Do not edit `sbNN_full.yaml` by hand and do not hand-edit `ips_calib.yaml` placement between
  cameras of one run.
- Deeper reference: `docs/03_calib.md`, `modules/ips_calib/README.md`, `calibration/README.md`;
  flows in `~/.swarmbotix/documents/sbcli_flow.md` §7 and §9 (`sbcli` skill).
