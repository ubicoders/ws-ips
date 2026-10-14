---
name: gcam-live-view
description: Show the image of one ws-ips camera in real time, either directly from the driver with the live_view example of gcam-mvsdk-rs (OpenCV window, fps overlay, 5 ms exposure) or through the running pipeline with sb topic listen --show on the ips_cam_gcam_pub image key, and check focus, framing, exposure and frame rate. Use when the user wants to see a camera live, aim or focus a ceiling camera, confirm streaming works, or check that a publisher really delivers frames. Rust driver is the default; gcam-mvsdk-py has the same example.
---

# gcam-live-view

Two ways to watch a camera. Pick by what is running:

| Situation | Use |
|---|---|
| Pipeline stopped, aiming or focusing a camera on a ladder | Driver live view (section A). Lowest latency, fps overlay, can record (see `gcam-collect`). |
| `sb up` or `sb run ips_cam_gcam_pub` already running | Pipeline view (section B). Does not open the camera a second time. |

A camera can be held by one program at a time. Never start the driver live view on a camera while
its `ips_cam_gcam_pub-sbNN` window is running; stop it first with `sb stop ips_cam_gcam_pub`.

## Prerequisites

- `gcam-list` shows the camera, no subnet WARNING.
- A display: a desktop session with `DISPLAY` set (`ssh -X` over SSH).
- For section A: OpenCV and libclang on the build machine. The workshop setup installs them with
  `bash rust-opencv-dependency.bash` at the repository root (`libopencv-dev llvm-18 libclang-18-dev`).
  A hand-run `cargo` needs `LIBCLANG_PATH` exported, for example `export LIBCLANG_PATH=/usr/lib/llvm-18/lib`.

## A. Driver live view

```bash
sb stop ips_cam_gcam_pub                       # free the cameras if the pipeline runs
cd drivers/gcam-mvsdk-rs
cargo run --release --features view --example live_view -- sb00
```

Options after `--`: the camera name or serial (may be omitted with exactly one camera),
`--exposure-us US` (default 5000, the IPS value, automatic exposure off), `--mtu BYTES` (GigE
packet size, 1500 unless every switch passes jumbo frames; the IPS default in `cam_params.yaml` is
`packet_bytes: 1500`), `--config FILE` (recording options, default `examples/record.yaml`).

Keys in the window: `q` or Esc quits, `r` starts a recording, `c` stops it (recording is covered
in `gcam-collect`).

What to check while watching:

- **Frame rate.** The overlay shows displayed frames per second. The IPS cameras run at about
  90 fps on their 1280x1024 preset; a steady 60 means `rmem_max` is below 32 MB (`gcam-mvsdk-rs
  doctor` names the sysctl).
- **Exposure.** 5 ms is the IPS value: longer blurs marker corners under motion. If the image is
  dark at 5 ms, add light or gain; do not raise the exposure for the pipeline.
- **Focus and framing.** Marker corners must be sharp over the whole floor area the camera covers.
  Overlap between neighbouring cameras is needed for `ips_track`.
- **Resolution.** The window scales the image; the camera must stay on its full 1280x1024 preset,
  because `ips_cam_gcam_pub` refuses other sizes (intrinsics match only that size).

A still frame instead of a window (headless machine):

```bash
gcam-mvsdk-rs grab sb00 -n 1 -o /tmp/check --exposure-us 5000 --mtu 1500
```

The printed `mean` (0..255) tells exposure at a glance: near 255 saturated, near 0 dark.

## B. Pipeline view

With the publisher running, read the shared-memory image key. `<device>` is from
`sb config get device`.

```bash
sb run ips_cam_gcam_pub:sb00                   # or sb up for everything
sb topic list                                  # find <device>/ws_ips/ips_cam_gcam_pub-sb00/iox2/image
sb topic listen --transport iceoryx2 /<device>/ws_ips/ips_cam_gcam_pub-sb00/iox2/image --show --no-data
sb topic listen --transport iceoryx2 /<device>/ws_ips/ips_cam_gcam_pub-sb00/iox2/image -n 1 --no-data   # header only: stamp, frame_id, size
```

`--show` opens the image in a window and updates it with every message. The header `frame_id` is
the camera name and the stamp is the capture time; the difference between two stamps is the real
frame interval.

The detector's annotated view (markers drawn on the frame) is the same command on
`/<device>/ws_ips/ips_detect-sb00/iox2/debug_image` after setting `debug: true` in
`modules/ips_detect/parameters/detector_params.yaml` and `sb run ips_detect`. All four at once:
`debug_show_images: true` in `modules/ips_viz/parameters/viz_params.yaml`, then `rerun` and
`sb run ips_viz`.

## If it fails

| Output | Cause and fix |
|---|---|
| Build: `couldn't find any valid shared libraries matching: ['libclang.so', ...]` | `bash rust-opencv-dependency.bash`, then `export LIBCLANG_PATH=/usr/lib/llvm-18/lib`. |
| Build: `Failed to find installed OpenCV package using probes` | `libopencv-dev` missing; same script. |
| OpenCV error when the window opens | No display. Desktop session, `ssh -X`, or `xhost +local:` for a container. |
| `0 cameras found; pass a name or serial` | `gcam-list`. |
| `Error: DeviceNotFound { query: "sb00", ... }` | Camera not named yet, or not restarted after `rename`: `gcam-rename`. |
| Status `-13`, `32774` or `-45` | Publisher holds the camera: `sb stop ips_cam_gcam_pub`. |
| Repeated `no frame within 1 s (check --mtu, the firewall and rp_filter)` | Add `--mtu 1500`; then `gcam-mvsdk-rs doctor` for firewall and `rp_filter`. |
| `sb topic listen` shows nothing | `sb topic list` first; check the `ips_cam_gcam_pub-sb00` window under `sb attach` for a retry loop (camera held or misnamed). Shared-memory topics need `--transport iceoryx2`. |

## Python driver

```bash
sb stop ips_cam_gcam_pub
cd drivers/gcam-mvsdk-py
~/.gcam-mvsdk/py-venv/bin/python3 examples/live_view.py sb00 --exposure-us 5000 --mtu 1500
```

The viewer extra must be installed once: `bash drivers/gcam-mvsdk-py/install.bash --with-view`
(or `pip install -r examples/requirements.txt` in your own environment; `opencv-python` and
`opencv-python-headless` must not share one environment). Same keys, same `record.yaml`, one extra
option `--window TITLE`. Still frame: `gcam-mvsdk-py grab sb00 -n 1 -o /tmp/check` (no `--mtu`
option in the Python grab).

## Safety

The driver viewer sets Mono8 output, continuous trigger, manual exposure and the given packet size
for the session only. `ips_cam_gcam_pub` restores its own values from `cam_params.yaml` when it
restarts, so nothing persists on the camera. The pipeline view changes nothing at all.

Deeper reference: `drivers/gcam-mvsdk-rs/.claude/skills/run-live-view`, `camera-grab`.
