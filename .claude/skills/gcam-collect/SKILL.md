---
name: gcam-collect
description: Collect images from ws-ips cameras as PNG files, in one of three ways chosen by purpose, with gcam-mvsdk-rs grab for a handful of stills, the live_view recording (r starts, c stops, rate_hz and duration_s in examples/record.yaml) for a timed sequence while watching, or the ips_detect save option for frames captured inside the running pipeline from all four cameras at once. Use when the user wants test frames, a dataset of marker or calibration-board images, frames for a detector or intrinsics test, or recorded captures for docs/data. Rust driver is the default; gcam-mvsdk-py gives the same commands.
---

# gcam-collect

Three collectors. Choose by what the images are for:

| Purpose | Collector | Output |
|---|---|---|
| A few stills: exposure check, focus check, one marker image | A. `grab` | `<dir>/sbNN_0000.png`, ... |
| A timed sequence from one camera while watching it (board sweep, moving cube) | B. live view recording | `~/.gcam-mvsdk/data/<YYYYMMDD_HHMMSS>_sbNN/0.png`, ... |
| Frames from all four cameras exactly as the detectors see them, pipeline running | C. `ips_detect` `save:` | `<save>/sbNN/<seq>.png` |

All three write 8-bit grayscale PNG (Mono8, 1280x1024), the same pixels the detector consumes.
Use exposure 5000 us so collected frames match live conditions.

## Prerequisites

- `gcam-list` shows the cameras by name.
- A and B need the camera free: `sb stop ips_cam_gcam_pub` first. C needs the opposite, the
  pipeline running.
- Enough disk: one frame is 1.3 MB raw, about 0.5 to 1 MB as PNG. Four cameras at 90 Hz for one
  minute is about 20 000 files; use `save_frames` or `rate_hz` to cap it.

## A. Stills with grab

```bash
sb stop ips_cam_gcam_pub
gcam-mvsdk-rs grab sb00 -n 5 -o docs/data/sb00_check --exposure-us 5000 --mtu 1500
ls -l docs/data/sb00_check
```

Options: `-n/--count N` (default 1), `-o/--out DIR` (default `.`, created if missing),
`--exposure-us US`, `--timeout-ms MS` (default 2000), `--mtu BYTES`. Files are named
`<user-set name>_NNNN.png`. Each saved line prints `mean` (exposure check: near 255 saturated, near
0 dark) and `ts`, the camera stamp, whose differences give the real frame interval.

Loop over all four cameras:

```bash
for c in sb00 sb01 sb02 sb03; do gcam-mvsdk-rs grab $c -n 3 -o docs/data/stills/$c --exposure-us 5000 --mtu 1500; done
```

## B. Timed sequence from the live view

Set the rate and duration first. The file is `drivers/gcam-mvsdk-rs/examples/record.yaml`; copy it
instead of editing the submodule:

```bash
cp drivers/gcam-mvsdk-rs/examples/record.yaml /tmp/record.yaml
```

```yaml
duration_s: 30   # a recording stops after this many seconds
rate_hz: 5       # images saved per second; the camera keeps running at 90 fps
data_dir: /abs/path/to/ws-ips/docs/data   # default ~/.gcam-mvsdk/data
```

Only these three keys are accepted; a missing file means the defaults above. Then:

```bash
sb stop ips_cam_gcam_pub
cd drivers/gcam-mvsdk-rs
cargo run --release --features view --example live_view -- sb00 --exposure-us 5000 --mtu 1500 --config /tmp/record.yaml
```

Press `r` to start, move the board or cube through the field of view, press `c` to stop early or
wait for `duration_s`. The terminal prints `recording to <dir>` and `recording stopped: N images in
<dir>`. The overlay shows `REC 23 saved  4.6 s` while recording. One image is saved per
`1/rate_hz` seconds anchored to the key press, so 30 s at 5 Hz gives about 150 files, named
`0.png`, `1.png`, ... under `<data_dir>/<YYYYMMDD_HHMMSS>_sb00/`. Every recording gets a new
folder; nothing is overwritten. `q` quits and finishes a running recording first.

Rates for the workshop: calibration board sweep 2 to 5 Hz (distinct poses, not near-duplicates);
a moving cube for tracker tests 10 to 30 Hz.

## C. Frames from inside the pipeline

Captures from all four cameras with the publishers' own settings, taken by the detectors. Edit
`modules/ips_detect/parameters/detector_params.yaml`:

```yaml
save: "../../docs/data/capture01"   # empty string is off; a relative path is resolved from modules/ips_detect/
save_frames: 300                     # per detector; 0 is unlimited
```

```bash
sb run ips_detect                 # restarts the four detectors; frames land in <save>/sb00/<seq>.png .. sb03/
ls docs/data/capture01/sb00 | wc -l
```

Set `save: ""` back and `sb run ips_detect` again when done, or every later run writes files. The
detectors also save with `family: april` during a calibration flow (`sb up calib`), which is the
way to collect board images from all cameras for `ips_calib` tests. Frames are saved raw, without
annotations, and each file name is the frame's `seq`, the same number carried by the published
detection on `.../ips_detect-sbNN/zenoh/markers`, so images and detections line up exactly.

## Where the files go

`docs/data/` is the repository's place for recorded captures (`docs/data/.gitkeep` exists; the
module tests under `modules/*/tests/` are written to pick up recordings there once present).
Keep a short `README` or the exposure and board size in the folder name; large datasets stay out of
git (`*.png` under `docs/data/` is not ignored by default, so check `git status` before a commit).

## If it fails

| Output | Cause and fix |
|---|---|
| `timeout waiting for a frame` ... `No frames received.` | Packet size above the NIC MTU: `--mtu 1500`. Then firewall and `rp_filter` (`gcam-mvsdk-rs doctor`). |
| Status `-13`, `32774` or `-45` | A publisher holds the camera (A and B need it free): `sb stop ips_cam_gcam_pub`. |
| `Error: examples/record.yaml: unknown field ...` or `rate_hz must be a positive number` | Only `duration_s`, `rate_hz`, `data_dir`; positive numbers. |
| `Error: creating <dir>: Permission denied` | `data_dir` or `-o` not writable; pick a folder under the repository or your home. |
| Detectors run but `<save>` stays empty | Publishers not running (`sb topic list` shows no image keys), or the folder landed elsewhere: a relative `save:` is resolved from `modules/ips_detect/`, not from the shell. Check the `ips_detect-sb00` window under `sb attach`; it prints the save directory at startup. |
| Images all black or all white | Exposure: `grab --exposure-us` to try values, keep 5000 for the dataset and fix the lighting instead. |

## Python driver

```bash
sb stop ips_cam_gcam_pub
gcam-mvsdk-py grab sb00 -n 5 -o docs/data/sb00_check --exposure-us 5000       # no --mtu option; -o is --output-dir
cd drivers/gcam-mvsdk-py
~/.gcam-mvsdk/py-venv/bin/python3 examples/live_view.py sb00 --exposure-us 5000 --mtu 1500 --config /tmp/record.yaml
```

Same `record.yaml` keys, same key bindings, same folder layout. Viewer extra:
`bash drivers/gcam-mvsdk-py/install.bash --with-view`. Section C is driver-independent.

## Safety

A and B set Mono8, continuous trigger, manual exposure and packet size on the camera for the
session only; `ips_cam_gcam_pub` restores its values on restart. Nothing is deleted or overwritten
by any collector; `grab` overwrites only its own `sbNN_NNNN.png` names in the same `-o` folder.
Remember to turn `save:` off in `detector_params.yaml`.

Deeper reference: `drivers/gcam-mvsdk-rs/.claude/skills/camera-grab`, `run-live-view`.
