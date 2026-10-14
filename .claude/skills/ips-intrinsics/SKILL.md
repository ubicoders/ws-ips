---
name: ips-intrinsics
description: Produce the per-camera intrinsics file calibration/intrinsics/sbNN-camchain.yaml for any number of ws-ips cameras. Collects AprilGrid images from one camera with the gcam driver, runs the standalone sb app camcalib (Kalibr in Docker, installed with sb install https://github.com/swarmbotix/sb-camcalib) on that image folder, checks the resulting camchain.yaml, and copies it under the right name into calibration/intrinsics/ where ips_detect and ips_calib read it. Use when the user wants to calibrate camera intrinsics, add a camera's camchain, redo intrinsics after a lens or focus change, install or fix sb camcalib, or asks why ips_detect reports a missing or malformed camchain. Extrinsics are the separate ips-extrinsics skill.
---

# ips-intrinsics

One camchain per camera, named after the camera, in one folder:

```
gcam live view / grab  ->  <images>/0.png ...  ->  sb camcalib <images>  ->  <images>/calib/cam0/camchain.yaml
                                                                                    |  copy and rename
                                                       calibration/intrinsics/sbNN-camchain.yaml
                                                                                    |
                                                  ips_detect (K, dist for pose_from_corners), ips_calib (K copied into sbNN_full.yaml)
```

`camcalib` is a standalone tool: it knows nothing about ws-ips, works on any image folder, and writes
its results next to the images. This skill adds the ws-ips conventions around it: which images to
take, where the result goes, what it must contain.

## What ws-ips needs

`modules/ips_detect/src/intrinsics.rs` and `ips_calib` read `calibration/intrinsics/<name>-camchain.yaml`
and take from it:

| Field | Requirement |
|---|---|
| `intrinsics: [fx, fy, cx, cy]` | exactly 4 values, pinhole model |
| `distortion_coeffs: [k1, k2, p1, p2]` | at least 4 values, radtan model |
| `resolution: [1280, 1024]` | informational, but the images must be the camera's full 1280x1024 preset or K is wrong for the pipeline |

So the Kalibr model is `pinhole-radtan`, camcalib's default. Do not pass another `--models`. The
top-level key stays `cam0:` in every file (Kalibr's name for the first camera of a run); the camera
identity is the file name only: `sb00-camchain.yaml`, `sb01-camchain.yaml`, and so on, matching
`cameras:` in `modules/ips_cam_gcam_pub/parameters/cam_params.yaml`.

## Step 0: camcalib installed?

```bash
sb camcalib version          # prints image and Kalibr info; pulls swarmbotix/sb_kalibr:latest on first use (about 7 GB, once)
sb app list                  # camcalib listed, kind docker
sb doctor                    # [OK] app camcalib: kind docker, path ok, requires docker ok
```

Install when `sb camcalib` is unknown:

```bash
sb install https://github.com/swarmbotix/sb-camcalib     # registers the app; run as `sb camcalib ...` from anywhere
sb app update camcalib                                    # later, pull a newer version
```

Needs Docker on a Linux host; the user must be able to run `docker` without sudo. The container runs
with the caller's uid, so output files belong to the user.

Known packaging fault: the app's entry script may be committed without the executable bit, and
`sb camcalib` then stops with `app 'camcalib': .../camcalib is not executable; run chmod +x ...`.
Run the `chmod +x` it names (path under `~/.swarmbotix/apps/camcalib/`) and retry; the fix belongs
upstream in the sb-camcalib repository.

## Step 1: the target

Default target is the Kalibr AprilGrid 6x10 in `~/.swarmbotix/apps/camcalib/april_6x10_40mm.yaml`
(`tagSize: 0.0518`, `tagSpacing: 0.30038`; printable `targets/calib.io_kalibr_600x400_6x10_40.pdf`).
Measure the printed tags with a caliper. If the size or the gap differs, put a `target.yaml` into the
image folder with the measured values (`tagSpacing` is gap divided by tag size); it overrides the
default for that folder. A plain AprilTag or ArUco detector does not see this grid, because the tags
sit inside a checkerboard; only Kalibr's detector does.

## Step 2: collect images of one camera

One camera at a time, with the pipeline stopped (`sb stop ips_cam_gcam_pub`). Use the live view
recording from `gcam-collect`: it shows the image while the grid is moved and saves at a fixed rate.

```bash
cp drivers/gcam-mvsdk-rs/examples/record.yaml /tmp/record_intr.yaml
# edit: rate_hz: 2   duration_s: 40   data_dir: /abs/path/to/ws-ips/docs/data/intrinsics
cd drivers/gcam-mvsdk-rs
cargo run --release --features view --example live_view -- sb00 --exposure-us 5000 --mtu 1500 --config /tmp/record_intr.yaml
```

Press `r`, move the grid for the duration, `q`. The recording lands in
`docs/data/intrinsics/<YYYYMMDD_HHMMSS>_sb00/0.png ...`: a flat folder of Mono8 1280x1024 PNG files
with integer names, which camcalib accepts directly as a single camera (`cam0`).

What the images must cover (Kalibr's own rules, `docs/capture-and-targets.md` in the app):

- 30 to 60 usable frames; more adds time, not accuracy. `--step` and `--max-frames` thin a long
  recording afterwards.
- The grid over the whole image, including every corner and edge, at several distances.
- Tilt of 30 to 45 degrees about both axes, and some roll. Flat frontal views alone give a poor
  distortion estimate.
- Sharp: the grid still at each pose, 5 ms exposure, enough light. Blurred corners are discarded
  silently and the fit degrades.
- A ceiling camera is calibrated in place: hold the grid up to it on a pole, or calibrate on the
  bench before mounting with the focus ring then taped. Refocusing afterwards invalidates the file.

Headless alternative with stills: `gcam-mvsdk-rs grab sb00 -n 1 -o <folder> --exposure-us 5000`
repeated per pose; `grab` names files `sb00_0000.png`, whose stems are not integers, so rename them
to `0000.png ...` before camcalib (`camcalib inspect` reports the problem).

## Step 3: run camcalib

```bash
sb camcalib inspect docs/data/intrinsics/<stamp>_sb00                 # checks the folder, no calibration
sb camcalib docs/data/intrinsics/<stamp>_sb00                         # mono run, results in <folder>/calib/
sb camcalib docs/data/intrinsics/<stamp>_sb00 --step 2 --max-frames 60   # thin a long recording
sb camcalib docs/data/intrinsics/<stamp>_sb00 --focal 870             # if the result is NaN: manual focal init (px)
```

Output:

```
<folder>/calib/
  cam0/camchain.yaml      intrinsics and distortion, the file ws-ips needs
  cam0/results-cam.txt    reprojection error statistics
  cam0/report-cam.pdf     plots: coverage, residuals per frame
  manifest.json log.txt
```

Read `results-cam.txt`: a reprojection error of a few tenths of a pixel is good, above about one
pixel means blur, a wrong target yaml, or too little coverage. Open `report-cam.pdf` and check that
the detected corners cover the image.

Run each camera separately, as its own mono folder. Do not put several ceiling cameras into one
folder as `cam0/ cam1/ ...`: that is Kalibr's rig mode, which assumes synchronised frames of the
same grid and solves camera-to-camera extrinsics the IPS does not use (the extrinsics come from the
board and `ips-extrinsics`).

## Step 4: install the file

```bash
cp docs/data/intrinsics/<stamp>_sb00/calib/cam0/camchain.yaml calibration/intrinsics/sb00-camchain.yaml
grep -E "intrinsics|distortion_coeffs|distortion_model|resolution" calibration/intrinsics/sb00-camchain.yaml
```

Expected: `distortion_model: radtan`, `intrinsics:` with 4 values, `distortion_coeffs:` with 4
values, `resolution: [1280, 1024]`. The name is `<camera name>-camchain.yaml`, the camera name being
the one stored in the camera and listed in `cam_params.yaml`. Keep the `cam0:` key as Kalibr wrote
it. Then:

- `sb run ips_detect:sb00` reads the new file at startup; its window prints the loaded intrinsics
  or `'...sb00-camchain.yaml': no 'intrinsics: [...]' line` for a malformed one.
- `ips_calib` copies `K` and `dist` from it into `sbNN_full.yaml`, so a new camchain means redoing
  that camera's extrinsics (`ips-extrinsics`).

## Scaling to N cameras

Repeat steps 2 to 4 per camera; the result is one `sbNN-camchain.yaml` for every entry of
`cameras:` in `cam_params.yaml` (two cameras: `sb00`, `sb01`; eight: `sb00` to `sb07`). The
camchain folder is read by name, so no list anywhere has to change for intrinsics alone; the lists
that do change when a camera is added are in `ips-extrinsics`, section "Scaling". A loop over several
recordings:

```bash
for d in docs/data/intrinsics/*_sb0[0-3]; do
  cam=${d##*_}                                   # sb00 .. sb03 from the folder name
  sb camcalib "$d" && cp "$d/calib/cam0/camchain.yaml" "calibration/intrinsics/$cam-camchain.yaml"
done
```

Cameras of one model and lens still get their own file each; focus and assembly differ per unit.

## If it fails

| Output | Cause and fix |
|---|---|
| `` error: `camcalib` is not an sb command or an installed app. `` | Step 0: `sb install https://github.com/swarmbotix/sb-camcalib`. |
| `app 'camcalib': ... is not executable; run chmod +x ...` | Run the named `chmod +x`, retry. |
| Docker `permission denied` on the socket | Add the user to the `docker` group and log in again, or the pull fails. |
| First run hangs on `Pulling` | The 7 GB image download; wait, or `docker pull swarmbotix/sb_kalibr:latest` in another terminal to watch progress. |
| `inspect`: file stems must be integers | `grab` names (`sb00_0000.png`): rename to `0000.png`. |
| Result `nan` or an absurd focal length | `--focal <px>` with a guess (about 870 px for the IPS lenses at 1280x1024), more tilted views. |
| Few corners detected, high reprojection error | Blur, grid too far, wrong `target.yaml` (measure the print), grid not the Kalibr AprilGrid. |
| `ips_detect`: `intrinsics has N values, expected 4` | A non-pinhole model was used (`--models`); redo with the default `pinhole-radtan`. |
| `ips_calib`: `<inst>: intrinsics load failed (...), skipped` | File missing or misnamed for that camera; check `ls calibration/intrinsics/`. |
| Poses look right in one camera but wrong in the pipeline | Images taken at a non-default ROI or binning; recalibrate at full 1280x1024. |

## Safety

- `cp` into `calibration/intrinsics/` overwrites that camera's previous camchain, which the existing
  `sbNN_full.yaml` was built from. Keep the old file (`git diff` shows it; the repository tracks the
  folder) and redo the extrinsics for that camera.
- camcalib never modifies the input images; it only adds `calib/` next to them. `docs/data/` is the
  place for recordings, but PNG files there are not gitignored, so check `git status` before a
  commit.
- Deeper reference: `~/.swarmbotix/apps/camcalib/README.md` and `docs/` (input format, commands,
  capture and targets); the app's own skills `calib-mono`, `calib-rig`, `calib-imu` under its
  `.claude/skills/`; `calibration/README.md` here.
