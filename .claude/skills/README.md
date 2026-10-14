# ws-ips Claude skills

Task guides for Claude Code sessions in this repository, one `SKILL.md` per folder. They cover the
IPS as a whole: camera names `sb00`..`sb03`, the sb modules, the wire keys. The driver submodules
carry their own, more general skills under `drivers/gcam-mvsdk-rs/.claude/skills/` and
`drivers/gcam-mvsdk-py/.claude/skills/`; each skill here points to the one it builds on.

The Rust driver `gcam-mvsdk-rs` is the default (it is what `ips_cam_gcam_pub` links). Every skill
ends with the equivalent `gcam-mvsdk-py` commands; the two CLIs share subcommands, output format
and the SDK home `~/.gcam-mvsdk/`.

| Skill | Task |
|---|---|
| `gcam-list` | sweep the network, list the cameras, read names, serials, IPs, troubleshoot an empty list |
| `gcam-rename` | name a camera `sbNN` by serial number, power-cycle, keep `cam_params.yaml` and the flows consistent |
| `gcam-live-view` | watch one camera live, from the driver or from the running pipeline |
| `gcam-collect` | save PNG frames: `grab` stills, timed live-view recording, or pipeline capture with `ips_detect` `save:` |
| `ips-intrinsics` | per-camera intrinsics: AprilGrid images from the driver, `sb camcalib` (Kalibr in Docker, sb app), copy `camchain.yaml` to `calibration/intrinsics/sbNN-camchain.yaml` |
| `ips-extrinsics` | extrinsic calibration: AprilTag board as world frame, detectors on `april`, `sb up calib`, the window keys S C W Y, headless mode, residuals, scaling to N cameras |
| `sbcli` | the swarmbotix CLI `sb`: check or install it from https://github.com/swarmbotix/sbcli, update it, route every sb question to `~/.swarmbotix/documents/sbcli_*.md`, ws-ips cheat sheet |

Order for a fresh setup: `sbcli` (install sb, `sb install .`), `gcam-list`, `gcam-rename` per camera, `gcam-live-view` to aim
and focus, `ips-intrinsics` per camera, `ips-extrinsics` once the cameras are mounted.
