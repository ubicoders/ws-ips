# ips_calib

Subscribes to every detector's marker stream while the cameras look at the IPS calib board, accumulates the marker centres, and solves each camera's pose in the world frame by a rigid least-squares (Kabsch) fit against the board geometry in `calibration/ips_calib.yaml`. Writes one `sbNN_full.yaml` per camera. Run once per camera placement, not part of the live pipeline.

## Inputs

- `<device>/ws_ips/ips_detect-*/zenoh/markers` with the detectors set to `family: april`, `marker_size: 0.185`.
- `calibration/ips_calib.yaml`, `calibration/intrinsics/sbNN-camchain.yaml`.

## Outputs

- `calibration/extrinsics/sbNN_full.yaml`: `K`, distortion, `R`, `t` (`world = R * cam + t`).
- The reprojection residual per camera, printed. Under a centimetre is good.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| markers (all cameras) | zenoh | `<device>/ws_ips/ips_detect-*/zenoh/markers` |

## State at handout, and what you build

Scaffold. The subscriber, the accumulation (median per marker id), and the yaml writer are in place. You fill, in module 2:

| Stub | Where | Test |
|---|---|---|
| `fit_extrinsics`: paired world and camera-frame marker centres to `R`, `t` (Kabsch / Umeyama, SVD) | `ips_common/src/fit.rs` | synthetic pairs with known transform, then the recorded board frames |
| `reprojection_residual`: RMS of the fitted centres against the measured ones | `ips_common/src/fit.rs` | same |

Each camera needs at least 3 marker ids in view or it is skipped.

## Run

```bash
sb run ips_calib            # this module alone, in its own tmux session
sb stop ips_calib
```

Parameters, when the module has them, live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml.
