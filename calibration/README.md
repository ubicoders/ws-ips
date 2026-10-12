# calibration

Data shared by several modules, kept outside all of them.

| Path | What | Written by | Read by |
|---|---|---|---|
| `intrinsics/sbNN-camchain.yaml` | Kalibr output per camera: `K`, distortion model and coefficients | Kalibr (module 2 take-home) | `ips_detect`, `ips_calib`, `ips_track` |
| `ips_calib.yaml` | the IPS calib board: AprilTag 36h11 ids, tag size, gap, layout, and where the board sits in the world frame | the instructor, measured once | `ips_calib` |
| `extrinsics/sbNN_full.yaml` | per camera `K`, distortion, and `R`, `t` in the world frame (`world = R * cam + t`, camera centre at `t`) | `ips_calib` | `ips_track`, and `ips_viz` through the cam_param service |

Frames: camera optical frame is RDF (+x right, +y down, +z forward), metres. The world frame is defined by the calib board placement in `ips_calib.yaml`.
