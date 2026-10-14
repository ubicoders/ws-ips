---
name: gcam-rename
description: Give one HuaTeng camera of the ws-ips system its IPS name (sb00, sb01, sb02 or sb03) with gcam-mvsdk-rs rename, selected by serial number, verify it, power-cycle the camera, and keep cam_params.yaml and the flow files consistent. Use when the user asks to name, rename or relabel a camera, when gcam-list shows an empty, duplicate or stray name, or when a replacement camera is installed. Writes the camera's non-volatile memory, so only on the serial the user confirmed. Rust driver is the default; gcam-mvsdk-py gives the same command.
---

# gcam-rename

The user-set name (GigE `DeviceUserID`) stored inside each camera is the instance id of the whole
pipeline: `ips_cam_gcam_pub --id sb00` opens the camera that reports `sb00`, and the wire key
becomes `<device>/ws_ips/ips_cam_gcam_pub-sb00/iox2/image`. The names are fixed by
`modules/ips_cam_gcam_pub/parameters/cam_params.yaml` (`cameras: [sb00, sb01, sb02, sb03]`) and by
the `instances:` lists in `sb.flow.yaml` and `calib.flow.yaml`. Rename cameras to match those
files; do not edit the files to match a camera.

## When to use

- A new or replacement camera arrives with an empty name.
- Two cameras carry the same name, or a camera carries a name outside `sb00`..`sb03`.
- Cameras were moved between ceiling positions and the names should follow the positions.

## Prerequisites

- `gcam-list` shows the camera with its serial number.
- Nothing holds the camera: `sb stop ips_cam_gcam_pub` (or `sb down`) first. Rename fails with
  status `-13`, `32774` or `-45` while a publisher streams.

## Steps

1. Read the current state and collect the serial numbers:

   ```bash
   gcam-mvsdk-rs list
   ```

2. Decide the mapping serial to name from the ceiling position (index 0 is `sb00`, the first
   entry of `cam_params.yaml`; other modules refer to a camera by that index). Check the target name
   is not already in use by another camera in the `list` output.

3. Confirm with the user, quoting the exact command: "Rename camera SN `<SN>` (currently
   `<old name>`) to `sb01`?" Proceed only after an explicit yes. Never infer the serial from a
   position in the list.

4. Write and verify:

   ```bash
   gcam-mvsdk-rs rename 056090423003 sb01
   gcam-mvsdk-rs list                   # User-set name line shows sb01
   gcam-mvsdk-rs info sb01              # selection by name works
   ```

5. Power-cycle the camera (unplug the PoE cable or switch the port off and on). The name is in
   non-volatile memory at once, but discovery reports the new name only after a restart, and
   `ips_cam_gcam_pub` selects by the discovered name.

6. Repeat for the other cameras, then check all four at once:

   ```bash
   gcam-mvsdk-rs list --json | python3 -c 'import json,sys; print(sorted(str(c["user_name"]) for c in json.load(sys.stdin)))'
   ```

   Expected `['sb00', 'sb01', 'sb02', 'sb03']`.

7. Start the publisher for the renamed camera and confirm the image key appears:

   ```bash
   sb run ips_cam_gcam_pub:sb01
   sb topic list                        # contains <device>/ws_ips/ips_cam_gcam_pub-sb01/iox2/image
   ```

Library equivalent (Rust):

```rust
let cam = sdk.open(&sdk.find_device("056090423003")?)?;
println!("current name: {:?}", cam.user_name()?);
cam.set_user_name("sb01")?;
```

## Name rules

- At most 31 bytes, ASCII letters, digits, `-`, `_`. None of `\ / : * ? " < > |`.
- For the IPS only `sb00`, `sb01`, `sb02`, `sb03`: two-digit index, matching `cam_params.yaml`.
- Unique on the network. The tool does not enforce it; with a duplicate, selection by name returns
  the first match and one camera of the pipeline silently opens the wrong unit.

## Reading the output

```text
SN 056090423003: user-set name "" -> "sb01"
The name is stored in the camera's non-volatile memory and survives power cycles.
```

`warning: the camera reports ... after writing ...` means the read-back differs: power-cycle and
`list` again.

## If it fails

| Output | Cause and fix |
|---|---|
| `invalid argument: name ...` | Breaks a name rule. Nothing was written. |
| `no camera matches "<SN>"; available: ...` | Wrong serial, or a name was given. `rename` takes only serial numbers; copy it from `list`. |
| Status `-13`, `32774` (0x8006) or `-45` | Camera held by a publisher or another host. `sb stop ips_cam_gcam_pub`, `pgrep -a ips_cam_gcam_pub`, wait a few seconds, retry. |
| `list` still shows the old name | Camera not yet restarted. Power-cycle it. |
| `ips_cam_gcam_pub-sb01` window keeps retrying | Discovery still reports the old name, or two cameras carry `sb01`. `list`, fix the duplicate, power-cycle. |

## Python driver

```bash
gcam-mvsdk-py list
gcam-mvsdk-py rename 056090423003 sb01
gcam-mvsdk-py list
gcam-mvsdk-py info sb01
```

```python
import gcam_mvsdk_py
sdk = gcam_mvsdk_py.load()
with sdk.open("056090423003") as cam:
    print("current name:", cam.user_name)
    cam.user_name = "sb01"
```

## Safety

- Rename only the serial the user named explicitly and confirmed.
- Renaming changes which camera every module opens. Tell the user which ceiling position now
  publishes under which key, and that `calibration/extrinsics/sbNN_full.yaml` belongs to the
  position, not to the serial: a camera that changes name needs that position's extrinsics
  re-done with `sb up calib` if it also moved.

Deeper reference: `drivers/gcam-mvsdk-rs/.claude/skills/camera-rename`.
