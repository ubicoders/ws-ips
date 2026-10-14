---
name: gcam-list
description: Sweep the LAN for the HuaTeng GigE cameras of the ws-ips indoor positioning system and list them (user-set name sb00..sb03, serial, IP, host NIC) with gcam-mvsdk-rs list, then explain the output, flag a camera on the wrong subnet, and troubleshoot an empty list with gcam-mvsdk-rs doctor. Use when the user asks which cameras are on the network, whether sb01 is there, what the serial or IP of a camera is, or before renaming, viewing or recording a camera. Rust driver is the default; the Python driver gcam-mvsdk-py gives the same commands.
---

# gcam-list

Find every camera of the IPS on the network and read its identity. The four ceiling cameras must
appear with the user-set names `sb00`, `sb01`, `sb02`, `sb03`, because `ips_cam_gcam_pub` selects
a camera by that name (`modules/ips_cam_gcam_pub/parameters/cam_params.yaml`).

## When to use

- "Which cameras are on the network?", "Is sb02 up?", "What is the serial of sb00?"
- Before `gcam-rename`, `gcam-live-view` or `gcam-collect`, to pick the camera.
- After cabling, after a power cycle, or after `sb up` shows an `ips_cam_gcam_pub-sbNN` window
  that keeps retrying.

## Prerequisites

- Driver installed (setup step 3): `bash drivers/gcam-mvsdk-rs/install.bash`. The command
  `gcam-mvsdk-rs` lands in `~/.cargo/bin`; the vendor library lands in `~/.gcam-mvsdk/`.
- No privileges. GigE discovery is a UDP broadcast on port 3956; `sudo` changes nothing.

## Steps

1. Check the host first. Fix every finding before trusting an empty list:

   ```bash
   gcam-mvsdk-rs doctor
   ```

   It reports the SDK path, `rp_filter`, the firewall, each NIC with its subnet, and the
   receive buffer size. The IPS needs `rp_filter` 0 on the camera NIC, UDP allowed from the camera
   subnet, one subnet for the PC and all cameras, and `net.core.rmem_max` of 32 MB (below that the
   cameras throttle to about 60 Hz).

2. Sweep the network:

   ```bash
   gcam-mvsdk-rs list              # one block per camera; opens each camera briefly to read its name
   gcam-mvsdk-rs list --no-open    # while ips_cam_gcam_pub is streaming: enumerate only, no names
   gcam-mvsdk-rs list --json       # for scripts
   ```

   Use `--no-open` whenever `sb up` or `sb run ips_cam_gcam_pub` is running, otherwise the name
   line reads `<could not open: ...>` for every streaming camera.

3. Check the four names are present exactly once each:

   ```bash
   gcam-mvsdk-rs list --json | python3 -c 'import json,sys; print(sorted(str(c["user_name"]) for c in json.load(sys.stdin)))'
   ```

   Expected: `['sb00', 'sb01', 'sb02', 'sb03']`. A `None`, an `<empty>`, a duplicate or a stray
   name means `gcam-rename`.

4. Select one camera afterwards by name or serial:

   ```bash
   gcam-mvsdk-rs info sb00
   ```

## Reading the output

| Line | Meaning for the IPS |
|---|---|
| `User-set name` | Must be `sb00`..`sb03`. `<empty>`: never named. `<could not open: ...>`: held by a publisher or another host. |
| `Serial number` | Factory serial; the only stable key for `rename`. Write it down per ceiling position. |
| `Camera` | Camera IP, mask, gateway, MAC. The MAC is the key for `setip`. |
| `Host NIC` | The PC's card that heard the camera. All four should answer on the same NIC. |
| `WARNING : camera is not in the host NIC subnet; use setip` | Discovery works but streaming will not. Move the camera: `gcam-mvsdk-rs setip <MAC> <IP> <MASK> 0.0.0.0` with a free IP in the NIC's subnet, then `list` again (some units need a power cycle). Confirm MAC and address with the user first. |

In JSON, the same warning is `gige.same_subnet: false`.

## If it fails

| Output | Next step |
|---|---|
| `No cameras found.` | Work through `doctor` in order: `rp_filter` (`sudo sysctl -w net.ipv4.conf.all.rp_filter=0 net.ipv4.conf.<nic>.rp_filter=0`), firewall (allow UDP from the camera subnet), NIC subnet (static address in the camera subnet), then power and cabling. A probe independent of the SDK: `python3 -I drivers/gcam-mvsdk-rs/.claude/skills/camera-discovery/gvcp_probe.py`. |
| Three cameras instead of four | The missing one: check its link light, its switch port, and whether it sits on another subnet (a 169.254.x.x address answers only with `rp_filter` 0). |
| `could not find the HuaTeng MVSDK library` | Re-run `bash drivers/gcam-mvsdk-rs/install.bash`; see `drivers/gcam-mvsdk-rs/.claude/skills/sdk-setup`. |
| `<could not open: ...>` with status `-13`, `32774` or `-45` | Another program holds the camera. `sb stop ips_cam_gcam_pub`, or use `--no-open`. Wait a few seconds for the heartbeat to expire. |

Report what was observed (commands, counts, warnings). Never claim a camera exists without a
`list` or probe result.

## Python driver

Same commands, same output, with `gcam-mvsdk-py` in place of `gcam-mvsdk-rs`. Installer:
`bash drivers/gcam-mvsdk-py/install.bash` (environment `~/.gcam-mvsdk/py-venv`, same SDK home, so
one SDK download serves both drivers).

```bash
gcam-mvsdk-py doctor
gcam-mvsdk-py list
gcam-mvsdk-py list --no-open
gcam-mvsdk-py list --json
```

From Python:

```python
import gcam_mvsdk_py
sdk = gcam_mvsdk_py.load()
for dev in sdk.enumerate():
    print(dev.serial, dev.friendly_name)
```

## Safety

`list` only reads. Without `--no-open` it opens each camera for a moment, which fails harmlessly
on a camera a publisher is streaming from. Prefer `--no-open` while the pipeline runs.

Deeper reference: `drivers/gcam-mvsdk-rs/.claude/skills/camera-list`, `camera-discovery`,
`camera-setip` (and the same names under `drivers/gcam-mvsdk-py/.claude/skills/`).
