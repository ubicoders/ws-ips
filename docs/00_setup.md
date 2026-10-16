# 00 Setup

Done before or during module 1.

1. Ubuntu 24 on a PC or laptop, no virtual machine, no WSL. Rust stable, then `bash rust-opencv-dependency.bash` at the repository root for OpenCV and libclang.
2. The sb CLI (0.8.0 or later) installed and `sb doctor` passing.
3. The camera SDK: `bash drivers/gcam-mvsdk-rs/install.bash` after the clone below. It installs the `gcam-mvsdk-rs` command and downloads the vendor library into `~/.gcam-mvsdk/`. Nothing from the vendor is committed to this repository.
4. Clone with submodules, set your device name, install the workspace:

   ```bash
   git clone --recurse-submodules https://github.com/ubicoders/ws-ips
   cd ws-ips
   sb config set device <yourname>
   sb install .
   ```

   The device name goes first because the generated code bakes it in. `sb install .` reads the committed `sb.workspace.yaml` and the three flow files beside it, `sb.flow.yaml` (the flow `default`), `calib.flow.yaml` (the flow `calib`) and `relay.flow.yaml` (the flow `relay`): it creates and activates the sb workspace `ws_ips`, installs the three flows (`-- flows: default, calib, relay`), imports `messages/custom/*.proto` into your sb message vault and compiles them, generates each module's `swarmbotix_io/` from its `sb.module.yaml`, and renders the tracker's service router `swarmbotix_io/services/cam_param.rs` when it is missing. Run it again after pulling, after editing `sb.workspace.yaml`, a flow file, an `sb.module.yaml` or a `.proto`, and after moving the clone. An existing service router is never overwritten, so after changing the device name also run `sb io update --services ips_track` to re-render it. `sb ws check` reports anything out of date.
5. `sb up` starts the default flow, `sb.flow.yaml`: it builds the four live modules (`cargo build --release`), then starts each in the session `sb-ws_ips`, one tmux window per instance: four camera windows, four detector windows, `ips_track` and `ips_viz`. `ips_calib` is not a node of the default flow; it runs later in the calibration flow (`sb up calib`, see [03_calib.md](03_calib.md)). Nor is the optional `ips_image_relay`, which republishes half-size images for network peers: the relay flow, `sb up relay`, runs the same pipeline with it (see [06_relay.md](06_relay.md)). Building `ips_detect` needs libclang for its opencv crate, and so does building `ips_image_relay`: sb sets `LIBCLANG_PATH` to `/usr/lib/llvm-18/lib` from each module's `sb.module.yaml`; if your LLVM lives elsewhere, export `LIBCLANG_PATH` before `sb up`. With no cameras yet, the detectors wait and `ips_track` publishes empty heartbeats; that is expected.

Check from a second machine on the LAN, through the zenoh router:

```bash
sb topic list
```
