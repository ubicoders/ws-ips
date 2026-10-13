# 00 Setup

Done before or during module 1.

1. Ubuntu 24 on a PC or laptop, no virtual machine, no WSL. Rust stable, tmux.
2. The sb CLI (0.5.x) installed and `sb doctor` passing.
3. The camera SDK: `bash drivers/gcam-mvsdk-rs/install.bash` after the clone below. It installs the `gcam-mvsdk-rs` command and downloads the vendor library into `~/.gcam-mvsdk/`. Nothing from the vendor is committed to this repository.
4. Clone with submodules, set your device name, run the setup script:

   ```bash
   git clone --recurse-submodules https://github.com/ubicoders/ws-ips
   cd ws-ips
   sb config set device <yourname>
   sb install .
   ```

   The device name goes first because the generated code bakes it in. `sb install .` reads the committed `flow.yaml`: it creates and activates the sb workspace `ws_ips`, imports `messages/custom/*.proto` into your sb message vault and compiles them, and generates each module's `swarmbotix_io/` from its `sb.dev.yml`. Run it again after pulling, after editing `flow.yaml`, an `sb.dev.yml` or a `.proto`, and after moving the clone. After changing the device name, also run `sb io update --services ips_track`: sb leaves service routers alone on install, and the tracker's run script only renders a missing one. `sb ws check` reports anything out of date.
5. `sb up` starts every module in `flow.yaml`, one tmux window each, in the session `sb-ws_ips`. With no cameras yet, the detectors wait and `ips_track` publishes empty heartbeats; that is expected.

Check from a second machine on the LAN, through the zenoh router:

```bash
sb topic list
```
