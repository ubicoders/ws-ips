# 00 Setup

Done before or during module 1.

1. Ubuntu 24 on a PC or laptop, no virtual machine, no WSL. Rust stable, tmux.
2. The sb CLI (0.2.x) installed and `sb doctor` passing.
3. The camera SDK: `bash drivers/gcam-mvsdk-rs/install.bash` after the clone below. It installs the `gcam-mvsdk-rs` command and downloads the vendor library into `~/.gcam-mvsdk/`. Nothing from the vendor is committed to this repository.
4. Clone with submodules, set your device name, run the setup script:

   ```bash
   git clone --recurse-submodules https://github.com/ubicoders/ws-ips
   cd ws-ips
   sb config set device <yourname>
   bash workspace/setup.sh
   ```

   The device name goes first because the generated code bakes it in. The script creates and activates the sb workspace `ws_ips`, installs `workspace/flow.yaml` with this clone's path, writes each module's `sb.dev.yml`, copies `messages/custom/*.proto` into your sb message vault and compiles them, and generates each module's `swarmbotix_io/`. Run it again after changing a message or an `sb.dev.yml.in`.
5. `sb up` starts every module in `flow.yaml`, one tmux window each, in the session `sb-ws_ips`. With no cameras yet, the detectors wait and `ips_track` publishes empty heartbeats; that is expected.

Check from a second machine on the LAN, through the zenoh router:

```bash
sb topic list
```
