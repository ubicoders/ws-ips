# 00 Setup

Done before or during module 1.

1. Ubuntu 24 on a PC or laptop, no virtual machine, no WSL. Docker installed.
2. The sb CLI installed and `sb doctor` passing.
3. Clone with submodules and run the setup script:

   ```bash
   git clone --recurse-submodules https://github.com/ubicoders/ws-ips
   cd ws-ips
   bash workspace/setup.sh
   ```

   This creates and activates the sb workspace `ws_ips`, installs `workspace/flow.yaml` with this clone's path, imports `messages/custom/*.proto` into your sb message vault, and pulls `drivers/`.
4. Set your device name once: `sb config` (it becomes the first segment of every wire key, `<device>/ws_ips/...`).
5. `sb up` starts every module in `flow.yaml` in tmux. With no cameras yet, only `ips_viz` will have something to say; that is expected.

Check from a second machine on the LAN, through the zenoh router:

```bash
sb topic list
```
