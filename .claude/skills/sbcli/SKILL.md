---
name: sbcli
description: Use the swarmbotix CLI `sb` (sbcli) correctly in ws-ips. Checks that sb is installed and healthy (sb --version, sb doctor), installs or updates it from the GitHub releases of https://github.com/swarmbotix/sbcli when it is missing or too old, and routes every question about an sb verb, file or error to the reference document shipped at ~/.swarmbotix/documents/sbcli_*.md before answering. Use whenever a task involves sb install, sb up, sb run, sb stop, sb down, sb attach, sb topic, sb ws, sb io, sb config, sb message, sb doctor, sb.workspace.yaml, sb.flow.yaml, calib.flow.yaml, sb.module.yaml, swarmbotix_io, wire keys, the tmux session sb-ws_ips, or any "sb: command not found" or sb error message.
---

# sbcli

`sb` is the swarmbotix CLI: one binary that installs a workspace from its committed files, generates
each module's pub/sub code, builds and launches modules under tmux, and inspects live topics on zenoh
and iceoryx2. ws-ips is one such workspace (`ws_ips`, flows `default` and `calib`); every module under
`modules/` is an sb module. Nothing in sb is a runtime: it is build-time and launch-time orchestration,
and generated code calls zenoh and iceoryx2 directly.

## Rule: the shipped documents are the reference

The installer places the complete reference under `~/.swarmbotix/documents/`. Open the matching
file before explaining a verb, a flag, a file format or an error; do not answer from memory, and do
not search the web for them. `sb <verb> --help` is the authority on what this build accepts.

| Question about | Read |
|---|---|
| What sb is, the three on-disk roots, topic naming, which doc covers what | `sbcli.md` (start here; §4 command map, §8 "where to look when") |
| `sb up`, `sb run`, `sb stop`, `sb down`, `sb attach`, tmux windows, `--fg`, exit codes, "module still running after stop" | `sbcli_launch.md` |
| `sb.flow.yaml`, `calib.flow.yaml`: `nodes:`, `instances:`, `wires:`, several flows, the active flow, keeping a tool out of `sb up` | `sbcli_flow.md` |
| `sb.workspace.yaml`: `modules:`, `messages:`, the installed copy, migrating a 0.7 flow file | `sbcli_workspace.md` |
| `sb ws *`, `sb install <repo>`, `sb ws check`, `active` marker, stale installed copy | `sbcli_ws.md` |
| `sb.module.yaml`: build and start defaults per language, `scripts:`, `env`, `requires`, `platforms`, Docker via compose | `sbcli_module.md` |
| `sb init` (adopt a project as a module) | `sbcli_init.md` |
| `sb io add pub/sub/service`, `sb io update`, encodings `--proto/--fb/--raw`, `--key`, `TopicOverrides` | `sbcli_pubsub.md` (§14 flag reference) |
| Wiring generated `swarmbotix_io/` files into Cargo, Python, C++ builds; the swarm instance pattern | `sbcli_pubsub_consuming.md` |
| Services, request/response over zenoh queryables (the tracker's `cam_param` service) | `sbcli_reqres.md` |
| `sb message *`: the vault, `.proto` files, compile, capacities, `Image1280x1024` and the fixed-size image types | `sbcli_messages.md` |
| `sb config *`: layering, `device`, tool paths, `show --sources`, `get` | `sbcli_config.md` |
| `sb doctor`: every check, OK / WARN / FAIL / SKIP meaning, exit code | `sbcli_doctor.md` |
| `sb update`, `sb --version`, update notice | `sbcli_update.md` |
| `sb install <git-url>` for app packages, `sb app *`, `sb.app.yml` | `sbcli_app.md` |
| `sb topic list / listen / pub / prune` | no shipped document yet; use `sb topic <verb> --help` and the examples below |

Quick way to find a section: `grep -n "^##" ~/.swarmbotix/documents/sbcli_launch.md`, then read
the range. The documents are long (400 to 1400 lines each); read the section, not the whole file.

## Step 0: is sb installed and healthy?

```bash
command -v sb && sb --version         # expect ~/.swarmbotix/bin/sb, 0.8.0 or later for ws-ips
ls ~/.swarmbotix/documents/           # the reference documents
sb doctor                             # tooling, vault, transports, the active workspace rows
```

`sb --version` also prints `update available: <ver>` when a newer release exists (cached 24 h,
`SB_NO_UPDATE_CHECK=1` skips the check). `sb doctor` exits non-zero only on `[FAIL]`; a `[WARN]`
such as `ws source ... is newer than the installed copy; run sb install` means the repository files
changed after the last `sb install .`.

The swarmbotix home `~/.swarmbotix/` holds: `bin/sb`, `documents/`, `sb.config.yml` (host config,
`device:` among others), `active` (name of the active workspace), `workspaces/<name>/` (installed
copy of each workspace with its `flows/` and `active_flow`), `messages/<style>/` (the proto vault and
generated bindings), `apps/`, `VERSION`, `uninstall.sh`.

## Step 1: install when missing

Source: https://github.com/swarmbotix/sbcli. A release is one zip per architecture with the
installer inside; the README's installation section is the procedure. Linux:

```bash
uname -m                                             # x86_64 or aarch64
# download swarmbotix-<version>-linux-<arch>.zip (and the .sha256) from
# https://github.com/swarmbotix/sbcli/releases   into a fresh folder, not inside ~/.swarmbotix
sha256sum -c swarmbotix-<version>-linux-x86_64.zip.sha256
unzip swarmbotix-<version>-linux-x86_64.zip
./swarmbotix-<version>-linux-x86_64/install.sh
```

The installer creates `~/.swarmbotix/{bin,messages,documents}/`, installs the binary to
`~/.swarmbotix/bin/`, merges the proto vault without overwriting edits, replaces `documents/`, seeds
`sb.config.yml` only if none exists, places an uninstaller, and offers to add `~/.swarmbotix/bin` to
`PATH`. If `sb` is still not found afterwards, add the line the installer printed to `~/.bashrc`
and open a new shell. Binaries link glibc 2.17, so any Ubuntu 20.04 or newer loads them.

Windows: `Expand-Archive` the `windows-x86_64` zip and run `install.ps1`.

From source (`git clone https://github.com/swarmbotix/sbcli && cargo build --release`) yields the
binary only, without the vault, the documents and the config template. Do not use it for a
workshop machine.

Runtime tools sb shells out to, reported by `sb doctor`: `protoc` (libprotoc 3.21+), `flatc` (24.x+),
`libzenohc.so` (zenoh 1.9), `libiceoryx2_ffi_c.so` (iceoryx2 0.9.3), `tmux` (3.2+). The two
transport pins are exact: an iceoryx2 process built against another patch version cannot see the
services of this one.

After installing, for ws-ips (`docs/00_setup.md`):

```bash
sb config set device <yourname>       # first segment of every wire key; do this before sb install
cd /path/to/ws-ips && sb install .    # workspace ws_ips, flows default and calib, messages, swarmbotix_io per module
sb doctor
```

## Step 2: update when too old

```bash
sb update --check                     # exit 10 when a newer release exists
sb update                             # download, verify SHA-256, run the installer in place; asks Proceed? [y/N]
sb update --version 0.8.0 --yes       # a specific version, no prompt
```

Only an installed `~/.swarmbotix/bin/sb` updates itself. `bin/sb`, `documents/` and `VERSION` are
replaced; `sb.config.yml`, workspaces, edited messages and apps are kept. Ask the user before
running `sb update` on a machine that is mid-workshop: a new sb may change file formats
(`sbcli.md` §10 lists deprecations and removals per release).

## ws-ips cheat sheet

Every command works from any directory; sb finds the modules through the installed workspace.
`<device>` is `sb config get device`.

```bash
sb install .                          # after pulling, after editing sb.workspace.yaml, a flow file, an sb.module.yaml or a .proto
sb ws check                           # the workspace rows of doctor, for ws_ips only

sb up                                 # build, then start every node of sb.flow.yaml in tmux session sb-ws_ips
sb up calib                           # the calibration flow instead (refused while a session exists: sb down first)
sb down                               # kill the session
sb attach                             # tmux attach; Ctrl+b w switches windows, named <module>-<id>

sb run ips_cam_gcam_pub               # one window per instance of the active flow (sb00..sb03)
sb run ips_cam_gcam_pub:sb00          # one instance; running again restarts it in place
sb run --fg ips_detect:sb00           # no tmux, this terminal, Ctrl+C stops
sb stop ips_detect                    # close its windows (plus scripts.stop if declared)

sb topic list                         # every live topic on both transports, as wire keys
sb topic listen /<device>/ws_ips/ips_track/zenoh/cube_poses -n 1
sb topic listen --transport iceoryx2 /<device>/ws_ips/ips_cam_gcam_pub-sb00/iox2/image --show --no-data
sb topic prune                        # clear (dead) iceoryx2 registrations after a crash

sb io update ips_track                # regenerate one module's swarmbotix_io/ without touching its yaml
sb io update --services ips_track     # re-render the service router (after changing the device name)
sb config show --sources              # merged config and which layer set each key
```

Wire key shape: `<device>/<workspace>/<module>[-<id>]/<transport>/<topic>`, no leading `/`
(`listen` and `pub` accept one and drop it). Shared-memory topics need the `iox2` segment or
`--transport iceoryx2`. Zenoh publishers tag frames with their type, so `--type` is rarely needed.

## If it fails

| Output | Cause and fix |
|---|---|
| `sb: command not found` | Not installed, or `~/.swarmbotix/bin` not on `PATH` (`ls ~/.swarmbotix/bin/sb`). Step 1. |
| `module <m> is not a node of flow <f>` | `sb run` follows the active flow. `sb down && sb up calib` for `ips_calib`; `sbcli_flow.md` §3, §9. |
| `sb up` refuses, a session exists | `sb down` first, or `sb run` for one module. `sbcli_launch.md` §2. |
| `[WARN] ws source ... newer than the installed copy` | `sb install .` at the repository root. |
| `[WARN] ws io files` missing generated files | `sb io update <module>`, or `sb install .`. |
| `schema: null` and hex from `sb topic listen` | Publisher predates the type tag: `sb io update <module>`, rebuild, restart. |
| `(dead)` rows in `sb topic list` | A publisher died without cleanup: `sb topic prune`. |
| `[FAIL] libiceoryx2` or version mismatch | Library absent or another patch version; `sbcli_doctor.md` §4.4. |
| Build fails in the tmux window and the window stays open | Intended: the window keeps the output. Fix, then `sb run <module>` restarts it in place. `sbcli_launch.md` §8, §9. |
| `` `<verb>` is not an sb command or an installed app `` | Removed verb (`sb pub`, `sb sub`, `sb list`, `sb service`) or typo; `sb io` replaced them. `sbcli.md` §10. |
| `this sb is <path>, not the installed <sb_home>/bin/sb` | A dev build; `sb update` only updates an installed sb. |

## Safety

- `sb install .`, `sb io update` and `sb io add` rewrite `swarmbotix_io/` (gitignored) and may edit
  `sb.module.yaml`; `sb ws uninstall --clean` and `sb ws delete` remove installed copies. Confirm
  before anything that deletes.
- `sb config set device` changes every wire key the generated code bakes in; it needs `sb install .`
  and `sb io update --services ips_track` afterwards, and other machines subscribing by key must
  follow.
- Do not run `cargo fmt` at a module root: it follows the `#[path]` includes and rewrites the
  generated `swarmbotix_io/` files.
