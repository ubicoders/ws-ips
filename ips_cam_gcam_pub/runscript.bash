#!/usr/bin/env bash
# ips_cam_gcam_pub/runscript.bash
# Invoked by `sb run ips_cam_gcam_pub [-- args]` / `sb up`, or directly from any cwd.
# Every argument is forwarded to the binary.
# No --id: every camera listed in parameters/cam_params.yaml, one thread each.
# --id sbNN: that camera only.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

# Standalone tmux wrap (same as the sb template): run directly, re-exec inside
# a tmux session named ips_cam_gcam_pub[-<id>]-dev so the output survives an exit.
# Under `sb up` / `sb run` $TMUX is already set and this block is skipped.
# SB_NO_TMUX=1 skips it too (CI, IDE runners).
if [[ -z "${TMUX:-}" ]] && [[ "${SB_NO_TMUX:-0}" != "1" ]] && command -v tmux >/dev/null 2>&1; then
  __sb_self="$PWD/$(basename "${BASH_SOURCE[0]}")"
  __sb_args=""
  for __arg in "$@"; do __sb_args="$__sb_args $(printf %q "$__arg")"; done
  __sb_id_suffix=""; __sb_prev=""
  for __arg in "$@"; do
    if [[ "$__sb_prev" == "--id" ]]; then __sb_id_suffix="-$__arg"; break; fi
    __sb_prev="$__arg"
  done
  exec tmux new -A -s "ips_cam_gcam_pub${__sb_id_suffix}-dev" \
    "trap '' INT; (trap - INT; bash $(printf %q "${__sb_self}")${__sb_args}); ec=\$?; echo; echo \"[app exited \$ec - press any key to close]\"; read -n 1"
fi

# The vendor SDK discovers GigE cameras over raw sockets, so the binary needs
# CAP_NET_RAW. cargo relinks drop file capabilities, so build first and set
# them on the result when missing (asks for sudo once per rebuild).
cargo build --release
__bin="target/release/ips_cam_gcam_pub"
if ! getcap "$__bin" 2>/dev/null | grep -q cap_net_raw; then
  echo "setting cap_net_raw,cap_net_admin on $__bin (sudo)" >&2
  sudo setcap cap_net_raw,cap_net_admin+ep "$__bin"
fi
exec "$__bin" "$@"
