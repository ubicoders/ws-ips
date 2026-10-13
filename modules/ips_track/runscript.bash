#!/usr/bin/env bash
# modules/ips_track/runscript.bash
# Invoked by `sb run ips_track [-- args]` / `sb up`, or directly from any cwd.
# Every argument is forwarded to the binary.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

# Standalone tmux wrap (same as the sb template): run directly, re-exec inside
# a tmux session named ips_track[-<id>]-dev so the output survives an exit.
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
  exec tmux new -A -s "ips_track${__sb_id_suffix}-dev" \
    "trap '' INT; (trap - INT; bash $(printf %q "${__sb_self}")${__sb_args}); ec=\$?; echo; echo \"[app exited \$ec - press any key to close]\"; read -n 1"
fi

# The cam_param service router is a user-owned file that `sb install .` leaves
# alone; render it when missing (fresh clone). After a device-name change,
# refresh it with: sb io update --services ips_track
[[ -f swarmbotix_io/services/cam_param.rs ]] || sb io update --services ips_track

exec cargo run --release -- "$@"
