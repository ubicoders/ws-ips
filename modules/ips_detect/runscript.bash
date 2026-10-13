#!/usr/bin/env bash
# modules/ips_detect/runscript.bash
# Invoked by `sb run ips_detect [-- args]` / `sb up`, or directly from any cwd.
#
#   --id sbNN     detect on that one camera, in this window.
#   (no --id)     open one tmux window per camera index in parameters/detector_params.yaml,
#                 named ips_detect-sbNN, each running `--id sbNN` as its own
#                 process, then exit. `sb stop ips_detect` closes them
#                 (stopscript.bash); `sb run ips_detect` replaces them.
#   SB_NO_TMUX=1  no tmux at all: one process, one thread per camera.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
MODULE=ips_detect

# The --id value, if one was passed.
ID=""; __prev=""
for __arg in "$@"; do
  if [[ "$__prev" == "--id" ]]; then ID="$__arg"; break; fi
  __prev="$__arg"
done

# Standalone tmux wrap (same as the sb template): run directly, re-exec inside
# a tmux session named ips_detect[-<id>]-dev so the output survives an exit.
# Under `sb up` / `sb run` $TMUX is already set and this block is skipped.
# SB_NO_TMUX=1 skips it too (CI, IDE runners).
if [[ -z "${TMUX:-}" ]] && [[ "${SB_NO_TMUX:-0}" != "1" ]] && command -v tmux >/dev/null 2>&1; then
  __sb_self="$PWD/$(basename "${BASH_SOURCE[0]}")"
  __sb_args=""
  for __arg in "$@"; do __sb_args="$__sb_args $(printf %q "$__arg")"; done
  exec tmux new -A -s "${MODULE}${ID:+-$ID}-dev" \
    "trap '' INT; (trap - INT; bash $(printf %q "${__sb_self}")${__sb_args}); ec=\$?; echo; echo \"[app exited \$ec - press any key to close]\"; read -n 1"
fi

# The opencv crate dlopens libclang at build time (feature clang-runtime).
export LIBCLANG_PATH="${LIBCLANG_PATH:-/usr/lib/llvm-18/lib}"
BIN="target/release/$MODULE"

# One camera, or no tmux: run the binary in this window.
if [[ -n "$ID" ]] || [[ -z "${TMUX:-}" ]]; then
  cargo build --release
  exec "$BIN" "$@"
fi

# Fan-out: one window per camera index in parameters/detector_params.yaml
# (index N -> sbNN, the camera's name in modules/ips_cam_gcam_pub/parameters/cam_params.yaml).
mapfile -t IDX < <(grep -E '^[[:space:]]*cameras:' parameters/detector_params.yaml | sed 's/#.*//' | grep -oE '[0-9]+')
if (( ${#IDX[@]} == 0 )); then
  echo "error: no cameras in parameters/detector_params.yaml" >&2
  exit 1
fi
IDS=()
for i in "${IDX[@]}"; do IDS+=("$(printf 'sb%02d' "$i")"); done
cargo build --release   # once here, so the detector windows do not race for the cargo lock
SESSION="$(tmux display-message -p '#S')"
SELF="$PWD/$(basename "${BASH_SOURCE[0]}")"
for id in "${IDS[@]}"; do
  win="$MODULE-$id"
  tmux kill-window -t "$SESSION:$win" 2>/dev/null || true
  tmux new-window -d -t "$SESSION:" -n "$win" \
    "trap '' INT; (trap - INT; bash $(printf %q "$SELF") --id $id); ec=\$?; echo; echo \"[$win exited \$ec - press any key to close]\"; read -n 1"
done
echo "$MODULE: ${#IDS[@]} detector window(s) in tmux session '$SESSION': ${IDS[*]/#/$MODULE-}"
echo "stop all: sb stop $MODULE"
