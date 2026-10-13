#!/usr/bin/env bash
# ips_cam_gcam_pub/runscript.bash
# Invoked by `sb run ips_cam_gcam_pub [-- args]` / `sb up`, or directly from any cwd.
#
#   --id sbNN     publish that one camera, in this window.
#   (no --id)     open one tmux window per camera listed in parameters/cam_params.yaml,
#                 named ips_cam_gcam_pub-sbNN, each running `--id sbNN` as its own
#                 process, then exit. `sb stop ips_cam_gcam_pub` closes them
#                 (stopscript.bash); `sb run ips_cam_gcam_pub` replaces them.
#   SB_NO_TMUX=1  no tmux at all: one process, one thread per camera.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
MODULE=ips_cam_gcam_pub

# The --id value, if one was passed.
ID=""; __prev=""
for __arg in "$@"; do
  if [[ "$__prev" == "--id" ]]; then ID="$__arg"; break; fi
  __prev="$__arg"
done

# Standalone tmux wrap (same as the sb template): run directly, re-exec inside
# a tmux session named ips_cam_gcam_pub[-<id>]-dev so the output survives an exit.
# Under `sb up` / `sb run` $TMUX is already set and this block is skipped.
# SB_NO_TMUX=1 skips it too (CI, IDE runners).
if [[ -z "${TMUX:-}" ]] && [[ "${SB_NO_TMUX:-0}" != "1" ]] && command -v tmux >/dev/null 2>&1; then
  __sb_self="$PWD/$(basename "${BASH_SOURCE[0]}")"
  __sb_args=""
  for __arg in "$@"; do __sb_args="$__sb_args $(printf %q "$__arg")"; done
  exec tmux new -A -s "${MODULE}${ID:+-$ID}-dev" \
    "trap '' INT; (trap - INT; bash $(printf %q "${__sb_self}")${__sb_args}); ec=\$?; echo; echo \"[app exited \$ec - press any key to close]\"; read -n 1"
fi

# No privileges: GigE discovery works from a plain user process on this setup
# (verified with the vendor SDK and no capability). If `gcam-mvsdk-rs list`
# finds no camera, `gcam-mvsdk-rs doctor` names the host setting to fix.
BIN="target/release/$MODULE"
build() {
  cargo build --release
}

# One camera, or no tmux: run the binary in this window.
if [[ -n "$ID" ]] || [[ -z "${TMUX:-}" ]]; then
  build
  exec "$BIN" "$@"
fi

# Fan-out: one window per camera named in parameters/cam_params.yaml.
mapfile -t IDS < <(grep -E '^[[:space:]]*cameras:' parameters/cam_params.yaml | sed 's/#.*//' | grep -oE 'sb[0-9]+')
if (( ${#IDS[@]} == 0 )); then
  echo "error: no cameras in parameters/cam_params.yaml" >&2
  exit 1
fi
build   # once here, so the camera windows do not race for the cargo lock
SESSION="$(tmux display-message -p '#S')"
SELF="$PWD/$(basename "${BASH_SOURCE[0]}")"
for id in "${IDS[@]}"; do
  win="$MODULE-$id"
  tmux kill-window -t "$SESSION:$win" 2>/dev/null || true
  tmux new-window -d -t "$SESSION:" -n "$win" \
    "trap '' INT; (trap - INT; bash $(printf %q "$SELF") --id $id); ec=\$?; echo; echo \"[$win exited \$ec - press any key to close]\"; read -n 1"
done
echo "$MODULE: ${#IDS[@]} camera window(s) in tmux session '$SESSION': ${IDS[*]/#/$MODULE-}"
echo "stop all: sb stop $MODULE"
