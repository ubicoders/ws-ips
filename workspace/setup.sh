#!/usr/bin/env bash
# One-time setup of this clone for the sb CLI (0.2.x). Safe to run again.
#   1. pull the driver submodules
#   2. create and activate the sb workspace ws_ips
#   3. install workspace/flow.yaml with <REPO> replaced by this clone's path
#   4. write every module's sb.dev.yml from its sb.dev.yml.in (same replacement)
#   5. put messages/custom/*.proto into the sb message vault and compile them
#   6. generate every module's swarmbotix_io/ from its sb.dev.yml
#
# Steps 4 and 6 exist because the generated files bake in this machine's device
# name, the workspace, and absolute paths. They are gitignored and rebuilt here.
set -euo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS=ws_ips
MODULES=(ips_cam_gcam_pub ips_detect ips_calib ips_track ips_viz)
SB_HOME="${SB_HOME:-$HOME/.swarmbotix}"
VAULT="$SB_HOME/messages/ros2/message_definitions"

command -v sb >/dev/null || { echo "sb not found. Install the sb CLI first (docs/00_setup.md)." >&2; exit 1; }
sb config get device >/dev/null 2>&1 || { echo "no device name set. Run: sb config set device <name>" >&2; exit 1; }

# sb switches to a vault found under the cwd; run every sb command from $HOME
# so this repository's messages/ folder is never mistaken for one.
sb() { (cd "$HOME" && command sb "$@"); }

echo "-- submodules"
git -C "$REPO" submodule update --init --recursive

echo "-- workspace $WS"
sb ws list 2>/dev/null | grep -qE "^\*? *$WS\$" || sb ws create "$WS"
sb ws set "$WS"

echo "-- flow.yaml"
mkdir -p "$SB_HOME/workspaces/$WS"
sed "s#<REPO>#$REPO#g" "$REPO/workspace/flow.yaml" > "$SB_HOME/workspaces/$WS/flow.yaml"

echo "-- sb.dev.yml per module"
for m in "${MODULES[@]}"; do
  sed "s#<REPO>#$REPO#g" "$REPO/$m/sb.dev.yml.in" > "$REPO/$m/sb.dev.yml"
  echo "   $m"
done

echo "-- messages -> vault ($VAULT/custom)"
mkdir -p "$VAULT/custom"
for f in "$REPO"/messages/custom/*.proto; do
  cp "$f" "$VAULT/custom/"
  echo "   $(basename "$f")"
done
sb message compile ros2/custom/MarkerDetections
sb message compile ros2/custom/CubePoses

echo "-- swarmbotix_io per module"
# Re-render every entry of a module's sb.dev.yml. `sb pub edit` needs -m or -t,
# so the publisher's own type is passed back; `sb sub edit` re-renders from the
# yaml as is. Parsing relies on the fixed layout of the sb.dev.yml.in files.
render_module() {
  local m="$1" yml="$REPO/$1/sb.dev.yml" section="" name="" topic="" type=""
  mkdir -p "$REPO/$m/swarmbotix_io"
  while IFS= read -r line; do
    case "$line" in
      publishers:*)  section=pub ;;
      subscribers:*) section=sub ;;
      *"- name:"*)   name="$(sed -E 's/.*name: *([^ #]+).*/\1/' <<<"$line")"; topic=""; type="" ;;
      *"topic:"*)    topic="$(sed -E 's/.*topic: *([^ #]+).*/\1/' <<<"$line")" ;;
      *"type:"*)     type="$(sed -E 's/.*type: *([^ #]+).*/\1/' <<<"$line")"
                     if [[ $section == pub ]]; then sb pub edit "$name" -m "$type" --module "$m"
                     else sb sub edit "$topic" --module "$m"; fi ;;
    esac
  done < <(grep -vE '^\s*#' "$yml")
}
for m in "${MODULES[@]}"; do
  echo "   $m"
  render_module "$m"
done
sb service init --module ips_track   # swarmbotix_io/service.rs for the cam_param queryable

echo "-- done. Next: sb up   (then `sb attach`, or `tmux attach -t sb-$WS`)"
