#!/usr/bin/env bash
# One-time setup of this clone for the sb CLI.
#   1. pull the driver submodules
#   2. create and activate the sb workspace ws_ips
#   3. install workspace/flow.yaml with <REPO> replaced by this clone's path
#   4. import messages/custom/*.proto into the sb message vault
# Safe to run again; every step is idempotent.
set -euo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS=ws_ips

command -v sb >/dev/null || { echo "sb not found. Install the sb CLI first (docs/00_setup.md)." >&2; exit 1; }

echo "-- submodules"
git -C "$REPO" submodule update --init --recursive

echo "-- workspace $WS"
sb ws list 2>/dev/null | grep -qE "^\*? *$WS\$" || sb ws create "$WS"
sb ws set "$WS"

echo "-- flow.yaml"
SB_HOME="${SB_HOME:-$HOME/.swarmbotix}"
sed "s#<REPO>#$REPO#g" "$REPO/workspace/flow.yaml" > "$SB_HOME/workspaces/$WS/flow.yaml"

echo "-- messages"
for f in "$REPO"/messages/custom/*.proto; do
  [ -e "$f" ] || { echo "   (no protos yet in messages/custom/)"; break; }
  echo "   $(basename "$f")"
  # TODO: replace with the sb message import command for this sb version
  # (see `sb message --help`). Until then copy by hand into
  # $SB_HOME/messages/ros2/message_definitions/custom/
done

echo "-- done. Next: sb up"
