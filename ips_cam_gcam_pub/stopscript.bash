#!/usr/bin/env bash
# ips_cam_gcam_pub/stopscript.bash
# `sb stop ips_cam_gcam_pub` kills the supervisor in the module's window first,
# then runs this script. The cameras themselves run in their own windows,
# ips_cam_gcam_pub-sbNN (see runscript.bash); close them here. Each process
# holds nothing outside itself (iceoryx2 ports close with it).
set -uo pipefail
MODULE=ips_cam_gcam_pub
command -v tmux >/dev/null 2>&1 || exit 0
n=0
while IFS= read -r target; do
  if tmux kill-window -t "$target" 2>/dev/null; then
    echo "closed $target"
    n=$((n + 1))
  fi
done < <(tmux list-windows -a -F '#S:#I #W' 2>/dev/null | awk -v m="$MODULE-" 'index($2, m) == 1 { print $1 }')
echo "$MODULE: closed $n camera window(s)"
exit 0
