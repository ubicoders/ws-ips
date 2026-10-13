#!/usr/bin/env bash
# ips_detect/stopscript.bash
# `sb stop ips_detect` kills the supervisor in the module's window first, then
# runs this script. The detectors themselves run in their own windows,
# ips_detect-sbNN (see runscript.bash); close them here. Each process holds
# nothing outside itself (iceoryx2 ports and zenoh sessions close with it).
set -uo pipefail
MODULE=ips_detect
command -v tmux >/dev/null 2>&1 || exit 0
n=0
while IFS= read -r target; do
  if tmux kill-window -t "$target" 2>/dev/null; then
    echo "closed $target"
    n=$((n + 1))
  fi
done < <(tmux list-windows -a -F '#S:#I #W' 2>/dev/null | awk -v m="$MODULE-" 'index($2, m) == 1 { print $1 }')
echo "$MODULE: closed $n detector window(s)"
exit 0
