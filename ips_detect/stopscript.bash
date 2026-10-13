#!/usr/bin/env bash
# ips_detect/stopscript.bash
# `sb stop ips_detect` kills the module's process in its tmux window first, then
# runs this script. The module holds no state outside that process (iceoryx2
# ports and zenoh sessions close with it), so there is nothing left to clean up.
exit 0
