#!/usr/bin/env bash
# ips_track/stopscript.bash
# `sb stop ips_track` kills the module's process in its tmux window first, then
# runs this script. The module holds no state outside that process (iceoryx2
# ports and zenoh sessions close with it), so there is nothing left to clean up.
exit 0
