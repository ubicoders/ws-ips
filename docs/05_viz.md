# 05 Viz and off the host

Module 3, last part. `rerun` then `sb run ips_viz`. Read the position plot for jitter and note the frame-to-pose latency: `ws-px4` needs that number for the estimator delay.

Then the zenoh router, and `sb topic listen` on the cube_poses key from a phone or a second machine.
