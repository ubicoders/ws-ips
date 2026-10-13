# 05 Viz and off the host

Module 3, last part. `rerun` then `sb run ips_viz`. Read the position plot for jitter and note the frame-to-pose latency: the stamp on every cube pose is the capture time of the camera frame it came from, so latency is the difference to the wall clock. `ws-px4` needs that number for the estimator delay.

Then the zenoh router, and `sb topic listen` on the cube_poses key from a phone or a second machine.
