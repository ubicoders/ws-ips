# 01 Cameras

Module 2, first half. Name each camera, put it on the network, and see its frames. The tools are the driver's own command, installed in setup step 3.

- `gcam-mvsdk-rs doctor` checks the host: raw-socket capability, `rp_filter`, firewall, subnet, and the receive buffer size. Fix what it flags before anything else.
- `gcam-mvsdk-rs list` shows every camera on the LAN with serial, user-set name, IP and the host NIC it answers on.
- `gcam-mvsdk-rs rename <serial> sb00` writes the user-set name into one camera. The name persists and is the instance id on the wire. Power-cycle the camera afterwards: `ips_cam_gcam_pub` selects a camera by the name it reports during discovery, and that follows a rename only after a restart.
- `gcam-mvsdk-rs setip <mac> <ip> <mask> <gateway>` moves a camera that sits outside the PC's subnet.
- `gcam-mvsdk-rs grab sb00` saves a test frame; check focus and exposure on it.
- `sb run ips_cam_gcam_pub:sb00` publishes that camera. `sb topic list` shows the image key.

Rules: exposure 5 ms (motion blur kills corner detection), jumbo frames only if the switch supports them (`packet_bytes` in `modules/ips_cam_gcam_pub/parameters/cam_params.yaml`), one subnet for cameras and PC, and a 32 MB socket receive buffer or the cameras throttle themselves to about 60 Hz.
