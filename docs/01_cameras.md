# 01 Cameras

Module 2, first half. Name each camera, put it on the network, and see its frames.

- `cam_gige_ht/camconfig/list_cameras.py` lists every camera on the LAN with serial, name, and IP.
- `cam_gige_ht/camconfig/nameit.py` writes a user-set name (`sb00` to `sb03`) into one camera. The name persists.
- `sb run cam_gige_ht --id sb00` publishes that camera. `sb topic list` shows the image key.

Rules: exposure 5 ms (motion blur kills corner detection), jumbo frames only if the switch supports them, one subnet for cameras and PC.
