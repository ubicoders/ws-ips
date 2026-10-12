# cam_gige_ht

Reads one HuaTeng GigE camera (MONO8, 1280x1024) through the MVSDK driver in `drivers/` and publishes every frame over iceoryx2 shared memory. One container per camera, selected by the camera's user-set name (`CAM_NAME`). Full at handout: camera capture is not a lesson of this workshop.

## Inputs

- The camera on the LAN, named with `camconfig/nameit.py` (`sb00` to `sb03`).
- `EXPOSURE_US` (default 5000) and `CAM_NAME` from `workspace/flow.yaml` instances.

## Outputs

- `ros2/sensor_msgs/Image1280x1024`, one iceoryx2 service per camera, about 90 Hz.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| image | iceoryx2 | `<device>/ws_ips/cam_gige_ht-sbNN/iox2/image` |

## State at handout, and what you build

Full. Nothing to build. `camconfig/` holds the helpers to list cameras, name a camera, and set its IP; run them before module 2.

## Run

```bash
sb run cam_gige_ht            # this module alone, in its own tmux session
sb stop cam_gige_ht
```

Parameters, when the module has them, live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml.
