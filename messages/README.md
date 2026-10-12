# messages

Source of the two payload messages the pipeline carries over zenoh. They are sb vault messages: the `.proto` files live here so every clone builds, and `workspace/setup.sh` imports them into the local sb message vault, where the module build scripts read them.

| File | Message | Published by | Carried on |
|---|---|---|---|
| `custom/MarkerDetections.proto` | one camera's marker detections for one frame: frame sequence, timestamp, camera instance, marker id, corners, pose in the camera frame | `ips_detect` | `.../ips_detect-sbNN/zenoh/markers` |
| `custom/CubePoses.proto` | the fused world pose of every tracked cube: `R`, `t`, euler angles, quaternion, velocity, contributing marker ids, camera count, residual RMS, liveness | `ips_track` | `.../ips_track/zenoh/cube_poses` |

Edit a message with `sb message edit custom/<Name>` and copy the result back here, so the repository stays the source of truth.
