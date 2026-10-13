# messages

Source of the two payload messages the pipeline carries over zenoh. They are sb vault messages: the `.proto` files live here so every clone builds, and `workspace/setup.sh` copies them into the local sb message vault (`~/.swarmbotix/messages/ros2/message_definitions/custom/`) and compiles them there.

sb emits no Rust for protobuf messages. Each Rust module therefore has its own `build.rs` that runs `prost-build` on these two files out of the vault, the same way the original project does. Nothing is shared between modules beyond the `.proto` text.

| File | Message | Published by | Carried on |
|---|---|---|---|
| `custom/MarkerDetections.proto` | one camera's marker detections for one frame: stamp, camera instance, frame sequence, and per marker the id, family, position, orientation, euler angles and distance in the camera frame | `ips_detect` | `.../ips_detect-sbNN/zenoh/markers` |
| `custom/CubePoses.proto` | the fused world pose of every tracked cube: `R`, `t`, euler angles, quaternion, velocity, contributing marker ids, camera count, residual RMS, liveness | `ips_track` | `.../ips_track/zenoh/cube_poses` |

The image messages (`ros2/sensor_msgs/Image1280x1024`, `Image1280x1024Color`) are standard vault messages that ship with sb; they are not kept here.

Edit a message here, then run `workspace/setup.sh` again so the vault copy and the compiled bindings follow. Keep this folder flat: a `messages/<style>/message_definitions/` layout would make every `sb` command run inside the repository treat it as the vault.
