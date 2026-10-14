# messages

Source of the two payload messages the pipeline carries over zenoh. The `.proto` files live here and are the single source: each Rust module's `build.rs` runs `prost-build` on `../../messages/custom/*.proto` directly, so every clone builds before any sb command has run. Nothing is shared between modules beyond the `.proto` text.

The `messages:` entry of `sb.workspace.yaml` makes `sb install .` import the two files into the local sb message vault (`~/.swarmbotix/messages/ros2/message_definitions/custom/`) and compiles them there. That copy is for the sb tooling only: the `sb io` generators need it to know the type when generating `swarmbotix_io/`, and `sb topic listen` decodes these frames against it, using the type every generated zenoh publisher stamps on the wire (`application/protobuf;ros2/custom/<Leaf>`).

| File | Message | Published by | Carried on |
|---|---|---|---|
| `custom/MarkerDetections.proto` | one camera's marker detections for one frame: stamp, camera instance, frame sequence, and per marker the id, family, position, orientation, euler angles and distance in the camera frame | `ips_detect` | `.../ips_detect-sbNN/zenoh/markers` |
| `custom/CubePoses.proto` | the fused world pose of every tracked cube: `R`, `t`, euler angles, quaternion, velocity, contributing marker ids, camera count, residual RMS, liveness | `ips_track` | `.../ips_track/zenoh/cube_poses` |

The image messages (`ros2/sensor_msgs/Image1280x1024`, `Image1280x1024Color`) are standard vault messages that ship with sb; they are not kept here.

Edit a message here, rebuild the modules (cargo picks the change up through `rerun-if-changed`), and run `sb install .` again so the vault copy follows (`--force` when the vault copy differs). Keep this folder flat: a `messages/<style>/message_definitions/` layout would make every `sb` command run inside the repository treat it as the vault.
