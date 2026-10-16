# ips_image_relay

Subscribes to one camera's two iceoryx2 image streams, the camera frame from `ips_cam_gcam_pub` and the annotated frame from `ips_detect`, halves each with OpenCV, and republishes them over zenoh under the same topic names. Shared memory can be opened only on the host, so this is how a network peer such as the Flutter app gets the images. One instance per camera, same id as the camera it reads. Full at handout. Optional: it is not a node of the default flow, so `sb up` never starts it; the relay flow (`relay.flow.yaml`, `sb up relay`) runs the pipeline with it. Nothing upstream or downstream changes: `ips_detect`, `ips_track` and `ips_viz` keep reading the full-resolution iceoryx2 streams.

## Inputs

- `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image` (iceoryx2), `ros2/sensor_msgs/Image1280x1024`, MONO8.
- `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image` (iceoryx2), `ros2/sensor_msgs/Image1280x1024Color`, RGB8. It exists only while `ips_detect` runs with `debug: true` in `detector_params.yaml`, so it may never come; the camera image is relayed regardless.
- `parameters/relay_params.yaml`: `cameras` (indices into cam_params.yaml, 0 -> sb00), `scale`, `every`.

Each subscriber is opened with its publisher's key at startup, as `ips_detect` does for the camera image. The relay waits for each of the two services on its own and prints a waiting line until it appears.

## Outputs

- `ros2/sensor_msgs/Image` (protobuf, the vault's variable-size type) over zenoh on `image`: 640x512 `mono8`, step 640, 327680 bytes of data.
- The same type on `debug_image`: 640x512 `rgb8`, step 1920, 983040 bytes of data, while the detector publishes its debug frame.

Sizes are for the default `scale: 2`. Every message carries the encoding tag `application/protobuf;ros2/sensor_msgs/Image`, so `sb topic listen` decodes it without `--type`. Header stamp, `frame_id` and encoding are copied from the source frame, so a viewer can line an image up with its markers and with the cube poses. The downscale is OpenCV `resize` with `INTER_AREA`, at scale 2 the average of each 2x2 block.

## Wire keys

| Topic | Transport | Key |
|---|---|---|
| image (subscribed) | iceoryx2 | `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image` |
| debug_image (subscribed) | iceoryx2 | `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image` |
| image | zenoh | `<device>/ws_ips/ips_image_relay-sbNN/zenoh/image` |
| debug_image | zenoh | `<device>/ws_ips/ips_image_relay-sbNN/zenoh/debug_image` |

In `relay.flow.yaml` the camera with the relay's id feeds it beside the detector (`ips_cam_gcam_pub[{id}].image: [ips_detect.image, ips_image_relay.image]`), and so does the detector with that id (`ips_detect[{id}].debug_image: ips_image_relay.debug_image`). As for the other modules, code generation does not read the wires: the program sets both subscriber keys itself at startup.

## Layout

```
src/main.rs          --id parsing, relay_params.yaml, one thread per camera (all cameras in one process when started by hand without --id), the publish loop, the Hz line
src/source.rs        one iceoryx2 stream, opened with its publisher's key once that service exists; the waiting line
src/relay.rs         one stream: keep every Nth frame, downscale, fill sensor_msgs/Image with the source's stamp, frame_id and encoding
src/scale.rs         frame geometry check, OpenCV resize with INTER_AREA
src/rate.rs          rate meter
parameters/relay_params.yaml
sb.module.yaml       subscribers `image`, `debug_image` (iceoryx2), publishers `image`, `debug_image` (zenoh); LIBCLANG_PATH under platforms.<os>.env
src/lib.rs           crate root: the swarmbotix_io includes, the prost types, the modules above
build.rs             prost on the vault's sensor_msgs/Image.proto and its imports
```

Depends on opencv (`imgproc` only), iceoryx2 `=0.9.3`, zenoh 1.9, prost. Needs `LIBCLANG_PATH` at build time only, like `ips_detect` (the opencv crate parses the OpenCV headers with libclang). sb sets it from `platforms.linux.env` in `sb.module.yaml`, default `/usr/lib/llvm-18/lib`; export `LIBCLANG_PATH` to override it, and export it yourself for a `cargo build` by hand. `sensor_msgs/Image` is not in this repository's `messages/`: `build.rs` reads it from the sb message vault (`~/.swarmbotix/messages/ros2/message_definitions/`, or under `$SWARMBOTIX_HOME`), which `sb install .` populates, so install the workspace before the first build.

## State at handout, and what you build

Full. Nothing to build.

## Run

```bash
sb down && sb up relay              # the pipeline plus one relay per camera; plain `sb up relay` when no session runs
sb run ips_image_relay              # relay flow active: builds, then one tmux window and process per instance (ips_image_relay-sbNN)
sb run ips_image_relay:sb00         # one camera
sb run --fg ips_image_relay:sb00    # one camera in this terminal, no tmux
sb stop ips_image_relay             # closes the relay windows; the pipeline keeps running
sb down && sb up                    # back to the pipeline without the relay
```

While the default flow is active, `sb run ips_image_relay` is refused (`module ips_image_relay is not a node of flow default`). Beside the default flow, start the relay by hand from this folder instead, once it is built (`cargo build --release`, with `LIBCLANG_PATH` exported):

```bash
target/release/ips_image_relay              # every camera in relay_params.yaml, one process, one thread each
target/release/ips_image_relay --id sb00    # one camera
```

Every 5 s each relay prints one line with the rate of both streams (`ips_image_relay-sbNN image ... Hz, debug_image ... Hz`); `debug_image` reads 0.0 Hz while the detectors run with `debug: false`. From the host or a second machine on the LAN:

```bash
sb topic list                                                          # the relay keys with their rates
sb topic listen /<device>/ws_ips/ips_image_relay-sb00/zenoh/image -n 1    # decodes one sensor_msgs/Image
```

Parameters live in `parameters/` and are read at startup. No value is passed on the command line that also exists in the yaml; `--id` only narrows the list. Every instance id in `relay.flow.yaml` must be one of the cameras `relay_params.yaml` selects (by index into `cam_params.yaml`).

| Key | Effect |
|---|---|
| `cameras` | `[0, 1, 2, 3]`, indices into cam_params.yaml; `--id sb00` runs one |
| `scale` | integer divisor of both sides, `2` gives 640x512; must divide 1280 and 1024 (1, 2, 4, 8, ...) |
| `every` | relay every Nth received frame per stream; `1` relays all |

`every` is the knob for a Wi-Fi peer that needs less bandwidth. A mono frame at `scale: 2` is 327680 bytes, so at the cameras' 90 Hz the four mono streams together are about 118 MB/s before zenoh framing, and a debug frame is three times the size of a mono one. `every: N` divides the rate of both streams by N.
