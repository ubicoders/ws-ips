# sb issue: `sb topic listen --transport iceoryx2` fails on a typed image service

Reported from the ws-ips workshop repository, 2026-10-12, against the live pipeline.

**Environment:** sb 0.3.0, Linux x86_64 (Ubuntu 24.04), iceoryx2 0.9.3.

## Steps

1. A module publishes `ros2/sensor_msgs/Image1280x1024` (an iox2 fixed-size type compiled with `sb message compile ros2/sensor_msgs/Image1280x1024 --iox2`) through the sb-generated iceoryx2 publisher. `sb topic list` shows it:

   ```
   - [i]      - Hz  PID 3747136   Image1280x1024   elcom/ws_ips/ips_cam_gcam_pub-sb00/iox2/image
   ```

2. ```bash
   sb topic listen --transport iceoryx2 /elcom/ws_ips/ips_cam_gcam_pub-sb00/iox2/image -n 1
   ```

## Result

```
error: iox2 service "elcom/ws_ips/ips_cam_gcam_pub-sb00/iox2/image" open_or_create as `[u8]` failed:
PublishSubscribeOpenOrCreateError::PublishSubscribeOpenError(IncompatibleTypes).
If the service already exists with a typed payload, this hits IncompatibleTypes —
wait for L5's typed introspection or use `--transport zenoh`.
```

No frame is shown.

## Expected

One JSON line per frame (header fields, width, height, encoding, step, data_len; the pixel data may be summarised or truncated), the same as for a zenoh topic. The type is known: `sb topic list` prints it, and its iox2 struct is in the vault's `message_targets`.

## Cause, from the behaviour

The listener opens the service as an untyped `[u8]` payload. iceoryx2 checks the payload type recorded in the service and refuses with `IncompatibleTypes`, because every sb-generated iox2 publisher creates the service with the compiled struct type. The error text itself says typed introspection is pending.

## Request

Open the service with the type that `sb topic list` already resolved for it (the same type name it prints in its own output), decode the fixed-size struct with the vault's definition, and print the fields. For the `data` array, print `data_len` and the first bytes, or an option such as `--no-data`.

## Impact

None of the camera streams in the workshop can be inspected from the CLI. The only live view of the images is the detector's debug image path into rerun. A student whose detector shows `markers=0` cannot tell from sb whether the camera stream itself is arriving.

## Workaround

Read the publisher's console (frame counter and measured rate) and the subscriber's console (`first frame from <key>`). For the picture: `debug: true` in `ips_detect/parameters/detector_params.yaml` plus `debug_show_images: true` in `ips_viz/parameters/viz_params.yaml`, and rerun shows the frames.
