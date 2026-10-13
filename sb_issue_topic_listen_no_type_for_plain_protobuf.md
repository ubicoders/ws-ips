# sb issue: `sb topic listen` cannot decode a protobuf topic that carries no swarmbotix header

Reported from the ws-ips workshop repository, 2026-10-12, against the live pipeline.

**Environment:** sb 0.3.0, Linux x86_64 (Ubuntu 24.04), zenoh 1.9.0, protoc 35.0.

## Steps

1. A vault message in the `ros2` style with no `swarmbotix/header/Header` field, published as plain protobuf through the sb-generated zenoh publisher (whose generated code says "encoding is the caller's job"):

   ```
   ros2/custom/CubePoses      fields: stamp_sec, stamp_nanosec, frame_seq, repeated CubePose cubes
   ```

   Declared in the publishing module's `sb.dev.yml`:

   ```yaml
   publishers:
     - name: cube_poses
       topic: cube_poses
       type: ros2/custom/CubePoses
       transport: zenoh
   ```

2. With the publisher running:

   ```bash
   sb topic listen /elcom/ws_ips/ips_track/zenoh/cube_poses
   ```

   Also tried the bare form from inside the publishing module's folder (`cd ips_track && sb topic listen cube_poses`).

## Result

Frames arrive but are not decoded, in both forms:

```
{"transport":"zenoh","topic":"/elcom/ws_ips/ips_track/zenoh/cube_poses","schema":null,"hash":null,"encoding":null,"payload":{"kind":"hex","bytes":"08afc8b6d6061095d1c70c18b13a"}}
```

The bytes are valid protobuf for the declared type (`08..` stamp_sec, `10..` stamp_nanosec, `18..` frame_seq).

## Expected

One JSON line per frame with the decoded fields, as for any type sb has the `.proto` for. The type is known to sb: it is declared in the publisher's `sb.dev.yml`, which the fully-qualified path identifies (`<device>/<workspace>/<module>/<transport>/<topic>`), and its `.proto` is compiled in the vault.

## Cause, from the behaviour

The decoder identifies the schema only by probing the payload for a `swarmbotix/header/Header` as field 1 and reading `metadata.msg_type` (`--raw` is documented as "skip the Header probe"). A message without that header gives `schema: null`, and there is no other way to tell `sb topic listen` the type: no `--type` option, and the bare-topic form resolves the key but not the type from `sb.dev.yml`.

Adding the header to the message is not a small fix for the user: `header/Header` lives in the `swarmbotix` style, cross-style imports are not supported, so a `ros2`-style message cannot embed it without moving to the other style, which changes the wire type name everywhere.

## Request

Either of:

1. `sb topic listen --type <style>/<namespace>/<Leaf>` to name the schema explicitly.
2. When the fully-qualified path names a module registered in the active `flow.yaml` (or the cwd module for the bare form), take the type from that module's `sb.dev.yml` entry for the topic when the header probe finds nothing.

## Impact

Any protobuf topic without the swarmbotix header is uninspectable from the CLI. For the sb-generated zenoh publishers that is the default situation, since they hand the caller raw bytes and do not add a header. In the workshop this is both pipeline payloads; students fall back to the module consoles.

## Workaround

None inside sb. Read the module's console output, or write a one-off subscriber.
