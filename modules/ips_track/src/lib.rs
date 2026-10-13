//! `ips_track`: fuse every camera's marker detections into rigid cube poses.
//!
//! The binary (`src/main.rs`) owns the parameters, the extrinsics, the fixed-rate
//! tick loop and the publish path. This library holds everything it builds on:
//! the sb-generated wire code, the prost types, the cube model and fit, the
//! output filters, the rate meter and the cam_param service routes.

// sb-generated zenoh subscriber for `ros2/custom/MarkerDetections`. Generated,
// do not edit. main() uses only its `TopicOverrides` to render the fan-in key
// `<device>/ws_ips/ips_detect-$*/zenoh/markers`, then declares its own
// subscriber and drains every sample (the generated `try_recv_latest` keeps only
// the newest one, which would drop the other cameras' frames).
//
// The three generated modules are not ours to edit, so clippy is silenced on
// them (the subscriber trips `useless_conversion`).
#[path = "../swarmbotix_io/subscribers/markers.rs"]
#[allow(clippy::all)]
pub mod markers_sub;

// sb-generated zenoh publisher for `ros2/custom/CubePoses` on
// `<device>/ws_ips/ips_track/zenoh/cube_poses`. Generated, do not edit.
#[path = "../swarmbotix_io/publishers/cube_poses.rs"]
#[allow(clippy::all)]
pub mod cube_poses_pub;

// sb service router, rendered by `sb install .` from the `services:` entry of
// sb.dev.yml (`cam_param`). Generated, do not edit.
#[path = "../swarmbotix_io/services/cam_param.rs"]
#[allow(clippy::all)]
pub mod service;

// prost types for `custom/MarkerDetections` and `custom/CubePoses` (package
// `swarmbotix.custom`): Vec3, Quat, MarkerDetection, MarkerDetections, Mat3,
// CubePose, CubePoses. build.rs compiles them from the repository's messages/.
pub mod msgs {
    include!(concat!(env!("OUT_DIR"), "/swarmbotix.custom.rs"));
}

pub mod cam_param;
pub mod cube;
pub mod filter;
pub mod filter_kf;
pub mod filter_lp;
pub mod filter_one_euro;
pub mod fps;
