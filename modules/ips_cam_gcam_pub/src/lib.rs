//! `ips_cam_gcam_pub`: reads HuaTeng GigE cameras through `gcam-mvsdk-rs` and
//! publishes every MONO8 1280x1024 frame over iceoryx2 shared memory.
//!
//! The binary (`src/main.rs`) parses `--id`, runs one thread per camera and the
//! publish loop. This library holds what the binary and the tests share.

// sb-generated iceoryx2 publisher for `ros2/sensor_msgs/Image1280x1024`. Do not
// edit: `sb install .` regenerates it per clone. Its baked module segment is
// `ips_cam_gcam_pub`; each camera thread overrides it to `ips_cam_gcam_pub-<id>`.
#[path = "../swarmbotix_io/publishers/image.rs"]
pub mod image;

pub mod capture;
