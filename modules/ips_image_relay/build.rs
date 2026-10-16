//! Generate the prost types for `ros2/sensor_msgs/Image` from the sb message vault.
//!
//! The relay publishes the stock ROS 2 `sensor_msgs/Image`, which this repository's
//! `messages/` folder does not carry: its definition lives in the vault that
//! `sb install .` populates, `<sb home>/messages/ros2/message_definitions/`, where
//! `<sb home>` is `$SWARMBOTIX_HOME` if set, else `$HOME/.swarmbotix`.
//!
//! `sensor_msgs/Image.proto` imports `std_msgs/Header.proto`, which imports
//! `builtin_interfaces/Time.proto`, so the vault folder is the include root and
//! prost-build emits one file per package into `$OUT_DIR`: `sensor_msgs.rs`,
//! `std_msgs.rs` and `builtin_interfaces.rs`. `lib.rs` includes all three as
//! sibling modules under `msgs`, so the `super::std_msgs::Header` and
//! `super::builtin_interfaces::Time` paths prost writes resolve. protoc comes
//! from `protoc-bin-vendored`, so no system protoc is needed.

use std::path::PathBuf;

/// `$SWARMBOTIX_HOME`, else `$HOME/.swarmbotix` (`%USERPROFILE%` on Windows).
fn sb_home() -> PathBuf {
    if let Some(h) = std::env::var_os("SWARMBOTIX_HOME").filter(|h| !h.is_empty()) {
        return PathBuf::from(h);
    }
    let home = std::env::var_os("HOME")
        .filter(|h| !h.is_empty())
        .or_else(|| std::env::var_os("USERPROFILE").filter(|h| !h.is_empty()))
        .unwrap_or_else(|| {
            panic!(
                "neither SWARMBOTIX_HOME nor HOME is set: cannot locate the sb message vault \
                 (<sb home>/messages/ros2/message_definitions)"
            )
        });
    PathBuf::from(home).join(".swarmbotix")
}

fn main() {
    println!("cargo:rerun-if-env-changed=SWARMBOTIX_HOME");
    println!("cargo:rerun-if-env-changed=HOME");

    let root = sb_home()
        .join("messages")
        .join("ros2")
        .join("message_definitions");
    let image = root.join("sensor_msgs").join("Image.proto");
    let header = root.join("std_msgs").join("Header.proto");
    let time = root.join("builtin_interfaces").join("Time.proto");

    for f in [&image, &header, &time] {
        assert!(
            f.exists(),
            "proto not found: {} (include root {}). ips_image_relay compiles the stock \
             ros2 sensor_msgs/Image from the sb message vault; run `sb install .` at the \
             repository root to populate it, or set SWARMBOTIX_HOME to the sb home folder \
             that holds messages/ros2/message_definitions/",
            f.display(),
            root.display()
        );
    }

    let protoc = protoc_bin_vendored::protoc_bin_path().expect("vendored protoc binary");

    // Only Image.proto is named; protoc follows its imports from the include
    // root, and prost-build emits code for the imported packages as well.
    prost_build::Config::new()
        .protoc_executable(protoc)
        .compile_protos(&[&image], &[&root])
        .expect("compile ros2 sensor_msgs/Image.proto from the sb message vault");

    for f in [&image, &header, &time] {
        println!("cargo:rerun-if-changed={}", f.display());
    }
}
