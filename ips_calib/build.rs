//! Generate the prost types for the IPS protobuf payloads from the repository.
//!
//! The `.proto` source of truth is `messages/custom/` at the repository root, one
//! level above this crate. Both `MarkerDetections.proto` and `CubePoses.proto` are
//! `package swarmbotix.custom`, so prost emits a single `swarmbotix.custom.rs`
//! that `lib.rs` includes as the `msgs` module. protoc comes from
//! `protoc-bin-vendored`, so the build needs no system protoc on PATH.

use std::path::PathBuf;

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let root = manifest.join("../messages");
    let markers = root.join("custom/MarkerDetections.proto");
    let cubes = root.join("custom/CubePoses.proto");

    for f in [&markers, &cubes] {
        assert!(
            f.exists(),
            "proto not found: {}. This crate compiles messages/custom/*.proto of the \
             repository it sits in; build it from inside a full ws-ips clone.",
            f.display()
        );
    }

    let protoc = protoc_bin_vendored::protoc_bin_path().expect("vendored protoc binary");

    // CubePoses.proto imports custom/MarkerDetections.proto, so messages/ is the
    // include root.
    prost_build::Config::new()
        .protoc_executable(protoc)
        .compile_protos(&[&markers, &cubes], &[&root])
        .expect("compile messages/custom/{MarkerDetections,CubePoses}.proto");

    println!("cargo:rerun-if-changed={}", markers.display());
    println!("cargo:rerun-if-changed={}", cubes.display());
}
