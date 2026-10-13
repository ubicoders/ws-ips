//! Generate the prost types for the IPS protobuf payloads from this repository's
//! `messages/` folder (the sibling of this module folder).
//!
//! prost-build emits `$OUT_DIR/swarmbotix.custom.rs` (package `swarmbotix.custom`),
//! which `lib.rs` includes as the `msgs` module: Vec3, Quat, MarkerDetection,
//! MarkerDetections, Mat3, CubePose, CubePoses. protoc comes from
//! `protoc-bin-vendored`, so no system protoc is needed.

use std::path::{Path, PathBuf};

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let raw_root = manifest.join("../..").join("messages");
    // Canonicalise so protoc sees an include root without `..` segments.
    let root = std::fs::canonicalize(&raw_root).unwrap_or_else(|e| {
        panic!(
            "messages folder not found at {} ({e}). ips_detect must sit in modules/ next to the \
             repository's messages/ folder, which holds custom/MarkerDetections.proto \
             and custom/CubePoses.proto",
            raw_root.display()
        )
    });
    let markers = root.join("custom").join("MarkerDetections.proto");
    let cubes = root.join("custom").join("CubePoses.proto");

    for f in [&markers, &cubes] {
        assert!(
            f.exists(),
            "proto not found: {} (include root {}). The repository's messages/custom/ \
             folder must hold MarkerDetections.proto and CubePoses.proto",
            f.display(),
            root.display()
        );
    }

    let protoc = protoc_bin_vendored::protoc_bin_path().expect("vendored protoc binary");

    // CubePoses.proto imports custom/MarkerDetections.proto, so messages/ is the
    // include root. Both files are package swarmbotix.custom, so prost emits a
    // single swarmbotix.custom.rs holding every type.
    let protos: [&Path; 2] = [&markers, &cubes];
    prost_build::Config::new()
        .protoc_executable(protoc)
        .compile_protos(&protos, &[&root])
        .expect("compile messages/custom/{MarkerDetections,CubePoses}.proto");

    println!("cargo:rerun-if-changed={}", markers.display());
    println!("cargo:rerun-if-changed={}", cubes.display());
}
