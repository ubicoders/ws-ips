//! Generate the prost types for the IPS payloads from the repository's
//! `messages/` folder (`custom/MarkerDetections.proto`, `custom/CubePoses.proto`).
//!
//! Both files are `package swarmbotix.custom`, so prost emits one
//! `$OUT_DIR/swarmbotix.custom.rs`, which `lib.rs` includes as `msgs`.
//! protoc comes from `protoc-bin-vendored`.

use std::path::PathBuf;

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let joined = manifest.join("../..").join("messages");
    let root = joined.canonicalize().unwrap_or_else(|e| {
        panic!(
            "messages folder not found at {} ({e}); ips_track must sit in modules/ of the ws-ips repository, next to messages/",
            joined.display()
        )
    });

    let markers = root.join("custom").join("MarkerDetections.proto");
    let cubes = root.join("custom").join("CubePoses.proto");
    for f in [&markers, &cubes] {
        assert!(
            f.exists(),
            "proto not found: {} (expected under the repository's messages/custom/)",
            f.display()
        );
    }

    let protoc = protoc_bin_vendored::protoc_bin_path().expect("vendored protoc binary");

    // CubePoses.proto imports "custom/MarkerDetections.proto", so the messages
    // root is the include path.
    prost_build::Config::new()
        .protoc_executable(protoc)
        .compile_protos(&[&markers, &cubes], &[&root])
        .expect("compile messages/custom/{MarkerDetections,CubePoses}.proto");

    println!("cargo:rerun-if-changed={}", markers.display());
    println!("cargo:rerun-if-changed={}", cubes.display());
}
