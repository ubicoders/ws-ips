//! Generate the prost types for the CubePoses payload from the repository's
//! `messages/` folder.
//!
//! `CubePoses.proto` imports `custom/MarkerDetections.proto` (for `Vec3` and
//! `Quat`), so both are compiled with `messages/` as the include root. Both are
//! `package swarmbotix.custom`, so prost emits one `$OUT_DIR/swarmbotix.custom.rs`,
//! which `lib.rs` includes as the `msgs` module. protoc comes from
//! `protoc-bin-vendored`, so the build needs no system protoc.

use std::path::PathBuf;

fn main() {
    let protoc = protoc_bin_vendored::protoc_bin_path().expect("vendored protoc binary");
    std::env::set_var("PROTOC", protoc);

    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let root = manifest.join("..").join("messages");
    let markers = root.join("custom/MarkerDetections.proto");
    let cubes = root.join("custom/CubePoses.proto");

    for f in [&markers, &cubes] {
        assert!(
            f.exists(),
            "proto not found: {}. ips_viz compiles the repository's messages/custom/*.proto; \
             run the build from a full ws-ips checkout (include root = {})",
            f.display(),
            root.display()
        );
    }

    prost_build::compile_protos(&[&markers, &cubes], &[&root])
        .expect("compile messages/custom/{MarkerDetections,CubePoses}.proto");

    println!("cargo:rerun-if-changed={}", markers.display());
    println!("cargo:rerun-if-changed={}", cubes.display());
}
