//! cam_param decoding tests. A rerun viewer and the live pipeline are not
//! available here, so these cover the JSON the cam_param service replies with.

use ips_viz::visualize::CameraExtrinsics;
use serde_json::{json, Value};

/// R and t of `../../calibration/extrinsics/sb00_full.yaml` at the time of writing.
#[rustfmt::skip]
const SB00_R: [[f64; 3]; 3] = [
    [0.774544063, -0.220794956, 0.592731880],
    [0.632253133, 0.243041344, -0.735654049],
    [0.018370351, 0.944553065, 0.327844533],
];
const SB00_T: [f64; 3] = [-1.575608, 2.711741, -1.763480];

/// The reply ips_track's cam_param service sends for sb00: the yaml as JSON
/// plus the 3x4 `Rt = [R | t]` it adds.
fn sb00_reply() -> Value {
    let rt: Vec<Vec<f64>> = (0..3)
        .map(|i| {
            let mut row = SB00_R[i].to_vec();
            row.push(SB00_T[i]);
            row
        })
        .collect();
    json!({
        "camera": "sb00",
        "world_frame": "RDF",
        "units": "m",
        "K": [
            [873.166931, 0.0, 652.784546],
            [0.0, 872.446797, 491.361374],
            [0.0, 0.0, 1.0]
        ],
        "dist": [-0.09405738, 0.09031716, -0.00001179, 0.00057490],
        "R": SB00_R,
        "euler_xyz_deg": [65.9798, 36.3511, 15.9110],
        "t": SB00_T,
        "residual_rms_m": 0.038748,
        "markers_used": [0, 1, 2, 3, 4, 5],
        "frames": 116,
        "Rt": rt,
    })
}

#[test]
fn extrinsics_from_cam_param_reply() {
    let e = CameraExtrinsics::from_json(&sb00_reply(), "sb00").expect("decode sb00 reply");
    for (i, row) in SB00_R.iter().enumerate() {
        for (j, &v) in row.iter().enumerate() {
            assert_eq!(e.r[(i, j)], v, "R[{i}][{j}]");
        }
    }
    for (i, &v) in SB00_T.iter().enumerate() {
        assert_eq!(e.t[i], v, "t[{i}]");
    }
    // A calibrated rotation: orthonormal, det +1.
    assert!((e.r.determinant() - 1.0).abs() < 1e-6);
    assert!((e.r.transpose() * e.r - nalgebra::Matrix3::identity()).norm() < 1e-6);
}

#[test]
fn extrinsics_service_error_reply() {
    // An unknown camera is answered with a normal `{ "error": .. }` object.
    let doc = json!({
        "error": "no params for 'sb07': No such file or directory (os error 2)",
        "available": ["sb00", "sb01", "sb02", "sb03"],
    });
    let err = CameraExtrinsics::from_json(&doc, "sb07")
        .unwrap_err()
        .to_string();
    assert!(
        err.starts_with("sb07: service: no params for 'sb07'"),
        "{err}"
    );
}
