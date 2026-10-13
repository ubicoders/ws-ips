//! `fit_cube` on synthetic, noise-free observations of a known cube pose.
//!
//! `docs/data` holds no recorded zenoh capture yet, so these tests are synthetic
//! only; the recorded-capture test named in the README is still to come.

use ips_track::cube::{fit_cube, Candidate, CubeModel};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

fn model() -> CubeModel {
    CubeModel::new(0.1, 0.1, 0.1, 0.05)
}

/// Geodesic angle (radians) between two rotation matrices.
fn angle_between(a: &Matrix3<f64>, b: &Matrix3<f64>) -> f64 {
    let qa = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(*a));
    let qb = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(*b));
    qa.angle_to(&qb)
}

/// The exact world marker pose a face shows for a known cube pose
/// `(r_true, t_true)`: the noise-free observation the fit must invert.
fn obs(model: &CubeModel, face: i32, r_true: &Matrix3<f64>, t_true: &Vector3<f64>) -> Candidate {
    let (r_body, t_body) = model.face_pose(face);
    Candidate {
        camera: format!("cam{face}"),
        id: face,
        r_world_mk: r_true * r_body,
        t_world_mk: r_true * t_body + t_true,
        r_body_mk: r_body,
        t_body_mk: t_body,
    }
}

#[test]
fn fit_recovers_known_pose_from_all_faces() {
    let model = model();
    let r_true = *Rotation3::from_euler_angles(0.1, -0.2, 0.5).matrix();
    let t_true = Vector3::new(0.12, -0.34, -0.80);
    let cands: Vec<Candidate> = (0..5).map(|f| obs(&model, f, &r_true, &t_true)).collect();

    // Both the reject pass and use_all must land on the truth for clean data.
    for reject in [Some((0.04, 35.0)), None] {
        let f = fit_cube(&cands, model.marker * 0.5, reject).expect("fit");
        assert!(
            (f.t - t_true).norm() < 1e-6,
            "t off ({reject:?}): {:?}",
            f.t
        );
        assert!(angle_between(&f.r, &r_true) < 1e-6, "R off ({reject:?})");
        assert!(f.residual_rms < 1e-6, "rms ({reject:?}) {}", f.residual_rms);
        assert_eq!(f.num_cameras, 5);
        assert_eq!(f.marker_ids, vec![0, 1, 2, 3, 4]);
    }
}

#[test]
fn single_marker_is_well_posed() {
    let model = model();
    let r_true = *Rotation3::from_euler_angles(-0.3, 0.2, 1.0).matrix();
    let t_true = Vector3::new(-0.05, 0.2, -0.7);
    let cands = vec![obs(&model, 2, &r_true, &t_true)];

    for reject in [Some((0.04, 35.0)), None] {
        let f = fit_cube(&cands, model.marker * 0.5, reject).expect("fit");
        assert!(
            (f.t - t_true).norm() < 1e-6,
            "t off ({reject:?}): {:?}",
            f.t
        );
        assert!(angle_between(&f.r, &r_true) < 1e-6, "R off ({reject:?})");
    }
}

#[test]
fn rejects_a_gross_depth_outlier() {
    let model = model();
    let r_true = *Rotation3::from_euler_angles(0.0, 0.0, 0.3).matrix();
    let t_true = Vector3::new(0.1, 0.1, -0.8);
    let mut cands: Vec<Candidate> = (0..4).map(|f| obs(&model, f, &r_true, &t_true)).collect();
    // Face 4: a single marker whose world centre is 1 m off (bad PnP depth).
    let mut bad = obs(&model, 4, &r_true, &t_true);
    bad.t_world_mk += Vector3::new(1.0, 0.0, 0.0);
    cands.push(bad);

    let f = fit_cube(&cands, model.marker * 0.5, Some((0.04, 35.0))).expect("fit");
    assert!(
        (f.t - t_true).norm() < 1e-6,
        "outlier leaked into t: {:?}",
        f.t
    );
    assert!(
        !f.marker_ids.contains(&4),
        "outlier not rejected: {:?}",
        f.marker_ids
    );
    assert_eq!(f.num_cameras, 4);
}
