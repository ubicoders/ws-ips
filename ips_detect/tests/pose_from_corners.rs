//! Acceptance test for `markers::pose_from_corners` (module 2).
//!
//! Synthesises marker poses in the camera optical frame, projects the four
//! marker corners into the image with `calib3d::project_points` and the real
//! sb00 Kalibr intrinsics (including distortion), recovers the pose from those
//! corners, and checks it against the truth.
//!
//! There is no recorded-frame test yet: `docs/data` is empty. When captures of
//! the calib board land there, add a test that detects the board in a frame and
//! checks the known tag distances.

use ips_detect::intrinsics::{self, Intrinsics};
use ips_detect::markers::pose_from_corners;
use opencv::core::{Mat, Point2f, Point3f, Vec3d, Vector};
use opencv::prelude::*;
use opencv::{aruco, calib3d};
use std::path::Path;

type M3 = [[f64; 3]; 3];

const TVEC_TOL_M: f64 = 1e-3;
const ROT_TOL_DEG: f64 = 0.1;

fn sb00() -> Intrinsics {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../calibration/intrinsics");
    Intrinsics::load(&intrinsics::path_for(&dir, "sb00")).expect("load sb00 camchain")
}

fn rot_x(deg: f64) -> M3 {
    let (s, c) = deg.to_radians().sin_cos();
    [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
}
fn rot_y(deg: f64) -> M3 {
    let (s, c) = deg.to_radians().sin_cos();
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}
fn rot_z(deg: f64) -> M3 {
    let (s, c) = deg.to_radians().sin_cos();
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}
fn mul(a: &M3, b: &M3) -> M3 {
    let mut r = [[0.0; 3]; 3];
    for (i, row) in r.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    r
}

fn rvec_from_matrix(m: &M3) -> Vec3d {
    let src = Mat::from_slice_2d(m).unwrap();
    let mut dst = Mat::default();
    calib3d::rodrigues_def(&src, &mut dst).unwrap();
    Vec3d::from([
        *dst.at::<f64>(0).unwrap(),
        *dst.at::<f64>(1).unwrap(),
        *dst.at::<f64>(2).unwrap(),
    ])
}

fn matrix_from_rvec(r: &Vec3d) -> M3 {
    let mut dst = Mat::default();
    calib3d::rodrigues_def(r, &mut dst).unwrap();
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = *dst.at_2d::<f64>(i as i32, j as i32).unwrap();
        }
    }
    m
}

/// Angle of the rotation `a^T b`, in degrees.
fn rotation_error_deg(a: &M3, b: &M3) -> f64 {
    let trace: f64 = (0..3)
        .map(|i| (0..3).map(|k| a[k][i] * b[k][i]).sum::<f64>())
        .sum();
    ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos().to_degrees()
}

/// Marker corners in ArUco order (top-left, top-right, bottom-right,
/// bottom-left), marker frame X right, Y up, Z out of the face.
fn marker_corners(size: f32) -> Vector<Point3f> {
    let h = size / 2.0;
    Vector::from_slice(&[
        Point3f::new(-h, h, 0.0),
        Point3f::new(h, h, 0.0),
        Point3f::new(h, -h, 0.0),
        Point3f::new(-h, -h, 0.0),
    ])
}

fn project(intr: &Intrinsics, size: f32, rvec: &Vec3d, tvec: &Vec3d) -> Vector<Point2f> {
    let mut img: Vector<Point2f> = Vector::new();
    calib3d::project_points_def(
        &marker_corners(size),
        rvec,
        tvec,
        &intr.camera_matrix,
        &intr.dist_coeffs,
        &mut img,
    )
    .unwrap();
    img
}

/// Project a known pose, recover it, and check both halves.
fn round_trip(name: &str, r_true: M3, t_true: [f64; 3], size: f32) {
    let intr = sb00();
    let rvec = rvec_from_matrix(&r_true);
    let tvec = Vec3d::from(t_true);
    let corners = project(&intr, size, &rvec, &tvec);
    for p in corners.iter() {
        assert!(
            p.x >= 0.0 && p.y >= 0.0 && p.x < intr.width as f32 && p.y < intr.height as f32,
            "{name}: corner {p:?} falls outside the image, fix the test pose"
        );
    }

    let (r_est, t_est) = pose_from_corners(&corners, &intr, size).expect("pose_from_corners");

    let dt = ((t_est[0] - t_true[0]).powi(2)
        + (t_est[1] - t_true[1]).powi(2)
        + (t_est[2] - t_true[2]).powi(2))
    .sqrt();
    let dr = rotation_error_deg(&r_true, &matrix_from_rvec(&r_est));
    assert!(
        dt < TVEC_TOL_M,
        "{name}: tvec off by {dt:.6} m (got {t_est:?}, want {t_true:?})"
    );
    assert!(dr < ROT_TOL_DEG, "{name}: rotation off by {dr:.4} deg");
}

// A marker facing the camera head-on has its Z axis pointing back at the
// camera and its Y axis up in the image, which is a 180 deg turn about camera X.
fn facing() -> M3 {
    rot_x(180.0)
}

#[test]
fn frontal_on_axis() {
    round_trip("frontal", facing(), [0.0, 0.0, 2.0], 0.087);
}

#[test]
fn tilted_compound() {
    let r = mul(
        &mul(&facing(), &rot_x(30.0)),
        &mul(&rot_y(-25.0), &rot_z(60.0)),
    );
    round_trip("tilt-compound", r, [0.5, 0.4, 3.0], 0.087);
}

#[test]
fn wrong_corner_count_is_an_error() {
    let intr = sb00();
    let three: Vector<Point2f> = Vector::from_slice(&[
        Point2f::new(600.0, 500.0),
        Point2f::new(650.0, 500.0),
        Point2f::new(650.0, 550.0),
    ]);
    assert!(pose_from_corners(&three, &intr, 0.087).is_err());
}

/// Same answer as the OLD detector, which called
/// `aruco::estimate_pose_single_markers`: the frame convention downstream
/// (ips_calib, ips_track) depends on it.
#[test]
fn matches_aruco_estimate_pose_single_markers() {
    let intr = sb00();
    let size = 0.087_f32;
    let r = mul(&mul(&facing(), &rot_x(20.0)), &rot_y(15.0));
    let corners = project(
        &intr,
        size,
        &rvec_from_matrix(&r),
        &Vec3d::from([0.2, -0.1, 2.2]),
    );

    let mut all: Vector<Vector<Point2f>> = Vector::new();
    all.push(corners.clone());
    let mut rvecs: Vector<Vec3d> = Vector::new();
    let mut tvecs: Vector<Vec3d> = Vector::new();
    aruco::estimate_pose_single_markers_def(
        &all,
        size,
        &intr.camera_matrix,
        &intr.dist_coeffs,
        &mut rvecs,
        &mut tvecs,
    )
    .unwrap();
    let (r_ref, t_ref) = (rvecs.get(0).unwrap(), tvecs.get(0).unwrap());

    let (r_est, t_est) = pose_from_corners(&corners, &intr, size).expect("pose_from_corners");
    for k in 0..3 {
        assert!(
            (t_est[k] - t_ref[k]).abs() < 1e-6,
            "tvec {t_est:?} vs aruco {t_ref:?}"
        );
    }
    let dr = rotation_error_deg(&matrix_from_rvec(&r_ref), &matrix_from_rvec(&r_est));
    assert!(dr < 1e-3, "rotation differs from aruco by {dr} deg");
}
