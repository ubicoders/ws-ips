//! Rigid least-squares fit of a camera's pose from paired marker centres.
//!
//! `cam[i]` is a board marker's centre in the camera optical frame (RDF, metres,
//! the median of what the detector reported), `world[i]` the same marker's centre
//! from the board model. The camera pose is the rigid transform with
//! `world = R * cam + t` (camera to world), so `t` is the camera centre in world.

use nalgebra::{Matrix3, Vector3};

/// Kabsch / Umeyama without scale: the `(R, t)` minimising
/// `sum |world[i] - (R * cam[i] + t)|^2`, with `R` a proper rotation (det +1).
///
/// Returns `None` when the slices differ in length, hold fewer than three pairs,
/// or the camera points are collinear (the rotation about that line is then
/// unobservable).
// STUB-BOUNDARY: module 2
pub fn fit_extrinsics(
    cam: &[Vector3<f64>],
    world: &[Vector3<f64>],
) -> Option<(Matrix3<f64>, Vector3<f64>)> {
    if cam.len() != world.len() || cam.len() < 3 {
        return None;
    }
    let n = cam.len() as f64;
    let cc = cam.iter().fold(Vector3::zeros(), |a, p| a + p) / n;
    let cw = world.iter().fold(Vector3::zeros(), |a, p| a + p) / n;

    // Cross-covariance of the centred sets.
    let mut h = Matrix3::zeros();
    for (pc, pw) in cam.iter().zip(world.iter()) {
        h += (pc - cc) * (pw - cw).transpose();
    }

    let svd = h.svd(true, true);

    // Degenerate geometry: all points equal, or all on one line. A plane (the
    // board) is fine: one singular value is zero and the reflection fix below
    // picks the proper rotation.
    let mut s: Vec<f64> = svd.singular_values.iter().copied().collect();
    s.sort_by(|a, b| b.total_cmp(a));
    if !s[0].is_finite() || s[0] <= 0.0 || s[1] <= 1e-12 * s[0] {
        return None;
    }

    let u = svd.u?;
    let v = svd.v_t?.transpose();
    let ut = u.transpose();
    // Reflection fix: if V * U^T is a reflection, flip the axis of the smallest
    // singular value so R is a proper rotation.
    let d = (v * ut).determinant().signum();
    let r = v * Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, d)) * ut;
    let t = cw - r * cc;
    if r.iter().chain(t.iter()).all(|x| x.is_finite()) {
        Some((r, t))
    } else {
        None
    }
}

/// RMS of `|world[i] - (R * cam[i] + t)|` over the pairs, metres. Pairs by index;
/// extra entries in the longer slice are ignored. Zero for no pairs.
// STUB-BOUNDARY: module 2
pub fn reprojection_residual(
    cam: &[Vector3<f64>],
    world: &[Vector3<f64>],
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
) -> f64 {
    let n = cam.len().min(world.len());
    if n == 0 {
        return 0.0;
    }
    let sse: f64 = cam
        .iter()
        .zip(world.iter())
        .map(|(pc, pw)| (pw - (r * pc + t)).norm_squared())
        .sum();
    (sse / n as f64).sqrt()
}
