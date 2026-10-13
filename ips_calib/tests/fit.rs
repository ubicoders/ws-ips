//! Acceptance tests for `fit_extrinsics` and `reprojection_residual` (module 2),
//! plus the board model they are fitted against.
//!
//! All point sets are synthetic. `docs/data` holds no recorded board detections
//! yet, so there is no recorded-stream test; add one here when a capture lands.

use ips_calib::board::{self, CalibConfig};
use ips_calib::fit::{fit_extrinsics, reprojection_residual};
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use std::path::Path;

/// xorshift64*: deterministic, dependency-free random numbers.
struct Rng(u64);

impl Rng {
    fn next_f64(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        let x = self.0.wrapping_mul(0x2545_f491_4f6c_dd1d);
        (x >> 11) as f64 / (1u64 << 53) as f64
    }
    /// Uniform in `[lo, hi)`.
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }
    fn vec3(&mut self, half: f64) -> Vector3<f64> {
        Vector3::new(
            self.range(-half, half),
            self.range(-half, half),
            self.range(-half, half),
        )
    }
    fn rotation(&mut self) -> Matrix3<f64> {
        let q = Quaternion::new(
            self.range(-1.0, 1.0),
            self.range(-1.0, 1.0),
            self.range(-1.0, 1.0),
            self.range(-1.0, 1.0),
        );
        UnitQuaternion::from_quaternion(q)
            .to_rotation_matrix()
            .into_inner()
    }
}

/// Rejects nearly collinear sets. Their rotation about the line is barely
/// observable, so no fit recovers it to 1e-9; that is the data, not the solver.
fn well_spread(pts: &[Vector3<f64>]) -> bool {
    let c = pts.iter().fold(Vector3::zeros(), |a, p| a + p) / pts.len() as f64;
    let cov = pts
        .iter()
        .fold(Matrix3::zeros(), |a, p| a + (p - c) * (p - c).transpose());
    let mut ev: Vec<f64> = cov.symmetric_eigenvalues().iter().copied().collect();
    ev.sort_by(|a, b| b.total_cmp(a));
    ev[1] > 1e-2 * ev[0]
}

/// `n` points from `gen`, redrawn until [`well_spread`].
fn draw(rng: &mut Rng, n: usize, gen: fn(&mut Rng) -> Vector3<f64>) -> Vec<Vector3<f64>> {
    loop {
        let pts: Vec<Vector3<f64>> = (0..n).map(|_| gen(rng)).collect();
        if well_spread(&pts) {
            return pts;
        }
    }
}

fn apply(r: &Matrix3<f64>, t: &Vector3<f64>, pts: &[Vector3<f64>]) -> Vec<Vector3<f64>> {
    pts.iter().map(|p| r * p + t).collect()
}

fn max_abs(m: impl IntoIterator<Item = f64>) -> f64 {
    m.into_iter().fold(0.0, |a, x| a.max(x.abs()))
}

fn assert_recovers(cam: &[Vector3<f64>], r_true: &Matrix3<f64>, t_true: &Vector3<f64>) {
    let world = apply(r_true, t_true, cam);
    let (r, t) = fit_extrinsics(cam, &world).expect("a fit");
    let dr = max_abs((r - r_true).iter().copied());
    let dt = max_abs((t - t_true).iter().copied());
    assert!(dr < 1e-9, "R off by {dr:e} ({} points)", cam.len());
    assert!(dt < 1e-9, "t off by {dt:e} ({} points)", cam.len());
    let rms = reprojection_residual(cam, &world, &r, &t);
    assert!(rms < 1e-9, "residual {rms:e}");
}

#[test]
fn recovers_random_rigid_transforms() {
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    for trial in 0..200 {
        let n = 3 + trial % 28;
        let cam = draw(&mut rng, n, |g| g.vec3(2.0));
        let r = rng.rotation();
        let t = rng.vec3(5.0);
        assert_recovers(&cam, &r, &t);
    }
}

#[test]
fn reflected_set_still_gives_a_proper_rotation() {
    let mut rng = Rng(7);
    let mirror = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0));
    for _ in 0..50 {
        let cam = draw(&mut rng, 8, |g| g.vec3(1.0));
        let m = rng.rotation() * mirror;
        let world = apply(&m, &rng.vec3(3.0), &cam);
        let (r, t) = fit_extrinsics(&cam, &world).expect("a fit");
        assert!(
            (r.determinant() - 1.0).abs() < 1e-9,
            "det {}",
            r.determinant()
        );
        let ortho = max_abs((r.transpose() * r - Matrix3::identity()).iter().copied());
        assert!(ortho < 1e-9, "R^T R off identity by {ortho:e}");
        assert!(reprojection_residual(&cam, &world, &r, &t).is_finite());
    }
}

#[test]
fn fewer_than_three_points_is_none() {
    let p = [Vector3::new(0.1, 0.2, 1.0), Vector3::new(0.3, -0.2, 1.5)];
    assert!(fit_extrinsics(&[], &[]).is_none());
    assert!(fit_extrinsics(&p[..1], &p[..1]).is_none());
    assert!(fit_extrinsics(&p, &p).is_none());
    let q = [p[0], p[1], Vector3::new(1.0, 1.0, 2.0)];
    assert!(fit_extrinsics(&q, &q[..2]).is_none(), "mismatched lengths");
}

fn shipped_board() -> CalibConfig {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../calibration/ips_calib.yaml");
    CalibConfig::load(&path).expect("calibration/ips_calib.yaml")
}

#[test]
fn board_model_places_origin_and_pitch() {
    let cfg = shipped_board();
    let world = board::build_world_model(&cfg).unwrap();
    let origin = Vector3::from(cfg.placement.position_mm) / 1000.0;
    let rot = board::euler_xyz_deg_to_mat(cfg.placement.orientation_deg);
    let pitch = cfg.pitch_m();
    assert!((pitch - 0.215).abs() < 1e-12, "185 mm tag + 30 mm gap");

    let id1 = world[&1];
    assert!(
        (id1 - origin).norm() < 1e-12,
        "id 1 at {id1:?}, want {origin:?}"
    );
    let id2 = world[&2];
    let want = origin + rot * Vector3::new(pitch, 0.0, 0.0);
    assert!(
        (id2 - want).norm() < 1e-12,
        "id 2 at {id2:?}, want {want:?}"
    );
    // Rows grow downward (+y), so id 4 sits one pitch below id 1.
    let id4 = world[&4];
    let want = origin + rot * Vector3::new(0.0, pitch, 0.0);
    assert!(
        (id4 - want).norm() < 1e-12,
        "id 4 at {id4:?}, want {want:?}"
    );
    assert_eq!(world.len(), 6);
}
