//! cube: the marker-cube geometry model and the multi-camera rigid fit.
//!
//! A cube carries five ArUco markers (the sixth face is the mount, left blank):
//! face 0 = TOP, 1 = BACK, 2 = RIGHT, 3 = FRONT, 4 = LEFT. Marker ids pack
//! 5 per cube: cube `k` (0-based) owns ids `5k ..= 5k+4`, so `cube = id / 5`,
//! `face = id % 5`.
//!
//! ## Frames
//!
//! * Marker frame (as ips_detect reports it): origin at the marker centre, +X
//!   right, +Y up, +Z out of the marker face. The wire carries `R_cam_mk`
//!   (quaternion) and `t_cam_mk` (centre) in the camera optical frame (RDF).
//! * Body frame: origin at the cube centre, RDF. Face normals map to the axes as
//!   +X RIGHT, -X LEFT, +Y BACK, -Y FRONT, -Z TOP, +Z BOTTOM (the blank mount
//!   face). The TOP marker sits at `-Z * hz`.
//! * World frame: RDF, metres, the frame the `sbNN_full.yaml` extrinsics put the
//!   cameras in (`world = R * cam + t`).
//!
//! `T_body_mk` is fixed by construction (below). Each detection is lifted into
//! the world frame ([`lift_to_world`]) and paired with its body-frame pose into a
//! [`Candidate`]; [`fit_cube`] then solves the single `world = R * body + t` that
//! best explains every face seen by every camera at once.

use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};
use std::fs;
use std::path::Path;

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// Markers per cube (TOP, BACK, RIGHT, FRONT, LEFT).
pub const FACES_PER_CUBE: i32 = 5;

/// `(cube_id 0-based, face 0..4)` for a marker id. Euclidean div/rem keeps the
/// face in `0..5` and the cube id at `>= 0` even for a negative id.
pub fn cube_and_face(id: i32) -> (u32, i32) {
    let cube = id.div_euclid(FACES_PER_CUBE);
    let face = id.rem_euclid(FACES_PER_CUBE);
    (cube.max(0) as u32, face)
}

// ---- cube geometry model ----------------------------------------------------

/// Half-extents of the cube along the body axes (X toward RIGHT, Y toward BACK,
/// Z toward the BOTTOM mount). For a true cube all three are equal (edge / 2).
pub struct CubeModel {
    pub hx: f64,
    pub hy: f64,
    pub hz: f64,
    /// Marker side length (metres). Sets the lever for the fit's orientation terms.
    pub marker: f64,
}

impl CubeModel {
    /// Build from the physical box dimensions (metres): width along X
    /// (left to right), length along Y (front to back), height along Z (top to
    /// bottom), plus the ArUco marker side length.
    pub fn new(width: f64, length: f64, height: f64, marker: f64) -> Self {
        Self {
            hx: width / 2.0,
            hy: length / 2.0,
            hz: height / 2.0,
            marker,
        }
    }

    /// `T_body_mk` for face `0..4`: the marker frame's pose in the body frame, as
    /// `(R_body_mk, t_body_mk)`. The rotation's columns are the marker's X, Y, Z
    /// axes in the body frame; the translation is the face centre.
    pub fn face_pose(&self, face: i32) -> (Matrix3<f64>, Vector3<f64>) {
        let t = match face {
            0 => Vector3::new(0.0, 0.0, -self.hz), // TOP   at -Z
            1 => Vector3::new(0.0, self.hy, 0.0),  // BACK  at +Y
            2 => Vector3::new(self.hx, 0.0, 0.0),  // RIGHT at +X
            3 => Vector3::new(0.0, -self.hy, 0.0), // FRONT at -Y
            4 => Vector3::new(-self.hx, 0.0, 0.0), // LEFT  at -X
            _ => Vector3::zeros(),
        };
        (face_rotation(face), t)
    }
}

/// `R_body_mk` for each face: columns are the marker's X, Y, Z axes in the body
/// frame. From the cube net: each marker's +Z is its outward face normal, every
/// side marker's up arrow (+Y_mk) points toward the TOP face (-Z_body), and the
/// TOP marker's up arrow folds toward FRONT (-Y_body). All determinants are +1.
fn face_rotation(face: i32) -> Matrix3<f64> {
    #[rustfmt::skip]
    let m = match face {
        // TOP: mkX = +X_body, mkY = -Y_body, mkZ = -Z_body.
        0 => Matrix3::new( 1.0, 0.0,  0.0,
                           0.0,-1.0,  0.0,
                           0.0, 0.0, -1.0),
        // BACK: mkX = +X_body, mkY = -Z_body, mkZ = +Y_body.
        1 => Matrix3::new( 1.0, 0.0, 0.0,
                           0.0, 0.0, 1.0,
                           0.0,-1.0, 0.0),
        // RIGHT: mkX = -Y_body, mkY = -Z_body, mkZ = +X_body.
        2 => Matrix3::new( 0.0, 0.0, 1.0,
                          -1.0, 0.0, 0.0,
                           0.0,-1.0, 0.0),
        // FRONT: mkX = -X_body, mkY = -Z_body, mkZ = -Y_body.
        3 => Matrix3::new(-1.0, 0.0,  0.0,
                           0.0, 0.0, -1.0,
                           0.0,-1.0,  0.0),
        // LEFT: mkX = +Y_body, mkY = -Z_body, mkZ = -X_body.
        4 => Matrix3::new( 0.0, 0.0, -1.0,
                           1.0, 0.0,  0.0,
                           0.0,-1.0,  0.0),
        _ => Matrix3::identity(),
    };
    m
}

// ---- camera extrinsics (sbNN_full.yaml) -------------------------------------

/// One camera's pose in the world frame from `sbNN_full.yaml`:
/// `world = R * cam + t`, camera centre at `t`.
pub struct CameraExtrinsics {
    pub r: Matrix3<f64>,
    pub t: Vector3<f64>,
}

impl CameraExtrinsics {
    /// Parse `R` (3x3) and `t` (3) out of an `sbNN_full.yaml`. The other fields
    /// (K, dist, euler, provenance) are ignored here; the cam_param service
    /// serves the whole file.
    pub fn load(path: &Path) -> Res<Self> {
        #[derive(serde::Deserialize)]
        struct Full {
            #[serde(rename = "R")]
            r: Vec<Vec<f64>>,
            t: Vec<f64>,
        }
        let text = fs::read_to_string(path)
            .map_err(|e| format!("cannot read extrinsics '{}': {e}", path.display()))?;
        let y: Full =
            serde_yaml::from_str(&text).map_err(|e| format!("parse '{}': {e}", path.display()))?;
        if y.r.len() != 3 || y.r.iter().any(|row| row.len() != 3) {
            return Err(format!("'{}': R must be 3x3", path.display()).into());
        }
        if y.t.len() != 3 {
            return Err(format!("'{}': t must have 3 values", path.display()).into());
        }
        #[rustfmt::skip]
        let r = Matrix3::new(
            y.r[0][0], y.r[0][1], y.r[0][2],
            y.r[1][0], y.r[1][1], y.r[1][2],
            y.r[2][0], y.r[2][1], y.r[2][2],
        );
        let t = Vector3::new(y.t[0], y.t[1], y.t[2]);
        Ok(Self { r, t })
    }
}

// ---- per-detection body/world correspondence --------------------------------

/// One marker detection lifted into the world frame, paired with that marker's
/// fixed pose in the body frame. A single correspondence for the rigid fit:
/// ideally `t_world_mk = R_wb * t_body_mk + t_wb`.
pub struct Candidate {
    pub camera: String,
    pub id: i32,
    /// Observed marker rotation in world, `R_world_mk` (= `ext.R * R_cam_mk`).
    pub r_world_mk: Matrix3<f64>,
    /// Observed marker centre in world, `t_world_mk`.
    pub t_world_mk: Vector3<f64>,
    /// Fixed marker rotation in body, `R_body_mk` (from the cube model).
    pub r_body_mk: Matrix3<f64>,
    /// Fixed marker centre in body, `t_body_mk` (the face centre).
    pub t_body_mk: Vector3<f64>,
}

/// Lift a marker pose from the camera optical frame into the world frame with
/// the camera's extrinsics (`world = R * cam + t`). Returns
/// `(R_world_mk, t_world_mk)`.
// STUB-BOUNDARY: module 3
pub fn lift_to_world(
    ext: &CameraExtrinsics,
    r_cam_mk: &Matrix3<f64>,
    t_cam_mk: &Vector3<f64>,
) -> (Matrix3<f64>, Vector3<f64>) {
    let r_world_mk = ext.r * r_cam_mk;
    let t_world_mk = ext.r * t_cam_mk + ext.t;
    (r_world_mk, t_world_mk)
}

/// Build one body/world correspondence from a marker detection. `r_cam_mk` and
/// `t_cam_mk` are the marker's pose in the camera optical frame (off the wire);
/// `ext` lifts it to world. The known `T_body_mk` is stored alongside so
/// [`fit_cube`] can fit the cube pose over all markers at once, rather than
/// backing out one cube pose per marker and averaging (which lever-arms each
/// marker's noisy rotation into the centre).
pub fn candidate_from_marker(
    model: &CubeModel,
    ext: &CameraExtrinsics,
    camera: &str,
    id: i32,
    r_cam_mk: &Matrix3<f64>,
    t_cam_mk: &Vector3<f64>,
) -> Candidate {
    let (_, face) = cube_and_face(id);
    let (r_body_mk, t_body_mk) = model.face_pose(face);
    let (r_world_mk, t_world_mk) = lift_to_world(ext, r_cam_mk, t_cam_mk);
    Candidate {
        camera: camera.to_string(),
        id,
        r_world_mk,
        t_world_mk,
        r_body_mk,
        t_body_mk,
    }
}

// ---- one rigid least-squares fit per cube -----------------------------------

/// The fitted world pose of one cube.
pub struct FusedPose {
    pub r: Matrix3<f64>,
    pub t: Vector3<f64>,
    /// XYZ Euler degrees of `r` (`R = Rx * Ry * Rz`).
    pub euler_deg: [f64; 3],
    /// `r` as a unit quaternion `[x, y, z, w]`.
    pub quat: [f64; 4],
    pub marker_ids: Vec<i32>,
    /// Distinct cameras whose detections survived the reject pass and were fit.
    pub cameras: Vec<String>,
    pub num_cameras: usize,
    /// RMS of the per-marker centre residual of the final fit, metres.
    pub residual_rms: f64,
}

/// Fit one rigid world pose to all of a cube's marker detections by least
/// squares, the rigid (Kabsch/Umeyama) analog of `solvePnP`. Knowing
/// `T_body_mk` for every face, find the single `(R, t)` that best maps the
/// body-frame layout onto the markers observed in world:
///
/// ```text
/// minimise  sum_i | R * body_marker_i + t - world_marker_i |^2
/// ```
///
/// solved in closed form with a 3x3 SVD. Each detection contributes its centre
/// plus three axis tips at `lever` metres along the marker's X, Y, Z, so the
/// same solve honours positions and orientations. With three or more faces the
/// centres (spanning the cube) pin the rotation; a lone marker's axis tips still
/// determine it, so the fit never goes singular.
///
/// `reject = None` (`use_all`) fits every candidate once. `reject =
/// Some((pos_tol_m, ang_tol_deg))` runs the greedy reject pass: while the fit
/// leaves a detection whose centre is more than `pos_tol_m` off, or whose
/// orientation is more than `ang_tol_deg` off (the planar-marker flip), the
/// single worst offender is dropped and the pose refit. One-at-a-time removal
/// stays stable when a gross outlier contaminates the first fit, and never strips
/// below one marker. Returns `None` for an empty input.
// STUB-BOUNDARY: module 3
pub fn fit_cube(cands: &[Candidate], lever: f64, reject: Option<(f64, f64)>) -> Option<FusedPose> {
    /// Closed-form rigid fit (Kabsch, no scale): the `(R, t)` minimising
    /// `sum |R p_i + t - q_i|^2` over the correspondences in `active`. Each
    /// detection expands to four point pairs (centre and three axis tips). The
    /// determinant term keeps R a proper rotation, never a reflection.
    fn kabsch(
        cands: &[Candidate],
        active: &[usize],
        lever: f64,
    ) -> Option<(Matrix3<f64>, Vector3<f64>)> {
        // Body points p (known) and world points q (observed).
        let mut p: Vec<Vector3<f64>> = Vec::with_capacity(active.len() * 4);
        let mut q: Vec<Vector3<f64>> = Vec::with_capacity(active.len() * 4);
        for &i in active {
            let c = &cands[i];
            p.push(c.t_body_mk);
            q.push(c.t_world_mk);
            for k in 0..3 {
                p.push(c.t_body_mk + c.r_body_mk.column(k) * lever);
                q.push(c.t_world_mk + c.r_world_mk.column(k) * lever);
            }
        }

        let n = p.len() as f64;
        let pc = p.iter().fold(Vector3::zeros(), |acc, v| acc + v) / n;
        let qc = q.iter().fold(Vector3::zeros(), |acc, v| acc + v) / n;

        // Cross-covariance H = sum (p_i - pc)(q_i - qc)^T, then
        // R = V * diag(1, 1, det(V U^T)) * U^T.
        let mut h = Matrix3::zeros();
        for (pk, qk) in p.iter().zip(q.iter()) {
            h += (pk - pc) * (qk - qc).transpose();
        }
        let svd = h.svd(true, true);
        let u = svd.u?;
        let v = svd.v_t?.transpose();
        let mut d = Matrix3::identity();
        d[(2, 2)] = (v * u.transpose()).determinant().signum();
        let r = v * d * u.transpose();
        let t = qc - r * pc;
        Some((r, t))
    }

    /// Package a solved `(r, t)` and its `active` correspondences into a
    /// [`FusedPose`]: centre RMS, sorted unique marker ids, distinct cameras.
    fn finalize(
        cands: &[Candidate],
        active: &[usize],
        r: Matrix3<f64>,
        t: Vector3<f64>,
    ) -> FusedPose {
        let sse: f64 = active
            .iter()
            .map(|&i| (r * cands[i].t_body_mk + t - cands[i].t_world_mk).norm_squared())
            .sum();
        let rms = (sse / active.len() as f64).sqrt();

        let mut ids: Vec<i32> = active.iter().map(|&i| cands[i].id).collect();
        ids.sort_unstable();
        ids.dedup();
        let mut cams: Vec<String> = active.iter().map(|&i| cands[i].camera.clone()).collect();
        cams.sort_unstable();
        cams.dedup();

        let co = unit_quat(&r).coords; // [x, y, z, w]
        FusedPose {
            r,
            t,
            euler_deg: mat_to_euler_xyz_deg(&r),
            quat: [co[0], co[1], co[2], co[3]],
            marker_ids: ids,
            num_cameras: cams.len(),
            cameras: cams,
            residual_rms: rms,
        }
    }

    /// Geodesic angle (radians) between two rotation matrices.
    fn rotation_angle(a: &Matrix3<f64>, b: &Matrix3<f64>) -> f64 {
        unit_quat(a).angle_to(&unit_quat(b))
    }

    if cands.is_empty() {
        return None;
    }
    let mut active: Vec<usize> = (0..cands.len()).collect();
    let (mut r, mut t) = kabsch(cands, &active, lever)?;

    if let Some((pos_tol, ang_tol_deg)) = reject {
        let ang_tol = ang_tol_deg.to_radians();
        // Drop the worst tolerance-busting marker, refit, repeat. `score > 1`
        // means the marker breaks the position or the orientation tolerance
        // (whichever by more).
        while active.len() > 1 {
            let mut worst: Option<(usize, f64)> = None; // (slot in `active`, score)
            for (slot, &i) in active.iter().enumerate() {
                let pos_res = (r * cands[i].t_body_mk + t - cands[i].t_world_mk).norm();
                let ang_res = rotation_angle(&(r * cands[i].r_body_mk), &cands[i].r_world_mk);
                let score = (pos_res / pos_tol).max(ang_res / ang_tol);
                if score > worst.map_or(0.0, |(_, s)| s) {
                    worst = Some((slot, score));
                }
            }
            match worst {
                Some((slot, score)) if score > 1.0 => {
                    active.remove(slot);
                    match kabsch(cands, &active, lever) {
                        Some((r2, t2)) => {
                            r = r2;
                            t = t2;
                        }
                        None => break,
                    }
                }
                _ => break,
            }
        }
    }

    Some(finalize(cands, &active, r, t))
}

// ---- rotation helpers -------------------------------------------------------

/// Build a unit quaternion from `(x, y, z, w)` and return its rotation matrix.
pub fn quat_to_mat(x: f64, y: f64, z: f64, w: f64) -> Matrix3<f64> {
    UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
        .to_rotation_matrix()
        .into_inner()
}

/// A rotation matrix as a unit quaternion (the matrix is trusted to be a proper
/// rotation).
pub fn unit_quat(r: &Matrix3<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(*r))
}

/// XYZ Euler degrees (`R = Rx * Ry * Rz`) of a rotation matrix, the same
/// convention as ips_detect's `euler_deg` and the `euler_xyz_deg` of the
/// extrinsics yaml.
pub fn mat_to_euler_xyz_deg(r: &Matrix3<f64>) -> [f64; 3] {
    let m = |i: usize, j: usize| r[(i, j)];
    let sy = m(0, 2).clamp(-1.0, 1.0);
    let ry = sy.asin();
    let cy = (1.0 - sy * sy).sqrt();
    let (rx, rz) = if cy > 1e-6 {
        ((-m(1, 2)).atan2(m(2, 2)), (-m(0, 1)).atan2(m(0, 0)))
    } else if sy > 0.0 {
        (m(1, 0).atan2(m(1, 1)), 0.0)
    } else {
        ((-m(1, 0)).atan2(m(1, 1)), 0.0)
    };
    [rx.to_degrees(), ry.to_degrees(), rz.to_degrees()]
}
