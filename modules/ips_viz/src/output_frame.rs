//! The tracker's output axis convention, undone for drawing.
//!
//! ips_track relabels the axes of the poses it publishes according to its
//! `parameters/output_frame.yaml` and names the result in `CubePoses.frame`
//! (`RDF` = the board frame, `FRD`, `FRU`; see `ips_track::output_frame`, the
//! authority on the matrices). The camera poses that ips_viz draws come from the
//! cam_param service in the board frame, so this module reads the name off every
//! message and maps the cube poses back into the board frame: rerun shows the
//! same scene whatever the convention on the wire.
//!
//! `C` holds the output axes as rows in board coordinates;
//! `p_board = C^T p_out`, `R_board = C^T R_out C`. Kept in step with the tracker.

use crate::msgs::{CubePose, Mat3, Quat, Vec3};
use nalgebra::{Matrix3, Quaternion, Rotation3, UnitQuaternion, Vector3};

const FORWARD: [f64; 3] = [0.0, -1.0, 0.0];
const RIGHT: [f64; 3] = [1.0, 0.0, 0.0];
const DOWN: [f64; 3] = [0.0, 0.0, 1.0];
const UP: [f64; 3] = [0.0, 0.0, -1.0];

/// The tracker's output convention and the matrix that undoes it.
#[derive(Debug, Clone)]
pub struct OutputFrame {
    pub name: String,
    /// `C^T`: output -> board.
    ct: Matrix3<f64>,
    identity: bool,
}

impl OutputFrame {
    /// From the `frame` field of a received `CubePoses`. An empty name is a
    /// publisher that predates the field and means the board frame.
    pub fn from_name(frame: &str) -> Result<Self, String> {
        let rows = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| {
            Matrix3::new(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2])
        };
        let name = frame.trim().to_ascii_uppercase();
        let name = if name.is_empty() { "RDF".to_string() } else { name };
        let c = match name.as_str() {
            "RDF" => Matrix3::identity(),
            "FRD" => rows(FORWARD, RIGHT, DOWN),
            "FRU" => rows(FORWARD, RIGHT, UP),
            other => {
                return Err(format!(
                    "cube_poses: unknown frame '{other}' (expected RDF, FRD or FRU)"
                ))
            }
        };
        Ok(Self {
            identity: name == "RDF",
            name,
            ct: c.transpose(),
        })
    }

    pub fn is_identity(&self) -> bool {
        self.identity
    }

    /// Whether this was built from `frame` (same name after trimming and
    /// upper-casing; empty = RDF).
    pub fn matches(&self, frame: &str) -> bool {
        let n = frame.trim().to_ascii_uppercase();
        if n.is_empty() { self.name == "RDF" } else { self.name == n }
    }

    /// The received cubes with position, rotation, euler, quaternion and velocity
    /// back in the board frame. `RDF` returns them unchanged.
    pub fn to_board(&self, cubes: &[CubePose]) -> Vec<CubePose> {
        if self.identity {
            return cubes.to_vec();
        }
        cubes.iter().map(|c| self.cube_to_board(c)).collect()
    }

    fn vec(&self, v: &Vec3) -> Vec3 {
        let b = self.ct * Vector3::new(v.x, v.y, v.z);
        Vec3 {
            x: b[0],
            y: b[1],
            z: b[2],
        }
    }

    fn cube_to_board(&self, c: &CubePose) -> CubePose {
        let mut out = c.clone();
        if !c.alive {
            return out; // pose fields are cleared on heartbeats
        }
        out.position = c.position.as_ref().map(|p| self.vec(p));
        out.velocity = c.velocity.as_ref().map(|v| self.vec(v));
        // The rotation: prefer the matrix, fall back to the quaternion.
        let r_out = match (&c.rotation, &c.orientation) {
            (Some(m), _) if m.m.len() == 9 => Some(Matrix3::from_row_slice(&m.m)),
            (_, Some(q)) => Some(
                UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z))
                    .to_rotation_matrix()
                    .into_inner(),
            ),
            _ => None,
        };
        if let Some(r_out) = r_out {
            let r = self.ct * r_out * self.ct.transpose();
            let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(r));
            let e = mat_to_euler_xyz_deg(&r);
            out.rotation = Some(Mat3 {
                m: r.transpose().as_slice().to_vec(), // row-major
            });
            out.orientation = Some(Quat {
                x: q.coords[0],
                y: q.coords[1],
                z: q.coords[2],
                w: q.coords[3],
            });
            out.euler_deg = Some(Vec3 {
                x: e[0],
                y: e[1],
                z: e[2],
            });
        }
        out
    }
}

/// XYZ Euler degrees (`R = Rx * Ry * Rz`), the tracker's convention.
fn mat_to_euler_xyz_deg(r: &Matrix3<f64>) -> [f64; 3] {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(p: [f64; 3], r: Matrix3<f64>) -> CubePose {
        let q = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(r));
        CubePose {
            cube_id: 1,
            alive: true,
            position: Some(Vec3 { x: p[0], y: p[1], z: p[2] }),
            rotation: Some(Mat3 { m: r.transpose().as_slice().to_vec() }),
            orientation: Some(Quat { x: q.coords[0], y: q.coords[1], z: q.coords[2], w: q.coords[3] }),
            velocity: Some(Vec3 { x: 1.0, y: 2.0, z: 3.0 }),
            ..Default::default()
        }
    }

    #[test]
    fn rdf_is_passthrough() {
        let f = OutputFrame::from_name("RDF").unwrap();
        assert!(f.is_identity());
        let c = pose([1.0, 2.0, 3.0], Matrix3::identity());
        let b = f.to_board(&[c.clone()]);
        assert_eq!(b[0].position, c.position);
    }

    #[test]
    fn frd_and_fru_undo_the_tracker_relabel() {
        // Board-frame truth: cube 1 m toward the top row, 0.5 m up, yawed 30 deg.
        let p_board = Vector3::new(0.0, -1.0, -0.5);
        let r_board = Rotation3::from_euler_angles(0.2, -0.4, 0.5).into_inner();
        for (name, c) in [
            ("FRD", Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
            ("FRU", Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0)),
        ] {
            // What the tracker publishes for that truth.
            let p_out = c * p_board;
            let r_out = c * r_board * c.transpose();
            let f = OutputFrame::from_name(name).unwrap();
            let b = &f.to_board(&[pose(p_out.into(), r_out)])[0];
            let p = b.position.as_ref().unwrap();
            assert!((Vector3::new(p.x, p.y, p.z) - p_board).norm() < 1e-12, "{name} pos");
            let m = Matrix3::from_row_slice(&b.rotation.as_ref().unwrap().m);
            assert!((m - r_board).norm() < 1e-12, "{name} rot");
            let q = b.orientation.as_ref().unwrap();
            let rq = UnitQuaternion::from_quaternion(Quaternion::new(q.w, q.x, q.y, q.z))
                .to_rotation_matrix()
                .into_inner();
            assert!((rq - r_board).norm() < 1e-12, "{name} quat");
            let v = b.velocity.as_ref().unwrap();
            let vb = c.transpose() * Vector3::new(1.0, 2.0, 3.0);
            assert!((Vector3::new(v.x, v.y, v.z) - vb).norm() < 1e-12, "{name} vel");
        }
    }

    #[test]
    fn heartbeat_untouched_and_unknown_rejected() {
        let f = OutputFrame::from_name("frd").unwrap();
        let hb = CubePose { cube_id: 2, alive: false, ..Default::default() };
        assert_eq!(f.to_board(&[hb.clone()])[0], hb);
        assert!(OutputFrame::from_name("ENU").is_err());
        assert!(OutputFrame::from_name("").unwrap().is_identity());
    }
}
