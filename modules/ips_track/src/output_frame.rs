//! Output axis convention of the published cube poses
//! (`parameters/output_frame.yaml`).
//!
//! Everything in the tracker is computed in the board frame, the RDF frame that
//! calibration defines (x along the board's top row, y toward its bottom row, z
//! down into the floor). This module relabels the axes of the finished pose right
//! before it goes on the wire, so that a consumer can get, say, PX4's FRD
//! directly. `RDF` is the identity and the default.
//!
//! The matrix `C` holds the output axes as rows, as unit vectors in the board
//! frame. The same relabelling is applied to the world side and to the cube body
//! side (FRONT = forward, RIGHT = right, BOTTOM = down), so
//!
//! ```text
//! p_out = C * p_board           R_out = C * R_board * C^T
//! ```
//!
//! and `R_out` is a proper rotation for every convention, including the
//! left-handed FRU (`det(C) = -1` cancels in the conjugation). ips_viz applies
//! the inverse to draw every convention identically.

use nalgebra::{Matrix3, Vector3};
use std::path::Path;

/// The named conventions. Directions in the board frame: forward = -y (toward
/// the board's top row), right = +x, down = +z, up = -z.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Convention {
    /// right, down, forward: the board frame itself. No conversion.
    Rdf,
    /// forward, right, down (PX4 body, NED-like).
    Frd,
    /// forward, right, up.
    Fru,
}

const FORWARD: [f64; 3] = [0.0, -1.0, 0.0];
const RIGHT: [f64; 3] = [1.0, 0.0, 0.0];
const DOWN: [f64; 3] = [0.0, 0.0, 1.0];
const UP: [f64; 3] = [0.0, 0.0, -1.0];

impl Convention {
    /// Parse the yaml value, case-insensitive.
    pub fn parse(s: &str) -> Result<Self, String> {
        match s.trim().to_ascii_uppercase().as_str() {
            "RDF" => Ok(Convention::Rdf),
            "FRD" => Ok(Convention::Frd),
            "FRU" => Ok(Convention::Fru),
            other => Err(format!(
                "output_frame: unknown convention '{other}' (expected RDF, FRD or FRU)"
            )),
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Convention::Rdf => "RDF",
            Convention::Frd => "FRD",
            Convention::Fru => "FRU",
        }
    }

    /// `C`: output x, y, z axes as rows, in board coordinates.
    pub fn matrix(self) -> Matrix3<f64> {
        let rows = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| {
            Matrix3::new(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2])
        };
        match self {
            Convention::Rdf => Matrix3::identity(),
            Convention::Frd => rows(FORWARD, RIGHT, DOWN),
            Convention::Fru => rows(FORWARD, RIGHT, UP),
        }
    }
}

/// The yaml file: one key.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct File {
    convention: String,
}

/// The loaded convention with its matrix.
#[derive(Debug, Clone)]
pub struct OutputFrame {
    pub convention: Convention,
    c: Matrix3<f64>,
}

impl OutputFrame {
    pub fn new(convention: Convention) -> Self {
        Self {
            convention,
            c: convention.matrix(),
        }
    }

    pub fn from_yaml(text: &str) -> Result<Self, String> {
        let f: File = serde_yaml::from_str(text).map_err(|e| format!("output_frame: {e}"))?;
        Ok(Self::new(Convention::parse(&f.convention)?))
    }

    pub fn load(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("cannot read '{}': {e}", path.display()))?;
        Self::from_yaml(&text)
            .map_err(|e| format!("parse '{}': {e}", path.display()))
    }

    /// `RDF`: the output is the board frame, nothing to convert.
    pub fn is_identity(&self) -> bool {
        self.convention == Convention::Rdf
    }

    /// A board-frame vector (position, velocity) in the output frame.
    pub fn vector(&self, v: &Vector3<f64>) -> Vector3<f64> {
        self.c * v
    }

    /// A board-frame body->world rotation in the output frame (both sides relabelled).
    pub fn rotation(&self, r: &Matrix3<f64>) -> Matrix3<f64> {
        self.c * r * self.c.transpose()
    }

    /// Inverse of [`vector`](Self::vector): an output-frame vector back in the board frame.
    pub fn vector_to_board(&self, v: &Vector3<f64>) -> Vector3<f64> {
        self.c.transpose() * v
    }

    /// Inverse of [`rotation`](Self::rotation).
    pub fn rotation_to_board(&self, r: &Matrix3<f64>) -> Matrix3<f64> {
        self.c.transpose() * r * self.c
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Rotation3;

    fn frame(c: Convention) -> OutputFrame {
        OutputFrame::new(c)
    }

    #[test]
    fn parses_case_insensitive_and_rejects_unknown() {
        assert_eq!(Convention::parse("frd").unwrap(), Convention::Frd);
        assert_eq!(Convention::parse(" FRU ").unwrap(), Convention::Fru);
        assert!(Convention::parse("ENU").is_err());
        assert!(OutputFrame::from_yaml("convention: RDF\n").unwrap().is_identity());
        assert!(OutputFrame::from_yaml("convention: FRD\nextra: 1\n").is_err());
    }

    #[test]
    fn matrices_are_orthonormal_and_rdf_is_identity() {
        for c in [Convention::Rdf, Convention::Frd, Convention::Fru] {
            let m = c.matrix();
            assert!((m * m.transpose() - Matrix3::identity()).norm() < 1e-12, "{c:?}");
        }
        assert_eq!(Convention::Rdf.matrix(), Matrix3::identity());
        assert!((Convention::Frd.matrix().determinant() - 1.0).abs() < 1e-12);
        assert!((Convention::Fru.matrix().determinant() + 1.0).abs() < 1e-12);
    }

    #[test]
    fn frd_relabels_board_axes() {
        // A point 1 m toward the board's top row (board -y) is 1 m forward.
        let f = frame(Convention::Frd);
        let p = f.vector(&Vector3::new(0.0, -1.0, 0.0));
        assert!((p - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
        // Board +x is right (output y), board +z stays down (output z).
        let p = f.vector(&Vector3::new(2.0, 0.0, 3.0));
        assert!((p - Vector3::new(0.0, 2.0, 3.0)).norm() < 1e-12);
    }

    #[test]
    fn fru_flips_down_to_up() {
        let f = frame(Convention::Fru);
        // 2 m above the floor is board z = -2, output z = +2.
        let p = f.vector(&Vector3::new(0.0, 0.0, -2.0));
        assert!((p - Vector3::new(0.0, 0.0, 2.0)).norm() < 1e-12);
    }

    #[test]
    fn rotations_stay_proper_and_round_trip() {
        let r = Rotation3::from_euler_angles(0.3, -0.7, 1.9).into_inner();
        for c in [Convention::Rdf, Convention::Frd, Convention::Fru] {
            let f = frame(c);
            let out = f.rotation(&r);
            assert!((out.determinant() - 1.0).abs() < 1e-12, "{c:?} det");
            assert!((out * out.transpose() - Matrix3::identity()).norm() < 1e-12);
            assert!((f.rotation_to_board(&out) - r).norm() < 1e-12, "{c:?} round trip");
            let v = Vector3::new(0.4, -1.2, 2.5);
            assert!((f.vector_to_board(&f.vector(&v)) - v).norm() < 1e-12);
        }
    }

    #[test]
    fn yaw_about_down_is_yaw_about_down_in_frd() {
        // A cube turned 90 deg about board z (down). In FRD the body z is also
        // down and the relabelling is a pure rotation, so the angle is preserved.
        let r = Rotation3::from_axis_angle(&Vector3::z_axis(), 90f64.to_radians()).into_inner();
        let out = frame(Convention::Frd).rotation(&r);
        let expect = Rotation3::from_axis_angle(&Vector3::z_axis(), 90f64.to_radians()).into_inner();
        assert!((out - expect).norm() < 1e-12);
        // Identity stays identity in every convention.
        for c in [Convention::Frd, Convention::Fru] {
            assert!((frame(c).rotation(&Matrix3::identity()) - Matrix3::identity()).norm() < 1e-12);
        }
    }
}
