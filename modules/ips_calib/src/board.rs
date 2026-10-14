//! The IPS calib board: the board section of `parameters/ips_calib.yaml` to
//! world-frame marker centres.
//!
//! The config is in millimetres; everything returned here is in metres. The world
//! frame, the board layout and its placement (origin marker, position, tilt) all
//! come from the yaml, loaded with the run settings by `setup::Params`.

use nalgebra::{Matrix3, Vector3};
use std::collections::HashMap;

/// The board keys of `parameters/ips_calib.yaml`: `world_frame`, `board`,
/// `placement`. Flattened into `setup::Params`, which checks the file's keys.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct CalibConfig {
    pub world_frame: String,
    pub board: Board,
    pub placement: Placement,
}

#[derive(Debug, Clone, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Board {
    /// Fiducial dictionary the board is printed in (`april` / `aruco`). The
    /// detectors must run the same family; ips_calib warns when they do not.
    pub family: String,
    pub marker_size_mm: f64,
    pub marker_gap_mm: f64,
    /// Marker id per `[row][col]`; row 0 = top, col 0 = left.
    pub layout: Vec<Vec<i32>>,
}

#[derive(Debug, Clone, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Placement {
    /// This marker's centre is placed at `position_mm`.
    pub origin_id: i32,
    pub position_mm: [f64; 3],
    /// XYZ Euler of the board in the world frame, `R = Rx * Ry * Rz`.
    pub orientation_deg: [f64; 3],
}

impl CalibConfig {
    /// Centre-to-centre distance of neighbouring markers, metres.
    pub fn pitch_m(&self) -> f64 {
        (self.board.marker_size_mm + self.board.marker_gap_mm) / 1000.0
    }

    /// Tag side, metres. The detectors' `marker_size` must match it.
    pub fn marker_size_m(&self) -> f64 {
        self.board.marker_size_mm / 1000.0
    }
}

/// XYZ Euler (degrees) to rotation matrix `R = Rx * Ry * Rz` (the convention of
/// the detectors' `euler_deg`).
pub fn euler_xyz_deg_to_mat(d: [f64; 3]) -> Matrix3<f64> {
    let (rx, ry, rz) = (d[0].to_radians(), d[1].to_radians(), d[2].to_radians());
    #[rustfmt::skip]
    let mx = Matrix3::new(1.0, 0.0, 0.0,  0.0, rx.cos(), -rx.sin(),  0.0, rx.sin(), rx.cos());
    #[rustfmt::skip]
    let my = Matrix3::new(ry.cos(), 0.0, ry.sin(),  0.0, 1.0, 0.0,  -ry.sin(), 0.0, ry.cos());
    #[rustfmt::skip]
    let mz = Matrix3::new(rz.cos(), -rz.sin(), 0.0,  rz.sin(), rz.cos(), 0.0,  0.0, 0.0, 1.0);
    mx * my * mz
}

/// Inverse of [`euler_xyz_deg_to_mat`]: the `R = Rx * Ry * Rz` angles (degrees) of
/// a rotation matrix. Only for the human-readable `euler_xyz_deg` field; `R` stays
/// the source of truth. `R02 = sin(ry)`, so `ry = asin`; `rx` and `rz` come from
/// the remaining elements, and the ry = +-90 deg gimbal case (where they couple)
/// is resolved by pinning `rz = 0`.
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

/// Each marker id's world-frame centre (metres): lay the grid out relative to
/// `origin_id`, rotate by the board tilt, translate to the placement.
pub fn build_world_model(cfg: &CalibConfig) -> Result<HashMap<i32, Vector3<f64>>, String> {
    let pitch = cfg.pitch_m();
    let mut rc: HashMap<i32, (usize, usize)> = HashMap::new();
    for (r, row) in cfg.board.layout.iter().enumerate() {
        for (c, &id) in row.iter().enumerate() {
            if rc.insert(id, (r, c)).is_some() {
                return Err(format!("marker id {id} appears twice in the board layout"));
            }
        }
    }
    let &(r0, c0) = rc
        .get(&cfg.placement.origin_id)
        .ok_or_else(|| format!("origin_id {} is not in the layout", cfg.placement.origin_id))?;
    let rot = euler_xyz_deg_to_mat(cfg.placement.orientation_deg);
    let origin = Vector3::from(cfg.placement.position_mm) / 1000.0;
    let mut world = HashMap::new();
    for (&id, &(r, c)) in &rc {
        // x grows with column (right), y grows with row (down), z = 0 in the board plane.
        let local = Vector3::new(
            (c as f64 - c0 as f64) * pitch,
            (r as f64 - r0 as f64) * pitch,
            0.0,
        );
        world.insert(id, rot * local + origin);
    }
    Ok(world)
}
