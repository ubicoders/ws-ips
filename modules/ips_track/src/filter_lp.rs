//! `filter_lp`: exponential low-pass (EMA) smoothing of fitted cube poses.
//!
//! The tracker builds one [`LowPass`] for the run, feeds it each cube's fresh
//! [`cube::FusedPose`] through [`LowPass::update`], and publishes the returned
//! [`Filtered`] pose. Position is smoothed with a per-axis EMA and the
//! orientation with its rotation analog (SLERP):
//!
//! ```text
//! pos = a_pos * pos_prev + (1 - a_pos) * pos_new     (EMA, per axis)
//! rot = slerp(rot_prev -> rot_new, 1 - a_ang)        (rotation analog, no euler wrap)
//! ```
//!
//! `a = 0` is passthrough, `a -> 1` is heavy smoothing. The first sample of each
//! cube seeds the state with the raw fit.
//!
//! See [`crate::filter_one_euro`] for an adaptive variant (speed-dependent
//! cutoff, less lag while moving) and [`crate::filter_kf`] for the
//! velocity-tracking Kalman filter.

use std::collections::HashMap;

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

use crate::cube;

/// Publish-ready smoothed pose: position plus the rotation in every form the
/// tracker writes onto a `CubePose`. The common return of every filter, so the
/// publish path does not depend on which one runs.
pub struct Filtered {
    pub pos: Vector3<f64>,
    pub rot: Matrix3<f64>,
    pub euler_deg: [f64; 3],
    /// `[x, y, z, w]`.
    pub quat: [f64; 4],
    /// World-frame linear velocity (m/s), or `None` for filters that carry no
    /// velocity state. Only the Kalman filter estimates it; the others leave it
    /// `None` so the wire `velocity` field stays unset.
    pub vel: Option<Vector3<f64>>,
}

impl Filtered {
    /// Build the publish-ready forms from a smoothed position and orientation, so
    /// every filter derives matrix, euler and quaternion from one rotation.
    /// Velocity is left unset; see [`Filtered::with_vel`].
    pub fn from_pose(pos: Vector3<f64>, rot: UnitQuaternion<f64>) -> Self {
        let r = rot.to_rotation_matrix().into_inner();
        let c = rot.coords; // [x, y, z, w]
        Self {
            pos,
            rot: r,
            euler_deg: cube::mat_to_euler_xyz_deg(&r),
            quat: [c[0], c[1], c[2], c[3]],
            vel: None,
        }
    }

    /// As [`from_pose`](Self::from_pose), carrying an estimated world-frame
    /// linear velocity (the Kalman filter's velocity state).
    pub fn with_vel(pos: Vector3<f64>, rot: UnitQuaternion<f64>, vel: Vector3<f64>) -> Self {
        Self {
            vel: Some(vel),
            ..Self::from_pose(pos, rot)
        }
    }
}

/// A [`cube::FusedPose`]'s orientation as a `UnitQuaternion`. `FusedPose::quat`
/// is `[x, y, z, w]`; nalgebra's `Quaternion::new` takes `(w, i, j, k)`. Shared
/// with the other filter modules.
pub(crate) fn fused_quat(f: &cube::FusedPose) -> UnitQuaternion<f64> {
    UnitQuaternion::from_quaternion(Quaternion::new(f.quat[3], f.quat[0], f.quat[1], f.quat[2]))
}

/// Per-cube EMA state: the last published position and orientation.
struct State {
    pos: Vector3<f64>,
    rot: UnitQuaternion<f64>,
}

/// Exponential low-pass smoother holding one [`State`] per cube id.
pub struct LowPass {
    /// Smoothing factor on position, weighting the previous value (`0..=1`).
    a_pos: f64,
    /// Smoothing factor on orientation, weighting the previous rotation (`0..=1`).
    a_ang: f64,
    state: HashMap<u32, State>,
}

impl LowPass {
    /// `a_pos` and `a_ang` weight the previous value (position, angle); both in
    /// `0..=1`, `0` passthrough, close to `1` heavy smoothing.
    pub fn new(a_pos: f64, a_ang: f64) -> Self {
        Self {
            a_pos,
            a_ang,
            state: HashMap::new(),
        }
    }

    /// Smooth a fresh fit for cube `cid` against its stored state, update the
    /// state, and return the publish-ready pose. The first call for a cube seeds
    /// the state with the raw fit and returns it unchanged.
    pub fn update(&mut self, cid: u32, f: &cube::FusedPose) -> Filtered {
        let q_new = fused_quat(f);
        let (pos, rot) = match self.state.get(&cid) {
            None => (f.t, q_new),
            Some(prev) => {
                let pos = self.a_pos * prev.pos + (1.0 - self.a_pos) * f.t;
                // SLERP prev to new by (1 - a_ang) keeps weight a_ang on the
                // previous rotation. try_slerp is None only for an antipodal pair
                // (never at ~90 Hz); fall back to the new pose then.
                let rot = prev
                    .rot
                    .try_slerp(&q_new, 1.0 - self.a_ang, 1e-6)
                    .unwrap_or(q_new);
                (pos, rot)
            }
        };
        self.state.insert(cid, State { pos, rot });
        Filtered::from_pose(pos, rot)
    }
}
