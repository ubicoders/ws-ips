//! `filter_one_euro`: adaptive low-pass (the 1 Euro filter) for fitted cube poses.
//!
//! The fixed-`a` EMA in [`crate::filter_lp`] forces one tradeoff: more smoothing
//! means more lag on a moving cube. The 1 Euro filter (Casiez, Roussel and Vogel,
//! 2012, <https://gery.casiez.net/1euro/>) escapes it by raising the low-pass
//! cutoff with the measured speed:
//!
//! ```text
//! cutoff = min_cutoff + beta * |velocity|
//! a      = smoothing_factor(cutoff, dt)        a in (0, 1], weight on the NEW value
//! out    = a * new + (1 - a) * prev
//! ```
//!
//! Standing still gives a small cutoff and heavy smoothing (no jitter). Moving
//! fast gives a large cutoff and light smoothing (low lag, where motion would
//! expose it).
//!
//! Tuning, per channel (position and orientation):
//!   - `min_cutoff` (Hz): lower means more smoothing at rest (less jitter, more rest lag).
//!   - `beta`: higher means less lag while moving (but lets jitter back in).
//!
//! Position runs one 1 Euro filter per axis (metres, speed in m/s). Orientation
//! runs the rotation analog: the speed is the geodesic angular rate (rad/s)
//! between the filtered previous and the new quaternion, and the cutoff drives a
//! SLERP weight.

use std::collections::HashMap;
use std::time::Instant;

use nalgebra::{UnitQuaternion, Vector3};

use crate::cube;
use crate::filter_lp::{fused_quat, Filtered};

/// Derivative-filter cutoff (Hz). Fixed at the 1 Euro filter's standard default;
/// it only de-noises the velocity estimate that drives the adaptive cutoff.
const D_CUTOFF: f64 = 1.0;

/// 1 Euro smoothing factor for a given `cutoff` (Hz) and timestep `dt` (s):
/// `a = 1 / (1 + tau / dt)` with `tau = 1 / (2 pi cutoff)`. `a` tends to 1 as
/// `dt` grows or the cutoff rises (trust the new sample).
fn smoothing_factor(cutoff: f64, dt: f64) -> f64 {
    let tau = 1.0 / (2.0 * std::f64::consts::PI * cutoff);
    1.0 / (1.0 + tau / dt)
}

/// Per-cube 1 Euro state: filtered position and its filtered per-axis velocity,
/// filtered orientation and its filtered angular rate, and the last sample time.
struct State {
    pos: Vector3<f64>,
    dpos: Vector3<f64>,
    rot: UnitQuaternion<f64>,
    drot: f64,
    last: Instant,
}

/// Adaptive (1 Euro) smoother holding one [`State`] per cube id.
pub struct OneEuro {
    min_cutoff_pos: f64,
    beta_pos: f64,
    min_cutoff_ang: f64,
    beta_ang: f64,
    state: HashMap<u32, State>,
}

impl OneEuro {
    /// `min_cutoff_*` (Hz) sets the smoothing at rest; `beta_*` how fast the
    /// cutoff opens with speed (position channel, orientation channel).
    pub fn new(min_cutoff_pos: f64, beta_pos: f64, min_cutoff_ang: f64, beta_ang: f64) -> Self {
        Self {
            min_cutoff_pos,
            beta_pos,
            min_cutoff_ang,
            beta_ang,
            state: HashMap::new(),
        }
    }

    /// Smooth a fresh fit for cube `cid` sampled at `now`, update the state, and
    /// return the publish-ready pose. The first call for a cube seeds the state
    /// (velocity 0) and returns the raw fit; `dt` is the gap to that cube's
    /// previous sample, so per-cube rates are handled independently.
    pub fn update(&mut self, cid: u32, now: Instant, f: &cube::FusedPose) -> Filtered {
        let q_new = fused_quat(f);

        let prev = match self.state.get(&cid) {
            None => {
                // Seed: no history, so no velocity. Publish the raw fit.
                self.state.insert(
                    cid,
                    State {
                        pos: f.t,
                        dpos: Vector3::zeros(),
                        rot: q_new,
                        drot: 0.0,
                        last: now,
                    },
                );
                return Filtered::from_pose(f.t, q_new);
            }
            Some(s) => s,
        };

        // Clamp dt so a zero gap (same Instant) cannot blow up 1 / dt.
        let dt = now.duration_since(prev.last).as_secs_f64().max(1e-4);

        // Position: one 1 Euro filter per axis.
        let ad = smoothing_factor(D_CUTOFF, dt);
        let raw_vel = (f.t - prev.pos) / dt;
        let dpos = ad * raw_vel + (1.0 - ad) * prev.dpos; // low-passed velocity
        let mut pos = Vector3::zeros();
        for i in 0..3 {
            let cutoff = self.min_cutoff_pos + self.beta_pos * dpos[i].abs();
            let a = smoothing_factor(cutoff, dt);
            pos[i] = a * f.t[i] + (1.0 - a) * prev.pos[i];
        }

        // Orientation: rotation analog, speed = geodesic angular rate.
        let angle = (prev.rot.inverse() * q_new).angle(); // [0, pi]
        let raw_rate = angle / dt;
        let drot = ad * raw_rate + (1.0 - ad) * prev.drot; // low-passed angular rate
        let cutoff = self.min_cutoff_ang + self.beta_ang * drot;
        let a = smoothing_factor(cutoff, dt);
        // SLERP prev to new by `a` (weight on the new sample). try_slerp is None
        // only for an antipodal pair (never at ~90 Hz); fall back to the new pose.
        let rot = prev.rot.try_slerp(&q_new, a, 1e-6).unwrap_or(q_new);

        self.state.insert(
            cid,
            State {
                pos,
                dpos,
                rot,
                drot,
                last: now,
            },
        );
        Filtered::from_pose(pos, rot)
    }
}
