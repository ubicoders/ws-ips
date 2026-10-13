//! filter: the output smoother applied to each published cube pose, selected
//! once at startup by the `filter:` value of `tracker_params.yaml`.
//!
//!   - `none`: publish the raw fit.
//!   - `lp`: exponential low-pass (EMA) on position and SLERP on orientation,
//!     `lp_pose` and `lp_angle` ([`crate::filter_lp`]).
//!   - `oneeuro`: adaptive 1 Euro filter, the `oe_*` values
//!     ([`crate::filter_one_euro`]).
//!   - `kf`: constant-velocity Kalman on position and a multiplicative EKF on
//!     orientation, the `kf_*` values ([`crate::filter_kf`]). The only filter that
//!     fills `CubePose.velocity`.
//!
//! Each variant owns its per-cube state; the tick loop feeds it every fresh fit
//! through [`Filter::smooth`] and publishes what comes back.

use std::time::Instant;

use crate::cube::FusedPose;
use crate::filter_kf::KalmanFilter;
use crate::filter_lp::{Filtered, LowPass};
use crate::filter_one_euro::OneEuro;

/// The filter name and every filter's tuning, as read from
/// `tracker_params.yaml`. Only the values of the selected filter are used.
pub struct Tuning {
    /// `none` | `lp` | `oneeuro` | `kf` (a few aliases are accepted).
    pub name: String,
    pub lp_pose: f64,
    pub lp_angle: f64,
    pub oe_min_cutoff_pos: f64,
    pub oe_beta_pos: f64,
    pub oe_min_cutoff_ang: f64,
    pub oe_beta_ang: f64,
    pub kf_q_pos: f64,
    pub kf_r_pos: f64,
    pub kf_q_ang: f64,
    pub kf_r_ang: f64,
    pub kf_lookahead_ms: u64,
}

/// The active output smoother.
pub enum Filter {
    /// No smoothing: publish the raw fit unchanged.
    Off,
    /// Exponential low-pass (EMA), see [`crate::filter_lp`].
    Lp(LowPass),
    /// Adaptive 1 Euro filter, see [`crate::filter_one_euro`].
    OneEuro(OneEuro),
    /// Constant-velocity Kalman plus MEKF, see [`crate::filter_kf`].
    Kalman(KalmanFilter),
}

impl Filter {
    /// Build the filter named by `t.name` with its tuning from `t`. An unknown
    /// name is an error naming the accepted ones.
    pub fn select(t: &Tuning) -> Result<Self, String> {
        match t.name.trim().to_ascii_lowercase().as_str() {
            "none" | "off" | "raw" => Ok(Filter::Off),
            "lp" | "ema" | "lowpass" => Ok(Filter::Lp(LowPass::new(t.lp_pose, t.lp_angle))),
            "oneeuro" | "one-euro" | "one_euro" | "1euro" | "1e" => {
                Ok(Filter::OneEuro(OneEuro::new(
                    t.oe_min_cutoff_pos,
                    t.oe_beta_pos,
                    t.oe_min_cutoff_ang,
                    t.oe_beta_ang,
                )))
            }
            "kf" | "kalman" => Ok(Filter::Kalman(KalmanFilter::new(
                t.kf_q_pos,
                t.kf_r_pos,
                t.kf_q_ang,
                t.kf_r_ang,
                t.kf_lookahead_ms as f64 / 1000.0,
            ))),
            other => Err(format!(
                "unknown filter '{other}' (use none | lp | oneeuro | kf)"
            )),
        }
    }

    /// One startup banner line naming the filter and its tuning.
    pub fn describe(&self, t: &Tuning) -> String {
        match self {
            Filter::Off => "filter: none  (raw fit; set filter: lp | oneeuro | kf to smooth)".to_string(),
            Filter::Lp(_) => format!(
                "filter: low-pass (EMA)  lp_pose={:.2}  lp_angle={:.2}  (pose = a*prev + (1-a)*new)",
                t.lp_pose, t.lp_angle
            ),
            Filter::OneEuro(_) => format!(
                "filter: one-euro  pos[min_cutoff={:.2} Hz beta={:.3}]  ang[min_cutoff={:.2} Hz beta={:.3}]",
                t.oe_min_cutoff_pos, t.oe_beta_pos, t.oe_min_cutoff_ang, t.oe_beta_ang
            ),
            Filter::Kalman(_) => format!(
                "filter: kalman (CV + MEKF)  pos[q={:.3} m²/s³ r={:.3} m]  ang[q={:.3} rad²/s³ r={:.3} rad]  lookahead={} ms",
                t.kf_q_pos, t.kf_r_pos, t.kf_q_ang, t.kf_r_ang, t.kf_lookahead_ms
            ),
        }
    }

    /// Smooth (or pass through) cube `cid`'s fresh fit `f`, sampled at `now`, and
    /// return the publish-ready pose. Its `vel` (world frame, m/s) is `Some` only
    /// for the Kalman filter, so the wire `velocity` field follows the filter.
    pub fn smooth(&mut self, cid: u32, now: Instant, f: &FusedPose) -> Filtered {
        match self {
            // Raw passthrough keeps the fit's own euler and quaternion verbatim,
            // so `none` is a true no-op.
            Filter::Off => Filtered {
                pos: f.t,
                rot: f.r,
                euler_deg: f.euler_deg,
                quat: f.quat,
                vel: None,
            },
            Filter::Lp(lp) => lp.update(cid, f),
            Filter::OneEuro(oe) => oe.update(cid, now, f),
            Filter::Kalman(kf) => kf.update(cid, now, f),
        }
    }
}
