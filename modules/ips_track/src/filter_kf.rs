//! `filter_kf`: Kalman filter for fitted cube poses.
//!
//! The velocity-tracking smoother: a constant-velocity Kalman filter on position
//! and a multiplicative (error-state) EKF on orientation. Unlike the fixed gains
//! of [`crate::filter_lp`] and [`crate::filter_one_euro`], the gain comes from the
//! noise model every frame, which buys two things a fixed-`a` low-pass cannot:
//!
//!   1. Per-frame measurement noise from fit quality. Each fit reports its centre
//!      `residual_rms` and the number of contributing cameras. The measurement
//!      variance is inflated by the residual and divided by the camera count, so
//!      a clean multi-camera fit is trusted more (gain close to 1) and a
//!      single-camera or high-residual fit less (the model coasts on velocity).
//!   2. Predict-ahead. Because the state carries velocity, the estimate can be
//!      rolled forward by the pipeline latency, cancelling the lag every
//!      smoothing filter otherwise adds. `lookahead = 0` publishes the estimate at
//!      the fit instant.
//!
//! ## Position: constant-velocity KF, one per axis
//!
//! The three world axes are independent under the diagonal noise model, so
//! position is three decoupled 2-state filters `x = [pos, vel]` ([`Cv1`]), with
//! the continuous white-noise acceleration (CWNA) model:
//!
//! ```text
//! predict:  x- = F x,  P- = F P F^T + Q     F = [[1, dt], [0, 1]]
//!           Q = q_pos * [[dt^3/3, dt^2/2], [dt^2/2, dt]]   (accel PSD q_pos, m^2/s^3)
//! update:   z = measured pos,  H = [1, 0]
//!           S = P-_00 + R,  K = P- H^T / S,  x = x- + K (z - x-_0),  P = (I - K H) P-
//! ```
//!
//! ## Orientation: multiplicative EKF (MEKF), constant body rate
//!
//! Orientation is a nominal quaternion `q` (world from body) plus a body-frame
//! angular velocity `w`, with the covariance carried over the 6-D error state
//! `[dtheta, dw]` (`dtheta` a small-angle rotation vector, local error convention
//! `q_true = q * exp(dtheta)`). For a constant body rate the error dynamics are
//! `dtheta' = -[w]x dtheta + dw`, `dw' = 0`, giving
//!
//! ```text
//! predict:  q- = q * exp(w dt),  w- = w
//!           F = [[exp(-[w]x dt), I dt], [0, I]]      (exp(-[w]x dt) = R(-w dt))
//!           Q = q_ang * [[I dt^3/3, I dt^2/2], [I dt^2/2, I dt]]   (ang-accel PSD, rad^2/s^3)
//! update:   dtheta_meas = Log(q-^-1 * q_meas),  H = [I, 0]
//!           S = P_tt + R I,  K = P H^T S^-1,  e = K dtheta_meas
//!           inject: q <- q * exp(e_theta),  w <- w + e_w,  P <- (I - K H) P
//! ```
//!
//! Per-cube state (both sub-filters and the last sample time) lives here; the
//! caller feeds fresh fits with a timestamp and reads back a [`Filtered`] pose,
//! like the other filters. After a dropout longer than [`GAP_RESET_S`] the stored
//! velocity is stale, so the cube is reseeded rather than extrapolated.

use std::collections::HashMap;
use std::time::Instant;

use nalgebra::{Matrix2, Matrix3, Matrix6, Matrix6x3, UnitQuaternion, Vector2, Vector3};

use crate::cube;
use crate::filter_lp::{fused_quat, Filtered};

/// Reseed a cube's filter when the gap since its last sample exceeds this (s):
/// after a dropout the stored velocity no longer describes the motion.
const GAP_RESET_S: f64 = 0.5;

/// Floor on `dt` (s) so a zero gap between two samples at the same `Instant`
/// cannot blow up the process noise or the gain.
const DT_MIN: f64 = 1e-4;

/// Seed covariance for the velocity states, which the first sample cannot
/// observe: large enough (about (1 m/s)^2 and (3 rad/s)^2) that the first few
/// frames pull the velocity straight from the position and orientation changes.
const INIT_VEL_VAR: f64 = 1.0;
const INIT_ANGVEL_VAR: f64 = 10.0;

/// One world axis of a constant-velocity Kalman filter: state `[pos, vel]` with a
/// 2x2 covariance, position the only measurement, CWNA process noise.
#[derive(Clone, Copy)]
struct Cv1 {
    /// `[pos, vel]`.
    x: Vector2<f64>,
    p: Matrix2<f64>,
}

impl Cv1 {
    /// Seed from the first position measurement: velocity unknown (0, wide
    /// prior), position at the measurement with its variance.
    fn seed(pos: f64, r: f64) -> Self {
        Self {
            x: Vector2::new(pos, 0.0),
            p: Matrix2::new(r, 0.0, 0.0, INIT_VEL_VAR),
        }
    }

    /// Roll the state forward `dt` s under the constant-velocity model with
    /// accel PSD `q` (m^2/s^3).
    // STUB-BOUNDARY: module 3
    fn predict(&mut self, dt: f64, q: f64) {
        let f = Matrix2::new(1.0, dt, 0.0, 1.0);
        self.x = f * self.x;
        let (dt2, dt3) = (dt * dt, dt * dt * dt);
        let qm = Matrix2::new(dt3 / 3.0, dt2 / 2.0, dt2 / 2.0, dt) * q;
        self.p = f * self.p * f.transpose() + qm;
        self.symmetrize();
    }

    /// Correct with a position measurement `z` of variance `r` (m^2).
    // STUB-BOUNDARY: module 3
    fn update(&mut self, z: f64, r: f64) {
        let s = self.p[(0, 0)] + r;
        let k = Vector2::new(self.p[(0, 0)] / s, self.p[(1, 0)] / s);
        self.x += k * (z - self.x[0]);
        // P <- (I - K H) P with H = [1, 0], so K H carries K in its first column.
        let kh = Matrix2::new(k[0], 0.0, k[1], 0.0);
        self.p = (Matrix2::identity() - kh) * self.p;
        self.symmetrize();
    }

    /// Position predicted `lookahead` s past the last update (predict-ahead).
    fn pos_at(&self, lookahead: f64) -> f64 {
        self.x[0] + self.x[1] * lookahead
    }

    /// Estimated velocity (m/s), the second state. Constant under the CV model,
    /// so it is the same whether or not the position is rolled ahead.
    fn vel(&self) -> f64 {
        self.x[1]
    }

    /// Re-impose covariance symmetry, which the plain `(I - K H) P` update
    /// erodes over many steps.
    fn symmetrize(&mut self) {
        self.p = (self.p + self.p.transpose()) * 0.5;
    }
}

/// Multiplicative EKF for orientation: nominal `(q, w)` (quaternion world from
/// body, body-frame angular rate) with a 6x6 covariance over `[dtheta, dw]`.
#[derive(Clone)]
struct Mekf {
    q: UnitQuaternion<f64>,
    /// Body-frame angular velocity, rad/s.
    w: Vector3<f64>,
    p: Matrix6<f64>,
}

impl Mekf {
    /// Seed from the first orientation measurement: rate unknown (0, wide
    /// prior), orientation at the measurement with its variance.
    fn seed(q: UnitQuaternion<f64>, r: f64) -> Self {
        let mut p = Matrix6::zeros();
        for i in 0..3 {
            p[(i, i)] = r;
            p[(i + 3, i + 3)] = INIT_ANGVEL_VAR;
        }
        Self {
            q,
            w: Vector3::zeros(),
            p,
        }
    }

    /// Roll nominal and covariance forward `dt` s under the constant-body-rate
    /// model with angular-accel PSD `q` (rad^2/s^3).
    // STUB-BOUNDARY: module 3
    fn predict(&mut self, dt: f64, q: f64) {
        // Nominal: integrate the body rate onto the quaternion (right-multiply).
        self.q *= UnitQuaternion::from_scaled_axis(self.w * dt);

        // Error-state transition F = [[exp(-[w]x dt), I dt], [0, I]]. The top-left
        // block is exactly the rotation matrix of -w dt.
        let theta = UnitQuaternion::from_scaled_axis(-self.w * dt)
            .to_rotation_matrix()
            .into_inner();
        let mut f = Matrix6::identity();
        f.fixed_view_mut::<3, 3>(0, 0).copy_from(&theta);
        f.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(Matrix3::identity() * dt));

        // CWNA process noise per 3-axis block, same kinematic form as position.
        let (dt2, dt3) = (dt * dt, dt * dt * dt);
        let mut qm = Matrix6::zeros();
        let i3 = Matrix3::identity();
        qm.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(i3 * (q * dt3 / 3.0)));
        qm.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(i3 * (q * dt2 / 2.0)));
        qm.fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(i3 * (q * dt2 / 2.0)));
        qm.fixed_view_mut::<3, 3>(3, 3).copy_from(&(i3 * (q * dt)));

        self.p = f * self.p * f.transpose() + qm;
        self.symmetrize();
    }

    /// Correct with an orientation measurement `q_meas` of variance `r` (rad^2).
    // STUB-BOUNDARY: module 3
    fn update(&mut self, q_meas: UnitQuaternion<f64>, r: f64) {
        // Innovation in the tangent space: the small rotation from nominal to
        // measurement, in the body frame (matches the local error convention).
        let dtheta = (self.q.inverse() * q_meas).scaled_axis();

        // H = [I, 0], so P H^T is the first three columns of P; S = P_tt + R I.
        let pht: Matrix6x3<f64> = self.p.fixed_columns::<3>(0).clone_owned();
        let s = self.p.fixed_view::<3, 3>(0, 0).clone_owned() + Matrix3::identity() * r;
        let s_inv = match s.try_inverse() {
            Some(si) => si,
            None => return, // singular innovation covariance (never at ~90 Hz): skip
        };
        let k: Matrix6x3<f64> = pht * s_inv;
        let e = k * dtheta;

        // Inject the correction into the nominal and reset the error to 0.
        self.q *= UnitQuaternion::from_scaled_axis(Vector3::new(e[0], e[1], e[2]));
        self.w += Vector3::new(e[3], e[4], e[5]);

        // P <- (I - K H) P with H = [I, 0], so K H carries K in its first 3 columns.
        let mut kh = Matrix6::zeros();
        kh.fixed_columns_mut::<3>(0).copy_from(&k);
        self.p = (Matrix6::identity() - kh) * self.p;
        self.symmetrize();
    }

    /// Orientation predicted `lookahead` s past the last update (predict-ahead).
    fn read(&self, lookahead: f64) -> UnitQuaternion<f64> {
        self.q * UnitQuaternion::from_scaled_axis(self.w * lookahead)
    }

    /// Re-impose covariance symmetry eroded by the `(I - K H) P` update.
    fn symmetrize(&mut self) {
        self.p = (self.p + self.p.transpose()) * 0.5;
    }
}

/// Per-cube Kalman state: the three position sub-filters, the orientation MEKF,
/// and the time of the last sample (for `dt` and the dropout reset).
struct State {
    pos: [Cv1; 3],
    rot: Mekf,
    last: Instant,
}

/// Constant-velocity Kalman smoother holding one [`State`] per cube id. Tuning is
/// the process-noise PSDs (`q_*`, how readily the model yields to the
/// measurements), the base measurement-noise stds (`r_*`, one clean camera), and
/// the predict-ahead horizon.
pub struct KalmanFilter {
    /// Position accel PSD (m^2/s^3). Higher trusts the model less, follows faster.
    q_pos: f64,
    /// Base position measurement std (m) at one clean camera, residual-free.
    r_pos: f64,
    /// Angular-accel PSD (rad^2/s^3).
    q_ang: f64,
    /// Base orientation measurement std (rad) at one clean camera, residual-free.
    r_ang: f64,
    /// Predict-ahead horizon (s); `0` publishes the estimate at the fit instant.
    lookahead: f64,
    state: HashMap<u32, State>,
}

impl KalmanFilter {
    /// `q_pos`, `q_ang`: process-noise PSDs (position m^2/s^3, orientation
    /// rad^2/s^3). `r_pos`, `r_ang`: base measurement-noise stds at one clean
    /// camera (m, rad). `lookahead_s`: predict-ahead horizon (s, `0` = none).
    pub fn new(q_pos: f64, r_pos: f64, q_ang: f64, r_ang: f64, lookahead_s: f64) -> Self {
        Self {
            q_pos,
            r_pos,
            q_ang,
            r_ang,
            lookahead: lookahead_s,
            state: HashMap::new(),
        }
    }

    /// Per-frame measurement variances `(R_pos, R_ang)` from the fit quality: the
    /// base variance inflated by the squared residual (relative to the base
    /// position noise) and divided by the camera count. The same dimensionless
    /// quality factor scales both channels.
    fn meas_var(&self, f: &cube::FusedPose) -> (f64, f64) {
        let n = f.num_cameras.max(1) as f64;
        let excess = (f.residual_rms / self.r_pos).powi(2);
        let quality = (1.0 + excess) / n;
        (self.r_pos.powi(2) * quality, self.r_ang.powi(2) * quality)
    }

    /// Smooth a fresh fit for cube `cid` sampled at `now`, update the state, and
    /// return the publish-ready pose. The first call for a cube, or the first
    /// after a gap longer than [`GAP_RESET_S`], seeds the state (velocity 0) and
    /// returns the raw fit. Otherwise the position KFs and the orientation MEKF
    /// predict by the per-cube `dt`, correct against the fit, and the readout is
    /// rolled `lookahead` s ahead.
    // STUB-BOUNDARY: module 3
    pub fn update(&mut self, cid: u32, now: Instant, f: &cube::FusedPose) -> Filtered {
        let q_meas = fused_quat(f);
        let (r_pos, r_ang) = self.meas_var(f);

        let reseed = match self.state.get(&cid) {
            Some(s) => now.duration_since(s.last).as_secs_f64() > GAP_RESET_S,
            None => true,
        };
        if reseed {
            self.state.insert(
                cid,
                State {
                    pos: [
                        Cv1::seed(f.t[0], r_pos),
                        Cv1::seed(f.t[1], r_pos),
                        Cv1::seed(f.t[2], r_pos),
                    ],
                    rot: Mekf::seed(q_meas, r_ang),
                    last: now,
                },
            );
            // Velocity is unobservable from one frame, so it seeds at 0. Still
            // report it (as Some) so the wire field's presence follows the chosen
            // filter, not the per-cube warm-up.
            return Filtered::with_vel(f.t, q_meas, Vector3::zeros());
        }

        let s = self
            .state
            .get_mut(&cid)
            .expect("state present when not reseeding");
        let dt = now.duration_since(s.last).as_secs_f64().max(DT_MIN);
        for i in 0..3 {
            s.pos[i].predict(dt, self.q_pos);
            s.pos[i].update(f.t[i], r_pos);
        }
        s.rot.predict(dt, self.q_ang);
        s.rot.update(q_meas, r_ang);
        s.last = now;

        let pos = Vector3::new(
            s.pos[0].pos_at(self.lookahead),
            s.pos[1].pos_at(self.lookahead),
            s.pos[2].pos_at(self.lookahead),
        );
        let vel = Vector3::new(s.pos[0].vel(), s.pos[1].vel(), s.pos[2].vel());
        Filtered::with_vel(pos, s.rot.read(self.lookahead), vel)
    }
}
