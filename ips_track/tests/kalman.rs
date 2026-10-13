//! `KalmanFilter` on synthetic, noise-free fits: holding a static pose and
//! tracking constant linear and angular velocity.

use std::time::{Duration, Instant};

use ips_track::cube::FusedPose;
use ips_track::filter_kf::KalmanFilter;
use ips_track::filter_lp::Filtered;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

const DT: Duration = Duration::from_millis(11); // ~90 Hz, the tick rate

/// A minimal `FusedPose` carrying what the filter reads (position, quaternion,
/// camera count, residual); the other fields are filler.
fn fp(t: Vector3<f64>, q: UnitQuaternion<f64>) -> FusedPose {
    let c = q.coords; // [x, y, z, w]
    FusedPose {
        r: q.to_rotation_matrix().into_inner(),
        t,
        euler_deg: [0.0; 3],
        quat: [c[0], c[1], c[2], c[3]],
        marker_ids: vec![],
        cameras: vec![],
        num_cameras: 4,
        residual_rms: 0.0,
    }
}

fn ufrom(f: &Filtered) -> UnitQuaternion<f64> {
    let [x, y, z, w] = f.quat;
    UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
}

fn kf() -> KalmanFilter {
    KalmanFilter::new(1.0, 0.01, 2.0, 0.02, 0.0)
}

#[test]
fn static_pose_is_held_exactly() {
    let mut k = kf();
    let t = Vector3::new(0.5, 0.5, -1.0);
    let q = UnitQuaternion::from_euler_angles(0.3, 0.2, -0.5);
    let mut now = Instant::now();
    let mut out = k.update(0, now, &fp(t, q));
    for _ in 0..60 {
        now += DT;
        out = k.update(0, now, &fp(t, q));
    }
    // A constant input drives the innovation (and the learned velocity) to zero,
    // so the estimate sits on the measurement.
    assert!((out.pos - t).norm() < 1e-6, "pos drifted: {:?}", out.pos);
    assert!(ufrom(&out).angle_to(&q) < 1e-6, "rot drifted");
}

#[test]
fn tracks_constant_velocity_with_low_lag() {
    let mut k = kf();
    let p0 = Vector3::new(-0.3, 0.1, 0.8);
    let v = Vector3::new(0.5, -0.2, 0.0); // m/s
    let mut now = Instant::now();
    let mut out = k.update(0, now, &fp(p0, UnitQuaternion::identity()));
    for step in 1..=80 {
        now += DT;
        let truth = p0 + v * (step as f64 * DT.as_secs_f64());
        out = k.update(0, now, &fp(truth, UnitQuaternion::identity()));
    }
    let truth = p0 + v * (80.0 * DT.as_secs_f64());
    // The constant-velocity model is exact for constant-velocity truth, so with
    // noise-free measurements the lag converges far below the per-step motion
    // (5.5 mm in x).
    assert!(
        (out.pos - truth).norm() < 2e-3,
        "lag too large: est {:?} truth {:?}",
        out.pos,
        truth
    );
    // The velocity estimate is finite and close to the truth.
    let vel = out.vel.expect("kf reports velocity");
    assert!(vel.iter().all(|x| x.is_finite()));
    assert!((vel - v).norm() < 0.05, "velocity off: {vel:?}");
}

#[test]
fn tracks_constant_angular_rate() {
    let mut k = kf();
    let wb = Vector3::new(0.0, 0.0, 2.0); // body rate, rad/s
    let step_q = UnitQuaternion::from_scaled_axis(wb * DT.as_secs_f64());
    let mut truth = UnitQuaternion::from_euler_angles(0.1, -0.2, 0.0);
    let mut now = Instant::now();
    let mut out = k.update(0, now, &fp(Vector3::zeros(), truth));
    for _ in 0..120 {
        now += DT;
        truth *= step_q; // body-frame right-multiply, matching the MEKF model
        out = k.update(0, now, &fp(Vector3::zeros(), truth));
    }
    assert!(
        ufrom(&out).angle_to(&truth) < 1e-2,
        "ang lag too large: {} rad",
        ufrom(&out).angle_to(&truth)
    );
}
