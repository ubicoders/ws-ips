//! intrinsics: per-camera intrinsics from the Kalibr camchain YAML, and the
//! marker pose helpers built on them.
//!
//! Each `sbNN` camera was calibrated with Kalibr; the result lives in
//! `calibration/intrinsics/sbNN-camchain.yaml` as a pinhole projection
//! `[fx, fy, cx, cy]` plus 4 radtan distortion coeffs `[k1, k2, p1, p2]`. We load
//! that into the OpenCV matrices the pose solver needs, then turn each detected
//! marker's image corners into a pose relative to the camera optical frame
//! (RDF: `+x` right, `+y` down, `+z` forward), in metres.
//!
//! The translation scale is set entirely by the physical marker side length
//! (`marker_size`, metres); get that wrong and every distance is off by the same
//! factor.

use crate::markers::{self, Detections, MarkerPose};
use opencv::calib3d;
use opencv::core::Mat;
use std::path::{Path, PathBuf};

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// One marker's pose in the camera optical frame. Same type as
/// [`markers::MarkerPose`]; this module adds `orientation()` and `distance()`.
pub type Pose = MarkerPose;

/// Pinhole intrinsics plus radtan distortion for one camera, pre-built into the
/// `CV_64F` matrices the pose solver consumes.
pub struct Intrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// radtan distortion: `[k1, k2, p1, p2]`.
    pub dist: [f64; 4],
    pub width: i32,
    pub height: i32,
    /// 3x3 `CV_64F` camera matrix.
    pub camera_matrix: Mat,
    /// 1x4 `CV_64F` distortion vector.
    pub dist_coeffs: Mat,
    /// The YAML the values came from (for logging).
    pub source: PathBuf,
}

/// A marker's orientation, derived from its Rodrigues `rvec`, expressed in the
/// camera optical frame (RDF: `+x` right, `+y` down, `+z` forward).
pub struct Orientation {
    /// Unit quaternion `[x, y, z, w]` (canonical; gimbal-lock-free).
    pub quat: [f64; 4],
    /// Tait-Bryan XYZ Euler angles in degrees: rotation about X, then Y, then Z
    /// (`R = Rx * Ry * Rz`). Gimbal lock at the Y term = +-90 deg.
    pub euler_deg: [f64; 3],
}

impl MarkerPose {
    /// Straight-line distance from the camera to the marker centre, in metres.
    pub fn distance(&self) -> f64 {
        let [x, y, z] = [self.tvec[0], self.tvec[1], self.tvec[2]];
        (x * x + y * y + z * z).sqrt()
    }

    /// Quaternion plus XYZ Euler (degrees) for this pose, in the camera optical
    /// frame. The quaternion comes straight from the axis-angle `rvec`; the
    /// Euler angles are read off the Rodrigues rotation matrix (`R = Rx * Ry * Rz`).
    pub fn orientation(&self) -> Orientation {
        let [rx, ry, rz] = [self.rvec[0], self.rvec[1], self.rvec[2]];
        let theta = (rx * rx + ry * ry + rz * rz).sqrt();

        // Quaternion directly from axis-angle: q = (axis * sin(theta/2), cos(theta/2)).
        let quat = if theta < 1e-9 {
            [0.0, 0.0, 0.0, 1.0]
        } else {
            let half = 0.5 * theta;
            let s = half.sin() / theta; // sin(theta/2)/theta, so rvec * s = axis * sin(theta/2)
            [rx * s, ry * s, rz * s, half.cos()]
        };

        // Rodrigues rotation matrix from the (unit) axis; theta = 0 yields identity.
        let (kx, ky, kz) = if theta < 1e-9 {
            (0.0, 0.0, 0.0)
        } else {
            (rx / theta, ry / theta, rz / theta)
        };
        let (c, s, t) = (theta.cos(), theta.sin(), 1.0 - theta.cos());
        // Only the elements the XYZ extraction needs.
        let m00 = t * kx * kx + c;
        let m01 = t * kx * ky - s * kz;
        let m02 = t * kx * kz + s * ky;
        let m12 = t * ky * kz - s * kx;
        let m22 = t * kz * kz + c;

        // R = Rx * Ry * Rz  =>  ax = atan2(-m12, m22), ay = asin(m02), az = atan2(-m01, m00).
        let ax = (-m12).atan2(m22);
        let ay = m02.clamp(-1.0, 1.0).asin();
        let az = (-m01).atan2(m00);
        let euler_deg = [ax.to_degrees(), ay.to_degrees(), az.to_degrees()];

        Orientation { quat, euler_deg }
    }
}

/// `<dir>/<inst>-camchain.yaml`, e.g. `.../calibration/intrinsics/sb00-camchain.yaml`.
pub fn path_for(dir: &Path, inst: &str) -> PathBuf {
    dir.join(format!("{inst}-camchain.yaml"))
}

impl Intrinsics {
    /// Load and parse one camera's camchain YAML and build its OpenCV matrices.
    pub fn load(path: &Path) -> Res<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("cannot read calibration '{}': {e}", path.display()))?;

        let intr = find_list(&text, "intrinsics")
            .ok_or_else(|| format!("'{}': no 'intrinsics: [...]' line", path.display()))?;
        if intr.len() != 4 {
            return Err(format!(
                "'{}': intrinsics has {} values, expected 4 (fx fy cx cy)",
                path.display(),
                intr.len()
            )
            .into());
        }
        let dist_v = find_list(&text, "distortion_coeffs")
            .ok_or_else(|| format!("'{}': no 'distortion_coeffs: [...]' line", path.display()))?;
        if dist_v.len() < 4 {
            return Err(format!(
                "'{}': distortion_coeffs has {} values, expected >=4 (k1 k2 p1 p2)",
                path.display(),
                dist_v.len()
            )
            .into());
        }
        // resolution is informational; fall back to the fixed sensor size if absent.
        let res = find_list(&text, "resolution").unwrap_or_default();
        let width = res.first().copied().unwrap_or(crate::camera::WIDTH as f64) as i32;
        let height = res.get(1).copied().unwrap_or(crate::camera::HEIGHT as f64) as i32;

        let (fx, fy, cx, cy) = (intr[0], intr[1], intr[2], intr[3]);
        let dist = [dist_v[0], dist_v[1], dist_v[2], dist_v[3]];

        // 3x3 camera matrix and 1x4 distortion, both CV_64F (f64 slices give CV_64FC1).
        let camera_matrix = Mat::from_slice_2d(&[[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])?;
        let dist_coeffs = Mat::from_slice_2d(&[dist])?;

        Ok(Self {
            fx,
            fy,
            cx,
            cy,
            dist,
            width,
            height,
            camera_matrix,
            dist_coeffs,
            source: path.to_path_buf(),
        })
    }

    /// One-line summary for the startup banner.
    pub fn summary(&self) -> String {
        format!(
            "fx={:.1} fy={:.1} cx={:.1} cy={:.1}  dist=[{:.4}, {:.4}, {:.5}, {:.5}]  {}x{}",
            self.fx,
            self.fy,
            self.cx,
            self.cy,
            self.dist[0],
            self.dist[1],
            self.dist[2],
            self.dist[3],
            self.width,
            self.height
        )
    }

    /// Estimate the pose of every detected marker, given the physical marker side
    /// length (metres). Output order matches `det.ids` / `det.corners`.
    pub fn estimate_poses(&self, det: &Detections, marker_len: f32) -> Res<Vec<Pose>> {
        let n = det.count();
        let mut poses = Vec::with_capacity(n);
        for i in 0..n {
            let corners = det.corners.get(i)?;
            let (rvec, tvec) = markers::pose_from_corners(&corners, self, marker_len)?;
            poses.push(Pose {
                id: det.ids.get(i)?,
                rvec,
                tvec,
            });
        }
        Ok(poses)
    }

    /// Draw the marker's RGB coordinate axes (x=red, y=green, z=blue) onto a BGR
    /// `canvas`, projected with this camera's intrinsics. `axis_len` is in metres.
    pub fn draw_axes(&self, canvas: &mut Mat, pose: &Pose, axis_len: f32) -> Res<()> {
        calib3d::draw_frame_axes(
            canvas,
            &self.camera_matrix,
            &self.dist_coeffs,
            &pose.rvec,
            &pose.tvec,
            axis_len,
            2,
        )?;
        Ok(())
    }
}

/// Pull the first `key: [a, b, c]` list out of `text` and parse it as `f64`s.
/// Matches the key at the start of a (whitespace-trimmed) line, so
/// `distortion_coeffs` vs `distortion_model` etc. don't collide. Handles
/// scientific notation.
pub fn find_list(text: &str, key: &str) -> Option<Vec<f64>> {
    for line in text.lines() {
        let t = line.trim_start();
        if let Some(rest) = t.strip_prefix(key) {
            if !rest.starts_with(':') {
                continue; // e.g. key "distortion" must not match "distortion_model:"
            }
            let lb = rest.find('[')?;
            let rb = rest.find(']')?;
            let mut out = Vec::new();
            for part in rest[lb + 1..rb].split(',') {
                let p = part.trim();
                if p.is_empty() {
                    continue;
                }
                out.push(p.parse().ok()?);
            }
            return Some(out);
        }
    }
    None
}
