//! markers: the family/dictionary table, the OpenCV fiducial detector, and the
//! per-marker pose from its four image corners.
//!
//! OpenCV on this host is 4.6, so ArUco and AprilTag live in the contrib `aruco`
//! module (not `objdetect`, which only absorbed them in 4.7). Both families are
//! different `PREDEFINED_DICTIONARY_NAME` values fed to the same detector, so one
//! [`MarkerDetector`] covers both.

use crate::intrinsics::Intrinsics;
use opencv::core::{Mat, Point2f, Point3f, Ptr, Scalar, Vec3d, Vector};
use opencv::prelude::*;
use opencv::{aruco, calib3d};

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// Fiducial family.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    Aruco,
    April,
}

/// A resolved family token: the family, its index, and the OpenCV dictionary it
/// maps to. `label` is the text carried in `MarkerDetection.family`.
pub struct Spec {
    pub family: Family,
    pub index: usize,
    pub dict: aruco::PREDEFINED_DICTIONARY_NAME,
    pub label: &'static str,
}

/// Parse a family token into a [`Spec`].
///
/// | token    | dictionary            |
/// |----------|-----------------------|
/// | `aruco0` | `DICT_4X4_250`        |
/// | `aruco1` | `DICT_5X5_250`        |
/// | `april0` | `DICT_APRILTAG_36h11` |
/// | `april1` | `DICT_APRILTAG_36h10` |
pub fn parse_fm(s: &str) -> Result<Spec, String> {
    use aruco::PREDEFINED_DICTIONARY_NAME::*;
    let (rest, family) = if let Some(n) = s.strip_prefix("aruco") {
        (n, Family::Aruco)
    } else if let Some(n) = s.strip_prefix("april") {
        (n, Family::April)
    } else {
        return Err(format!(
            "unknown family '{s}' (expected aruco<N> or april<N>; valid: aruco0, aruco1, april0, april1)"
        ));
    };
    let index: usize = rest
        .parse()
        .map_err(|_| format!("family '{s}': '{rest}' is not a dictionary index"))?;
    let (dict, label) = match (family, index) {
        (Family::Aruco, 0) => (DICT_4X4_250, "aruco0 (ArUco 4x4_250)"),
        (Family::Aruco, 1) => (DICT_5X5_250, "aruco1 (ArUco 5x5_250)"),
        (Family::April, 0) => (DICT_APRILTAG_36h11, "april0 (AprilTag 36h11)"),
        (Family::April, 1) => (DICT_APRILTAG_36h10, "april1 (AprilTag 36h10)"),
        _ => {
            return Err(format!(
                "family '{s}': index {index} out of range (valid: aruco0, aruco1, april0, april1)"
            ))
        }
    };
    Ok(Spec {
        family,
        index,
        dict,
        label,
    })
}

/// Result of one detection pass. `corners[i]` are the 4 image-space corners of
/// the marker whose id is `ids[i]`, in ArUco order (top-left, top-right,
/// bottom-right, bottom-left).
pub struct Detections {
    pub ids: Vector<i32>,
    pub corners: Vector<Vector<Point2f>>,
}

impl Detections {
    /// Detected ids as a plain `Vec` (for logging and iteration).
    pub fn ids_vec(&self) -> Vec<i32> {
        self.ids.iter().collect()
    }

    /// Number of markers detected this frame.
    pub fn count(&self) -> usize {
        self.ids.len()
    }
}

/// OpenCV ArUco/AprilTag detector bound to one predefined dictionary.
pub struct MarkerDetector {
    dictionary: Ptr<aruco::Dictionary>,
    params: Ptr<aruco::DetectorParameters>,
}

impl MarkerDetector {
    /// Build a detector for the given predefined dictionary.
    pub fn new(dict: aruco::PREDEFINED_DICTIONARY_NAME) -> Res<Self> {
        let dictionary = aruco::get_predefined_dictionary(dict)?;
        let params = aruco::DetectorParameters::create()?;
        Ok(Self { dictionary, params })
    }

    /// Detect markers in a grayscale (`CV_8UC1`) image.
    pub fn detect(&self, gray: &Mat) -> Res<Detections> {
        let mut corners: Vector<Vector<Point2f>> = Vector::new();
        let mut ids: Vector<i32> = Vector::new();
        let mut rejected: Vector<Vector<Point2f>> = Vector::new();
        aruco::detect_markers(
            gray,
            &self.dictionary,
            &mut corners,
            &mut ids,
            &self.params,
            &mut rejected,
        )?;
        Ok(Detections { ids, corners })
    }

    /// Draw detected marker outlines and ids onto `canvas` (3-channel BGR), in place.
    pub fn draw(&self, canvas: &mut Mat, det: &Detections) -> Res<()> {
        aruco::draw_detected_markers(
            canvas,
            &det.corners,
            &det.ids,
            Scalar::new(0.0, 255.0, 0.0, 0.0), // green (BGR)
        )?;
        Ok(())
    }
}

/// One marker's estimated pose, relative to the camera optical frame
/// (RDF: `+x` right, `+y` down, `+z` forward into the scene). The marker frame is
/// X right, Y up, Z out of the printed face, origin at the marker centre, so
/// `cam = R(rvec) * marker + tvec`. `orientation()` and `distance()` are in
/// `intrinsics.rs`.
pub struct MarkerPose {
    pub id: i32,
    /// Rodrigues rotation vector.
    pub rvec: Vec3d,
    /// Translation `[x, y, z]` of the marker centre, in metres.
    pub tvec: Vec3d,
}

/// Pose of one square marker of side `marker_size` (metres) from its four image
/// corners, in ArUco order (top-left, top-right, bottom-right, bottom-left), using
/// the camera's `K` and distortion. Returns `(rvec, tvec)` in the camera optical
/// frame, same convention as `aruco::estimate_pose_single_markers`: object points
/// `(-s/2, s/2, 0), (s/2, s/2, 0), (s/2, -s/2, 0), (-s/2, -s/2, 0)`, solved with
/// `solvePnP` (iterative).
// STUB-BOUNDARY: module 2
pub fn pose_from_corners(
    corners: &Vector<Point2f>,
    intr: &Intrinsics,
    marker_size: f32,
) -> Res<(Vec3d, Vec3d)> {
    if corners.len() != 4 {
        return Err(format!(
            "pose_from_corners: expected 4 corners, got {}",
            corners.len()
        )
        .into());
    }
    let h = marker_size / 2.0;
    let object: Vector<Point3f> = Vector::from_slice(&[
        Point3f::new(-h, h, 0.0),
        Point3f::new(h, h, 0.0),
        Point3f::new(h, -h, 0.0),
        Point3f::new(-h, -h, 0.0),
    ]);
    let mut rvec = Mat::default();
    let mut tvec = Mat::default();
    let ok = calib3d::solve_pnp(
        &object,
        corners,
        &intr.camera_matrix,
        &intr.dist_coeffs,
        &mut rvec,
        &mut tvec,
        false,
        calib3d::SOLVEPNP_ITERATIVE,
    )?;
    if !ok {
        return Err("pose_from_corners: solvePnP found no solution".into());
    }
    let r = Vec3d::from([
        *rvec.at::<f64>(0)?,
        *rvec.at::<f64>(1)?,
        *rvec.at::<f64>(2)?,
    ]);
    let t = Vec3d::from([
        *tvec.at::<f64>(0)?,
        *tvec.at::<f64>(1)?,
        *tvec.at::<f64>(2)?,
    ]);
    Ok((r, t))
}
