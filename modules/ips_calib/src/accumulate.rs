//! Accumulate the detectors' marker stream per camera, per marker id, and reduce
//! each id to the median of its observed centres.
//!
//! Every decoded `MarkerDetections` is one frame of the camera named in it. The
//! marker positions are metres in that camera's optical frame (RDF), the same
//! units as the board model.

use crate::markers_sub;
use crate::msgs::MarkerDetections;
use nalgebra::Vector3;
use std::collections::{HashMap, HashSet};

/// Module segment of the detectors on the wire; their instances are `ips_detect-sbNN`.
pub const DETECT_MODULE: &str = "ips_detect";

/// `<device>/ws_ips/ips_detect-$*/zenoh/markers`: every detector instance. The
/// wildcard is `$*` (zenoh sub-segment wildcard), since the instance is a suffix
/// of the module segment. Rendered through the generated `TopicOverrides` so the
/// device and workspace come from the sb-generated constants.
pub fn markers_key() -> String {
    markers_sub::TopicOverrides {
        full: Some(format!(
            "{}/{}/{DETECT_MODULE}-$*/{}/{}",
            markers_sub::DEFAULT_DEVICE,
            markers_sub::DEFAULT_WORKSPACE,
            markers_sub::TRANSPORT_SEGMENT,
            markers_sub::DEFAULT_TOPIC,
        )),
        ..Default::default()
    }
    .key_expr()
}

/// Instance name of camera index `i`: `0 -> sb00`.
pub fn instance_name(i: u32) -> String {
    format!("sb{i:02}")
}

/// Component-wise median, robust to the odd misdetection. `v` must not be empty.
pub fn median_vec(v: &[Vector3<f64>]) -> Vector3<f64> {
    let comp = |sel: fn(&Vector3<f64>) -> f64| {
        let mut xs: Vec<f64> = v.iter().map(sel).collect();
        xs.sort_by(|a, b| a.total_cmp(b));
        xs[xs.len() / 2]
    };
    Vector3::new(comp(|p| p.x), comp(|p| p.y), comp(|p| p.z))
}

/// Something worth telling the operator, at most once per camera.
#[derive(Debug, Clone, PartialEq)]
pub enum Notice {
    /// A detector not in `calib_params.yaml` `cameras` is publishing.
    Ignored { camera: String },
    /// A camera's detector runs another marker family than the board's.
    WrongFamily {
        camera: String,
        family: String,
        board_family: String,
    },
}

impl std::fmt::Display for Notice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Notice::Ignored { camera } => write!(
                f,
                "  '{camera}': not in calib_params.yaml cameras, its markers are ignored"
            ),
            Notice::WrongFamily {
                camera,
                family,
                board_family,
            } => write!(
                f,
                "warning: {camera} reports family '{family}', the board is '{board_family}'. \
                 Its detector is not set to the board family, so this camera's fit will be wrong. \
                 Set the detectors to the board's family and marker_size and restart them."
            ),
        }
    }
}

/// Per camera: frame count and every observed centre per marker id.
pub struct Accumulator {
    targets: Vec<String>,
    frames_goal: u64,
    board_family: String,
    frames: HashMap<String, u64>,
    samples: HashMap<String, HashMap<i32, Vec<Vector3<f64>>>>,
    ignored: HashSet<String>,
    family_warned: HashSet<String>,
}

impl Accumulator {
    /// `targets`: the cameras to accumulate, in report order. `frames_goal`: a
    /// camera stops accumulating once it has this many frames. `board_family`:
    /// the expected prefix of every marker's `family` string (`april`).
    pub fn new(targets: Vec<String>, frames_goal: u64, board_family: &str) -> Self {
        Self {
            targets,
            frames_goal,
            board_family: board_family.to_string(),
            frames: HashMap::new(),
            samples: HashMap::new(),
            ignored: HashSet::new(),
            family_warned: HashSet::new(),
        }
    }

    /// Take one decoded message. Returns a notice the first time a camera is
    /// ignored or reports the wrong family.
    pub fn ingest(&mut self, msg: &MarkerDetections) -> Option<Notice> {
        let cam = msg.camera.as_str();
        if !self.targets.iter().any(|t| t == cam) {
            return self
                .ignored
                .insert(cam.to_string())
                .then(|| Notice::Ignored {
                    camera: cam.to_string(),
                });
        }
        let n = self.frames.entry(cam.to_string()).or_default();
        if *n >= self.frames_goal {
            return None;
        }
        *n += 1;

        let mut notice = None;
        let per_cam = self.samples.entry(cam.to_string()).or_default();
        for m in &msg.markers {
            // A marker of another family is not on the board. It is dropped so
            // a detector left on ArUco cannot write a wrong extrinsics file.
            if !m.family.starts_with(&self.board_family) {
                if self.family_warned.insert(cam.to_string()) {
                    notice = Some(Notice::WrongFamily {
                        camera: cam.to_string(),
                        family: m.family.clone(),
                        board_family: self.board_family.clone(),
                    });
                }
                continue;
            }
            let Some(p) = &m.position else { continue };
            let v = Vector3::new(p.x, p.y, p.z);
            if v.iter().all(|x| x.is_finite()) {
                per_cam.entry(m.id).or_default().push(v);
            }
        }
        notice
    }

    /// Frames counted for `cam` so far.
    pub fn frames(&self, cam: &str) -> u64 {
        self.frames.get(cam).copied().unwrap_or(0)
    }

    /// Every target camera reached the frame goal.
    pub fn done(&self) -> bool {
        self.targets
            .iter()
            .all(|c| self.frames(c) >= self.frames_goal)
    }

    /// `sb00:120  sb01:87 ...` for the once-per-second progress line.
    pub fn progress(&self) -> String {
        self.targets
            .iter()
            .map(|c| format!("{c}:{}", self.frames(c)))
            .collect::<Vec<_>>()
            .join("  ")
    }

    /// The ids `cam` observed that are also on the board, ascending, each with the
    /// median of its observed centres (camera frame), paired with its board centre
    /// (world frame). Returns `(ids, cam_pts, world_pts)`.
    pub fn matched(
        &self,
        cam: &str,
        world: &HashMap<i32, Vector3<f64>>,
    ) -> (Vec<i32>, Vec<Vector3<f64>>, Vec<Vector3<f64>>) {
        let Some(per_cam) = self.samples.get(cam) else {
            return (Vec::new(), Vec::new(), Vec::new());
        };
        let mut ids: Vec<i32> = per_cam
            .iter()
            .filter(|(id, v)| !v.is_empty() && world.contains_key(id))
            .map(|(&id, _)| id)
            .collect();
        ids.sort_unstable();
        let cam_pts = ids.iter().map(|id| median_vec(&per_cam[id])).collect();
        let world_pts = ids.iter().map(|id| world[id]).collect();
        (ids, cam_pts, world_pts)
    }
}
