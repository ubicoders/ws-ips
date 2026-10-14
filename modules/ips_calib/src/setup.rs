//! Everything a calibration run loads before it touches the wire: the run
//! parameters, the board model, each camera's intrinsics, and the marker stream
//! subscription. Shared by the headless one-shot (`headless.rs`) and the
//! interactive window (`session.rs`), so neither repeats the loading.

use crate::accumulate;
use crate::board::{self, CalibConfig};
use crate::msgs::MarkerDetections;
use crate::writer::{self, Camchain};

use nalgebra::Vector3;
use prost::Message;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use zenoh::handlers::FifoChannelHandler;
use zenoh::pubsub::Subscriber;
use zenoh::sample::Sample;
use zenoh::Wait;

/// `parameters/calib_params.yaml`.
#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Params {
    /// Camera indices, `0 -> sb00`.
    pub cameras: Vec<u32>,
    /// Marker frames to accumulate per camera before fitting.
    pub frames: u64,
    /// Overall cap on the accumulation, seconds. Headless mode only.
    pub timeout_s: f64,
    /// A camera that saw fewer distinct board ids is skipped.
    pub min_markers: usize,
    /// `true`: the interactive window. `false`: the headless one-shot.
    #[serde(default = "default_ui")]
    pub ui: bool,
    /// A residual above this is shown amber in the window, metres.
    #[serde(default = "default_rms_warn_m")]
    pub rms_warn_m: f64,
}

fn default_ui() -> bool {
    true
}

fn default_rms_warn_m() -> f64 {
    0.01
}

impl Params {
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("cannot read params '{}': {e}", path.display()))?;
        let p: Params =
            serde_yaml::from_str(&text).map_err(|e| format!("parse '{}': {e}", path.display()))?;
        if p.cameras.is_empty() {
            return Err(format!("'{}': cameras is empty", path.display()));
        }
        if p.frames == 0 {
            return Err(format!("'{}': frames must be at least 1", path.display()));
        }
        if !(p.timeout_s.is_finite() && p.timeout_s > 0.0) {
            return Err(format!(
                "'{}': timeout_s must be a positive number",
                path.display()
            ));
        }
        if p.min_markers < 3 {
            return Err(format!(
                "'{}': min_markers must be at least 3 (a rigid fit needs three points)",
                path.display()
            ));
        }
        if !(p.rms_warn_m.is_finite() && p.rms_warn_m > 0.0) {
            return Err(format!(
                "'{}': rms_warn_m must be a positive number",
                path.display()
            ));
        }
        Ok(p)
    }
}

/// Resolved file locations, all from this crate's folder.
pub struct Paths {
    pub repo: PathBuf,
    pub params: PathBuf,
    pub board: PathBuf,
    pub intrinsics: PathBuf,
    pub extrinsics: PathBuf,
}

impl Paths {
    pub fn resolve() -> Self {
        let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        let repo = crate_dir
            .parent()
            .and_then(|modules| modules.parent())
            .expect("the crate sits in modules/ inside the repository")
            .to_path_buf();
        let calib = repo.join("calibration");
        Self {
            params: crate_dir.join("parameters/calib_params.yaml"),
            board: calib.join("ips_calib.yaml"),
            intrinsics: calib.join("intrinsics"),
            extrinsics: calib.join("extrinsics"),
            repo,
        }
    }
}

/// What both run modes start from.
pub struct Setup {
    pub params: Params,
    pub paths: Paths,
    pub cfg: CalibConfig,
    /// Each board marker id's centre in the world frame, metres.
    pub world: HashMap<i32, Vector3<f64>>,
    /// The cameras with intrinsics, in `calib_params.yaml` order: `(instance, camchain)`.
    pub cams: Vec<(String, Camchain)>,
}

impl Setup {
    /// Load the parameters, the board, and every camera's camchain, printing the
    /// startup banner. A camera without a camchain is skipped and not waited for.
    pub fn load() -> Result<Self, String> {
        let paths = Paths::resolve();
        let params = Params::load(&paths.params)?;
        let cfg = CalibConfig::load(&paths.board)?;
        let world = board::build_world_model(&cfg)?;

        let mut targets: Vec<String> = Vec::new();
        for &i in &params.cameras {
            let name = accumulate::instance_name(i);
            if !targets.contains(&name) {
                targets.push(name);
            }
        }

        println!(
            "ips_calib: world_frame={}  pitch={:.4} m  origin_id={}",
            cfg.world_frame,
            cfg.pitch_m(),
            cfg.placement.origin_id
        );
        println!(
            "board: {}  tag={:.4} m  (the detectors must run family: {}, marker_size: {})",
            cfg.board.family,
            cfg.marker_size_m(),
            cfg.board.family,
            cfg.marker_size_m()
        );
        println!(
            "targets: {}   frames/cam={}  timeout={}s  min_markers={}  ui={}",
            targets.join(", "),
            params.frames,
            params.timeout_s,
            params.min_markers,
            params.ui
        );
        println!("params file:  {}", paths.params.display());
        println!("board file:   {}", paths.board.display());
        println!(
            "output files: {}",
            writer::full_yaml_path(&paths.extrinsics, "sbNN").display()
        );

        let mut cams: Vec<(String, Camchain)> = Vec::new();
        for inst in &targets {
            let path = writer::camchain_path(&paths.intrinsics, inst);
            match Camchain::load(&path) {
                Ok(c) => {
                    println!("  {inst}: {}  {}", path.display(), c.summary());
                    cams.push((inst.clone(), c));
                }
                Err(e) => println!("  {inst}: intrinsics load failed ({e}), skipped"),
            }
        }
        if cams.is_empty() {
            return Err("none of the requested cameras has intrinsics".to_string());
        }
        Ok(Self {
            params,
            paths,
            cfg,
            world,
            cams,
        })
    }

    /// The camera instance names, in report order.
    pub fn instances(&self) -> Vec<String> {
        self.cams.iter().map(|(inst, _)| inst.clone()).collect()
    }
}

/// The fan-in subscription on every detector's marker stream.
///
/// Declared directly: the generated `MarkersSubscriber`'s `try_recv_latest`
/// collapses the queue to the newest sample, which would drop the other cameras'
/// frames. Every frame counts here, so [`MarkerStream::drain`] uses `try_recv`.
pub struct MarkerStream {
    pub key: String,
    sub: Subscriber<FifoChannelHandler<Sample>>,
    /// Kept so the subscriber outlives nothing it needs; dropped after `sub`.
    _session: zenoh::Session,
    undecodable: u64,
}

impl MarkerStream {
    pub fn open() -> Result<Self, String> {
        let key = accumulate::markers_key();
        let session = zenoh::open(zenoh::Config::default())
            .wait()
            .map_err(|e| format!("zenoh open: {e}"))?;
        let sub = session
            .declare_subscriber(key.as_str())
            .wait()
            .map_err(|e| format!("zenoh subscribe '{key}': {e}"))?;
        println!("subscribing to '{key}'  (custom/MarkerDetections)\n");
        Ok(Self {
            key,
            sub,
            _session: session,
            undecodable: 0,
        })
    }

    /// Hand every queued message to `f` until the queue is empty or `f` returns
    /// `false`. Returns whether anything arrived. Undecodable samples are
    /// counted, and the first one is reported.
    pub fn drain(
        &mut self,
        mut f: impl FnMut(&MarkerDetections) -> bool,
    ) -> Result<bool, String> {
        let mut any = false;
        loop {
            let sample = match self.sub.try_recv() {
                Ok(Some(s)) => s,
                Ok(None) => return Ok(any),
                Err(e) => return Err(format!("zenoh subscriber '{}': {e}", self.key)),
            };
            any = true;
            let bytes = sample.payload().to_bytes();
            match MarkerDetections::decode(bytes.as_ref()) {
                Ok(msg) => {
                    if !f(&msg) {
                        return Ok(any);
                    }
                }
                Err(e) => {
                    if self.undecodable == 0 {
                        println!(
                            "warning: undecodable sample on '{}' ({e}), ignored",
                            sample.key_expr()
                        );
                    }
                    self.undecodable += 1;
                }
            }
        }
    }

    /// How many undecodable samples were ignored so far.
    pub fn undecodable(&self) -> u64 {
        self.undecodable
    }
}
