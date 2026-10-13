//! ips_detect: subscribe to the cameras listed in `parameters/detector_params.yaml`,
//! detect fiducial markers, estimate each marker's 3-D pose with that camera's
//! Kalibr intrinsics, and publish every frame's detections over zenoh as protobuf.
//!
//! Every setting comes from `parameters/detector_params.yaml` (cameras, family,
//! marker_size, debug, save, save_frames). The only command-line option is
//! `--id sbNN`, which narrows the run to that one camera. Without it, one
//! detector thread runs per camera in the yaml, all in this one process.
//!
//! Pose is the marker centre and orientation relative to the camera optical frame
//! (RDF: `+x` right, `+y` down, `+z` forward), in metres. Translation scale is set
//! by `marker_size`: it must match the printed tag (cube ArUco 0.087, calib board
//! 0.185) or every distance is off by that ratio, and ips_track's cameras then
//! disagree about the cube. Intrinsics are read from
//! `calibration/intrinsics/sbNN-camchain.yaml` at the repository root.
//!
//! Detections are published every frame on `<device>/ws_ips/ips_detect-sbNN/zenoh/markers`
//! (protobuf `custom/MarkerDetections`), stamped with the image's capture stamp;
//! the marker list is empty on frames with no detections, so consumers get a
//! steady heartbeat.

use ips_detect::camera::{self, Camera};
use ips_detect::debug_image_pub::{self, DebugImagePublisher};
use ips_detect::fps::FpsMeter;
use ips_detect::intrinsics::{self, Intrinsics};
use ips_detect::markers::{self, MarkerDetector};
use ips_detect::markers_pub::{self, MarkersPublisher};
use ips_detect::msgs::{MarkerDetection, MarkerDetections, Quat, Vec3};
use ips_detect::view;

use iceoryx2::prelude::*;
use opencv::core::{Mat, Point, Scalar, Vector};
use opencv::prelude::*;
use opencv::{imgcodecs, imgproc};
use prost::Message;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};
use zenoh::Wait;

/// Raised by the first thread whose `node.wait` reports SIGINT/SIGTERM.
/// iceoryx2 0.9.3 clears the signal once one node has seen it, so the other
/// camera threads learn about it from this flag instead of from their own node.
static STOP: AtomicBool = AtomicBool::new(false);

/// One poll-loop step: false once the process has been asked to stop.
fn keep_running(node: &Node<ipc::Service>, sleep: Duration) -> bool {
    if STOP.load(Ordering::Relaxed) {
        return false;
    }
    if node.wait(sleep).is_err() {
        STOP.store(true, Ordering::Relaxed);
        return false;
    }
    !STOP.load(Ordering::Relaxed)
}

const USAGE: &str =
    "usage: ips_detect [--id sbNN]   (every other setting is in parameters/detector_params.yaml)";

fn crate_dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

/// `cameras` accepts a list (`[0, 1, 2, 3]`) or a single index (`0`); both
/// normalise to a `Vec<u32>` via [`Indices::into_vec`].
#[derive(serde::Deserialize)]
#[serde(untagged)]
enum Indices {
    One(u32),
    Many(Vec<u32>),
}

impl Indices {
    fn into_vec(self) -> Vec<u32> {
        match self {
            Indices::One(i) => vec![i],
            Indices::Many(v) => v,
        }
    }
}

fn d_family() -> String {
    "aruco".to_string()
}
fn d_marker_size() -> f64 {
    0.087
}

/// All settings for `ips_detect`, read from `parameters/detector_params.yaml`.
/// Missing optional fields fall back to the `d_*` defaults.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct DetectorParams {
    /// Cameras to run: index N -> instance sbNN. One detector thread per camera.
    cameras: Indices,
    /// Marker dictionary: `aruco` (4x4_250) | `april` (36h11). Also accepts the
    /// `markers::parse_fm` tokens `aruco0`, `aruco1`, `april0`, `april1`.
    #[serde(default = "d_family")]
    family: String,
    #[serde(default = "d_marker_size")]
    marker_size: f64,
    #[serde(default)]
    debug: bool,
    /// Directory to save raw frames into; empty = off. Relative to this module folder.
    #[serde(default)]
    save: String,
    #[serde(default)]
    save_frames: u64,
}

/// Resolved run settings (the yaml, validated, narrowed by `--id`).
struct Settings {
    /// Instance names to drive, one thread each, de-duplicated in yaml order.
    insts: Vec<String>,
    /// Resolved token fed to `markers::parse_fm` ("aruco0" / "april0").
    fm: String,
    marker_size: f32,
    debug: bool,
    /// If set, save each raw frame under `<save_dir>/<inst>/<seq>.png`.
    save_dir: Option<PathBuf>,
    /// Stop saving after this many frames (0 = unlimited). Detection continues.
    save_frames: u64,
}

/// Everything one camera thread needs.
struct Job {
    inst: String,
    fm: String,
    marker_size: f32,
    debug: bool,
    save_dir: Option<PathBuf>,
    save_frames: u64,
    calib_path: PathBuf,
}

/// Parse the command line: nothing, or `--id <sbNN>`. Anything else is an error.
fn parse_cli<I: Iterator<Item = String>>(mut args: I) -> Result<Option<String>, String> {
    let mut id: Option<String> = None;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--id" => {
                let v = args
                    .next()
                    .ok_or("--id needs a camera name, e.g. --id sb00")?;
                if id.is_some() {
                    return Err("--id given more than once".to_string());
                }
                id = Some(v);
            }
            other => return Err(format!("unexpected argument '{other}'")),
        }
    }
    Ok(id)
}

/// Normalise a config `family` string to a `markers::parse_fm` token.
fn family_token(f: &str) -> Result<String, String> {
    match f {
        "aruco" | "aruco0" => Ok("aruco0".to_string()),
        "april" | "april0" => Ok("april0".to_string()),
        "aruco1" | "april1" => Ok(f.to_string()),
        other => Err(format!("invalid family '{other}' (use aruco or april)")),
    }
}

/// Validate the yaml and narrow it to `id` when given.
fn resolve(cfg: DetectorParams, id: Option<&str>) -> Result<Settings, String> {
    let fm = family_token(&cfg.family)?;
    let marker_size = cfg.marker_size as f32;
    if marker_size.is_nan() || marker_size <= 0.0 {
        return Err("marker_size must be > 0".to_string());
    }
    // De-dup the cameras (preserve order) so a repeated index doesn't start two
    // threads on the same camera.
    let mut insts: Vec<String> = Vec::new();
    for i in cfg.cameras.into_vec() {
        let inst = camera::instance_name(i);
        if !insts.contains(&inst) {
            insts.push(inst);
        }
    }
    if insts.is_empty() {
        return Err("cameras: needs at least one camera index (e.g. [0, 1, 2, 3])".to_string());
    }
    if let Some(id) = id {
        if !insts.iter().any(|s| s == id) {
            return Err(format!(
                "camera '{id}' is not in detector_params.yaml cameras [{}]",
                insts.join(", ")
            ));
        }
        insts = vec![id.to_string()];
    }
    let save_dir = if cfg.save.is_empty() {
        None
    } else {
        let p = PathBuf::from(&cfg.save);
        Some(if p.is_relative() {
            crate_dir().join(p)
        } else {
            p
        })
    };
    Ok(Settings {
        insts,
        fm,
        marker_size,
        debug: cfg.debug,
        save_dir,
        save_frames: cfg.save_frames,
    })
}

fn fail(msg: String, code: i32) -> ! {
    eprintln!("error: {msg}");
    eprintln!("{USAGE}");
    std::process::exit(code);
}

/// `<device>/ws_ips/ips_detect-<inst>/zenoh/markers`.
fn markers_overrides(inst: &str) -> markers_pub::TopicOverrides {
    markers_pub::TopicOverrides {
        module: Some(format!("{}-{inst}", markers_pub::DEFAULT_MODULE)),
        ..Default::default()
    }
}

/// `<device>/ws_ips/ips_detect-<inst>/iox2/debug_image`.
fn debug_overrides(inst: &str) -> debug_image_pub::TopicOverrides {
    debug_image_pub::TopicOverrides {
        module: Some(format!("{}-{inst}", debug_image_pub::DEFAULT_MODULE)),
        ..Default::default()
    }
}

/// Copy an RGB8 OpenCV `Mat` (1280x1024, `CV_8UC3`, contiguous) into the iox2
/// flat colour-image payload, stamping the fixed geometry and the header. The
/// stamp is the source image's capture stamp. A short `Mat` is clamped to the
/// buffer length. `encoding` is `rgb8`.
fn fill_color_image(
    msg: &mut debug_image_pub::Image1280x1024Color,
    rgb: &Mat,
    inst: &str,
    stamp_sec: i32,
    stamp_nanosec: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let src = rgb.data_bytes()?;
    let n = src.len().min(msg.data.len());
    msg.data[..n].copy_from_slice(&src[..n]);
    msg.data_len = n as u32;
    msg.width = camera::WIDTH as u32;
    msg.height = camera::HEIGHT as u32;
    msg.step = (camera::WIDTH * 3) as u32;
    msg.is_bigendian = 0;
    msg.header_stamp_sec = stamp_sec;
    msg.header_stamp_nanosec = stamp_nanosec;
    let fid = inst.as_bytes();
    let k = fid.len().min(msg.header_frame_id.len());
    msg.header_frame_id[..k].copy_from_slice(&fid[..k]);
    let enc = b"rgb8";
    msg.encoding[..enc.len()].copy_from_slice(enc);
    Ok(())
}

/// One camera's detect, pose, publish loop. Runs until SIGINT/SIGTERM (via
/// `node.wait`) or a fatal per-camera error, which is returned so the caller can
/// log it without taking the other cameras down. `fm` is re-parsed here so the
/// OpenCV dictionary stays thread-local (the thread owns all of its OpenCV, iox2
/// and zenoh state).
fn run_camera(job: Job) -> Result<(), Box<dyn std::error::Error>> {
    let Job {
        inst,
        fm,
        marker_size,
        debug,
        save_dir,
        save_frames,
        calib_path,
    } = job;
    let spec = markers::parse_fm(&fm)?;

    // Load this camera's intrinsics up front so a missing or garbled YAML fails
    // before we touch iox2.
    let intr = Intrinsics::load(&calib_path)?;

    // iox2 node, the subscriber on the camera's key, and the detector.
    let node = NodeBuilder::new().create::<ipc::Service>()?;
    let mut cam = Camera::open(&inst);
    let detector = MarkerDetector::new(spec.dict)?;

    // zenoh session and detections publisher (sync `Wait` API, no async runtime).
    // Each thread owns its own session so the borrow-bound publisher stays local.
    // zenoh's error is `Box<dyn Error + Send + Sync>`, which `?` won't coerce into
    // `Box<dyn Error>`; stringify it at each zenoh boundary.
    let z_session = zenoh::open(zenoh::Config::default())
        .wait()
        .map_err(|e| e.to_string())?;
    // Per-camera key: the instance is a suffix of the module segment,
    // <device>/ws_ips/ips_detect-sbNN/zenoh/markers. Consumers subscribe to all
    // cameras with the sub-segment wildcard ips_detect-$*.
    let overrides = markers_overrides(&inst);
    let pub_key = overrides.key_expr();
    let det_pub = MarkersPublisher::open_with(&z_session, overrides).map_err(|e| e.to_string())?;

    println!("[{inst}] family={fm} -> {}", spec.label);
    println!(
        "[{inst}] image   <- iox2  '{}'  (sensor_msgs/Image1280x1024)",
        cam.key
    );
    println!("[{inst}] markers -> zenoh '{pub_key}'  (custom/MarkerDetections)");
    println!(
        "[{inst}] intrinsics: {}  ({})",
        intr.summary(),
        intr.source.display()
    );
    println!("[{inst}] marker size: {marker_size:.4} m  (detector_params.yaml)");

    // Optional raw-frame saving (<save_dir>/<inst>/<seq>.png) for offline
    // debugging. Each saved frame's name is its frame_seq, matching the seq in
    // the published detection, so images and detections line up exactly.
    let save_cam_dir = match &save_dir {
        Some(root) => {
            let dir = root.join(&inst);
            std::fs::create_dir_all(&dir)
                .map_err(|e| format!("cannot create save dir '{}': {e}", dir.display()))?;
            let cap = if save_frames == 0 {
                "all".to_string()
            } else {
                save_frames.to_string()
            };
            println!(
                "[{inst}] saving raw frames to {}/  ({cap} frames)",
                dir.display()
            );
            Some(dir)
        }
        None => None,
    };
    let mut saved: u64 = 0;

    // Optional debug-image publisher (debug: true). Each camera publishes its
    // marker-annotated colour frame on its own iox2 service,
    // <device>/ws_ips/ips_detect-sbNN/iox2/debug_image, via the same module
    // override as the markers key. ips_viz tiles them. Shares this camera's node.
    let debug_pub = if debug {
        let ov = debug_overrides(&inst);
        let key = ov.service_name();
        let p = DebugImagePublisher::open_with(&node, ov)?;
        println!("[{inst}] debug image -> iox2 '{key}'  (sensor_msgs/Image1280x1024Color)");
        Some(p)
    } else {
        None
    };

    // Reusable buffers and the poll loop (exits on SIGINT/SIGTERM via node.wait).
    // `canvas` holds the BGR annotated frame; `rgb` its BGR to RGB copy for publish.
    let mut frame = Camera::make_frame()?;
    let mut canvas = Mat::default();
    let mut rgb = Mat::default();
    let mut fps = FpsMeter::new(Duration::from_secs(1));
    let mut seq: u64 = 0;

    while keep_running(&node, Duration::from_micros(1000)) {
        let Some(info) = cam.poll_into(&node, &mut frame)? else {
            continue;
        };
        seq += 1;

        // Save the raw frame if requested (name = seq = published frame_seq).
        if let Some(dir) = &save_cam_dir {
            if save_frames == 0 || saved < save_frames {
                let path = dir.join(format!("{seq:06}.png"));
                imgcodecs::imwrite(
                    path.to_str().unwrap_or_default(),
                    &frame,
                    &Vector::<i32>::new(),
                )?;
                saved += 1;
                if save_frames != 0 && saved == save_frames {
                    println!("[{inst}] saved {saved} frames to {}/", dir.display());
                }
            }
        }

        let hz = fps.tick(Instant::now());
        let det = detector.detect(&frame)?;
        // 3-D pose of each marker, in this camera's optical frame (metres, RDF).
        let poses = intr.estimate_poses(&det, marker_size)?;

        // The protobuf message for this frame, published below even when empty
        // (a steady heartbeat). The stamp is the image's capture stamp.
        let mut msg = MarkerDetections {
            stamp_sec: info.stamp_sec,
            stamp_nanosec: info.stamp_nanosec,
            camera: inst.clone(),
            frame_seq: seq,
            markers: Vec::with_capacity(poses.len()),
        };

        let hz_s = hz
            .map(|h| format!("{h:.1}"))
            .unwrap_or_else(|| "--".to_string());
        println!(
            "[{inst}] frame {seq}  data_len={}  {hz_s} Hz  markers={}  ids={:?}",
            info.data_len,
            det.count(),
            det.ids_vec()
        );
        // One line per marker: id, relative xyz, XYZ Euler (deg), distance.
        for p in &poses {
            let o = p.orientation();
            println!(
                "    [{inst}] id {:>3}   xyz=({:>7.3},{:>7.3},{:>7.3}) m   eul=({:>6.1},{:>6.1},{:>6.1}) deg   dist={:.3} m",
                p.id,
                p.tvec[0],
                p.tvec[1],
                p.tvec[2],
                o.euler_deg[0],
                o.euler_deg[1],
                o.euler_deg[2],
                p.distance()
            );
            msg.markers.push(MarkerDetection {
                id: p.id,
                family: spec.label.to_string(),
                position: Some(Vec3 {
                    x: p.tvec[0],
                    y: p.tvec[1],
                    z: p.tvec[2],
                }),
                orientation: Some(Quat {
                    x: o.quat[0],
                    y: o.quat[1],
                    z: o.quat[2],
                    w: o.quat[3],
                }),
                euler_deg: Some(Vec3 {
                    x: o.euler_deg[0],
                    y: o.euler_deg[1],
                    z: o.euler_deg[2],
                }),
                distance: p.distance(),
            });
        }

        // Publish this frame's detections (protobuf bytes over zenoh).
        det_pub
            .publish(&msg.encode_to_vec())
            .map_err(|e| e.to_string())?;

        if let Some(pubr) = &debug_pub {
            // Promote the mono frame to BGR, draw the markers in colour (green
            // outlines and ids), and project each marker's coordinate axes.
            imgproc::cvt_color_def(&frame, &mut canvas, imgproc::COLOR_GRAY2BGR)?;
            detector.draw(&mut canvas, &det)?;
            let axis_len = marker_size * 0.5;
            for p in &poses {
                intr.draw_axes(&mut canvas, p, axis_len)?;
            }

            // Off-scene legend (top-left): a status row, then one row per marker.
            // Rows correlate to markers by the id drawn on each marker, so the
            // labels never pile up over clustered detections.
            let yellow = Scalar::new(0.0, 255.0, 255.0, 0.0);
            let green = Scalar::new(0.0, 255.0, 0.0, 0.0);
            let mut rows: Vec<(String, Scalar)> = Vec::with_capacity(poses.len() + 1);
            rows.push((
                format!("[{inst}]  {hz_s} Hz  markers={}", det.count()),
                yellow,
            ));
            for p in &poses {
                let o = p.orientation();
                rows.push((
                    format!(
                        "id {:>3}  x{:>6.2} y{:>6.2} z{:>6.2}m  rpy{:>5.0},{:>5.0},{:>5.0}",
                        p.id,
                        p.tvec[0],
                        p.tvec[1],
                        p.tvec[2],
                        o.euler_deg[0],
                        o.euler_deg[1],
                        o.euler_deg[2]
                    ),
                    green,
                ));
            }
            view::draw_legend(&mut canvas, Point::new(12, 12), &rows)?;

            // OpenCV is BGR; rerun and sensor_msgs expect RGB. Convert, then copy
            // into a fresh iox2 payload and publish. The payload is 3.9 MB; the
            // detector thread runs on a large stack (see `main`) so the by-value
            // `publish` never overflows.
            imgproc::cvt_color_def(&canvas, &mut rgb, imgproc::COLOR_BGR2RGB)?;
            let mut img = debug_image_pub::Image1280x1024Color::default();
            fill_color_image(&mut img, &rgb, &inst, info.stamp_sec, info.stamp_nanosec)?;
            // The debug image is secondary to detection: a transient iox2 publish
            // hiccup must not take this camera's detector thread down.
            if let Err(e) = pubr.publish(img) {
                eprintln!("[{inst}] warning: debug image publish failed: {e}");
            }
        }
    }

    Ok(())
}

fn main() {
    let id = parse_cli(std::env::args().skip(1)).unwrap_or_else(|e| fail(e, 2));

    // Every other setting comes from detector_params.yaml.
    let config = crate_dir().join("parameters").join("detector_params.yaml");
    let text = std::fs::read_to_string(&config)
        .map_err(|e| format!("cannot read config '{}': {e}", config.display()))
        .unwrap_or_else(|e| fail(e, 1));
    let cfg: DetectorParams = serde_yaml::from_str(&text)
        .map_err(|e| format!("parse '{}': {e}", config.display()))
        .unwrap_or_else(|e| fail(e, 1));
    let settings = resolve(cfg, id.as_deref()).unwrap_or_else(|e| fail(e, 2));

    // Resolve the dictionary once: fail fast on a bad family before spawning.
    let spec = markers::parse_fm(&settings.fm).unwrap_or_else(|e| fail(e, 2));

    // Shared data path, resolved once: <repo>/calibration/intrinsics.
    let raw_dir = crate_dir()
        .join("../..")
        .join("calibration")
        .join("intrinsics");
    let calib_dir = std::fs::canonicalize(&raw_dir).unwrap_or(raw_dir);

    println!(
        "ips_detect: {} camera(s) [{}]  family={} -> {}",
        settings.insts.len(),
        settings.insts.join(", "),
        settings.fm,
        spec.label
    );
    println!("ips_detect: parameters {}", config.display());
    println!("ips_detect: intrinsics {}", calib_dir.display());
    println!("ips_detect: marker size {:.4} m", settings.marker_size);
    if settings.debug {
        println!(
            "ips_detect: debug=true, each camera publishes an annotated colour image on {}/{}/{}-sbNN/iox2/debug_image for ips_viz",
            debug_image_pub::DEFAULT_DEVICE,
            debug_image_pub::DEFAULT_WORKSPACE,
            debug_image_pub::DEFAULT_MODULE
        );
    }

    // One detector thread per camera. The process stays up until every thread
    // exits; each thread's `node.wait` returns on SIGINT/SIGTERM. A thread that
    // fails (e.g. missing intrinsics) logs and exits without stopping the others.
    let handles: Vec<_> = settings
        .insts
        .iter()
        .map(|inst| {
            let job = Job {
                inst: inst.clone(),
                fm: settings.fm.clone(),
                marker_size: settings.marker_size,
                debug: settings.debug,
                save_dir: settings.save_dir.clone(),
                save_frames: settings.save_frames,
                calib_path: intrinsics::path_for(&calib_dir, inst),
            };
            let name = inst.clone();
            thread::Builder::new()
                .name(inst.clone())
                // The iox2 colour payload is 3.9 MB and `publish` takes it by
                // value; give each detector thread plenty of stack so the move
                // never overflows the 2 MB default.
                .stack_size(32 * 1024 * 1024)
                .spawn(move || {
                    if let Err(e) = run_camera(job) {
                        eprintln!("[{name}] error: {e}");
                    }
                })
                .expect("spawn detector thread")
        })
        .collect();

    for h in handles {
        let _ = h.join();
    }
}
