//! ips_calib: one-shot extrinsic calibration of every IPS camera.
//!
//! Subscribes to every detector's marker stream (`ips_detect-$*`) while the
//! cameras look at the IPS calib board, accumulates each camera's marker centres
//! (camera frame, metres), and, using the board geometry in
//! `calibration/ips_calib.yaml`, solves every camera's pose in the world frame by
//! a rigid least-squares (Kabsch) fit of the board marker centres. Writes
//! `calibration/extrinsics/sbNN_full.yaml` per camera (K copied from the
//! camchain, plus R|t) and exits.
//!
//! The detectors must run the board's family and tag size (`family: april`,
//! `marker_size: 0.185`). No command-line options: the run settings are in
//! `parameters/calib_params.yaml`.

use ips_calib::accumulate::{self, Accumulator};
use ips_calib::board::{self, CalibConfig};
use ips_calib::fit;
use ips_calib::msgs::MarkerDetections;
use ips_calib::writer::{self, Camchain, FullYaml};

use prost::Message;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};
use zenoh::Wait;

const USAGE: &str = "usage: ips_calib  (no options; settings are in parameters/calib_params.yaml)";

/// `parameters/calib_params.yaml`.
#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct Params {
    /// Camera indices, `0 -> sb00`.
    cameras: Vec<u32>,
    /// Marker frames to accumulate per camera before fitting.
    frames: u64,
    /// Overall cap on the accumulation, seconds.
    timeout_s: f64,
    /// A camera that saw fewer distinct board ids is skipped.
    min_markers: usize,
}

impl Params {
    fn load(path: &Path) -> Result<Self, String> {
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
        Ok(p)
    }
}

/// Resolved file locations, all from this crate's folder.
struct Paths {
    repo: PathBuf,
    params: PathBuf,
    board: PathBuf,
    intrinsics: PathBuf,
    extrinsics: PathBuf,
}

impl Paths {
    fn resolve() -> Self {
        let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
        let repo = crate_dir
            .parent()
            .expect("the crate sits inside the repository")
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

fn main() {
    if let Some(a) = std::env::args().nth(1) {
        eprintln!("error: unexpected argument '{a}'");
        eprintln!("{USAGE}");
        std::process::exit(2);
    }
    match run() {
        Ok(0) => {
            eprintln!("error: no camera could be fitted");
            std::process::exit(1);
        }
        Ok(_) => {}
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
    }
}

/// The whole run. Returns how many `sbNN_full.yaml` were written.
fn run() -> Result<usize, String> {
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
        "targets: {}   frames/cam={}  timeout={}s  min_markers={}",
        targets.join(", "),
        params.frames,
        params.timeout_s,
        params.min_markers
    );
    println!("params file:  {}", paths.params.display());
    println!("board file:   {}", paths.board.display());
    println!(
        "output files: {}",
        writer::full_yaml_path(&paths.extrinsics, "sbNN").display()
    );

    // Each camera's camchain up front: K and dist go into its sbNN_full.yaml. A
    // camera without one is skipped and not waited for.
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
    let mut acc = Accumulator::new(
        cams.iter().map(|(inst, _)| inst.clone()).collect(),
        params.frames,
        &cfg.board.family,
    );

    // Fan-in subscriber, declared directly: the generated MarkersSubscriber's
    // try_recv_latest collapses the queue to the newest sample, which would drop
    // the other cameras' frames. Every frame counts here, so drain with try_recv.
    let key = accumulate::markers_key();
    let session = zenoh::open(zenoh::Config::default())
        .wait()
        .map_err(|e| format!("zenoh open: {e}"))?;
    let sub = session
        .declare_subscriber(key.as_str())
        .wait()
        .map_err(|e| format!("zenoh subscribe '{key}': {e}"))?;
    println!("subscribing to '{key}'  (custom/MarkerDetections)\n");

    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(params.timeout_s);
    let mut last_report = Instant::now();
    let mut undecodable: u64 = 0;
    while Instant::now() < deadline && !acc.done() {
        let mut any = false;
        while !acc.done() {
            let sample = match sub.try_recv() {
                Ok(Some(s)) => s,
                Ok(None) => break,
                Err(e) => return Err(format!("zenoh subscriber '{key}': {e}")),
            };
            any = true;
            let bytes = sample.payload().to_bytes();
            match MarkerDetections::decode(bytes.as_ref()) {
                Ok(msg) => {
                    if let Some(notice) = acc.ingest(&msg) {
                        println!("{notice}");
                    }
                }
                Err(e) => {
                    if undecodable == 0 {
                        println!(
                            "warning: undecodable sample on '{}' ({e}), ignored",
                            sample.key_expr()
                        );
                    }
                    undecodable += 1;
                }
            }
        }
        if !any {
            std::thread::sleep(Duration::from_millis(1));
        }
        if last_report.elapsed() >= Duration::from_secs(1) {
            println!("  accumulating  {}", acc.progress());
            last_report = Instant::now();
        }
    }
    if acc.done() {
        println!(
            "  every camera reached {} frames after {:.1} s",
            params.frames,
            start.elapsed().as_secs_f64()
        );
    } else {
        println!("  timeout after {} s  {}", params.timeout_s, acc.progress());
    }
    if undecodable > 0 {
        println!("  {undecodable} undecodable sample(s) ignored");
    }
    drop(sub);

    // Fit and write each camera.
    println!("\n=== results ===");
    let mut written = 0;
    for (inst, cc) in &cams {
        let n = acc.frames(inst);
        if n == 0 {
            println!("  {inst}: no frames received, skipped");
            continue;
        }
        // Matched (cam, world) pairs by id; median over the frames for each id.
        let (ids, cam_pts, world_pts) = acc.matched(inst, &world);
        if ids.len() < params.min_markers {
            println!(
                "  {inst}: only {} usable marker(s) ({n} frames), need >={}, skipped",
                ids.len(),
                params.min_markers
            );
            continue;
        }
        let Some((r, t)) = fit::fit_extrinsics(&cam_pts, &world_pts) else {
            println!("  {inst}: degenerate marker geometry (ids={ids:?}), skipped");
            continue;
        };
        let rms = fit::reprojection_residual(&cam_pts, &world_pts, &r, &t);

        let out = writer::full_yaml_path(&paths.extrinsics, inst);
        let label = writer::display_relative(&cc.source, &paths.repo);
        let yaml = FullYaml {
            camera: inst,
            world_frame: &cfg.world_frame,
            camchain: cc,
            camchain_label: &label,
            r,
            t,
            residual_rms_m: rms,
            markers_used: &ids,
            frames: n,
        };
        if let Err(e) = yaml.write(&out) {
            println!("  {inst}: cannot write '{}' ({e}), skipped", out.display());
            continue;
        }
        written += 1;
        println!(
            "  {inst}: t=({:>8.4},{:>8.4},{:>8.4}) m  rms={:.4} m  ids={:?}  frames={n}  -> {}",
            t[0],
            t[1],
            t[2],
            rms,
            ids,
            out.display()
        );
    }
    Ok(written)
}
