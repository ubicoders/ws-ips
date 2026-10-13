//! ips_viz: rerun.io viewer for the IPS pipeline.
//!
//! Subscribes to the fused cube poses ips_track publishes over zenoh
//! (`custom/CubePoses` on `<device>/ws_ips/ips_track/zenoh/cube_poses`) and
//! streams them, plus the static camera poses, to a running rerun viewer. The
//! tracker only publishes; the best-effort rerun streaming is this process's job,
//! so a viewer hiccup cannot stall tracking.
//!
//! It also subscribes over iceoryx2 to the per-camera color debug images that
//! ips_detect publishes when `debug: true` (`sensor_msgs/Image1280x1024Color` on
//! `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image`), one zero-copy subscriber
//! per camera, and logs the latest frame of each at `rerun_hz`. A camera that is
//! not publishing leaves its panel empty.
//!
//! The layout is driven by the four `debug_show_*` flags of `viz_params.yaml`,
//! each adding a column to the left of the always present 3D world view; see
//! `Visualizer::connect`.
//!
//! Two things the cube_poses message does not carry come from elsewhere:
//!   * cube box sizes, from `viz_params.yaml`;
//!   * camera poses, fetched once at startup from ips_track's cam_param service
//!     (GET `<device>/ws_ips/ips_track/zenoh/service/cam_param/sbNN`) and logged
//!     static. The request is retried a few rounds because the queryable is
//!     often not discovered yet when the first GET goes out; a camera that still
//!     does not answer is skipped with a warning and not drawn.
//!
//! No command-line options: every value comes from `parameters/viz_params.yaml`.
//! Start a rerun viewer first (`rerun`); with none up rerun buffers and
//! reconnects, so the order is not critical.

use ips_viz::cube_poses_sub::{self, CubePosesSubscriber};
use ips_viz::debug_image_sub::{self, DebugImageSubscriber};
use ips_viz::msgs::CubePoses;
use ips_viz::visualize::{CameraExtrinsics, CubeModel, Visualizer, VizParams};

use iceoryx2::prelude::*;
use prost::Message;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use zenoh::Wait;

const USAGE: &str =
    "usage: ips_viz   (no options; parameters come from parameters/viz_params.yaml)";

/// The module that publishes cube_poses and serves cam_param.
const TRACK_MODULE: &str = "ips_track";
/// The module that publishes the per-camera debug images.
const DETECT_MODULE: &str = "ips_detect";

/// How long to wait for the cam_param service to answer one camera per attempt.
const CAM_PARAM_TIMEOUT: Duration = Duration::from_secs(3);
/// How many rounds of GETs to try before a camera is given up on. The first GETs
/// can leave before this session has discovered ips_track's queryable (zenoh
/// scouting takes a moment after `open`), so a single 3 s attempt right after
/// startup misses cameras that a retry a second later finds.
const CAM_PARAM_ATTEMPTS: usize = 5;
/// Pause between two rounds of GETs.
const CAM_PARAM_RETRY_DELAY: Duration = Duration::from_secs(1);
/// After the startup rounds, cameras still without extrinsics are asked for
/// again this often from the main loop, so a tracker that starts or restarts
/// later (after `ips_calib`) still gets its cameras drawn.
const CAM_PARAM_REFRESH: Duration = Duration::from_secs(5);

/// Idle wait of the main loop when no cube_poses sample is queued.
const IDLE_WAIT: Duration = Duration::from_millis(2);
/// Period of the console echo (10 Hz).
const ECHO_PERIOD: Duration = Duration::from_millis(100);

/// `<device>/ws_ips/ips_track/zenoh/cube_poses`, from this module's generated
/// device and workspace constants.
fn cube_poses_key() -> String {
    format!(
        "{}/{}/{TRACK_MODULE}/zenoh/cube_poses",
        cube_poses_sub::DEFAULT_DEVICE,
        cube_poses_sub::DEFAULT_WORKSPACE
    )
}

/// `<device>/ws_ips/ips_track/zenoh/service/cam_param/<inst>`.
fn cam_param_key(inst: &str) -> String {
    format!(
        "{}/{}/{TRACK_MODULE}/zenoh/service/cam_param/{inst}",
        cube_poses_sub::DEFAULT_DEVICE,
        cube_poses_sub::DEFAULT_WORKSPACE
    )
}

/// `<device>/ws_ips/ips_detect-<inst>/iox2/debug_image`.
fn debug_image_key(inst: &str) -> String {
    format!(
        "{}/{}/{DETECT_MODULE}-{inst}/iox2/debug_image",
        debug_image_sub::DEFAULT_DEVICE,
        debug_image_sub::DEFAULT_WORKSPACE
    )
}

/// GET one camera's calibration JSON from the cam_param service. Returns the
/// first reply decoded as JSON, or an error if the service errored, sent
/// undecodable bytes, or did not answer within [`CAM_PARAM_TIMEOUT`].
fn fetch_cam_param(session: &zenoh::Session, inst: &str) -> Result<serde_json::Value, String> {
    let key = cam_param_key(inst);
    let replies = session
        .get(&key)
        .timeout(CAM_PARAM_TIMEOUT)
        .wait()
        .map_err(|e| format!("get '{key}': {e}"))?;
    if let Ok(reply) = replies.recv() {
        return match reply.into_result() {
            Ok(sample) => serde_json::from_slice(&sample.payload().to_bytes())
                .map_err(|e| format!("decode '{key}': {e}")),
            Err(err) => {
                let msg = err
                    .payload()
                    .try_to_string()
                    .map(|s| s.into_owned())
                    .unwrap_or_else(|_| "<non-utf8 error payload>".into());
                Err(format!("service replied error: {msg}"))
            }
        };
    }
    Err(format!("no reply on '{key}', is ips_track running?"))
}

/// Fetch every camera's extrinsics, retried for up to [`CAM_PARAM_ATTEMPTS`]
/// rounds. A camera that has not answered after the last round is reported and
/// left out, so it is not drawn.
fn fetch_one_extrinsics(session: &zenoh::Session, inst: &str) -> Result<CameraExtrinsics, String> {
    fetch_cam_param(session, inst)
        .and_then(|doc| CameraExtrinsics::from_json(&doc, inst).map_err(|e| e.to_string()))
}

/// Returns the extrinsics fetched and the cameras still missing, which the main
/// loop keeps asking for every [`CAM_PARAM_REFRESH`].
fn fetch_all_extrinsics(
    session: &zenoh::Session,
    insts: &[String],
) -> (HashMap<String, CameraExtrinsics>, Vec<String>) {
    let mut exts: HashMap<String, CameraExtrinsics> = HashMap::new();
    let mut pending: Vec<String> = insts.to_vec();
    let mut last_err: HashMap<String, String> = HashMap::new();
    for round in 1..=CAM_PARAM_ATTEMPTS {
        if pending.is_empty() {
            break;
        }
        if round > 1 {
            std::thread::sleep(CAM_PARAM_RETRY_DELAY);
        }
        let mut still_pending = Vec::new();
        for inst in pending {
            match fetch_one_extrinsics(session, &inst) {
                Ok(e) => {
                    exts.insert(inst, e);
                }
                Err(e) => {
                    last_err.insert(inst.clone(), e);
                    still_pending.push(inst);
                }
            }
        }
        pending = still_pending;
        if !pending.is_empty() && round < CAM_PARAM_ATTEMPTS {
            eprintln!(
                "cam_param: no answer yet for [{}], retry {}/{}",
                pending.join(", "),
                round + 1,
                CAM_PARAM_ATTEMPTS
            );
        }
    }
    for inst in &pending {
        eprintln!(
            "warning: {inst}: no usable extrinsics yet ({}), not drawn; asking again every {} s",
            last_err
                .get(inst)
                .map(String::as_str)
                .unwrap_or("unknown error"),
            CAM_PARAM_REFRESH.as_secs()
        );
    }
    (exts, pending)
}

fn fail(msg: String, code: i32) -> ! {
    eprintln!("error: {msg}");
    eprintln!("{USAGE}");
    std::process::exit(code);
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Single instance, no options: anything on the command line is an error.
    if let Some(arg) = std::env::args().nth(1) {
        fail(format!("unexpected argument '{arg}'"), 2);
    }

    // viz_params.yaml: cube geometry, which cameras to draw, layout flags.
    let params_path = VizParams::default_path();
    let cfg = VizParams::load(&params_path).unwrap_or_else(|e| fail(e, 1));
    let model = CubeModel::new(cfg.cube_width, cfg.cube_length, cfg.cube_height);
    // The full instance list drives both the debug-image subscribers and the 2x2
    // grid in the blueprint: every configured camera gets a panel even without
    // extrinsics, so the grid stays 2x2 and a silent camera just shows empty.
    let insts = cfg.camera_names();

    let cube_key = cube_poses_key();
    println!("ips_viz: params     {}", params_path.display());
    println!("ips_viz: cameras    [{}]", insts.join(", "));
    println!("ips_viz: cube_poses {cube_key}");
    for inst in &insts {
        println!("ips_viz: cam_param  {}", cam_param_key(inst));
    }

    // One zenoh session for the whole process: it fetches the static camera
    // extrinsics (request/response against ips_track's cam_param service) and
    // subscribes to the live cube_poses stream.
    let session = zenoh::open(zenoh::Config::default())
        .wait()
        .map_err(|e| e.to_string())?;

    let (exts, mut missing) = fetch_all_extrinsics(&session, &insts);
    let mut loaded: Vec<&String> = exts.keys().collect();
    loaded.sort();
    println!(
        "ips_viz: cube {:.0}x{:.0}x{:.0} mm (viz x{})  cameras drawn=[{}]",
        cfg.cube_width * 1000.0,
        cfg.cube_length * 1000.0,
        cfg.cube_height * 1000.0,
        cfg.cube_viz_scale,
        loaded
            .iter()
            .map(|s| s.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    );

    // Connect to the rerun viewer (ships the layout selected by the debug_show_*
    // flags) and log the static camera poses once. Best effort: rerun buffers and
    // reconnects if no viewer is up yet. `mut` because the per-cube plot styling
    // is logged lazily on first sight of each cube.
    let mut viz = Visualizer::connect(
        &insts,
        cfg.cube_viz_scale,
        cfg.debug_show_images,
        cfg.debug_show_pos_xyz,
        cfg.debug_show_vel_xyz,
        cfg.debug_show_ang_xyz,
    )?;
    if let Err(e) = viz.log_cameras(&exts) {
        eprintln!("warning: visualize: failed to log camera poses: {e}");
    }

    // zenoh: subscribe to ips_track's cube_poses on the session opened above.
    // The generated subscriber defaults to this module's own segment, so the
    // foreign key is passed verbatim through `full`.
    let sub = CubePosesSubscriber::open_with(
        &session,
        cube_poses_sub::TopicOverrides {
            full: Some(cube_key.clone()),
            ..Default::default()
        },
    )
    .map_err(|e| e.to_string())?;
    println!("subscribing to '{cube_key}'  (custom/CubePoses)\n");

    // iox2: one debug-image subscriber per camera, each on that camera's
    // verbatim service via `full` (iox2 has no wildcard). The service is created
    // on open even if the publisher is not up yet, so a camera running with
    // debug: false simply never delivers a frame (empty panel).
    //
    // Only opened when the image grid is part of the layout: with no panel to
    // feed there is no point subscribing or doing the per-frame RGB copy, so an
    // empty `img_subs` makes the image step below a no-op. The node is kept for
    // the loop's `node.wait`, which returns on SIGINT/SIGTERM.
    let node = if cfg.debug_show_images {
        Some(NodeBuilder::new().create::<ipc::Service>()?)
    } else {
        None
    };
    let mut img_subs: Vec<(String, DebugImageSubscriber)> = Vec::new();
    if let Some(node) = node.as_ref() {
        for inst in &insts {
            let full = debug_image_key(inst);
            let ov = debug_image_sub::TopicOverrides {
                full: Some(full.clone()),
                ..Default::default()
            };
            img_subs.push((inst.clone(), DebugImageSubscriber::open_with(node, ov)?));
            println!("subscribing to '{full}'  (sensor_msgs/Image1280x1024Color)");
        }
        println!(
            "{} debug image service(s), logging to rerun at {:.0} Hz\n",
            img_subs.len(),
            cfg.rerun_hz
        );
    } else {
        println!("debug_show_images: false, image grid off, not subscribing to debug images\n");
    }
    let img_period = Duration::from_secs_f64(1.0 / cfg.rerun_hz.max(1.0));
    let mut last_img = Instant::now();

    // Drain to the latest set each poll and mirror it to rerun. `try_recv_latest`
    // collapses any backlog to the newest sample, which is right for a viewer (an
    // intermediate dropped frame is invisible). A 10 Hz console echo makes it
    // verifiable from the terminal.
    let mut last_print = Instant::now();
    let mut last_refetch = Instant::now();
    loop {
        // Cameras the tracker could not answer for at startup (tracker not up yet,
        // or not calibrated yet) are asked for again until they arrive, and drawn
        // as soon as they do.
        if !missing.is_empty() && last_refetch.elapsed() >= CAM_PARAM_REFRESH {
            last_refetch = Instant::now();
            let mut found: HashMap<String, CameraExtrinsics> = HashMap::new();
            missing.retain(|inst| match fetch_one_extrinsics(&session, inst) {
                Ok(e) => {
                    found.insert(inst.clone(), e);
                    false
                }
                Err(_) => true,
            });
            if !found.is_empty() {
                let mut names: Vec<&String> = found.keys().collect();
                names.sort();
                println!(
                    "cam_param: extrinsics arrived for [{}]",
                    names.iter().map(|s| s.as_str()).collect::<Vec<_>>().join(", ")
                );
                if let Err(e) = viz.log_cameras(&found) {
                    eprintln!("warning: visualize: failed to log camera poses: {e}");
                }
            }
        }

        // Throttle the heavy color images to rerun_hz: drain each camera to its
        // newest frame and log it. A camera with nothing queued is left alone, so
        // its panel keeps the last frame (or stays empty if it never published).
        if last_img.elapsed() >= img_period {
            last_img = Instant::now();
            for (inst, isub) in &img_subs {
                match isub.try_recv_latest() {
                    Ok(Some(sample)) => {
                        let n = (sample.data_len as usize).min(sample.data.len());
                        let rgb = sample.data[..n].to_vec();
                        let stamp = sample.header_stamp_sec as f64
                            + sample.header_stamp_nanosec as f64 * 1e-9;
                        drop(sample); // release the shmem slot before the rerun copy
                        if let Err(e) = viz.log_debug_image(inst, stamp, rgb) {
                            eprintln!("warning: {inst}: log debug image failed: {e}");
                        }
                    }
                    Ok(None) => {}
                    Err(e) => eprintln!("warning: {inst}: image recv failed: {e}"),
                }
            }
        }

        match sub.try_recv_latest() {
            Ok(Some(bytes)) => {
                let msg = match CubePoses::decode(bytes.as_slice()) {
                    Ok(m) => m,
                    Err(_) => continue,
                };
                let stamp = msg.stamp_sec as f64 + msg.stamp_nanosec as f64 * 1e-9;
                if let Err(e) = viz.log_cubes(msg.frame_seq, stamp, &msg.cubes, &model) {
                    eprintln!("warning: visualize: log_cubes failed: {e}");
                }
                if last_print.elapsed() >= ECHO_PERIOD {
                    last_print = Instant::now();
                    let alive = msg.cubes.iter().filter(|c| c.alive).count();
                    println!(
                        "[viz] seq {}  cubes={} (alive {alive})",
                        msg.frame_seq,
                        msg.cubes.len()
                    );
                }
            }
            Ok(None) => match node.as_ref() {
                // iceoryx2 captures SIGINT/SIGTERM once it is in use, so the
                // loop must ask the node; `wait` errs on a termination request.
                Some(node) => {
                    if node.wait(IDLE_WAIT).is_err() {
                        println!("ips_viz: termination requested, exiting");
                        break;
                    }
                }
                None => std::thread::sleep(IDLE_WAIT),
            },
            Err(e) => {
                eprintln!("error: subscribe failed: {e}");
                break;
            }
        }
    }
    Ok(())
}
