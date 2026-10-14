//! ips_track: fuse every camera's marker detections into rigid cube poses.
//!
//! Subscribes to every detector's marker stream
//! (`<device>/ws_ips/ips_detect-$*/zenoh/markers`), lifts each detected marker
//! into the world frame with that camera's `sbNN_full.yaml` extrinsics, and,
//! knowing how the five markers sit on a cube, fits the single world pose per
//! cube that best maps the body-frame marker layout onto all observed markers
//! (`world = R * body + t`). Publishes `custom/CubePoses` on
//! `<device>/ws_ips/ips_track/zenoh/cube_poses` at a fixed `rate_hz`.
//!
//! Each tick takes the latest frame of every live camera (seen within
//! `stale_ms`), fits once per cube, smooths with the configured filter, and
//! publishes once, so the output rate is steady whatever the cameras' frames do.
//! `CubePoses.stamp_*` is the newest capture stamp among the frames that fed this
//! tick's fits. It never decreases: a tick that fits nothing repeats the last
//! stamp, and only the ticks before the first fit use the wall clock.
//!
//! Each `CubePose` carries a liveness heartbeat: a cube enters the roster on its
//! first fit and is reported on every tick after, `alive` while fitted within
//! `alive_ms`, then `alive=false` with the pose cleared. `last_seen_*` is the wall
//! clock of its last fit. `cubes` is empty only before the first detection.
//!
//! No command-line options: every value comes from
//! `parameters/tracker_params.yaml`. The tracker also hosts the cam_param service
//! (see `ips_track::cam_param`).

use ips_track::cam_param;
use ips_track::cube::{self, CameraExtrinsics, Candidate, CubeModel, FusedPose};
use ips_track::cube_poses_pub::{self, CubePosesPublisher};
use ips_track::filter::{Filter, Tuning};
use ips_track::fps::FpsMeter;
use ips_track::markers_sub;
use ips_track::msgs::{CubePose, CubePoses, MarkerDetections, Mat3, Quat, Vec3};

use nalgebra::{Matrix3, Vector3};
use prost::Message;
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use zenoh::Wait;

const USAGE: &str =
    "usage: ips_track  (no options; every value comes from parameters/tracker_params.yaml)";

/// The detector module whose marker streams the tracker fans in.
const DETECT_MODULE: &str = "ips_detect";

fn crate_dir() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

/// `path` with `..` resolved when it exists, for the banner; else as given.
fn display_path(path: PathBuf) -> PathBuf {
    path.canonicalize().unwrap_or(path)
}

/// Camera index to instance name: `0 -> "sb00"`, `3 -> "sb03"`.
fn instance_name(index: u32) -> String {
    format!("sb{index:02}")
}

/// The fan-in key `<device>/ws_ips/ips_detect-$*/zenoh/markers`, rendered
/// through the generated `TopicOverrides`. `$*` is zenoh's sub-segment wildcard:
/// it matches every `ips_detect-sbNN` instance.
fn markers_fan_in_key() -> String {
    markers_sub::TopicOverrides {
        full: Some(format!(
            "{}/{}/{}-$*/{}/{}",
            markers_sub::DEFAULT_DEVICE,
            markers_sub::DEFAULT_WORKSPACE,
            DETECT_MODULE,
            markers_sub::TRANSPORT_SEGMENT,
            markers_sub::DEFAULT_TOPIC,
        )),
        ..Default::default()
    }
    .key_expr()
}

fn fail(msg: String) -> ! {
    eprintln!("error: {msg}");
    std::process::exit(1);
}

/// Every parameter of `ips_track`, from `parameters/tracker_params.yaml`. Every
/// key is required: the yaml is the single source of the values.
#[derive(serde::Deserialize)]
struct TrackerParams {
    cube_width: f64,
    cube_height: f64,
    cube_length: f64,
    marker_size: f64,
    /// 0-based cube ids to fit and publish; markers of any other cube are
    /// ignored. Empty = every cube seen.
    cube_ids: Vec<u32>,
    /// Camera indices whose extrinsics to load (`i` loads `sb{i:02}_full.yaml`).
    cameras: Vec<u32>,
    /// `true`: fit every detected marker, no reject pass (`pos_tol`, `ang_tol`
    /// unused). `false`: greedy reject of markers that disagree past the
    /// tolerances, keeping the largest self-consistent subset.
    use_all: bool,
    pos_tol: f64,
    ang_tol: f64,
    /// Fixed fit and publish rate (Hz).
    rate_hz: f64,
    /// Drop a camera whose latest frame arrived longer ago than this.
    stale_ms: u64,
    /// A cube not fitted for this long publishes `alive=false`.
    alive_ms: u64,
    /// `none` | `lp` | `oneeuro` | `kf`.
    filter: String,
    lp_pose: f64,
    lp_angle: f64,
    oe_min_cutoff_pos: f64,
    oe_beta_pos: f64,
    oe_min_cutoff_ang: f64,
    oe_beta_ang: f64,
    kf_q_pos: f64,
    kf_r_pos: f64,
    kf_q_ang: f64,
    kf_r_ang: f64,
    kf_lookahead_ms: u64,
}

impl TrackerParams {
    fn tuning(&self) -> Tuning {
        Tuning {
            name: self.filter.clone(),
            lp_pose: self.lp_pose,
            lp_angle: self.lp_angle,
            oe_min_cutoff_pos: self.oe_min_cutoff_pos,
            oe_beta_pos: self.oe_beta_pos,
            oe_min_cutoff_ang: self.oe_min_cutoff_ang,
            oe_beta_ang: self.oe_beta_ang,
            kf_q_pos: self.kf_q_pos,
            kf_r_pos: self.kf_r_pos,
            kf_q_ang: self.kf_q_ang,
            kf_r_ang: self.kf_r_ang,
            kf_lookahead_ms: self.kf_lookahead_ms,
        }
    }
}

/// Resolved per-run settings handed to [`fuse_and_publish`].
struct Settings {
    /// `None` (`use_all: true`) fits every marker; `Some((pos_tol_m,
    /// ang_tol_deg))` runs the greedy reject pass.
    reject: Option<(f64, f64)>,
    stale: Duration,
    /// Fixed tick period (1 / `rate_hz`).
    period: Duration,
    /// Cube liveness window.
    alive: Duration,
    /// 0-based cube ids to keep; empty = keep all.
    cube_ids: HashSet<u32>,
}

/// Flatten a 3x3 into row-major 9 for the `Mat3` proto field.
fn flatten(r: &Matrix3<f64>) -> Vec<f64> {
    let mut v = Vec::with_capacity(9);
    for i in 0..3 {
        for j in 0..3 {
            v.push(r[(i, j)]);
        }
    }
    v
}

/// Wall clock since the Unix epoch.
fn wall_now() -> Duration {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
}

/// Rolling 1-second diagnostic: how many distinct cameras land in each fitted
/// cube, and how often each camera's markers survive the reject pass.
///
/// `in_hist` counts the cameras fed into the fit (before the reject pass),
/// `cams_hist` the cameras that survived it. Inputs high and survivors low
/// means the cameras disagree past `pos_tol`/`ang_tol` and the reject pass is
/// culling them (an extrinsics problem), not a detection problem.
struct Stats {
    start: Instant,
    fits: u64,
    in_hist: [u64; 5],   // index = distinct cameras fed in, 0..=4
    cams_hist: [u64; 5], // index = num_cameras surviving the fit, 0..=4
    contrib: HashMap<String, u64>,
}

impl Stats {
    fn new(now: Instant) -> Self {
        Self {
            start: now,
            fits: 0,
            in_hist: [0; 5],
            cams_hist: [0; 5],
            contrib: HashMap::new(),
        }
    }

    fn record(&mut self, f: &FusedPose, n_input: usize) {
        self.fits += 1;
        self.in_hist[n_input.min(4)] += 1;
        self.cams_hist[f.num_cameras.min(4)] += 1;
        for cam in &f.cameras {
            *self.contrib.entry(cam.clone()).or_default() += 1;
        }
    }

    /// Print a rollup and reset once a second has elapsed.
    fn maybe_report(&mut self, now: Instant) {
        if now.duration_since(self.start) < Duration::from_secs(1) {
            return;
        }
        let contrib_s = {
            let mut c: Vec<(&String, &u64)> = self.contrib.iter().collect();
            c.sort_by(|a, b| a.0.cmp(b.0));
            c.iter()
                .map(|(cam, n)| format!("{cam}:{n}"))
                .collect::<Vec<_>>()
                .join(" ")
        };
        println!(
            "[stats] {:.1}s fits={}  in[1:{} 2:{} 3:{} 4:{}]  cams[1:{} 2:{} 3:{} 4:{}]  contrib {{{}}}",
            now.duration_since(self.start).as_secs_f64(),
            self.fits,
            self.in_hist[1],
            self.in_hist[2],
            self.in_hist[3],
            self.in_hist[4],
            self.cams_hist[1],
            self.cams_hist[2],
            self.cams_hist[3],
            self.cams_hist[4],
            contrib_s,
        );
        *self = Stats::new(now);
    }
}

/// Per-cube liveness state for the heartbeat roster. A cube gets one on its
/// first successful fit and keeps it. `pose` is the last good fit (carried while
/// alive, cleared on the wire once stale); `last_seen` (monotonic) drives the
/// alive window, `last_seen_wall` is the wall clock for `last_seen_*`.
struct CubeTrack {
    pose: CubePose,
    last_seen: Instant,
    last_seen_wall: Duration,
    /// Cameras whose markers fed the last fit, sorted, for the `cams=[..]` echo.
    cameras: Vec<String>,
    /// Cameras fed into the last fit but fully culled by the reject pass,
    /// sorted, for the `rej=[..]` echo.
    rejected: Vec<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    if let Some(arg) = std::env::args().nth(1) {
        eprintln!("error: unexpected argument '{arg}'");
        eprintln!("{USAGE}");
        std::process::exit(2);
    }

    // Parameters.
    let params_path = crate_dir().join("parameters").join("tracker_params.yaml");
    let text = std::fs::read_to_string(&params_path)
        .unwrap_or_else(|e| fail(format!("cannot read '{}': {e}", params_path.display())));
    let cfg: TrackerParams = serde_yaml::from_str(&text)
        .unwrap_or_else(|e| fail(format!("parse '{}': {e}", params_path.display())));
    for (name, a) in [("lp_pose", cfg.lp_pose), ("lp_angle", cfg.lp_angle)] {
        if !(0.0..=1.0).contains(&a) {
            fail(format!(
                "tracker_params: {name}={a} out of range (must be 0..1)"
            ));
        }
    }
    if !cfg.rate_hz.is_finite() || cfg.rate_hz <= 0.0 {
        fail(format!(
            "tracker_params: rate_hz must be > 0 (got {})",
            cfg.rate_hz
        ));
    }
    let model = CubeModel::new(
        cfg.cube_width,
        cfg.cube_length,
        cfg.cube_height,
        cfg.marker_size,
    );
    let settings = Settings {
        reject: if cfg.use_all {
            None
        } else {
            Some((cfg.pos_tol, cfg.ang_tol))
        },
        stale: Duration::from_millis(cfg.stale_ms),
        period: Duration::from_secs_f64(1.0 / cfg.rate_hz),
        alive: Duration::from_millis(cfg.alive_ms),
        cube_ids: cfg.cube_ids.iter().copied().collect(),
    };
    let tuning = cfg.tuning();
    let mut filter =
        Filter::select(&tuning).unwrap_or_else(|e| fail(format!("tracker_params: {e}")));

    // Per-camera extrinsics. A camera without a usable file is skipped: its
    // detections cannot be lifted to world, so they are ignored on the wire.
    let ext_dir = display_path(
        crate_dir()
            .join("../..")
            .join("calibration")
            .join("extrinsics"),
    );
    let mut exts: HashMap<String, CameraExtrinsics> = HashMap::new();
    let mut ext_lines: Vec<String> = Vec::new();
    for &i in &cfg.cameras {
        let inst = instance_name(i);
        let path = ext_dir.join(format!("{inst}_full.yaml"));
        match CameraExtrinsics::load(&path) {
            Ok(e) => {
                ext_lines.push(format!("  {inst}: {}", path.display()));
                exts.insert(inst, e);
            }
            Err(e) => eprintln!("warning: {inst}: no usable extrinsics ({e}); camera skipped"),
        }
    }

    let mut loaded: Vec<&String> = exts.keys().collect();
    loaded.sort();
    let cube_ids_s = if settings.cube_ids.is_empty() {
        "all".to_string()
    } else {
        let mut v: Vec<u32> = settings.cube_ids.iter().copied().collect();
        v.sort_unstable();
        v.iter()
            .map(|c| c.to_string())
            .collect::<Vec<_>>()
            .join(",")
    };
    println!(
        "ips_track: cube {:.0}×{:.0}×{:.0} mm  marker {:.0} mm  cube_ids=[{}]  cameras=[{}]",
        cfg.cube_width * 1000.0,
        cfg.cube_length * 1000.0,
        cfg.cube_height * 1000.0,
        cfg.marker_size * 1000.0,
        cube_ids_s,
        loaded
            .iter()
            .map(|s| s.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    );
    println!("parameters: {}", params_path.display());
    println!("extrinsics: {}", ext_dir.display());
    for line in &ext_lines {
        println!("{line}");
    }
    if exts.is_empty() {
        eprintln!(
            "warning: no camera extrinsics loaded from {}; publishing empty heartbeats. \
             Run ips_calib, then restart ips_track.",
            ext_dir.display()
        );
    }
    match settings.reject {
        None => println!(
            "fit: use_all=true, least squares over every detected marker (no reject; pos_tol/ang_tol unused)  stale={} ms",
            settings.stale.as_millis()
        ),
        Some((pos_tol, ang_tol)) => println!(
            "fit: reject  pos_tol={pos_tol:.3} m  ang_tol={ang_tol:.0}°  stale={} ms",
            settings.stale.as_millis()
        ),
    }
    println!(
        "rate: {:.1} Hz fixed  (tick period {:.2} ms; latest frame per live camera each tick)",
        cfg.rate_hz,
        settings.period.as_secs_f64() * 1000.0
    );
    println!(
        "heartbeat: alive window {} ms  (a cube unseen longer publishes alive=false)",
        settings.alive.as_millis()
    );
    println!("{}", filter.describe(&tuning));

    // zenoh: the fan-in markers subscriber and the cube_poses publisher. The
    // generated MarkersSubscriber is not used: its try_recv_latest collapses the
    // fan-in queue to the newest sample, which would drop the other cameras'
    // frames. The subscriber is declared directly and drained with try_recv.
    let session = zenoh::open(zenoh::Config::default())
        .wait()
        .map_err(|e| e.to_string())?;
    let sub_key = markers_fan_in_key();
    let sub = session
        .declare_subscriber(sub_key.clone())
        .wait()
        .map_err(|e| e.to_string())?;
    let publisher = CubePosesPublisher::open(&session).map_err(|e| e.to_string())?;
    println!("subscribing to '{sub_key}'");
    println!(
        "publishing cube poses on '{}'  (custom/CubePoses)",
        cube_poses_pub::TopicOverrides::default().key_expr()
    );

    // The cam_param service, before the tick loop. Non-blocking; `_cam_service`
    // keeps its routes and session alive until main exits.
    let _cam_service = cam_param::start_cam_param_service(ext_dir.clone());
    println!();

    // Fixed-rate model: `pending` holds the latest frame per camera (the drain
    // keeps it current), `last_seen` is per-camera liveness. Every tick takes
    // whatever is latest per live camera and fits and publishes exactly once.
    let mut pending: HashMap<String, MarkerDetections> = HashMap::new();
    let mut last_seen: HashMap<String, Instant> = HashMap::new();
    let mut seq: u64 = 0;
    let mut fps = FpsMeter::new(Duration::from_secs(1));
    let mut last_print = Instant::now();
    let mut stats = Stats::new(Instant::now());
    // Liveness roster: every cube seen at least once.
    let mut tracks: HashMap<u32, CubeTrack> = HashMap::new();
    // Last published `CubePoses` stamp, so the stamp never decreases.
    let mut last_stamp: Option<(i32, u32)> = None;

    let period = settings.period;
    let mut next_tick = Instant::now() + period;
    loop {
        // Drain every queued frame; newer frames of a camera overwrite older ones.
        loop {
            let sample = match sub.try_recv() {
                Ok(Some(s)) => s,
                Ok(None) => break,
                Err(_) => break,
            };
            let bytes = sample.payload().to_bytes();
            let msg = match MarkerDetections::decode(bytes.as_ref()) {
                Ok(m) => m,
                Err(_) => continue,
            };
            let cam = msg.camera.clone();
            last_seen.insert(cam.clone(), Instant::now());
            pending.insert(cam, msg);
        }

        fuse_and_publish(
            &pending,
            &last_seen,
            &exts,
            &model,
            Instant::now(),
            &settings,
            &publisher,
            &mut seq,
            &mut fps,
            &mut last_print,
            &mut stats,
            &mut filter,
            &mut tracks,
            &mut last_stamp,
        )?;

        // Sleep to the next edge of the fixed clock. If a tick ran long, drop the
        // missed edge and re-phase from now rather than bursting to catch up.
        let now = Instant::now();
        if next_tick > now {
            std::thread::sleep(next_tick - now);
            next_tick += period;
        } else {
            next_tick = now + period;
        }
    }
}

/// Fit one set (latest frame per live camera) into cube poses and publish one
/// `CubePoses` message. The published `cubes` is the liveness roster (every cube
/// ever seen), not only this tick's fits: each freshly fitted cube refreshes its
/// [`CubeTrack`], then the whole roster is emitted, each cube stamped
/// `alive`/`last_seen_*` against the alive window. Echoes a throttled (~10 Hz)
/// summary to the console.
#[allow(clippy::too_many_arguments)]
fn fuse_and_publish(
    pending: &HashMap<String, MarkerDetections>,
    last_seen: &HashMap<String, Instant>,
    exts: &HashMap<String, CameraExtrinsics>,
    model: &CubeModel,
    now: Instant,
    settings: &Settings,
    publisher: &CubePosesPublisher,
    seq: &mut u64,
    fps: &mut FpsMeter,
    last_print: &mut Instant,
    stats: &mut Stats,
    filter: &mut Filter,
    tracks: &mut HashMap<u32, CubeTrack>,
    last_stamp: &mut Option<(i32, u32)>,
) -> Result<(), Box<dyn std::error::Error>> {
    // Body/world correspondences from every live camera's latest frame, by cube.
    // Only the cubes named in `cube_ids` are kept (empty = keep all).
    let mut by_cube: HashMap<u32, Vec<Candidate>> = HashMap::new();
    for (cam, m) in pending {
        let live = last_seen
            .get(cam)
            .is_some_and(|t| now.duration_since(*t) <= settings.stale);
        if !live {
            continue;
        }
        let ext = match exts.get(cam) {
            Some(e) => e,
            None => continue,
        };
        for det in &m.markers {
            let (cube_id, _) = cube::cube_and_face(det.id);
            if !settings.cube_ids.is_empty() && !settings.cube_ids.contains(&cube_id) {
                continue;
            }
            let (p, q) = match (&det.position, &det.orientation) {
                (Some(p), Some(q)) => (p, q),
                _ => continue,
            };
            let r_cam_mk = cube::quat_to_mat(q.x, q.y, q.z, q.w);
            let t_cam_mk = Vector3::new(p.x, p.y, p.z);
            let cand = cube::candidate_from_marker(model, ext, cam, det.id, &r_cam_mk, &t_cam_mk);
            by_cube.entry(cube_id).or_default().push(cand);
        }
    }

    // Wall clock of this tick: the last-seen stamp of every cube fitted now, and
    // the message stamp when nothing is fitted.
    let t = wall_now();
    // Newest capture stamp among the frames whose markers survived into a fit.
    let mut newest_stamp: Option<(i32, u32)> = None;

    // Axis-tip lever for the fit's orientation terms: half the marker side, so
    // the cube-spanning marker centres lead the rotation when 3 or more faces show.
    let lever = model.marker * 0.5;
    let mut cids: Vec<u32> = by_cube.keys().copied().collect();
    cids.sort_unstable();
    for cid in cids {
        let cands = &by_cube[&cid];
        // Distinct cameras fed into this cube's fit, before the reject pass.
        let mut input_cams: Vec<String> = cands
            .iter()
            .map(|c| c.camera.clone())
            .collect::<HashSet<_>>()
            .into_iter()
            .collect();
        input_cams.sort();
        let Some(f) = cube::fit_cube(cands, lever, settings.reject) else {
            continue;
        };
        stats.record(&f, input_cams.len());
        for cam in &f.cameras {
            if let Some(m) = pending.get(cam) {
                newest_stamp = newest_stamp.max(Some((m.stamp_sec, m.stamp_nanosec)));
            }
        }
        let o = filter.smooth(cid, now, &f);
        let pose = CubePose {
            cube_id: cid,
            position: Some(Vec3 {
                x: o.pos[0],
                y: o.pos[1],
                z: o.pos[2],
            }),
            rotation: Some(Mat3 { m: flatten(&o.rot) }),
            euler_deg: Some(Vec3 {
                x: o.euler_deg[0],
                y: o.euler_deg[1],
                z: o.euler_deg[2],
            }),
            orientation: Some(Quat {
                x: o.quat[0],
                y: o.quat[1],
                z: o.quat[2],
                w: o.quat[3],
            }),
            // World-frame linear velocity, Some only under the kf filter.
            velocity: o.vel.map(|v| Vec3 {
                x: v[0],
                y: v[1],
                z: v[2],
            }),
            marker_ids: f.marker_ids.clone(),
            num_cameras: f.num_cameras as u32,
            residual_rms: f.residual_rms,
            alive: true,
            last_seen_sec: t.as_secs() as i32,
            last_seen_nanosec: t.subsec_nanos(),
        };
        // f.cameras is already sorted and unique.
        let cams = f.cameras.clone();
        // Cameras fed in but fully culled by the reject pass.
        let rejected: Vec<String> = input_cams
            .iter()
            .filter(|c| !cams.contains(c))
            .cloned()
            .collect();
        tracks.insert(
            cid,
            CubeTrack {
                pose,
                last_seen: now,
                last_seen_wall: t,
                cameras: cams,
                rejected,
            },
        );
    }

    // Emit the full roster, sorted. A cube fitted within the alive window rides
    // with its last good pose and alive=true; one past the window stays as an
    // explicit alive=false heartbeat with the pose cleared.
    let mut roster: Vec<u32> = tracks.keys().copied().collect();
    roster.sort_unstable();
    let mut cubes: Vec<CubePose> = Vec::with_capacity(roster.len());
    for cid in roster {
        let track = &tracks[&cid];
        if now.duration_since(track.last_seen) <= settings.alive {
            cubes.push(track.pose.clone());
        } else {
            cubes.push(CubePose {
                cube_id: cid,
                alive: false,
                last_seen_sec: track.last_seen_wall.as_secs() as i32,
                last_seen_nanosec: track.last_seen_wall.subsec_nanos(),
                ..Default::default()
            });
        }
    }

    // One message per tick (an empty `cubes` is the pre-first-detection heartbeat).
    *seq += 1;
    // The stamp never goes backwards: a capture stamp lags the wall clock by the
    // detection and transport delay, so a fit right after a heartbeat tick would
    // otherwise step back in time. A tick with no contributing frame repeats the
    // last stamp; only the very first ticks fall back to the wall clock.
    let candidate = newest_stamp
        .or(*last_stamp)
        .unwrap_or((t.as_secs() as i32, t.subsec_nanos()));
    let (stamp_sec, stamp_nanosec) = match *last_stamp {
        Some(last) if last > candidate => last,
        _ => candidate,
    };
    *last_stamp = Some((stamp_sec, stamp_nanosec));
    let out = CubePoses {
        stamp_sec,
        stamp_nanosec,
        frame_seq: *seq,
        cubes,
    };
    publisher
        .publish(&out)
        .map_err(|e| e.to_string())?;

    // Throttled console echo (~10 Hz).
    let hz = fps.tick(now);
    if last_print.elapsed() >= Duration::from_millis(100) {
        *last_print = Instant::now();
        let hz_s = hz
            .map(|h| format!("{h:.0}"))
            .unwrap_or_else(|| "--".to_string());
        let alive_n = out.cubes.iter().filter(|c| c.alive).count();
        println!(
            "[track] seq {}  {hz_s} Hz  cubes={} (alive {alive_n})",
            *seq,
            out.cubes.len()
        );
        for c in &out.cubes {
            match (c.alive, c.position.as_ref(), c.euler_deg.as_ref()) {
                (true, Some(p), Some(e)) => {
                    let (cams, rej) = tracks
                        .get(&c.cube_id)
                        .map(|t| (t.cameras.join(", "), t.rejected.join(", ")))
                        .unwrap_or_default();
                    // Velocity only under the kf filter; blank for the others.
                    let vel_s = c
                        .velocity
                        .as_ref()
                        .map(|v| format!("  vel=({:>6.2},{:>6.2},{:>6.2}) m/s", v.x, v.y, v.z))
                        .unwrap_or_default();
                    println!(
                        "    cube {:<2} alive xyz=({:>7.3},{:>7.3},{:>7.3}) m  eul=({:>6.1},{:>6.1},{:>6.1})°{}  cams=[{}] rej=[{}] ids={:?} rms={:.3} m",
                        c.cube_id, p.x, p.y, p.z, e.x, e.y, e.z, vel_s, cams, rej, c.marker_ids, c.residual_rms
                    )
                }
                _ => println!(
                    "    cube {:<2} STALE  (alive=false, last_seen {}.{:09}s)",
                    c.cube_id, c.last_seen_sec, c.last_seen_nanosec
                ),
            }
        }
    }

    stats.maybe_report(now);
    Ok(())
}
