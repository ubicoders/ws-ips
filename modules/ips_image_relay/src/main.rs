//! ips_image_relay: make the cameras' iceoryx2 image streams readable over the
//! network. For each camera listed in `parameters/relay_params.yaml`, subscribe
//! to its two full-resolution shared-memory streams, downscale every frame by an
//! integer divisor with OpenCV (`INTER_AREA`), and republish it over zenoh as
//! protobuf `sensor_msgs/Image`, for zenoh peers (a phone app, a second machine)
//! that cannot read iceoryx2.
//!
//! Every setting comes from `parameters/relay_params.yaml` (cameras, scale,
//! every). The only command-line option is `--id sbNN`, which narrows the run
//! to that one camera. Without it, one relay thread runs per camera in the yaml,
//! all in this one process.
//!
//! Per camera sbNN, two streams, each relayed on its own:
//!
//! | In (iceoryx2) | Out (zenoh, protobuf `sensor_msgs/Image`) |
//! |---|---|
//! | `<device>/ws_ips/ips_cam_gcam_pub-sbNN/iox2/image`, `Image1280x1024`, MONO8 | `<device>/ws_ips/ips_image_relay-sbNN/zenoh/image` |
//! | `<device>/ws_ips/ips_detect-sbNN/iox2/debug_image`, `Image1280x1024Color`, RGB8 | `<device>/ws_ips/ips_image_relay-sbNN/zenoh/debug_image` |
//!
//! The topic segments match the sources; the module segment is
//! `ips_image_relay-sbNN`. `debug_image` exists only while ips_detect runs with
//! `debug: true`, so that stream may never arrive; the relay keeps waiting for
//! it and relays the camera image regardless. With the default `scale: 2` the
//! output is 640x512, a quarter of the bytes (about 0.3 MB mono, 1 MB colour).
//! Each output frame carries its source's header stamp, frame_id and encoding
//! (`mono8`, `rgb8`), its own width, height and unpadded step, and
//! `is_bigendian` 0. `every: N` relays only every Nth received frame per stream.
//!
//! Each thread prints `ips_image_relay-sbNN image 29.8 Hz, debug_image 0.0 Hz`
//! every 5 s: the rate of frames published on each output key.

use ips_image_relay::debug_image_pub::{self, DebugImagePublisher};
use ips_image_relay::image_pub::{self, ImagePublisher};
use ips_image_relay::rate::{RateCounter, REPORT_EVERY};
use ips_image_relay::relay::StreamRelay;
use ips_image_relay::source::{self, Stream};

use iceoryx2::prelude::*;
use std::path::Path;
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
    "usage: ips_image_relay [--id sbNN]   (every other setting is in parameters/relay_params.yaml)";

/// The fixed source geometry; `scale` must divide both so the output size is exact.
const SOURCE_WIDTH: u32 = 1280;
const SOURCE_HEIGHT: u32 = 1024;

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

fn d_scale() -> u32 {
    2
}
fn d_every() -> u32 {
    1
}

/// All settings for `ips_image_relay`, read from `parameters/relay_params.yaml`.
/// Missing optional fields fall back to the `d_*` defaults.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct RelayParams {
    /// Cameras to relay: index N -> instance sbNN. One relay thread per camera.
    cameras: Indices,
    /// Integer divisor of both image sides: 2 -> 640x512.
    #[serde(default = "d_scale")]
    scale: u32,
    /// Relay every Nth received frame per stream; 1 = all.
    #[serde(default = "d_every")]
    every: u32,
}

/// Resolved run settings (the yaml, validated, narrowed by `--id`).
struct Settings {
    /// Instance names to drive, one thread each, de-duplicated in yaml order.
    insts: Vec<String>,
    scale: u32,
    every: u64,
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

/// Validate the yaml and narrow it to `id` when given.
fn resolve(cfg: RelayParams, id: Option<&str>) -> Result<Settings, String> {
    if cfg.scale == 0
        || !SOURCE_WIDTH.is_multiple_of(cfg.scale)
        || !SOURCE_HEIGHT.is_multiple_of(cfg.scale)
    {
        return Err(format!(
            "scale: {} must be >= 1 and divide {SOURCE_WIDTH} and {SOURCE_HEIGHT} (1, 2, 4, 8, ...)",
            cfg.scale
        ));
    }
    if cfg.every == 0 {
        return Err("every: must be >= 1 (1 relays every frame)".to_string());
    }
    // De-dup the cameras (preserve order) so a repeated index doesn't start two
    // threads on the same camera.
    let mut insts: Vec<String> = Vec::new();
    for i in cfg.cameras.into_vec() {
        let inst = source::instance_name(i);
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
                "camera '{id}' is not in relay_params.yaml cameras [{}]",
                insts.join(", ")
            ));
        }
        insts = vec![id.to_string()];
    }
    Ok(Settings {
        insts,
        scale: cfg.scale,
        every: u64::from(cfg.every),
    })
}

fn fail(msg: String, code: i32) -> ! {
    eprintln!("error: {msg}");
    eprintln!("{USAGE}");
    std::process::exit(code);
}

/// `ips_image_relay-<inst>`, the module segment of this camera's output keys.
fn module_segment(inst: &str) -> String {
    format!("{}-{inst}", image_pub::DEFAULT_MODULE)
}

/// `<device>/ws_ips/ips_image_relay-<inst>/zenoh/image`.
fn image_overrides(inst: &str) -> image_pub::TopicOverrides {
    image_pub::TopicOverrides {
        module: Some(module_segment(inst)),
        ..Default::default()
    }
}

/// `<device>/ws_ips/ips_image_relay-<inst>/zenoh/debug_image`.
fn debug_image_overrides(inst: &str) -> debug_image_pub::TopicOverrides {
    debug_image_pub::TopicOverrides {
        module: Some(module_segment(inst)),
        ..Default::default()
    }
}

/// One camera's relay loop over both streams. Runs until SIGINT/SIGTERM (via
/// `node.wait`) or a fatal per-camera error, which is returned so the caller
/// can log it without taking the other cameras down. The thread owns its iox2
/// node and its publishers; the zenoh session is a clone of the process's one.
fn run_camera(
    inst: &str,
    session: &zenoh::Session,
    scale: u32,
    every: u64,
) -> Result<(), Box<dyn std::error::Error>> {
    let node = NodeBuilder::new().create::<ipc::Service>()?;

    let mut image = StreamRelay::new(inst, Stream::Image, scale, every);
    let mut debug = StreamRelay::new(inst, Stream::DebugImage, scale, every);

    // zenoh's error is `Box<dyn Error + Send + Sync>`, which `?` won't coerce
    // into `Box<dyn Error>`; stringify it at each zenoh boundary.
    let image_ov = image_overrides(inst);
    let image_key = image_ov.key_expr();
    let image_pub = ImagePublisher::open_with(session, image_ov).map_err(|e| e.to_string())?;
    let debug_ov = debug_image_overrides(inst);
    let debug_key = debug_ov.key_expr();
    let debug_pub = DebugImagePublisher::open_with(session, debug_ov).map_err(|e| e.to_string())?;

    let (out_w, out_h) = (SOURCE_WIDTH / scale, SOURCE_HEIGHT / scale);
    for (relay, key) in [(&image, &image_key), (&debug, &debug_key)] {
        let s = relay.source.stream;
        println!(
            "[{inst}] {:<11} <- iox2  '{}'  ({})",
            s.topic(),
            relay.source.key,
            s.source_type()
        );
        println!(
            "[{inst}] {:<11} -> zenoh '{key}'  (sensor_msgs/Image, {out_w}x{out_h})",
            s.topic()
        );
    }

    let label = module_segment(inst);
    let now = Instant::now();
    let mut image_rate = RateCounter::new(now);
    let mut debug_rate = RateCounter::new(now);
    let mut last_report = now;

    while keep_running(&node, Duration::from_micros(1000)) {
        if let Some(msg) = image.poll(&node)? {
            image_pub.publish(msg).map_err(|e| e.to_string())?;
            image_rate.tick();
        }
        if let Some(msg) = debug.poll(&node)? {
            debug_pub.publish(msg).map_err(|e| e.to_string())?;
            debug_rate.tick();
        }

        let now = Instant::now();
        if now.duration_since(last_report) >= REPORT_EVERY {
            last_report = now;
            println!(
                "{label} image {:.1} Hz, debug_image {:.1} Hz",
                image_rate.take_hz(now),
                debug_rate.take_hz(now)
            );
        }
    }

    Ok(())
}

fn main() {
    let id = parse_cli(std::env::args().skip(1)).unwrap_or_else(|e| fail(e, 2));

    // Every other setting comes from relay_params.yaml.
    let config = crate_dir().join("parameters").join("relay_params.yaml");
    let text = std::fs::read_to_string(&config)
        .map_err(|e| format!("cannot read config '{}': {e}", config.display()))
        .unwrap_or_else(|e| fail(e, 1));
    let cfg: RelayParams = serde_yaml::from_str(&text)
        .map_err(|e| format!("parse '{}': {e}", config.display()))
        .unwrap_or_else(|e| fail(e, 1));
    let settings = resolve(cfg, id.as_deref()).unwrap_or_else(|e| fail(e, 2));

    println!(
        "ips_image_relay: {} camera(s) [{}]  scale 1/{} -> {}x{}  every {}",
        settings.insts.len(),
        settings.insts.join(", "),
        settings.scale,
        SOURCE_WIDTH / settings.scale,
        SOURCE_HEIGHT / settings.scale,
        settings.every
    );
    println!("ips_image_relay: parameters {}", config.display());

    // One zenoh session for the process (peer mode, the default config). It is
    // Arc-backed: each thread gets a clone and declares its own publishers on it.
    let session = zenoh::open(zenoh::Config::default())
        .wait()
        .unwrap_or_else(|e| fail(format!("cannot open zenoh session: {e}"), 1));

    // One relay thread per camera. The process stays up until every thread
    // exits; each thread's `node.wait` returns on SIGINT/SIGTERM. A thread that
    // fails logs and exits without stopping the others.
    let handles: Vec<_> = settings
        .insts
        .iter()
        .map(|inst| {
            let inst = inst.clone();
            let session = session.clone();
            let (scale, every) = (settings.scale, settings.every);
            thread::Builder::new()
                .name(inst.clone())
                .spawn(move || {
                    if let Err(e) = run_camera(&inst, &session, scale, every) {
                        eprintln!("[{inst}] error: {e}");
                    }
                })
                .expect("spawn relay thread")
        })
        .collect();

    for h in handles {
        let _ = h.join();
    }
}
