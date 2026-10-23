//! `ips_cam_gcam_pub`: one thread per camera. Each thread finds its camera by
//! user-set name, configures it, and publishes every frame as
//! `sensor_msgs/Image1280x1024` on `<device>/ws_ips/ips_cam_gcam_pub-<id>/iox2/image`.
//!
//! Command line: `--id <sbNN>` or nothing. Everything else comes from
//! `parameters/cam_params.yaml`.

use std::collections::VecDeque;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use gcam_mvsdk_rs::{Camera, Error as SdkError, Sdk};
use iceoryx2::prelude::{ipc, Node, NodeBuilder, NodeName};

use ips_cam_gcam_pub::capture::{self, CamParams, GrabError};
use ips_cam_gcam_pub::image::{Image1280x1024, ImagePublisher};

/// Pause between attempts to find and open a camera.
const RETRY_PERIOD: Duration = Duration::from_secs(2);
/// While the same error repeats, remind at most this often.
const RETRY_REMINDER: Duration = Duration::from_secs(30);
/// Frames per timing line.
const TIMING_FRAMES: u32 = 60;
/// The payload is 1.3 MB and `publish` takes it by value.
const STACK_BYTES: usize = 32 * 1024 * 1024;

/// Raised by the first thread whose `node.wait` reports SIGINT/SIGTERM.
/// iceoryx2 clears the signal once one node has seen it, so the other camera
/// threads learn about it from here.
static STOP: AtomicBool = AtomicBool::new(false);

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

fn main() -> ExitCode {
    let args = std::env::args_os().skip(1).map(|a| a.to_string_lossy().into_owned());
    let id = match capture::parse_args(args) {
        Ok(id) => id,
        Err(e) => return usage_error(&e),
    };
    let params_path = capture::params_path();
    let params = match CamParams::load(&params_path) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(1);
        }
    };
    let cameras = match capture::select_cameras(&params.cameras, id.as_deref()) {
        Ok(c) => c,
        Err(e) => return usage_error(&e),
    };

    println!("ips_cam_gcam_pub: {} camera(s) [{}]", cameras.len(), cameras.join(", "));
    println!("  parameters   {}", params_path.display());
    println!(
        "  exposure {} us, frame-rate cap {} Hz (0 = none), packet <= {} bytes, {} receive buffers",
        params.exposure_us, params.frame_rate_hz, params.packet_bytes, params.num_buffers
    );
    for name in &cameras {
        println!("  {name} -> {}", capture::topic_overrides(name).service_name());
    }

    let sdk = match Sdk::load() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(1);
        }
    };
    println!(
        "  MVSDK {} from {}",
        sdk.version().unwrap_or_else(|| "(version unknown)".into()),
        sdk.library_path().display()
    );
    set_data_directory(&sdk);
    // SDK-wide GigE option, set once before any camera is opened. More
    // streaming buffers smooth out packet jitter at high frame rates.
    match sdk.gige_set_option("NumBuffers", &params.num_buffers.to_string()) {
        Ok(()) => println!("  NumBuffers   {}", params.num_buffers),
        Err(e) => eprintln!("warning: NumBuffers={} not set: {e}", params.num_buffers),
    }

    let params = Arc::new(params);
    let mut handles = Vec::new();
    let mut failed = false;
    for name in cameras {
        let sdk = sdk.clone();
        let params = Arc::clone(&params);
        let spawned = thread::Builder::new()
            .name(name.clone())
            .stack_size(STACK_BYTES)
            .spawn({
                let name = name.clone();
                move || match run_camera(&sdk, &name, &params) {
                    Ok(()) => true,
                    Err(e) => {
                        eprintln!("[{name}] FATAL: {e}");
                        false
                    }
                }
            });
        match spawned {
            Ok(h) => handles.push((name, h)),
            Err(e) => {
                eprintln!("[{name}] error: cannot start the camera thread: {e}");
                failed = true;
            }
        }
    }

    // The process stays up until every camera thread has returned: on
    // SIGINT/SIGTERM, or one by one on a fatal per-camera error (printed by
    // the thread itself when it happens).
    for (name, h) in handles {
        match h.join() {
            Ok(true) => {}
            Ok(false) => failed = true,
            Err(_) => {
                eprintln!("[{name}] camera thread panicked");
                failed = true;
            }
        }
    }
    if failed {
        ExitCode::from(1)
    } else {
        ExitCode::SUCCESS
    }
}

fn usage_error(msg: &str) -> ExitCode {
    eprintln!("error: {msg}");
    eprintln!("{}", capture::USAGE);
    ExitCode::from(2)
}

/// Sends the files the SDK keeps per camera and model (data files, parameter
/// groups A to D, log) to the SDK data directory, the same place the
/// `gcam-mvsdk-rs` command uses: `$GCAM_MVSDK_DATA_DIR` (set in sb.module.yaml
/// to the data folder of the driver checkout), else `<SDK home>/data`. Without
/// this the SDK writes into the working directory (this module folder).
fn set_data_directory(sdk: &Sdk) {
    let Some(dir) = gcam_mvsdk_rs::data_dir() else {
        return;
    };
    if let Err(e) = std::fs::create_dir_all(&dir) {
        eprintln!("warning: cannot create {}: {e}", dir.display());
        return;
    }
    match sdk.set_data_directory(&dir) {
        Ok(()) => println!("  SDK data     {}", dir.display()),
        Err(SdkError::Unsupported(_)) => {}
        Err(e) => eprintln!("warning: SDK data directory {} not set: {e}", dir.display()),
    }
}

/// How a streaming session ended.
enum StreamEnd {
    /// SIGINT/SIGTERM.
    Shutdown,
    /// The camera failed; close it and reconnect.
    CameraError(String),
    /// Reconnecting does not help; this camera thread stops.
    Fatal(String),
}

/// One camera, for the life of the process: node and publisher once, then
/// find, open, configure, stream, and reconnect after a camera error. An `Err`
/// ends this camera only; the other camera threads keep running.
fn run_camera(sdk: &Sdk, name: &str, params: &CamParams) -> Result<(), String> {
    let node_name = NodeName::new(&format!("ips_cam_gcam_pub-{name}"))
        .map_err(|e| format!("invalid iceoryx2 node name for {name:?}: {e:?}"))?;
    let node = NodeBuilder::new()
        .name(&node_name)
        .create::<ipc::Service>()
        .map_err(|e| format!("cannot create the iceoryx2 node: {e:?}"))?;
    let overrides = capture::topic_overrides(name);
    let key = overrides.service_name();
    let publisher =
        ImagePublisher::open_with(&node, overrides).map_err(|e| format!("cannot open the publisher on {key}: {e}"))?;

    // One heap payload per thread, refilled every frame. `publish` takes it by
    // value, so each frame moves a copy into the call, and the generated
    // publisher writes that copy into a loaned shared-memory slot.
    let mut payload = capture::new_payload();
    capture::init_header(&mut payload, name);
    println!(
        "[{name}] publishing on {key} ({} byte payload); looking for camera {name:?}",
        std::mem::size_of::<Image1280x1024>()
    );

    let mut retry = RetryLog::new(name);
    while keep_running(&node, Duration::ZERO) {
        let cam = match open_and_start(sdk, name, params) {
            Ok(cam) => {
                retry.succeeded();
                cam
            }
            Err(e) => {
                retry.failed(&e);
                if !keep_running(&node, RETRY_PERIOD) {
                    break;
                }
                continue;
            }
        };
        match stream(&node, &cam, &publisher, &mut payload, name) {
            StreamEnd::Shutdown => break,
            StreamEnd::CameraError(e) => {
                eprintln!("[{name}] camera error: {e}; closing the camera and reconnecting");
                drop(cam);
                if !keep_running(&node, RETRY_PERIOD) {
                    break;
                }
            }
            StreamEnd::Fatal(e) => return Err(format!("{e}. This camera thread stops")),
        }
    }
    println!("[{name}] stopped");
    Ok(())
}

/// Finds and opens the camera, configures it, and starts streaming.
fn open_and_start(sdk: &Sdk, name: &str, params: &CamParams) -> Result<Camera, String> {
    let cam = capture::open_by_name(sdk, name).map_err(|e| e.to_string())?;
    let report = capture::configure(&cam, params).map_err(|e| format!("configure: {e}"))?;
    cam.play().map_err(|e| format!("play: {e}"))?;
    let mut text = String::new();
    for line in report {
        text.push_str(&format!("[{name}] {line}\n"));
    }
    println!("{text}[{name}] streaming");
    Ok(cam)
}

/// The publish loop for one open, playing camera.
fn stream(
    node: &Node<ipc::Service>,
    cam: &Camera,
    publisher: &ImagePublisher,
    payload: &mut Image1280x1024,
    name: &str,
) -> StreamEnd {
    let mut timing = Timing::default();
    let mut fps = FpsMeter::new(Duration::from_secs(1));
    let mut last_rate_print = Instant::now();
    let mut published: u64 = 0;
    let mut timeouts_in_row: u32 = 0;
    let mut publish_errors: u64 = 0;
    let mut last_publish_error: Option<Instant> = None;

    while keep_running(node, Duration::ZERO) {
        let t0 = Instant::now();
        let grabbed = match capture::grab_into(cam, payload) {
            Ok(Some(g)) => g,
            Ok(None) => {
                timeouts_in_row += 1;
                if timeouts_in_row == 1 || timeouts_in_row.is_multiple_of(10) {
                    eprintln!("[{name}] no frame for {timeouts_in_row} s (grab time-out), still waiting");
                }
                continue;
            }
            Err(GrabError::Sdk(e)) => return StreamEnd::CameraError(e.to_string()),
            Err(GrabError::Geometry(m)) => return StreamEnd::Fatal(m),
        };
        if timeouts_in_row > 0 {
            println!("[{name}] frames again after {timeouts_in_row} grab time-out(s)");
            timeouts_in_row = 0;
        }

        let t2 = Instant::now();
        if let Err(e) = publisher.publish(*payload) {
            publish_errors += 1;
            if last_publish_error.is_none_or(|t| t.elapsed() >= Duration::from_secs(1)) {
                eprintln!("[{name}] publish failed ({publish_errors} so far): {e}");
                last_publish_error = Some(Instant::now());
            }
        } else {
            published += 1;
        }
        let t3 = Instant::now();

        timing.add(grabbed.grabbed_at - t0, t2 - grabbed.grabbed_at, t3 - t2, grabbed.cam_ts_100us);
        if timing.frames == TIMING_FRAMES {
            println!("[{name}] {}", timing.take_line());
        }
        let hz = fps.tick(t3);
        if t3.duration_since(last_rate_print) >= Duration::from_secs(1) {
            last_rate_print = t3;
            let hz = hz.map(|v| format!("{v:.2}")).unwrap_or_else(|| "--".into());
            println!("[{name}] measured {hz} Hz, {published} frames published");
        }
    }
    StreamEnd::Shutdown
}

/// Logs failed attempts to reach a camera. A group is a run of attempts that
/// fail with the same error: its first attempt is logged, then a reminder at
/// most every [`RETRY_REMINDER`]. A different error starts a new group.
struct RetryLog<'a> {
    name: &'a str,
    attempts: u32,
    first_failure: Option<Instant>,
    last_error: Option<String>,
    last_print: Option<Instant>,
}

impl<'a> RetryLog<'a> {
    fn new(name: &'a str) -> Self {
        RetryLog { name, attempts: 0, first_failure: None, last_error: None, last_print: None }
    }

    fn failed(&mut self, err: &str) {
        self.attempts += 1;
        let now = Instant::now();
        let since = now.duration_since(*self.first_failure.get_or_insert(now));
        let name = self.name;
        if self.last_error.as_deref() != Some(err) {
            eprintln!(
                "[{name}] camera not ready (attempt {}): {err}; retrying every {} s",
                self.attempts,
                RETRY_PERIOD.as_secs()
            );
            self.last_error = Some(err.to_string());
            self.last_print = Some(now);
        } else if self.last_print.is_none_or(|t| now.duration_since(t) >= RETRY_REMINDER) {
            eprintln!(
                "[{name}] still waiting for the camera ({} attempts over {} s): {err}",
                self.attempts,
                since.as_secs()
            );
            self.last_print = Some(now);
        }
    }

    fn succeeded(&mut self) {
        if self.attempts > 0 {
            println!("[{}] camera reached after {} failed attempt(s)", self.name, self.attempts);
        }
        *self = RetryLog::new(self.name);
    }
}

/// Sums for the timing line, printed every [`TIMING_FRAMES`] frames as the C++
/// publisher did. `cam_dt` is the interval between camera ticks (0.1 ms units).
#[derive(Default)]
struct Timing {
    frames: u32,
    grab_ms: f64,
    process_ms: f64,
    send_ms: f64,
    cam_dt_ms: f64,
    cam_dt_count: u32,
    prev_cam_ts: Option<u32>,
}

impl Timing {
    fn add(&mut self, grab: Duration, process: Duration, send: Duration, cam_ts_100us: u32) {
        self.frames += 1;
        self.grab_ms += grab.as_secs_f64() * 1e3;
        self.process_ms += process.as_secs_f64() * 1e3;
        self.send_ms += send.as_secs_f64() * 1e3;
        if let Some(prev) = self.prev_cam_ts {
            self.cam_dt_ms += f64::from(cam_ts_100us.wrapping_sub(prev)) * 0.1;
            self.cam_dt_count += 1;
        }
        self.prev_cam_ts = Some(cam_ts_100us);
    }

    /// Formats the averages and clears the sums (the last camera tick is kept).
    fn take_line(&mut self) -> String {
        let n = f64::from(self.frames.max(1));
        let cam_dt = if self.cam_dt_count > 0 { self.cam_dt_ms / f64::from(self.cam_dt_count) } else { 0.0 };
        let cam_fps = if cam_dt > 0.0 { 1000.0 / cam_dt } else { 0.0 };
        let line = format!(
            "timing avg({}): grab={:.2}ms process={:.2}ms send={:.2}ms total={:.2}ms  cam_dt={:.2}ms (={:.1} fps)",
            self.frames,
            self.grab_ms / n,
            self.process_ms / n,
            self.send_ms / n,
            (self.grab_ms + self.process_ms + self.send_ms) / n,
            cam_dt,
            cam_fps
        );
        *self = Timing { prev_cam_ts: self.prev_cam_ts, ..Timing::default() };
        line
    }
}

/// Sliding-window frame-rate meter: keeps the arrival times of the last
/// `window` and divides the frame count by the span they cover.
struct FpsMeter {
    window: Duration,
    ticks: VecDeque<Instant>,
}

impl FpsMeter {
    fn new(window: Duration) -> Self {
        FpsMeter { window, ticks: VecDeque::new() }
    }

    /// Records a frame at `now`; `None` until two frames share the window.
    fn tick(&mut self, now: Instant) -> Option<f64> {
        self.ticks.push_back(now);
        while let Some(&oldest) = self.ticks.front() {
            if now.duration_since(oldest) > self.window {
                self.ticks.pop_front();
            } else {
                break;
            }
        }
        let oldest = *self.ticks.front()?;
        let span = now.duration_since(oldest).as_secs_f64();
        if self.ticks.len() < 2 || span <= 0.0 {
            return None;
        }
        Some((self.ticks.len() - 1) as f64 / span)
    }
}
