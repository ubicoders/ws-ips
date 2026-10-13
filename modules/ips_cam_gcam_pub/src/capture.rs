//! Camera side of the publisher: the parameters file, the camera list and
//! `--id`, opening a camera by its user-set name, the configuration sequence,
//! and one grab into the iceoryx2 payload.
//!
//! The configuration order is the one the C++ publisher used (frame-speed tier,
//! packet size, frame rate, MONO8, continuous trigger, manual exposure, play).
//! `NumBuffers` is an SDK-wide option and is set once in `main`, before the
//! first camera is opened.

use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use gcam_mvsdk_rs::{Camera, OutputFormat, Sdk, TriggerMode};
use serde::Deserialize;

use crate::image::{self, Image1280x1024};

/// Image width on the wire. Fixed by the payload type `Image1280x1024`.
pub const WIDTH: u32 = 1280;
/// Image height on the wire.
pub const HEIGHT: u32 = 1024;
/// MONO8 bytes per frame (one byte per pixel).
pub const IMG_BYTES: usize = WIDTH as usize * HEIGHT as usize;
/// `encoding` field of the payload.
pub const ENCODING: &str = "mono8";
/// How long one `grab` waits for a frame before it reports a time-out.
pub const GRAB_TIMEOUT: Duration = Duration::from_secs(1);

/// One-line usage message, printed on any command-line error.
pub const USAGE: &str = "usage: ips_cam_gcam_pub [--id <sbNN>]   (no --id: every camera in parameters/cam_params.yaml)";

// ---------------------------------------------------------------------------
// Parameters and the camera list
// ---------------------------------------------------------------------------

/// `parameters/cam_params.yaml`. Every key is required; unknown keys are an
/// error so a typo does not silently fall back to a default.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CamParams {
    /// User-set camera names (`gcam-mvsdk-rs rename <serial> sb00`).
    pub cameras: Vec<String>,
    /// Manual exposure in microseconds.
    pub exposure_us: f64,
    /// Camera-side frame-rate cap in Hz; 0 (or below) removes the cap.
    pub frame_rate_hz: i32,
    /// Largest GigE packet size to select, in bytes (normally the NIC MTU).
    pub packet_bytes: u32,
    /// SDK receive buffers (`CameraGigeSetOption("NumBuffers", ..)`).
    pub num_buffers: u32,
}

impl CamParams {
    /// Parses and validates the yaml text.
    pub fn from_yaml(text: &str) -> Result<CamParams, String> {
        let params: CamParams = serde_yaml::from_str(text).map_err(|e| e.to_string())?;
        params.validate()?;
        Ok(params)
    }

    /// Reads, parses and validates the file at `path`.
    pub fn load(path: &Path) -> Result<CamParams, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
        Self::from_yaml(&text).map_err(|e| format!("{}: {e}", path.display()))
    }

    fn validate(&self) -> Result<(), String> {
        if self.cameras.is_empty() {
            return Err("`cameras` is empty".into());
        }
        for (i, name) in self.cameras.iter().enumerate() {
            gcam_mvsdk_rs::validate_user_name(name).map_err(|e| format!("`cameras[{i}]`: {e}"))?;
            if self.cameras[..i].contains(name) {
                return Err(format!("`cameras` lists {name:?} twice"));
            }
        }
        if !(self.exposure_us.is_finite() && self.exposure_us > 0.0) {
            return Err(format!("`exposure_us` must be positive, got {}", self.exposure_us));
        }
        if self.packet_bytes == 0 {
            return Err("`packet_bytes` must be positive".into());
        }
        if self.num_buffers == 0 {
            return Err("`num_buffers` must be at least 1".into());
        }
        Ok(())
    }
}

/// `parameters/cam_params.yaml` of this crate.
pub fn params_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("parameters").join("cam_params.yaml")
}

/// Parses the command line (without the program name). The only option is
/// `--id <sbNN>`; returns the id, or `None` when it is absent.
pub fn parse_args<I, S>(args: I) -> Result<Option<String>, String>
where
    I: IntoIterator<Item = S>,
    S: Into<String>,
{
    let mut args = args.into_iter().map(Into::into);
    let mut id: Option<String> = None;
    while let Some(arg) = args.next() {
        if arg != "--id" {
            return Err(format!("unexpected argument {arg:?}"));
        }
        let value = match args.next() {
            Some(v) if !v.is_empty() && !v.starts_with('-') => v,
            _ => return Err("--id needs a camera id, for example --id sb00".into()),
        };
        if id.replace(value).is_some() {
            return Err("--id given more than once".into());
        }
    }
    Ok(id)
}

/// The cameras this process runs: all of `cameras` without an id, or just the
/// one named by `--id`, which must be in the list.
pub fn select_cameras(cameras: &[String], id: Option<&str>) -> Result<Vec<String>, String> {
    match id {
        None => Ok(cameras.to_vec()),
        Some(id) if cameras.iter().any(|c| c == id) => Ok(vec![id.to_string()]),
        Some(id) => Err(format!(
            "unknown camera id {id:?}; parameters/cam_params.yaml lists {}",
            cameras.join(", ")
        )),
    }
}

/// Wire key overrides for one camera: module segment `ips_cam_gcam_pub-<id>`,
/// everything else from the generated defaults.
pub fn topic_overrides(id: &str) -> image::TopicOverrides {
    image::TopicOverrides {
        module: Some(format!("{}-{id}", image::DEFAULT_MODULE)),
        ..Default::default()
    }
}

// ---------------------------------------------------------------------------
// Payload header
// ---------------------------------------------------------------------------

/// A zeroed payload allocated straight on the heap. `Box::default()` would
/// build the 1.3 MB value on the stack first, which overflows a default 2 MB
/// thread stack in debug builds.
pub fn new_payload() -> Box<Image1280x1024> {
    // SAFETY: every field is an integer or a byte array, so all-zero bytes are
    // a valid value (the generated `Default` is `mem::zeroed` too).
    unsafe { Box::<Image1280x1024>::new_zeroed().assume_init() }
}

/// Writes the fields that stay the same for every frame of a camera.
pub fn init_header(msg: &mut Image1280x1024, frame_id: &str) {
    write_c_string(&mut msg.header_frame_id, frame_id);
    msg.height = HEIGHT;
    msg.width = WIDTH;
    write_c_string(&mut msg.encoding, ENCODING);
    msg.is_bigendian = 0;
    msg.step = WIDTH;
    msg.data_len = IMG_BYTES as u32;
}

/// Sets the header stamp. `t` is host wall-clock time taken right after
/// `grab` returned; the camera's own tick counter is relative and is not used.
pub fn set_stamp(msg: &mut Image1280x1024, t: SystemTime) {
    let since_epoch = t.duration_since(UNIX_EPOCH).unwrap_or_default();
    msg.header_stamp_sec = since_epoch.as_secs() as i32;
    msg.header_stamp_nanosec = since_epoch.subsec_nanos();
}

/// Copies `text` into a fixed NUL-padded field, truncated so at least one
/// terminating NUL remains.
fn write_c_string(dst: &mut [u8], text: &str) {
    dst.fill(0);
    let n = text.len().min(dst.len().saturating_sub(1));
    dst[..n].copy_from_slice(&text.as_bytes()[..n]);
}

/// Checks that a frame fits the wire payload exactly. `processed_len` is
/// `Frame::processed_len` with the output format set to MONO8. The geometry is
/// fixed by the payload type, so a mismatch is reported, never resized.
pub fn check_geometry(width: u32, height: u32, processed_len: usize) -> Result<(), String> {
    if width == WIDTH && height == HEIGHT && processed_len == IMG_BYTES {
        return Ok(());
    }
    Err(format!(
        "the camera delivers {width}x{height} ({processed_len} bytes after MONO8 processing), but the wire \
         payload Image1280x1024 needs exactly {WIDTH}x{HEIGHT} ({IMG_BYTES} bytes). Set the camera to its full \
         {WIDTH}x{HEIGHT} resolution (it reuses its last parameter set on open); this publisher does not resize"
    ))
}

// ---------------------------------------------------------------------------
// Opening and configuring
// ---------------------------------------------------------------------------

// Enumeration and open are serialised across the camera threads so two threads
// never enumerate or open at the same time.
static OPEN_LOCK: Mutex<()> = Mutex::new(());

/// Finds the camera whose user-set name (or serial number) is `name` and opens it.
///
/// The match uses the enumeration result only. The driver's `find_device` falls
/// back to opening every camera on the LAN to read its stored name when the
/// enumeration does not carry it; with one camera missing, that would probe the
/// cameras streaming in the sibling threads every retry. A camera renamed with
/// `gcam-mvsdk-rs rename` reports the new name after a power cycle.
pub fn open_by_name(sdk: &Sdk, name: &str) -> gcam_mvsdk_rs::Result<Camera> {
    let _guard = OPEN_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let devices = sdk.enumerate()?;
    let dev = devices
        .iter()
        .find(|d| d.friendly_name == name || d.serial == name)
        .ok_or_else(|| gcam_mvsdk_rs::Error::DeviceNotFound {
            query: name.to_string(),
            available: devices
                .iter()
                .map(|d| format!("{:?} (SN {})", d.friendly_name, d.serial))
                .collect(),
        })?;
    sdk.open(dev)
}

/// Configures an open camera in the C++ publisher's order, stopping before
/// `play`. Returns the lines worth printing once the camera is up.
pub fn configure(cam: &Camera, params: &CamParams) -> gcam_mvsdk_rs::Result<Vec<String>> {
    let mut report = Vec::new();
    let cap = cam.capability()?;
    report.push(format!(
        "opened sn={} sensor={} max={}x{} publish={WIDTH}x{HEIGHT} MONO8",
        cam.serial(),
        if cap.is_mono() { "mono" } else { "color" },
        cap.max_width(),
        cap.max_height(),
    ));

    let presets: Vec<String> = cap
        .image_sizes
        .iter()
        .map(|r| format!("[{}] {}x{} '{}'", r.index, r.width, r.height, r.description))
        .collect();
    report.push(format!("preset image sizes ({}): {}", presets.len(), presets.join(", ")));
    if let Ok(r) = cam.image_resolution() {
        report.push(format!(
            "current resolution: idx={} {}x{} (FOV {}x{} at +{},+{})",
            r.index, r.width, r.height, r.fov_width, r.fov_height, r.fov_offset_x, r.fov_offset_y
        ));
    }

    // Fastest frame-speed tier the camera advertises. Without it the SDK
    // stays on LOW/NORMAL, which caps the sensor below its rated rate.
    let tiers: Vec<String> = cap
        .frame_speeds
        .iter()
        .map(|s| format!("[{}] '{}'", s.index, s.description))
        .collect();
    report.push(format!("frame-speed tiers ({}): {}", tiers.len(), tiers.join(", ")));
    if let Some(target) = cap.frame_speeds.iter().map(|s| s.index).max() {
        cam.set_frame_speed(target)?;
        let got = cam.frame_speed().map(|v| v.to_string()).unwrap_or_else(|e| format!("? ({e})"));
        report.push(format!("frame-speed tier set to {target}, camera reports {got}"));
    }

    // Largest GigE packet that fits the path MTU. A failure is a warning, as
    // in the C++ publisher: the camera keeps its previous packet size.
    let sizes: Vec<String> = cap.pack_lengths.iter().map(|p| p.size.to_string()).collect();
    match cam.set_gige_packet_size(params.packet_bytes) {
        Ok(size) => report.push(format!(
            "packet size {size} bytes (limit {}, offered {})",
            params.packet_bytes,
            sizes.join("/")
        )),
        Err(e) => report.push(format!("warning: packet size not set: {e}")),
    }

    // The frame-speed tier sets the readout class; the SDK also enforces a
    // per-camera frame-rate limit. 0 removes it.
    let before = cam.frame_rate().map(|v| v.to_string()).unwrap_or_else(|_| "?".into());
    if let Err(e) = cam.set_frame_rate(params.frame_rate_hz) {
        report.push(format!("warning: frame rate {} not set: {e}", params.frame_rate_hz));
    }
    let after = cam.frame_rate().map(|v| v.to_string()).unwrap_or_else(|_| "?".into());
    report.push(format!(
        "frame-rate cap {before} -> {after} (requested {}, 0 = none)",
        params.frame_rate_hz
    ));

    // MONO8: the payload is one byte per pixel.
    cam.set_output_format(OutputFormat::Mono8)?;
    cam.set_trigger_mode(TriggerMode::Continuous)?;
    cam.set_auto_exposure(false)?;
    cam.set_exposure_us(params.exposure_us)?;
    let exposure = cam.exposure_us().map(|v| format!("{v:.1}")).unwrap_or_else(|_| "?".into());
    report.push(format!(
        "MONO8, continuous trigger, manual exposure {exposure} us (requested {})",
        params.exposure_us
    ));
    Ok(report)
}

// ---------------------------------------------------------------------------
// Grabbing
// ---------------------------------------------------------------------------

/// A frame that was written into the payload.
#[derive(Debug, Clone, Copy)]
pub struct Grabbed {
    /// Steady-clock time right after `grab` returned (for the timing line).
    pub grabbed_at: Instant,
    /// Camera tick in 0.1 ms, relative to camera start. Used only for the
    /// frame interval in the timing line.
    pub cam_ts_100us: u32,
}

/// Why a grab produced no frame (a time-out is `Ok(None)` instead).
#[derive(Debug)]
pub enum GrabError {
    /// The SDK failed. Close the camera and reconnect.
    Sdk(gcam_mvsdk_rs::Error),
    /// The frame does not fit the payload. Reconnecting does not help.
    Geometry(String),
}

impl std::fmt::Display for GrabError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GrabError::Sdk(e) => write!(f, "{e}"),
            GrabError::Geometry(m) => f.write_str(m),
        }
    }
}

impl std::error::Error for GrabError {}

/// Waits up to [`GRAB_TIMEOUT`] for the next frame, runs the SDK ISP straight
/// into `msg.data` and stamps the header. `Ok(None)` is a time-out. The SDK
/// frame buffer is released before this returns.
pub fn grab_into(cam: &Camera, msg: &mut Image1280x1024) -> Result<Option<Grabbed>, GrabError> {
    let frame = match cam.grab(GRAB_TIMEOUT) {
        Ok(f) => f,
        Err(e) if e.is_timeout() => return Ok(None),
        Err(e) => return Err(GrabError::Sdk(e)),
    };
    let stamp = SystemTime::now();
    let grabbed_at = Instant::now();

    let need = frame.processed_len().map_err(GrabError::Sdk)?;
    check_geometry(frame.width(), frame.height(), need).map_err(GrabError::Geometry)?;
    let info = frame.process_into(&mut msg.data).map_err(GrabError::Sdk)?;
    check_geometry(info.width, info.height, info.len).map_err(GrabError::Geometry)?;
    let cam_ts_100us = frame.timestamp_100us();
    drop(frame);

    set_stamp(msg, stamp);
    Ok(Some(Grabbed { grabbed_at, cam_ts_100us }))
}
