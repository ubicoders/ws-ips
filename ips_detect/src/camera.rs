//! camera: subscribe to one ips_cam_gcam_pub instance and hand frames to OpenCV.
//!
//! Wraps the sb-generated [`crate::image`] subscriber. The generated file's
//! default key is this module's own, so [`image_overrides`] replaces it with the
//! camera publisher's key via `TopicOverrides.full`. Each MONO8 frame is copied
//! into a reusable `CV_8UC1` `Mat`, and its capture stamp is handed back so the
//! detections carry it through.
//!
//! The generated subscriber uses iceoryx2 `open_or_create`, which would create
//! the camera's service when the camera is not up yet and make `sb topic list`
//! show an image service with no publisher behind it. So the subscriber is only
//! opened once the camera has created the service: [`Camera::poll_into`] probes
//! the service list every [`PROBE_EVERY`], prints a waiting line on the first
//! empty poll and repeats it every [`WAIT_LOG_EVERY`] until the first frame.

use crate::image;
use iceoryx2::prelude::*;
use opencv::core::{Mat, Scalar, CV_8UC1};
use opencv::prelude::*;
use std::time::{Duration, Instant};

/// Fixed sensor geometry (MONO8, 1 byte per pixel).
pub const WIDTH: i32 = 1280;
pub const HEIGHT: i32 = 1024;
pub const FRAME_BYTES: usize = (WIDTH * HEIGHT) as usize;

/// How often to repeat the waiting line until the first frame arrives.
pub const WAIT_LOG_EVERY: Duration = Duration::from_secs(5);
/// How often to look for the camera's service while it is not published yet.
pub const PROBE_EVERY: Duration = Duration::from_millis(200);

/// Whether an iceoryx2 service with exactly this name exists right now.
fn service_exists(name: &str) -> Result<bool, Box<dyn std::error::Error>> {
    let mut found = false;
    ipc::Service::list(Config::global_config(), |svc| {
        if svc.static_details.name().as_str() == name {
            found = true;
            CallbackProgression::Stop
        } else {
            CallbackProgression::Continue
        }
    })?;
    Ok(found)
}

/// Map a camera index from `detector_params.yaml` to its instance name:
/// `0 -> "sb00"`, `3 -> "sb03"`.
pub fn instance_name(index: u32) -> String {
    format!("sb{index:02}")
}

/// Overrides that point the generated subscriber at the camera publisher's key
/// `<device>/ws_ips/ips_cam_gcam_pub-<inst>/iox2/image`.
pub fn image_overrides(inst: &str) -> image::TopicOverrides {
    image::TopicOverrides {
        full: Some(format!(
            "{}/{}/ips_cam_gcam_pub-{inst}/iox2/image",
            image::DEFAULT_DEVICE,
            image::DEFAULT_WORKSPACE
        )),
        ..Default::default()
    }
}

/// What a received frame carried besides its pixels.
#[derive(Clone, Copy, Debug)]
pub struct FrameInfo {
    pub data_len: u32,
    /// The image's `header_stamp_sec` (capture time, set by the camera publisher).
    pub stamp_sec: i32,
    /// The image's `header_stamp_nanosec`.
    pub stamp_nanosec: u32,
}

/// A single camera subscription (data plane only).
pub struct Camera {
    pub name: String,
    /// The iceoryx2 service name subscribed to.
    pub key: String,
    /// `None` until the camera has created the service (see [`Camera::poll_into`]).
    sub: Option<image::ImageSubscriber>,
    started: Instant,
    /// When the service list was last probed while `sub` is `None`.
    last_probe: Option<Instant>,
    /// `None` before the first poll; then when the waiting line was last printed.
    last_wait_log: Option<Instant>,
    got_first: bool,
}

impl Camera {
    /// Prepare the subscription on `inst`'s camera key (see [`image_overrides`]).
    /// Nothing is opened yet: the subscriber is created by [`Camera::poll_into`]
    /// once the camera's service exists, so a waiting detector never creates it.
    pub fn open(inst: &str) -> Self {
        Self {
            name: inst.to_string(),
            key: image_overrides(inst).service_name(),
            sub: None,
            started: Instant::now(),
            last_probe: None,
            last_wait_log: None,
            got_first: false,
        }
    }

    /// Allocate a reusable destination `Mat` (`CV_8UC1`, `HEIGHT x WIDTH`).
    pub fn make_frame() -> Result<Mat, Box<dyn std::error::Error>> {
        Ok(Mat::new_rows_cols_with_default(
            HEIGHT,
            WIDTH,
            CV_8UC1,
            Scalar::all(0.0),
        )?)
    }

    /// Drain to the latest frame; if one arrived, copy its bytes into `dst` and
    /// return its [`FrameInfo`]. Zero-copy receive: the sample is dropped before
    /// returning so iceoryx2 can reclaim the shared-memory slot. Until the first
    /// frame, logs the waiting line every [`WAIT_LOG_EVERY`]. While the camera's
    /// service does not exist yet, probes for it every [`PROBE_EVERY`] and opens
    /// the subscriber on `node` as soon as it appears.
    pub fn poll_into(
        &mut self,
        node: &Node<ipc::Service>,
        dst: &mut Mat,
    ) -> Result<Option<FrameInfo>, Box<dyn std::error::Error>> {
        if self.sub.is_none() {
            let now = Instant::now();
            let due = self
                .last_probe
                .is_none_or(|t| now.duration_since(t) >= PROBE_EVERY);
            if due {
                self.last_probe = Some(now);
                if service_exists(&self.key)? {
                    self.sub = Some(image::ImageSubscriber::open_with(
                        node,
                        image_overrides(&self.name),
                    )?);
                }
            }
        }
        let latest = match &self.sub {
            Some(sub) => sub.try_recv_latest()?,
            None => None,
        };
        let info = match latest {
            Some(frame) => {
                let avail = (frame.data_len as usize)
                    .min(frame.data.len())
                    .min(FRAME_BYTES);
                dst.data_bytes_mut()?[..avail].copy_from_slice(&frame.data[..avail]);
                Some(FrameInfo {
                    data_len: frame.data_len,
                    stamp_sec: frame.header_stamp_sec,
                    stamp_nanosec: frame.header_stamp_nanosec,
                })
            }
            None => None,
        };

        if !self.got_first {
            let now = Instant::now();
            if info.is_some() {
                self.got_first = true;
                println!(
                    "[{}] first frame from {} after {:.1} s",
                    self.name,
                    self.key,
                    now.duration_since(self.started).as_secs_f64()
                );
            } else {
                match self.last_wait_log {
                    None => {
                        println!("waiting for camera {} on {}", self.name, self.key);
                        self.last_wait_log = Some(now);
                    }
                    Some(t) if now.duration_since(t) >= WAIT_LOG_EVERY => {
                        println!(
                            "waiting for camera {} on {} ({:.0} s)",
                            self.name,
                            self.key,
                            now.duration_since(self.started).as_secs_f64()
                        );
                        self.last_wait_log = Some(now);
                    }
                    Some(_) => {}
                }
            }
        }
        Ok(info)
    }
}
