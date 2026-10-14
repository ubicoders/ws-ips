//! images: one iceoryx2 subscription per camera, latest MONO8 frame kept as bytes.
//!
//! A copy of `ips_detect/src/camera.rs` by design: a module depends on nothing
//! in another module's folder (see the repository README). It differs in two
//! ways: the frame lands in a `Vec<u8>` instead of an OpenCV `Mat`, so this file
//! builds without OpenCV, and it remembers when the last frame arrived so the
//! window can show `NO IMAGE`.
//!
//! The generated subscriber uses iceoryx2 `open_or_create`, which would create
//! the camera's service when the camera is not up yet and make `sb topic list`
//! show an image service with no publisher behind it. So the subscriber is only
//! opened once the camera has created the service: [`Camera::poll`] probes the
//! service list every [`PROBE_EVERY`] until then.

use crate::image;
use iceoryx2::prelude::*;
use std::time::{Duration, Instant};

/// Fixed sensor geometry (MONO8, 1 byte per pixel).
pub const WIDTH: usize = 1280;
pub const HEIGHT: usize = 1024;
pub const FRAME_BYTES: usize = WIDTH * HEIGHT;

/// How often to look for the camera's service while it is not published yet.
pub const PROBE_EVERY: Duration = Duration::from_millis(200);

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// Whether an iceoryx2 service with exactly this name exists right now.
fn service_exists(name: &str) -> Res<bool> {
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

/// A single camera subscription and its latest frame.
pub struct Camera {
    pub name: String,
    /// The iceoryx2 service name subscribed to.
    pub key: String,
    /// The latest frame's pixels, `HEIGHT` rows of `WIDTH` bytes.
    pub frame: Vec<u8>,
    /// When the latest frame arrived; `None` before the first.
    pub last_frame_at: Option<Instant>,
    /// `None` until the camera has created the service.
    sub: Option<image::ImageSubscriber>,
    last_probe: Option<Instant>,
}

impl Camera {
    /// Prepare the subscription on `inst`'s camera key. Nothing is opened yet:
    /// the subscriber is created by [`Camera::poll`] once the service exists.
    pub fn open(inst: &str) -> Self {
        Self {
            name: inst.to_string(),
            key: image_overrides(inst).service_name(),
            frame: vec![0; FRAME_BYTES],
            last_frame_at: None,
            sub: None,
            last_probe: None,
        }
    }

    /// Drain to the latest frame and copy it into `frame`. Returns whether a new
    /// frame arrived. Zero-copy receive: the sample is dropped before returning
    /// so iceoryx2 can reclaim the shared-memory slot.
    pub fn poll(&mut self, node: &Node<ipc::Service>) -> Res<bool> {
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
                    println!("[{}] subscribed to {}", self.name, self.key);
                }
            }
        }
        let latest = match &self.sub {
            Some(sub) => sub.try_recv_latest()?,
            None => None,
        };
        let Some(sample) = latest else {
            return Ok(false);
        };
        let avail = (sample.data_len as usize)
            .min(sample.data.len())
            .min(FRAME_BYTES);
        self.frame[..avail].copy_from_slice(&sample.data[..avail]);
        self.last_frame_at = Some(Instant::now());
        Ok(true)
    }

    /// Whether a frame arrived within `within`.
    pub fn is_live(&self, now: Instant, within: Duration) -> bool {
        self.last_frame_at
            .is_some_and(|t| now.duration_since(t) <= within)
    }
}
