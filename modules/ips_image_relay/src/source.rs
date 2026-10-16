//! source: one iceoryx2 image stream of one camera instance, opened lazily.
//!
//! The relay reads two streams per camera ([`Stream::Image`] from
//! ips_cam_gcam_pub, [`Stream::DebugImage`] from ips_detect). The generated
//! subscribers' default keys are this module's own, so [`Stream::source_key`]
//! replaces them with the source publisher's key via `TopicOverrides.full`.
//!
//! The generated subscribers use iceoryx2 `open_or_create`, which would create
//! the source's service when the source is not up yet and make `sb topic list`
//! show an image service with no publisher behind it. So a subscriber is only
//! opened once its source has created the service: [`Source::poll`] probes the
//! service list every [`PROBE_EVERY`], prints a waiting line on the first empty
//! poll and repeats it every [`WAIT_LOG_EVERY`] until the first frame. Each
//! stream does this on its own; `debug_image` exists only while ips_detect runs
//! with `debug: true`, so it may never arrive.
//!
//! Frames are received zero-copy: [`Source::poll`] hands the caller a borrowed
//! [`FrameView`] of the shared-memory sample and drops the sample as soon as the
//! caller's closure returns, so the publisher gets its slot back promptly.

use crate::debug_image_sub::{self, DebugImageSubscriber, Image1280x1024Color};
use crate::image_sub::{self, Image1280x1024, ImageSubscriber};
use iceoryx2::prelude::*;
use std::time::{Duration, Instant};

/// How often to repeat the waiting line until the first frame arrives.
pub const WAIT_LOG_EVERY: Duration = Duration::from_secs(5);
/// How often to look for the source's service while it is not published yet.
pub const PROBE_EVERY: Duration = Duration::from_millis(200);

/// Map a camera index from `relay_params.yaml` to its instance name:
/// `0 -> "sb00"`, `3 -> "sb03"`.
pub fn instance_name(index: u32) -> String {
    format!("sb{index:02}")
}

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

/// The two image streams a camera instance relays.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stream {
    /// The camera's MONO8 frame, `ips_cam_gcam_pub-<inst>/iox2/image`.
    Image,
    /// The detector's annotated RGB8 frame, `ips_detect-<inst>/iox2/debug_image`.
    DebugImage,
}

impl Stream {
    /// Topic segment, the same on the iceoryx2 source and the zenoh output.
    pub fn topic(self) -> &'static str {
        match self {
            Stream::Image => image_sub::DEFAULT_TOPIC,
            Stream::DebugImage => debug_image_sub::DEFAULT_TOPIC,
        }
    }

    /// Bytes per pixel: 1 for MONO8, 3 for RGB8.
    pub fn channels(self) -> usize {
        match self {
            Stream::Image => 1,
            Stream::DebugImage => 3,
        }
    }

    /// The source's iceoryx2 payload type, for log lines.
    pub fn source_type(self) -> &'static str {
        match self {
            Stream::Image => "sensor_msgs/Image1280x1024",
            Stream::DebugImage => "sensor_msgs/Image1280x1024Color",
        }
    }

    /// The module that publishes this stream.
    pub fn source_module(self) -> &'static str {
        match self {
            Stream::Image => "ips_cam_gcam_pub",
            Stream::DebugImage => "ips_detect",
        }
    }

    /// The source publisher's service name,
    /// `<device>/ws_ips/<source_module>-<inst>/iox2/<topic>`.
    pub fn source_key(self, inst: &str) -> String {
        match self {
            Stream::Image => image_overrides(inst).service_name(),
            Stream::DebugImage => debug_image_overrides(inst).service_name(),
        }
    }
}

/// Overrides that point the generated subscriber at the camera publisher's key
/// `<device>/ws_ips/ips_cam_gcam_pub-<inst>/iox2/image`.
fn image_overrides(inst: &str) -> image_sub::TopicOverrides {
    image_sub::TopicOverrides {
        full: Some(format!(
            "{}/{}/{}-{inst}/{}/{}",
            image_sub::DEFAULT_DEVICE,
            image_sub::DEFAULT_WORKSPACE,
            Stream::Image.source_module(),
            image_sub::TRANSPORT_SEGMENT,
            image_sub::DEFAULT_TOPIC
        )),
        ..Default::default()
    }
}

/// Overrides that point the generated subscriber at the detector's key
/// `<device>/ws_ips/ips_detect-<inst>/iox2/debug_image`.
fn debug_image_overrides(inst: &str) -> debug_image_sub::TopicOverrides {
    debug_image_sub::TopicOverrides {
        full: Some(format!(
            "{}/{}/{}-{inst}/{}/{}",
            debug_image_sub::DEFAULT_DEVICE,
            debug_image_sub::DEFAULT_WORKSPACE,
            Stream::DebugImage.source_module(),
            debug_image_sub::TRANSPORT_SEGMENT,
            debug_image_sub::DEFAULT_TOPIC
        )),
        ..Default::default()
    }
}

/// A borrowed view of one received frame, aliasing the iceoryx2 sample. The
/// geometry fields are as the publisher wrote them and not yet validated.
pub struct FrameView<'a> {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    /// NUL-terminated `header.frame_id`.
    pub frame_id: &'a [u8],
    /// NUL-terminated `encoding` (`mono8`, `rgb8`).
    pub encoding: &'a [u8],
    pub width: u32,
    pub height: u32,
    pub step: u32,
    /// The whole fixed-size payload buffer; only `data[..data_len]` is meaningful.
    pub data: &'a [u8],
    pub data_len: u32,
}

/// Both flat payloads share their field names; one impl each.
macro_rules! frame_view_from {
    ($payload:ty) => {
        impl<'a> From<&'a $payload> for FrameView<'a> {
            fn from(p: &'a $payload) -> Self {
                FrameView {
                    stamp_sec: p.header_stamp_sec,
                    stamp_nanosec: p.header_stamp_nanosec,
                    frame_id: &p.header_frame_id,
                    encoding: &p.encoding,
                    width: p.width,
                    height: p.height,
                    step: p.step,
                    data: &p.data,
                    data_len: p.data_len,
                }
            }
        }
    };
}
frame_view_from!(Image1280x1024);
frame_view_from!(Image1280x1024Color);

/// The opened subscriber, one variant per payload type.
enum Sub {
    Image(ImageSubscriber),
    DebugImage(DebugImageSubscriber),
}

/// One stream subscription of one camera instance (data plane only).
pub struct Source {
    pub inst: String,
    pub stream: Stream,
    /// The iceoryx2 service name subscribed to.
    pub key: String,
    /// `None` until the source has created the service (see [`Source::poll`]).
    sub: Option<Sub>,
    started: Instant,
    /// When the service list was last probed while `sub` is `None`.
    last_probe: Option<Instant>,
    /// `None` before the first poll; then when the waiting line was last printed.
    last_wait_log: Option<Instant>,
    got_first: bool,
}

impl Source {
    /// Prepare the subscription on `inst`'s `stream` key (see [`Stream::source_key`]).
    /// Nothing is opened yet: the subscriber is created by [`Source::poll`] once
    /// the source's service exists, so a waiting relay never creates it.
    pub fn new(inst: &str, stream: Stream) -> Self {
        Self {
            inst: inst.to_string(),
            stream,
            key: stream.source_key(inst),
            sub: None,
            started: Instant::now(),
            last_probe: None,
            last_wait_log: None,
            got_first: false,
        }
    }

    /// Drain to the latest frame; if one arrived, call `f` with a borrowed view
    /// of it and return `f`'s result. The sample is dropped when `f` returns, so
    /// `f` should do only what needs the source pixels. While the source's
    /// service does not exist yet, probes for it every [`PROBE_EVERY`] and opens
    /// the subscriber on `node` as soon as it appears. Until the first frame,
    /// logs the waiting line every [`WAIT_LOG_EVERY`].
    pub fn poll<R>(
        &mut self,
        node: &Node<ipc::Service>,
        f: impl FnOnce(&FrameView<'_>) -> R,
    ) -> Result<Option<R>, Box<dyn std::error::Error>> {
        if self.sub.is_none() {
            let now = Instant::now();
            let due = self
                .last_probe
                .is_none_or(|t| now.duration_since(t) >= PROBE_EVERY);
            if due {
                self.last_probe = Some(now);
                if service_exists(&self.key)? {
                    self.sub = Some(match self.stream {
                        Stream::Image => Sub::Image(ImageSubscriber::open_with(
                            node,
                            image_overrides(&self.inst),
                        )?),
                        Stream::DebugImage => Sub::DebugImage(DebugImageSubscriber::open_with(
                            node,
                            debug_image_overrides(&self.inst),
                        )?),
                    });
                }
            }
        }

        // `&*sample` borrows the payload in shared memory (no copy); the sample
        // itself is dropped at the end of each closure, right after `f`.
        let out = match &self.sub {
            Some(Sub::Image(sub)) => sub
                .try_recv_latest()?
                .map(|sample| f(&FrameView::from(&*sample))),
            Some(Sub::DebugImage(sub)) => sub
                .try_recv_latest()?
                .map(|sample| f(&FrameView::from(&*sample))),
            None => None,
        };

        if !self.got_first {
            self.log_waiting(out.is_some());
        }
        Ok(out)
    }

    /// The first-frame line, or the waiting line on the first empty poll and
    /// every [`WAIT_LOG_EVERY`] after it.
    fn log_waiting(&mut self, got_frame: bool) {
        let now = Instant::now();
        let topic = self.stream.topic();
        let hint = match self.stream {
            Stream::Image => "",
            Stream::DebugImage => "  (published only while ips_detect runs with debug: true)",
        };
        if got_frame {
            self.got_first = true;
            println!(
                "[{}] first {topic} frame from {} after {:.1} s",
                self.inst,
                self.key,
                now.duration_since(self.started).as_secs_f64()
            );
            return;
        }
        match self.last_wait_log {
            None => {
                println!("[{}] waiting for {topic} on {}{hint}", self.inst, self.key);
                self.last_wait_log = Some(now);
            }
            Some(t) if now.duration_since(t) >= WAIT_LOG_EVERY => {
                println!(
                    "[{}] waiting for {topic} on {} ({:.0} s){hint}",
                    self.inst,
                    self.key,
                    now.duration_since(self.started).as_secs_f64()
                );
                self.last_wait_log = Some(now);
            }
            Some(_) => {}
        }
    }
}
