//! relay: one stream of one camera, from iceoryx2 sample to protobuf message.
//!
//! [`StreamRelay::poll`] receives the latest frame of its [`Source`], keeps
//! every `every`-th one, validates its geometry, downscales it while the
//! shared-memory sample is held, drops the sample, and fills a reusable
//! `sensor_msgs/Image` with the result for the caller to publish. Header stamp,
//! frame_id and encoding are copied from the source frame; width, height and
//! step describe the downscaled image, which is always unpadded.

use crate::msgs::builtin_interfaces::Time;
use crate::msgs::sensor_msgs::Image;
use crate::msgs::std_msgs::Header;
use crate::scale::{Downscaler, Geometry};
use crate::source::{FrameView, Source, Stream};
use iceoryx2::prelude::*;

/// Replace `dst` with the NUL-terminated string at the start of `src` (lossy UTF-8).
fn set_c_string(dst: &mut String, src: &[u8]) {
    let end = src.iter().position(|&b| b == 0).unwrap_or(src.len());
    dst.clear();
    dst.push_str(&String::from_utf8_lossy(&src[..end]));
}

/// What one received frame came to while its sample was held.
enum Outcome {
    /// Not an `every`-th frame.
    Skipped,
    /// Geometry did not check out; the reason.
    Malformed(String),
    /// Downscaled into the `Downscaler`, header fields set in the message.
    Ready,
}

/// Relay state for one stream of one camera.
pub struct StreamRelay {
    pub source: Source,
    scale: u32,
    every: u64,
    scaler: Downscaler,
    /// Reused for every published frame so its strings and buffer keep their capacity.
    msg: Image,
    /// Frames received so far (relayed or not), for the `every` decimation.
    received: u64,
    /// A malformed frame has been reported; later ones are skipped silently.
    warned_malformed: bool,
}

impl StreamRelay {
    /// Relay `inst`'s `stream`, downscaled by `scale` (>= 1), keeping every
    /// `every`-th received frame (>= 1). Nothing is opened until the first poll.
    pub fn new(inst: &str, stream: Stream, scale: u32, every: u64) -> Self {
        Self {
            source: Source::new(inst, stream),
            scale: scale.max(1),
            every: every.max(1),
            scaler: Downscaler::new(),
            msg: Image::default(),
            received: 0,
            warned_malformed: false,
        }
    }

    /// Poll the source once. Returns the downscaled message when a frame was
    /// received, kept and valid; `None` when there was nothing new, the frame
    /// was decimated away, or it was malformed (reported once, then skipped).
    /// Errors are iceoryx2 or OpenCV failures, fatal for this camera's thread.
    pub fn poll(
        &mut self,
        node: &Node<ipc::Service>,
    ) -> Result<Option<&Image>, Box<dyn std::error::Error>> {
        let stream = self.source.stream;
        let channels = stream.channels();
        let (scale, every) = (self.scale, self.every);
        let received = &mut self.received;
        let scaler = &mut self.scaler;
        let msg = &mut self.msg;

        let outcome = self.source.poll(node, |view: &FrameView<'_>| {
            *received += 1;
            if !(*received - 1).is_multiple_of(every) {
                return Ok(Outcome::Skipped);
            }
            let g = match Geometry::check(view, channels, scale) {
                Ok(g) => g,
                Err(reason) => return Ok(Outcome::Malformed(reason)),
            };
            scaler.resize(view, &g)?;

            let header = msg.header.get_or_insert_with(Header::default);
            header.stamp = Some(Time {
                sec: view.stamp_sec,
                nanosec: view.stamp_nanosec,
            });
            set_c_string(&mut header.frame_id, view.frame_id);
            set_c_string(&mut msg.encoding, view.encoding);
            msg.width = g.out_width as u32;
            msg.height = g.out_height as u32;
            msg.step = g.out_step() as u32;
            msg.is_bigendian = 0;
            Ok::<_, opencv::Error>(Outcome::Ready)
        })?;
        // The iceoryx2 sample is gone from here on; only the downscaled copy remains.

        match outcome.transpose()? {
            None | Some(Outcome::Skipped) => Ok(None),
            Some(Outcome::Malformed(reason)) => {
                if !self.warned_malformed {
                    self.warned_malformed = true;
                    eprintln!(
                        "[{}] warning: skipping malformed {} frame from {}: {reason} \
                         (further malformed frames are skipped silently)",
                        self.source.inst,
                        stream.topic(),
                        self.source.key
                    );
                }
                Ok(None)
            }
            Some(Outcome::Ready) => {
                let bytes = self.scaler.output_bytes()?;
                self.msg.data.clear();
                self.msg.data.extend_from_slice(bytes);
                Ok(Some(&self.msg))
            }
        }
    }
}
