//! scale: validate a received frame's geometry and downscale it with OpenCV.
//!
//! [`Geometry::check`] turns the publisher-written width, height, step and
//! data_len of a [`FrameView`] into trusted numbers, or says why the frame is
//! malformed. [`Downscaler::resize`] then wraps the shared-memory pixels in an
//! OpenCV `Mat` without copying and shrinks them by the integer divisor with
//! `INTER_AREA` (pixel-area averaging, the right filter for decimation) into a
//! reusable, contiguous output `Mat`.

use crate::source::FrameView;
use opencv::core::{Mat, Rect, Size};
use opencv::imgproc;
use opencv::prelude::*;

/// Validated geometry of one frame and its downscaled output.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Geometry {
    /// Source size in pixels.
    pub width: usize,
    pub height: usize,
    /// Bytes per pixel (1 MONO8, 3 RGB8).
    pub channels: usize,
    /// Source row stride in bytes: the sample's `step`, or `width * channels`
    /// when the publisher left it 0.
    pub step: usize,
    /// Output size in pixels: source size / scale, rounded down.
    pub out_width: usize,
    pub out_height: usize,
}

impl Geometry {
    /// Check `view` against its payload buffer for a `channels`-byte-per-pixel
    /// stream downscaled by `scale` (>= 1). Every bound is checked before any
    /// pixel is touched, so a garbled header can never index past the buffer.
    pub fn check(view: &FrameView<'_>, channels: usize, scale: u32) -> Result<Self, String> {
        let (width, height) = (view.width as usize, view.height as usize);
        if width == 0 || height == 0 {
            return Err(format!("empty frame {width}x{height}"));
        }
        let row = width
            .checked_mul(channels)
            .ok_or_else(|| format!("width {width} overflows"))?;
        let step = if view.step == 0 {
            row
        } else {
            view.step as usize
        };
        if step < row {
            return Err(format!(
                "step {step} is shorter than width*channels {row} ({width}x{channels})"
            ));
        }
        let len = view.data_len as usize;
        if len > view.data.len() {
            return Err(format!(
                "data_len {len} exceeds the payload capacity {}",
                view.data.len()
            ));
        }
        let need = step
            .checked_mul(height)
            .ok_or_else(|| format!("step*height overflows ({step}x{height})"))?;
        if len < need {
            return Err(format!(
                "data_len {len} is shorter than step*height {need} ({step}x{height})"
            ));
        }
        let scale = scale.max(1) as usize;
        let (out_width, out_height) = (width / scale, height / scale);
        if out_width == 0 || out_height == 0 {
            return Err(format!(
                "frame {width}x{height} is too small to divide by {scale}"
            ));
        }
        // need <= len <= capacity (3.9 MB), so every size here fits an i32.
        Ok(Self {
            width,
            height,
            channels,
            step,
            out_width,
            out_height,
        })
    }

    /// Output row length in bytes (the published `step`): no padding.
    pub fn out_step(&self) -> usize {
        self.out_width * self.channels
    }
}

/// Downscales frames into one reusable output `Mat`.
pub struct Downscaler {
    out: Mat,
}

impl Default for Downscaler {
    fn default() -> Self {
        Self::new()
    }
}

impl Downscaler {
    pub fn new() -> Self {
        Self {
            out: Mat::default(),
        }
    }

    /// Resize `view`'s pixels (validated by `g`) into the output `Mat`.
    ///
    /// The source is wrapped, not copied: a `CV_8UC1` `height x step` view of
    /// the bytes, cut to `width * channels` columns (dropping any row padding),
    /// then reinterpreted as `CV_8UC<channels>` `height x width`. OpenCV allows
    /// that channel reshape on a non-contiguous view because the row count stays
    /// the same. `resize` reallocates the output only when its size or type changes.
    pub fn resize(&mut self, view: &FrameView<'_>, g: &Geometry) -> opencv::Result<()> {
        let rows = g.height as i32;
        let bytes = &view.data[..g.step * g.height];
        let padded = Mat::new_rows_cols_with_data::<u8>(rows, g.step as i32, bytes)?;
        let unpadded = Mat::roi(
            &padded,
            Rect::new(0, 0, (g.width * g.channels) as i32, rows),
        )?;
        let src = unpadded.reshape(g.channels as i32, 0)?;
        imgproc::resize(
            &src,
            &mut self.out,
            Size::new(g.out_width as i32, g.out_height as i32),
            0.0,
            0.0,
            imgproc::INTER_AREA,
        )
    }

    /// The last resized frame's pixels, `out_height` rows of `out_step` bytes.
    pub fn output_bytes(&self) -> opencv::Result<&[u8]> {
        self.out.data_bytes()
    }
}
