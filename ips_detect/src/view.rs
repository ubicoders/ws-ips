//! view: drawing helpers for the annotated debug frame (`debug: true`).
//!
//! The frame is published over iceoryx2 for ips_viz; there is no OpenCV window.

use opencv::core::{self, Mat, Point, Rect, Scalar};
use opencv::imgproc;
use opencv::prelude::*;

type Res<T> = Result<T, Box<dyn std::error::Error>>;

const FONT: i32 = imgproc::FONT_HERSHEY_SIMPLEX;

/// Draw `text` at `org` as a black outline behind a coloured fill, so labels stay
/// legible over both the bright and dark regions of a mono frame. `color` is BGR;
/// `scale` is the font scale (about 0.7 to 0.9 reads well at 1280x1024).
pub fn put_label(img: &mut Mat, text: &str, org: Point, color: Scalar, scale: f64) -> Res<()> {
    imgproc::put_text(
        img,
        text,
        org,
        FONT,
        scale,
        Scalar::all(0.0),
        4,
        imgproc::LINE_AA,
        false,
    )?;
    imgproc::put_text(
        img,
        text,
        org,
        FONT,
        scale,
        color,
        2,
        imgproc::LINE_AA,
        false,
    )?;
    Ok(())
}

/// Alpha-blend a solid `color` panel into the `rect` region of `img`, in place,
/// so overlaid text stays legible over a busy scene. `rect` is clipped to the
/// image (a panel running off an edge is harmless); `alpha` is its opacity
/// (0 = invisible, 1 = solid).
pub fn dim_panel(img: &mut Mat, rect: Rect, color: Scalar, alpha: f64) -> Res<()> {
    let r = rect & Rect::new(0, 0, img.cols(), img.rows()); // clip to image
    if r.width <= 0 || r.height <= 0 {
        return Ok(());
    }
    let mut roi = img.roi_mut(r)?;
    let overlay = Mat::new_rows_cols_with_default(r.height, r.width, roi.typ(), color)?;
    let mut blended = Mat::default();
    core::add_weighted(&roi, 1.0 - alpha, &overlay, alpha, 0.0, &mut blended, -1)?;
    blended.copy_to(&mut roi)?;
    Ok(())
}

const LEGEND_SCALE: f64 = 0.7;
const LEGEND_ROW_H: i32 = 32; // vertical pitch between rows
const LEGEND_PAD: i32 = 14; // inner padding

/// Draw a translucent legend at `origin` (top-left corner of the panel): one row
/// per `(text, color)` entry, generously spaced and on a dimmed backdrop. The
/// panel is auto-sized to the widest row so nothing clips. Keeps labels out of
/// the scene instead of stacking them on top of clustered markers.
pub fn draw_legend(img: &mut Mat, origin: Point, rows: &[(String, Scalar)]) -> Res<()> {
    if rows.is_empty() {
        return Ok(());
    }
    // Measure the widest row so the panel fits the text exactly.
    let mut text_w = 0;
    let mut base = 0;
    for (t, _) in rows {
        let sz = imgproc::get_text_size(t, FONT, LEGEND_SCALE, 2, &mut base)?;
        text_w = text_w.max(sz.width);
    }
    let panel_w = text_w + LEGEND_PAD * 2;
    let panel_h = LEGEND_PAD * 2 + LEGEND_ROW_H * rows.len() as i32;
    dim_panel(
        img,
        Rect::new(origin.x, origin.y, panel_w, panel_h),
        Scalar::all(0.0),
        0.55,
    )?;

    let x = origin.x + LEGEND_PAD;
    let mut y = origin.y + LEGEND_PAD + 20; // baseline of the first row
    for (t, color) in rows {
        put_label(img, t, Point::new(x, y), *color, LEGEND_SCALE)?;
        y += LEGEND_ROW_H;
    }
    Ok(())
}
