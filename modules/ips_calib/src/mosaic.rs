//! mosaic: the one window's picture. Camera tiles in a grid, a text panel under
//! them, and the overwrite prompt. Draws only what `ui.rs` describes; it decides
//! nothing. Behind the `ui` feature (OpenCV highgui and imgproc).
//!
//! Geometry: each tile is the sensor frame at [`SCALE`], the grid is
//! `ceil(sqrt(n))` columns, and the panel spans the grid's width.

use crate::images;
use crate::ui::{ConfirmRow, Panel, TileStatus, Tone};

use opencv::core::{self, Mat, Point, Rect, Scalar, Size, CV_8UC1, CV_8UC3};
use opencv::imgproc;
use opencv::prelude::*;

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// Tile pixels per sensor pixel.
pub const SCALE: f64 = 0.5;
pub const TILE_W: i32 = images::WIDTH as i32 / 2;
pub const TILE_H: i32 = images::HEIGHT as i32 / 2;
pub const PANEL_H: i32 = 110;

const BORDER: i32 = 3;
const BAR_H: i32 = 6;
const PAD: i32 = 12;
const LABEL_RADIUS: i32 = 6;
const CONFIRM_W: i32 = 720;
const CONFIRM_ROW_H: i32 = 32;

const FONT: i32 = imgproc::FONT_HERSHEY_SIMPLEX;
const NAME_SCALE: f64 = 0.8;
const COUNTER_SCALE: f64 = 1.0;
const STATUS_SCALE: f64 = 0.6;
const LABEL_SCALE: f64 = 0.6;
const PANEL_TITLE_SCALE: f64 = 0.9;
const PANEL_TEXT_SCALE: f64 = 0.65;
const CHIP_SCALE: f64 = 0.7;

fn bgr(b: f64, g: f64, r: f64) -> Scalar {
    Scalar::new(b, g, r, 0.0)
}

fn tone_color(t: Tone) -> Scalar {
    match t {
        Tone::Neutral => bgr(220.0, 220.0, 220.0),
        Tone::Ok => bgr(0.0, 200.0, 0.0),
        Tone::Warn => bgr(0.0, 180.0, 255.0),
        Tone::Block => bgr(0.0, 0.0, 255.0),
    }
}

fn white() -> Scalar {
    bgr(235.0, 235.0, 235.0)
}

fn yellow() -> Scalar {
    bgr(0.0, 255.0, 255.0)
}

/// Grid of `n` tiles: `(cols, rows)`.
pub fn grid_for(n: usize) -> (i32, i32) {
    let n = n.max(1) as i32;
    let cols = (n as f64).sqrt().ceil() as i32;
    let rows = (n + cols - 1) / cols;
    (cols, rows)
}

/// A marker label at a sensor pixel.
pub struct Label {
    pub u: f64,
    pub v: f64,
    pub text: String,
}

/// Draw `text` at `org` as a black outline behind a coloured fill, legible over
/// both bright and dark regions of a mono frame.
fn put_label(img: &mut Mat, text: &str, org: Point, color: Scalar, scale: f64) -> Res<()> {
    imgproc::put_text(img, text, org, FONT, scale, Scalar::all(0.0), 4, imgproc::LINE_AA, false)?;
    imgproc::put_text(img, text, org, FONT, scale, color, 2, imgproc::LINE_AA, false)?;
    Ok(())
}

fn text_width(text: &str, scale: f64) -> Res<i32> {
    let mut base = 0;
    Ok(imgproc::get_text_size(text, FONT, scale, 2, &mut base)?.width)
}

/// Alpha-blend a solid `color` panel into `rect`, clipped to the image.
fn dim_panel(img: &mut Mat, rect: Rect, color: Scalar, alpha: f64) -> Res<()> {
    let r = rect & Rect::new(0, 0, img.cols(), img.rows());
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

/// The window's image and the scratch buffers behind it.
pub struct Canvas {
    pub img: Mat,
    cols: i32,
    rows: i32,
    small: Mat,
    tile: Mat,
}

impl Canvas {
    pub fn new(n_cams: usize) -> Res<Self> {
        let (cols, rows) = grid_for(n_cams);
        let img = Mat::new_rows_cols_with_default(
            rows * TILE_H + PANEL_H,
            cols * TILE_W,
            CV_8UC3,
            Scalar::all(0.0),
        )?;
        Ok(Self {
            img,
            cols,
            rows,
            small: Mat::new_rows_cols_with_default(TILE_H, TILE_W, CV_8UC1, Scalar::all(0.0))?,
            tile: Mat::new_rows_cols_with_default(TILE_H, TILE_W, CV_8UC3, Scalar::all(0.0))?,
        })
    }

    pub fn size(&self) -> Size {
        Size::new(self.img.cols(), self.img.rows())
    }

    fn tile_rect(&self, idx: usize) -> Rect {
        let i = idx as i32;
        Rect::new((i % self.cols) * TILE_W, (i / self.cols) * TILE_H, TILE_W, TILE_H)
    }

    fn grid_rect(&self) -> Rect {
        Rect::new(0, 0, self.cols * TILE_W, self.rows * TILE_H)
    }

    fn panel_rect(&self) -> Rect {
        Rect::new(0, self.rows * TILE_H, self.cols * TILE_W, PANEL_H)
    }

    /// One camera tile: the frame (or a dark `NO IMAGE` field), the marker
    /// labels, the name, the counter, the progress bar, the status line, and
    /// the border in the status tone.
    pub fn draw_tile(
        &mut self,
        idx: usize,
        name: &str,
        mono: Option<&Mat>,
        labels: &[Label],
        status: &TileStatus,
    ) -> Res<()> {
        let rect = self.tile_rect(idx);
        match mono {
            Some(m) => {
                imgproc::resize(
                    m,
                    &mut self.small,
                    Size::new(TILE_W, TILE_H),
                    0.0,
                    0.0,
                    imgproc::INTER_AREA,
                )?;
                imgproc::cvt_color_def(&self.small, &mut self.tile, imgproc::COLOR_GRAY2BGR)?;
            }
            None => {
                self.tile.set_to(&bgr(30.0, 30.0, 30.0), &core::no_array())?;
                let w = text_width("NO IMAGE", COUNTER_SCALE)?;
                put_label(
                    &mut self.tile,
                    "NO IMAGE",
                    Point::new((TILE_W - w) / 2, TILE_H / 2),
                    tone_color(Tone::Block),
                    COUNTER_SCALE,
                )?;
            }
        }
        {
            let mut roi = self.img.roi_mut(rect)?;
            self.tile.copy_to(&mut roi)?;
        }
        let img = &mut self.img;
        let color = tone_color(status.tone);

        for l in labels {
            let x = rect.x + (l.u * SCALE) as i32;
            let y = rect.y + (l.v * SCALE) as i32;
            if !rect.contains(Point::new(x, y)) {
                continue;
            }
            imgproc::circle(img, Point::new(x, y), LABEL_RADIUS, yellow(), 2, imgproc::LINE_AA, 0)?;
            put_label(img, &l.text, Point::new(x + 10, y - 10), yellow(), LABEL_SCALE)?;
        }

        put_label(img, name, Point::new(rect.x + PAD, rect.y + 30), white(), NAME_SCALE)?;
        if let Some(c) = &status.counter {
            let w = text_width(c, COUNTER_SCALE)?;
            put_label(
                img,
                c,
                Point::new(rect.x + rect.width - PAD - w, rect.y + 34),
                white(),
                COUNTER_SCALE,
            )?;
        }
        if let Some(f) = status.bar {
            let bar = Rect::new(rect.x + PAD, rect.y + 44, rect.width - 2 * PAD, BAR_H);
            imgproc::rectangle(img, bar, bgr(60.0, 60.0, 60.0), imgproc::FILLED, imgproc::LINE_8, 0)?;
            let fill = Rect::new(bar.x, bar.y, (bar.width as f32 * f.clamp(0.0, 1.0)) as i32, BAR_H);
            if fill.width > 0 {
                imgproc::rectangle(img, fill, color, imgproc::FILLED, imgproc::LINE_8, 0)?;
            }
        }
        put_label(
            img,
            &status.line,
            Point::new(rect.x + PAD, rect.y + rect.height - PAD),
            color,
            STATUS_SCALE,
        )?;
        let inner = Rect::new(
            rect.x + BORDER / 2,
            rect.y + BORDER / 2,
            rect.width - BORDER,
            rect.height - BORDER,
        );
        imgproc::rectangle(img, inner, color, BORDER, imgproc::LINE_8, 0)?;
        Ok(())
    }

    /// Tiles the grid has no camera for (a grid of 4 for 3 cameras).
    pub fn draw_empty_tiles(&mut self, n_cams: usize) -> Res<()> {
        for idx in n_cams..(self.cols * self.rows) as usize {
            let rect = self.tile_rect(idx);
            imgproc::rectangle(&mut self.img, rect, bgr(15.0, 15.0, 15.0), imgproc::FILLED, imgproc::LINE_8, 0)?;
        }
        Ok(())
    }

    /// The text panel under the grid.
    pub fn draw_panel(&mut self, panel: &Panel) -> Res<()> {
        let r = self.panel_rect();
        let img = &mut self.img;
        imgproc::rectangle(img, r, bgr(20.0, 20.0, 20.0), imgproc::FILLED, imgproc::LINE_8, 0)?;
        put_label(
            img,
            &panel.title,
            Point::new(PAD, r.y + 32),
            tone_color(panel.tone),
            PANEL_TITLE_SCALE,
        )?;
        if let Some(h) = &panel.hint {
            let w = text_width(h, PANEL_TEXT_SCALE)?;
            put_label(
                img,
                h,
                Point::new(r.x + r.width - PAD - w, r.y + 32),
                tone_color(Tone::Block),
                PANEL_TEXT_SCALE,
            )?;
        }
        put_label(
            img,
            &panel.instruction,
            Point::new(PAD, r.y + 64),
            white(),
            PANEL_TEXT_SCALE,
        )?;
        let mut x = PAD;
        let y = r.y + PANEL_H - 14;
        for (key, label) in &panel.chips {
            let k = format!("[{key}]");
            put_label(img, &k, Point::new(x, y), yellow(), CHIP_SCALE)?;
            x += text_width(&k, CHIP_SCALE)? + 8;
            put_label(img, label, Point::new(x, y), white(), CHIP_SCALE)?;
            x += text_width(label, CHIP_SCALE)? + 40;
        }
        Ok(())
    }

    /// The overwrite prompt, centred over the grid.
    pub fn draw_confirm(&mut self, rows: &[ConfirmRow], dir: &str) -> Res<()> {
        let grid = self.grid_rect();
        // Title, a header line naming the unit, one row per file, the keys.
        let h = 60 + CONFIRM_ROW_H * (rows.len() as i32 + 1) + 60;
        let bx = Rect::new(
            grid.x + (grid.width - CONFIRM_W) / 2,
            grid.y + (grid.height - h) / 2,
            CONFIRM_W,
            h,
        );
        dim_panel(&mut self.img, bx, Scalar::all(0.0), 0.85)?;
        let img = &mut self.img;
        imgproc::rectangle(img, bx, white(), 2, imgproc::LINE_8, 0)?;
        put_label(
            img,
            &format!("Write to {dir} ?"),
            Point::new(bx.x + 24, bx.y + 40),
            white(),
            PANEL_TITLE_SCALE,
        )?;
        let mut y = bx.y + 60 + 24;
        // Header, aligned with the columns below.
        for (text, dx) in [("residual rms (m)", 24), ("on disk", 300), ("this fit", 480)] {
            put_label(img, text, Point::new(bx.x + dx, y), white(), PANEL_TEXT_SCALE)?;
        }
        y += CONFIRM_ROW_H;
        for row in rows {
            let old = row
                .old_rms
                .map(|v| format!("old {v:.3} m"))
                .unwrap_or_else(|| "old  none".to_string());
            let (new, tone) = match row.new_rms {
                Some(v) => (format!("new {v:.3} m"), Tone::Ok),
                None => ("skipped, kept".to_string(), Tone::Block),
            };
            put_label(img, &row.file, Point::new(bx.x + 24, y), white(), PANEL_TEXT_SCALE)?;
            put_label(img, &old, Point::new(bx.x + 300, y), white(), PANEL_TEXT_SCALE)?;
            put_label(img, &new, Point::new(bx.x + 480, y), tone_color(tone), PANEL_TEXT_SCALE)?;
            y += CONFIRM_ROW_H;
        }
        put_label(
            img,
            "[Y] Overwrite      [N] Cancel",
            Point::new(bx.x + 24, bx.y + h - 20),
            yellow(),
            CHIP_SCALE,
        )?;
        Ok(())
    }
}
