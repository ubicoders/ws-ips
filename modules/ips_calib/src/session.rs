//! The interactive run (`ui: true`): one OpenCV window with every camera's live
//! frame and the board tags it sees, driven by keys through the state machine in
//! `ui.rs`. Collects only while the operator says so (S), fits on C, and writes
//! the `sbNN_full.yaml` files only once the overwrite prompt is confirmed (W, Y).
//!
//! This file owns the wire, the cameras, and the window. `ui.rs` decides what a
//! key means and what the panel says; `mosaic.rs` draws it. Every state change is
//! mirrored to stdout, so the terminal keeps a log of the run. Behind the `ui`
//! feature.

use crate::accumulate::Accumulator;
use crate::calibrate::{self, CamFit};
use crate::images::{self, Camera};
use crate::mosaic::{Canvas, Label};
use crate::msgs::MarkerDetections;
use crate::setup::{MarkerStream, Setup};
use crate::ui::{project, Action, CamView, FitSummary, Key, State, Ui, NO_IMAGE_AFTER};

use iceoryx2::prelude::*;
use opencv::core::{Mat, Scalar, CV_8UC1};
use opencv::highgui;
use opencv::prelude::*;
use std::time::Instant;

type Res<T> = Result<T, Box<dyn std::error::Error>>;

/// The window's name, which is also its title.
pub const WINDOW: &str = "ips_calib";
/// `wait_key` timeout per frame, milliseconds. It paces the loop and is where
/// highgui handles the window's events, so it stays short.
const WAIT_MS: i32 = 10;
/// `WND_PROP_VISIBLE` reads this while the window is shown and less once the
/// user has closed it.
const WINDOW_VISIBLE: f64 = 1.0;
/// Where the overwrite prompt says the files go, relative to the repository.
const EXTRINSICS_DIR: &str = "calibration/extrinsics/";

/// Whether the loop goes on after a frame or an action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Flow {
    Continue,
    Quit,
}

/// Everything the window needs between frames. Every per-camera `Vec` is in
/// `setup.cams` order.
struct Session<'a> {
    setup: &'a Setup,
    stream: MarkerStream,
    /// Declared before `node` so the subscribers are dropped before their node.
    cameras: Vec<Camera>,
    node: Node<ipc::Service>,
    /// Each camera's latest frame at sensor resolution, refreshed in place.
    monos: Vec<Mat>,
    /// Each camera's latest detector message. Read while idle too, so the
    /// operator can aim the cameras before collecting.
    latest: Vec<Option<MarkerDetections>>,
    /// `None` until S. Created on S, so a new collection never mixes in frames
    /// from before it.
    acc: Option<Accumulator>,
    /// `Some(Err(reason))` for a camera the fit skipped.
    fits: Vec<Option<Result<CamFit, String>>>,
    /// `residual_rms_m` of the file on disk, for the overwrite prompt.
    old_rms: Vec<Option<f64>>,
    ui: Ui,
    canvas: Canvas,
    /// Files written over the whole run, for the closing line.
    written: usize,
    /// The state last printed, so only changes are mirrored to stdout.
    last_state: State,
}

impl<'a> Session<'a> {
    /// Subscribe to the markers and the cameras and open the window.
    fn new(setup: &'a Setup) -> Res<Self> {
        let n = setup.cams.len();
        let stream = MarkerStream::open()?;
        let node = NodeBuilder::new().create::<ipc::Service>()?;
        let cameras = setup
            .cams
            .iter()
            .map(|(inst, _)| Camera::open(inst))
            .collect();
        let monos = (0..n).map(|_| full_frame()).collect::<Res<Vec<Mat>>>()?;
        let old_rms = setup
            .cams
            .iter()
            .map(|(inst, _)| calibrate::existing_rms(setup, inst))
            .collect();
        let canvas = Canvas::new(n)?;
        // GUI_NORMAL: no toolbar or status bar from a Qt-built highgui, the
        // panel is the window's only text.
        highgui::named_window(WINDOW, highgui::WINDOW_AUTOSIZE | highgui::WINDOW_GUI_NORMAL)?;
        let ui = Ui::new(setup.params.rms_warn_m, Instant::now());
        let last_state = ui.state();
        Ok(Self {
            setup,
            stream,
            cameras,
            node,
            monos,
            latest: vec![None; n],
            acc: None,
            fits: vec![None; n],
            old_rms,
            ui,
            canvas,
            written: 0,
            last_state,
        })
    }

    /// One frame: take in what arrived, show it, and act on the key.
    fn tick(&mut self) -> Res<Flow> {
        self.drain_markers()?;
        self.poll_cameras()?;
        let now = Instant::now();
        let views = self.views(now);
        self.draw(&views, now)?;

        let code = highgui::wait_key(WAIT_MS)?;
        if highgui::get_window_property(WINDOW, highgui::WND_PROP_VISIBLE)? < WINDOW_VISIBLE {
            return Ok(Flow::Quit);
        }
        for action in self.ui.handle(Key::from_code(code), &views, now) {
            if self.apply(action) == Flow::Quit {
                return Ok(Flow::Quit);
            }
        }
        self.mirror_state();
        Ok(Flow::Continue)
    }

    /// Keep every camera's latest message, and accumulate it while collecting.
    fn drain_markers(&mut self) -> Res<()> {
        // Disjoint borrows: the closure holds `latest` and `acc` while `drain`
        // holds the stream.
        let setup = self.setup;
        let Self {
            stream,
            latest,
            acc,
            ui,
            ..
        } = self;
        let collecting = ui.state() == State::Collecting;
        stream.drain(|msg| {
            let Some(idx) = setup.cams.iter().position(|(inst, _)| *inst == msg.camera) else {
                return true;
            };
            latest[idx] = Some(msg.clone());
            if collecting {
                if let Some(notice) = acc.as_mut().and_then(|a| a.ingest(msg)) {
                    println!("{notice}");
                }
            }
            true
        })?;
        if acc.as_ref().is_some_and(Accumulator::done) {
            ui.on_goal_reached();
        }
        Ok(())
    }

    /// Copy each camera's new frame, if any, into its full-resolution `Mat`.
    fn poll_cameras(&mut self) -> Res<()> {
        for (camera, mono) in self.cameras.iter_mut().zip(self.monos.iter_mut()) {
            if camera.poll(&self.node)? {
                mono.data_bytes_mut()?[..images::FRAME_BYTES]
                    .copy_from_slice(&camera.frame[..images::FRAME_BYTES]);
            }
        }
        Ok(())
    }

    /// Every camera's snapshot for this frame, as `ui.rs` reads it.
    fn views(&self, now: Instant) -> Vec<CamView> {
        let setup = self.setup;
        let family = setup.cfg.board.family.as_str();
        setup
            .cams
            .iter()
            .enumerate()
            .map(|(i, (inst, _))| {
                let markers = self.latest[i]
                    .as_ref()
                    .map(|m| m.markers.as_slice())
                    .unwrap_or_default();
                let mut ids_seen: Vec<i32> = markers
                    .iter()
                    .filter(|m| m.family.starts_with(family) && setup.world.contains_key(&m.id))
                    .map(|m| m.id)
                    .collect();
                ids_seen.sort_unstable();
                // A tag detected twice in one frame is still one tag for the fit.
                ids_seen.dedup();
                CamView {
                    inst: inst.clone(),
                    frames: self.acc.as_ref().map_or(0, |a| a.frames(inst)),
                    goal: setup.params.frames,
                    ids_seen,
                    family_ok: markers.iter().all(|m| m.family.starts_with(family)),
                    has_image: self.cameras[i].is_live(now, NO_IMAGE_AFTER),
                    fit: self.fits[i].as_ref().map(|f| match f {
                        Ok(f) => Ok(summary(f)),
                        Err(why) => Err(why.clone()),
                    }),
                    old_rms: self.old_rms[i],
                }
            })
            .collect()
    }

    /// The board tags in camera `idx`'s latest message, at their sensor pixels.
    fn labels(&self, idx: usize) -> Vec<Label> {
        let (_, camchain) = &self.setup.cams[idx];
        let family = self.setup.cfg.board.family.as_str();
        let Some(msg) = &self.latest[idx] else {
            return Vec::new();
        };
        msg.markers
            .iter()
            .filter(|m| m.family.starts_with(family))
            .filter_map(|m| {
                let p = m.position.as_ref()?;
                let (u, v) = project(camchain, [p.x, p.y, p.z])?;
                Some(Label {
                    u,
                    v,
                    text: format!("[{}]", m.id),
                })
            })
            .collect()
    }

    /// Draw the tiles, the panel, and the overwrite prompt, and show them.
    fn draw(&mut self, views: &[CamView], now: Instant) -> Res<()> {
        for (idx, view) in views.iter().enumerate() {
            // Without a live frame the last message is stale too, and tags over
            // the `NO IMAGE` field would only mislead.
            let labels = if view.has_image {
                self.labels(idx)
            } else {
                Vec::new()
            };
            let mono = view.has_image.then_some(&self.monos[idx]);
            let status = self.ui.tile_status(view);
            self.canvas
                .draw_tile(idx, &view.inst, mono, &labels, &status)?;
        }
        self.canvas.draw_empty_tiles(views.len())?;
        self.canvas.draw_panel(&self.ui.panel(views, now))?;
        if self.ui.state() == State::Confirm {
            self.canvas
                .draw_confirm(&Ui::confirm_rows(views), EXTRINSICS_DIR)?;
        }
        highgui::imshow(WINDOW, &self.canvas.img)?;
        Ok(())
    }

    /// Carry out what the state machine asked for.
    fn apply(&mut self, action: Action) -> Flow {
        match action {
            Action::StartCollect => {
                let setup = self.setup;
                self.acc = Some(Accumulator::new(
                    setup.instances(),
                    setup.params.frames,
                    &setup.cfg.board.family,
                ));
                self.fits.fill(None);
            }
            // Nothing to stop: ingest is gated on the Collecting state, which the
            // state machine has already left.
            Action::StopCollect => {}
            Action::Reset => {
                self.acc = None;
                self.fits.fill(None);
            }
            Action::Fit => self.fit(),
            Action::Write => self.write(),
            Action::Quit => return Flow::Quit,
        }
        Flow::Continue
    }

    /// Fit every camera from what was collected and print the results block.
    fn fit(&mut self) {
        let setup = self.setup;
        println!("\n=== results ===");
        for (i, (inst, _)) in setup.cams.iter().enumerate() {
            let fit = match &self.acc {
                Some(acc) => calibrate::fit_camera(setup, acc, inst),
                None => Err("no frames received".to_string()),
            };
            match &fit {
                Ok(f) => println!("{}", fit_line(inst, f)),
                Err(why) => println!("  {inst}: {why}, skipped"),
            }
            self.fits[i] = Some(fit);
            // Re-read: an earlier W in this run may have replaced the file, and
            // the overwrite prompt compares against what is on disk now.
            self.old_rms[i] = calibrate::existing_rms(setup, inst);
        }
    }

    /// Write every fitted camera's file. A failed write is reported and the
    /// other cameras are still written.
    fn write(&mut self) {
        let setup = self.setup;
        let mut n = 0;
        for ((inst, camchain), fit) in setup.cams.iter().zip(&self.fits) {
            let Some(Ok(fit)) = fit else { continue };
            match calibrate::write_camera(setup, inst, camchain, fit) {
                Ok(out) => {
                    println!("{}", calibrate::result_line(inst, fit, &out));
                    n += 1;
                }
                Err(e) => println!("  {inst}: {e}"),
            }
        }
        self.ui.on_written(n);
        self.written += n;
    }

    /// Print the state when it changed, so the terminal log follows the window.
    fn mirror_state(&mut self) {
        let state = self.ui.state();
        if state != self.last_state {
            println!("[ui] {state:?}");
            self.last_state = state;
        }
    }
}

/// One camera's frame buffer at sensor resolution (MONO8).
fn full_frame() -> Res<Mat> {
    Ok(Mat::new_rows_cols_with_default(
        images::HEIGHT as i32,
        images::WIDTH as i32,
        CV_8UC1,
        Scalar::all(0.0),
    )?)
}

/// A fit as the panel shows it.
fn summary(f: &CamFit) -> FitSummary {
    FitSummary {
        t: [f.t[0], f.t[1], f.t[2]],
        rms: f.rms,
        ids: f.ids.clone(),
    }
}

/// A fitted camera's results line. It has no file yet; once written, the line
/// is `calibrate::result_line`, which adds the path.
fn fit_line(inst: &str, f: &CamFit) -> String {
    format!(
        "  {inst}: t=({:>8.4},{:>8.4},{:>8.4}) m  rms={:.4} m  ids={:?}  frames={}",
        f.t[0], f.t[1], f.t[2], f.rms, f.ids, f.frames
    )
}

/// The whole interactive run: the window until the operator quits or closes it.
/// Quitting without writing is a choice, not a failure, so it returns `Ok`.
pub fn run(setup: &Setup) -> Result<(), String> {
    let mut session = Session::new(setup).map_err(|e| format!("window setup: {e}"))?;
    while session.tick().map_err(|e| format!("window: {e}"))? == Flow::Continue {}

    let written = session.written;
    let undecodable = session.stream.undecodable();
    drop(session);
    highgui::destroy_all_windows().map_err(|e| format!("window close: {e}"))?;

    if undecodable > 0 {
        println!("  {undecodable} undecodable sample(s) ignored");
    }
    match written {
        0 => println!("ips_calib: closed, no file written"),
        n => println!(
            "ips_calib: closed, {n} file(s) written to {}",
            setup.paths.extrinsics.display()
        ),
    }
    Ok(())
}
