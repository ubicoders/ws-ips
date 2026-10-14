//! The window's state machine and the text it shows. Pure: no OpenCV, no wire,
//! no files. `session.rs` feeds it keys and camera snapshots and executes the
//! actions it returns; `mosaic.rs` draws what [`Ui::panel`] and
//! [`Ui::tile_status`] describe.
//!
//! States: Idle, Collecting (S), Collected (goal reached or S again), Fitted (C),
//! Confirm (W), Written (Y). R returns to Idle from anywhere, Q quits, with a
//! confirmation when a fit would be lost.

use crate::writer::Camchain;
use std::time::{Duration, Instant};

/// How long a key hint stays on the panel.
pub const HINT_TTL: Duration = Duration::from_secs(1);
/// With no key since start, remind the operator to focus the window.
pub const FOCUS_HINT_AFTER: Duration = Duration::from_secs(20);
/// A camera with no frame for this long shows `NO IMAGE`.
pub const NO_IMAGE_AFTER: Duration = Duration::from_secs(2);
/// A rigid fit needs three points; fewer board ids and the camera is skipped.
const MIN_IDS: usize = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    Idle,
    Collecting,
    Collected,
    Fitted,
    Confirm,
    Written,
    QuitConfirm,
}

impl State {
    fn title(self) -> &'static str {
        match self {
            State::Idle => "IDLE",
            State::Collecting => "COLLECTING",
            State::Collected => "COLLECTED",
            State::Fitted => "CALIBRATED",
            State::Confirm => "CONFIRM",
            State::Written => "WRITTEN",
            State::QuitConfirm => "QUIT?",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Key {
    S,
    R,
    C,
    W,
    Y,
    N,
    Q,
    /// Any other key.
    Other(char),
    /// `waitKey` timed out.
    None,
}

impl Key {
    /// From an OpenCV `wait_key` code: `-1` is no key, `Esc` counts as `Q`,
    /// letters are case-insensitive.
    pub fn from_code(code: i32) -> Self {
        if code < 0 {
            return Key::None;
        }
        let c = (code & 0xFF) as u8;
        match c.to_ascii_lowercase() {
            27 => Key::Q,
            b's' => Key::S,
            b'r' => Key::R,
            b'c' => Key::C,
            b'w' => Key::W,
            b'y' => Key::Y,
            b'n' => Key::N,
            b'q' => Key::Q,
            other => Key::Other(other as char),
        }
    }
}

/// What the session must do after a key.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    StartCollect,
    StopCollect,
    Reset,
    Fit,
    Write,
    Quit,
}

/// Colour meaning, mapped to BGR only in `mosaic.rs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tone {
    Neutral,
    Ok,
    Warn,
    Block,
}

/// A fitted camera, as the panel shows it.
#[derive(Debug, Clone, PartialEq)]
pub struct FitSummary {
    /// Camera centre in world, metres.
    pub t: [f64; 3],
    pub rms: f64,
    pub ids: Vec<i32>,
}

/// One camera's snapshot for this frame, built by the session.
#[derive(Debug, Clone)]
pub struct CamView {
    pub inst: String,
    pub frames: u64,
    pub goal: u64,
    /// Board ids in the latest detector message.
    pub ids_seen: Vec<i32>,
    /// The latest message carried only the board's family.
    pub family_ok: bool,
    pub has_image: bool,
    /// `Some(Err(reason))` when the fit skipped this camera.
    pub fit: Option<Result<FitSummary, String>>,
    /// `residual_rms_m` of the file on disk, for the overwrite prompt.
    pub old_rms: Option<f64>,
}

/// The text panel under the camera grid.
#[derive(Debug, Clone, PartialEq)]
pub struct Panel {
    pub title: String,
    pub instruction: String,
    pub tone: Tone,
    /// Key and label, only the keys valid now.
    pub chips: Vec<(char, &'static str)>,
    pub hint: Option<String>,
}

/// One tile's decorations.
#[derive(Debug, Clone, PartialEq)]
pub struct TileStatus {
    /// Top-right counter while collecting, e.g. `142/300`.
    pub counter: Option<String>,
    /// Progress bar fill, 0 to 1, while collecting.
    pub bar: Option<f32>,
    /// Bottom line.
    pub line: String,
    pub tone: Tone,
}

/// One row of the overwrite prompt.
#[derive(Debug, Clone, PartialEq)]
pub struct ConfirmRow {
    pub file: String,
    pub old_rms: Option<f64>,
    /// `None` for a skipped camera.
    pub new_rms: Option<f64>,
}

pub struct Ui {
    state: State,
    rms_warn_m: f64,
    hint: Option<(String, Instant)>,
    started_at: Instant,
    last_key_at: Option<Instant>,
    collect_started_at: Option<Instant>,
    written: usize,
}

impl Ui {
    pub fn new(rms_warn_m: f64, now: Instant) -> Self {
        Self {
            state: State::Idle,
            rms_warn_m,
            hint: None,
            started_at: now,
            last_key_at: None,
            collect_started_at: None,
            written: 0,
        }
    }

    pub fn state(&self) -> State {
        self.state
    }

    /// Every camera reached its frame goal.
    pub fn on_goal_reached(&mut self) {
        if self.state == State::Collecting {
            self.state = State::Collected;
        }
    }

    /// The write finished; `n` files were written.
    pub fn on_written(&mut self, n: usize) {
        self.written = n;
        self.state = State::Written;
    }

    /// One key. Returns what the session must do, in order.
    pub fn handle(&mut self, key: Key, views: &[CamView], now: Instant) -> Vec<Action> {
        if key == Key::None {
            return Vec::new();
        }
        self.last_key_at = Some(now);
        use Action::*;
        use State::*;
        let (next, actions): (State, Vec<Action>) = match (self.state, key) {
            (_, Key::Q) if self.state == Fitted => (QuitConfirm, vec![]),
            (QuitConfirm, Key::Y) => (QuitConfirm, vec![Quit]),
            (QuitConfirm, Key::N) => (Fitted, vec![]),
            (QuitConfirm, _) => return self.refuse(key, now),
            (_, Key::Q) => (self.state, vec![Quit]),
            (Confirm, Key::Y) => (Confirm, vec![Write]),
            (Confirm, Key::N) => (Fitted, vec![]),
            (Confirm, _) => return self.refuse(key, now),
            (Idle, Key::S) => {
                if let Some(v) = views.iter().find(|v| !v.family_ok) {
                    return self.hint_now(
                        format!("{} runs another family, fix detector_params.yaml", v.inst),
                        now,
                    );
                }
                self.collect_started_at = Some(now);
                (Collecting, vec![StartCollect])
            }
            (Collecting, Key::S) => (Collected, vec![StopCollect]),
            (Collecting | Collected, Key::C) => (Fitted, vec![Fit]),
            (Fitted, Key::W) => {
                if views.iter().all(|v| !matches!(v.fit, Some(Ok(_)))) {
                    return self.hint_now("nothing to write, every camera skipped".into(), now);
                }
                (Confirm, vec![])
            }
            (Collecting | Collected | Fitted | Written, Key::R) => {
                self.collect_started_at = None;
                (Idle, vec![Reset])
            }
            _ => return self.refuse(key, now),
        };
        self.state = next;
        actions
    }

    fn refuse(&mut self, key: Key, now: Instant) -> Vec<Action> {
        let name = match key {
            Key::S => 'S',
            Key::R => 'R',
            Key::C => 'C',
            Key::W => 'W',
            Key::Y => 'Y',
            Key::N => 'N',
            Key::Q => 'Q',
            Key::Other(c) => c.to_ascii_uppercase(),
            Key::None => return Vec::new(),
        };
        self.hint_now(format!("{name} does nothing here"), now)
    }

    fn hint_now(&mut self, text: String, now: Instant) -> Vec<Action> {
        self.hint = Some((text, now));
        Vec::new()
    }

    fn live_hint(&self, now: Instant) -> Option<String> {
        if let Some((text, at)) = &self.hint {
            if now.duration_since(*at) <= HINT_TTL {
                return Some(text.clone());
            }
        }
        if self.state == State::Idle
            && self.last_key_at.is_none()
            && now.duration_since(self.started_at) >= FOCUS_HINT_AFTER
        {
            return Some("Click the window, then press a key".to_string());
        }
        None
    }

    fn tone_of_rms(&self, rms: f64) -> Tone {
        if rms <= self.rms_warn_m {
            Tone::Ok
        } else {
            Tone::Warn
        }
    }

    /// The panel for this frame.
    pub fn panel(&self, views: &[CamView], now: Instant) -> Panel {
        use State::*;
        let n = views.len();
        let (title, instruction, tone, chips): (String, String, Tone, Vec<(char, &str)>) =
            match self.state {
                Idle => {
                    let quit = ('Q', "Quit");
                    if let Some(v) = views.iter().find(|v| !v.family_ok) {
                        (
                            Idle.title().into(),
                            format!(
                                "{} detector runs another family, board is april. \
                                 Fix detector_params.yaml, restart detectors.",
                                v.inst
                            ),
                            Tone::Block,
                            vec![quit],
                        )
                    } else if let Some(v) = views.iter().find(|v| !v.has_image) {
                        (
                            Idle.title().into(),
                            format!("No images from {}. Check its camera window.", v.inst),
                            Tone::Block,
                            vec![('S', "Start anyway"), quit],
                        )
                    } else if let Some(v) = views.iter().find(|v| v.ids_seen.len() < MIN_IDS) {
                        (
                            Idle.title().into(),
                            format!(
                                "{} sees {} tag(s), needs {MIN_IDS}. Move the board or camera.",
                                v.inst,
                                v.ids_seen.len()
                            ),
                            Tone::Warn,
                            vec![('S', "Start anyway"), quit],
                        )
                    } else {
                        (
                            Idle.title().into(),
                            format!("Board visible on all {n} cameras. Keep it still, press S."),
                            Tone::Ok,
                            vec![('S', "Start collecting"), quit],
                        )
                    }
                }
                Collecting => {
                    let secs = self
                        .collect_started_at
                        .map(|t| now.duration_since(t).as_secs_f64())
                        .unwrap_or(0.0);
                    let dead: Vec<&str> = views
                        .iter()
                        .filter(|v| !v.has_image)
                        .map(|v| v.inst.as_str())
                        .collect();
                    let (text, tone) = if dead.is_empty() {
                        ("Keep the board still.".to_string(), Tone::Ok)
                    } else {
                        (
                            format!(
                                "Keep the board still. {} sends no frames, check camera.",
                                dead.join(", ")
                            ),
                            Tone::Warn,
                        )
                    };
                    (
                        format!("{}   {secs:.1} s", Collecting.title()),
                        text,
                        tone,
                        vec![('S', "Stop early"), ('C', "Calibrate now"), ('R', "Reset"), ('Q', "Quit")],
                    )
                }
                Collected => (
                    Collected.title().into(),
                    "Collection done. Press C to calibrate.".into(),
                    Tone::Ok,
                    vec![('C', "Calibrate"), ('R', "Reset"), ('Q', "Quit")],
                ),
                Fitted | QuitConfirm | Confirm => {
                    let good = views
                        .iter()
                        .filter(|v| matches!(&v.fit, Some(Ok(f)) if f.rms <= self.rms_warn_m))
                        .count();
                    let mut problems: Vec<String> = Vec::new();
                    for v in views {
                        match &v.fit {
                            Some(Ok(f)) if f.rms > self.rms_warn_m => {
                                problems.push(format!("{} residual {:.1} cm", v.inst, f.rms * 100.0))
                            }
                            Some(Err(why)) => problems.push(format!("{} skipped ({why})", v.inst)),
                            _ => {}
                        }
                    }
                    let title = format!("{}   {good} of {n} under {:.0} cm", Fitted.title(), self.rms_warn_m * 100.0);
                    let (text, tone) = if problems.is_empty() {
                        (
                            "All residuals under the limit. Press W to write the files.".to_string(),
                            Tone::Ok,
                        )
                    } else if good == 0 {
                        (format!("{}. Press R to redo.", problems.join(", ")), Tone::Block)
                    } else {
                        (
                            format!("{}. Write anyway, or R to redo.", problems.join(", ")),
                            Tone::Warn,
                        )
                    };
                    match self.state {
                        Confirm => (
                            title,
                            "Overwrite the files listed above?".into(),
                            tone,
                            vec![('Y', "Yes, overwrite"), ('N', "Cancel")],
                        ),
                        QuitConfirm => (
                            QuitConfirm.title().into(),
                            "Quit without saving the calibration?".into(),
                            Tone::Warn,
                            vec![('Y', "Yes, quit"), ('N', "Back")],
                        ),
                        _ => (
                            title,
                            text,
                            tone,
                            vec![('W', "Write files"), ('R', "Redo"), ('Q', "Quit without saving")],
                        ),
                    }
                }
                Written => (
                    Written.title().into(),
                    format!(
                        "Saved {} file(s). Set detectors back to aruco 0.087, then sb down && sb up. Press Q to close.",
                        self.written
                    ),
                    Tone::Ok,
                    vec![('R', "Calibrate again"), ('Q', "Quit")],
                ),
            };
        Panel {
            title,
            instruction,
            tone,
            chips,
            hint: self.live_hint(now),
        }
    }

    /// One tile's decorations for this frame.
    pub fn tile_status(&self, v: &CamView) -> TileStatus {
        use State::*;
        let counter = match self.state {
            Idle => None,
            _ => Some(format!("{}/{}", v.frames, v.goal)),
        };
        let bar = match self.state {
            Collecting => Some((v.frames as f32 / v.goal.max(1) as f32).min(1.0)),
            _ => None,
        };
        let (line, tone) = match self.state {
            Fitted | Confirm | QuitConfirm | Written => match &v.fit {
                Some(Ok(f)) => (
                    format!(
                        "rms {:.3} m  t {:.2} {:.2} {:.2}",
                        f.rms, f.t[0], f.t[1], f.t[2]
                    ),
                    self.tone_of_rms(f.rms),
                ),
                Some(Err(why)) => (format!("skipped: {why}"), Tone::Block),
                None => ("not fitted".to_string(), Tone::Block),
            },
            Collecting | Collected => {
                if !v.has_image {
                    ("NO IMAGE".to_string(), Tone::Block)
                } else {
                    (format!("ids {}", v.ids_seen.len()), Tone::Ok)
                }
            }
            Idle => {
                if !v.has_image {
                    ("NO IMAGE".to_string(), Tone::Block)
                } else if !v.family_ok {
                    ("wrong family".to_string(), Tone::Block)
                } else if v.ids_seen.len() < MIN_IDS {
                    (
                        format!("tags {}  NEED {MIN_IDS}", v.ids_seen.len()),
                        Tone::Warn,
                    )
                } else {
                    (format!("tags {}  OK", v.ids_seen.len()), Tone::Ok)
                }
            }
        };
        TileStatus {
            counter,
            bar,
            line,
            tone,
        }
    }

    /// The overwrite prompt's rows, one per camera.
    pub fn confirm_rows(views: &[CamView]) -> Vec<ConfirmRow> {
        views
            .iter()
            .map(|v| ConfirmRow {
                file: format!("{}_full.yaml", v.inst),
                old_rms: v.old_rms,
                new_rms: match &v.fit {
                    Some(Ok(f)) => Some(f.rms),
                    _ => None,
                },
            })
            .collect()
    }
}

/// Pixel of a camera-frame point through the pinhole `K`, distortion ignored
/// (a label a few pixels off is fine). `None` behind the camera.
pub fn project(k: &Camchain, p: [f64; 3]) -> Option<(f64, f64)> {
    if p[2] <= 0.0 || p[2].is_nan() {
        return None;
    }
    Some((k.fx * p[0] / p[2] + k.cx, k.fy * p[1] / p[2] + k.cy))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    /// A point on the optical axis lands on the principal point: the one place
    /// a wrong axis or scale in the overlay would mislead the operator.
    #[test]
    fn project_centre_pixel() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../calibration/intrinsics");
        let k = Camchain::load(&root.join("sb00-camchain.yaml")).unwrap();
        let (u, v) = project(&k, [0.0, 0.0, 2.0]).unwrap();
        assert!((u - k.cx).abs() < 1e-9 && (v - k.cy).abs() < 1e-9, "{u} {v}");
        assert!(project(&k, [0.1, 0.1, 0.0]).is_none());
    }
}
