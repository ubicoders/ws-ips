//! The window's state machine on its one intended path: S, goal reached, C, W,
//! Y walks Idle to Written and asks the session for exactly one start, one fit,
//! one write.

use ips_calib::ui::{Action, CamView, FitSummary, Key, State, Ui};
use std::time::Instant;

fn view(inst: &str, frames: u64, fit: Option<Result<FitSummary, String>>) -> CamView {
    CamView {
        inst: inst.to_string(),
        frames,
        goal: 300,
        ids_seen: vec![0, 1, 2, 3, 4, 5],
        family_ok: true,
        has_image: true,
        fit,
        old_rms: Some(0.039),
    }
}

#[test]
fn ui_happy_path() {
    let now = Instant::now();
    let mut ui = Ui::new(0.01, now);
    let idle = [view("sb00", 0, None), view("sb01", 0, None)];
    assert_eq!(ui.state(), State::Idle);

    assert_eq!(ui.handle(Key::S, &idle, now), vec![Action::StartCollect]);
    assert_eq!(ui.state(), State::Collecting);
    assert_eq!(ui.handle(Key::None, &idle, now), vec![]);

    ui.on_goal_reached();
    assert_eq!(ui.state(), State::Collected);

    let full = [view("sb00", 300, None), view("sb01", 300, None)];
    assert_eq!(ui.handle(Key::C, &full, now), vec![Action::Fit]);
    assert_eq!(ui.state(), State::Fitted);

    let fitted = [
        view(
            "sb00",
            300,
            Some(Ok(FitSummary {
                t: [-1.58, 2.71, -1.76],
                rms: 0.006,
                ids: vec![0, 1, 2, 3, 4, 5],
            })),
        ),
        view("sb01", 300, Some(Err("only 2 usable marker(s)".into()))),
    ];
    assert_eq!(ui.handle(Key::W, &fitted, now), vec![]);
    assert_eq!(ui.state(), State::Confirm);
    let panel = ui.panel(&fitted, now);
    assert_eq!(panel.chips.iter().map(|c| c.0).collect::<String>(), "YN");

    assert_eq!(ui.handle(Key::Y, &fitted, now), vec![Action::Write]);
    ui.on_written(1);
    assert_eq!(ui.state(), State::Written);
    assert!(ui.panel(&fitted, now).instruction.starts_with("Saved 1 file"));

    assert_eq!(ui.handle(Key::Q, &fitted, now), vec![Action::Quit]);
}
