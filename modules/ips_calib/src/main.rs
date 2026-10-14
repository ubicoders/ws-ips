//! ips_calib: extrinsic calibration of every IPS camera.
//!
//! Subscribes to every detector's marker stream (`ips_detect-$*`) while the
//! cameras look at the IPS calib board, accumulates each camera's marker centres
//! (camera frame, metres), and, using the board geometry in
//! `calibration/ips_calib.yaml`, solves every camera's pose in the world frame by
//! a rigid least-squares (Kabsch) fit of the board marker centres. Writes
//! `calibration/extrinsics/sbNN_full.yaml` per camera (K copied from the
//! camchain, plus R|t).
//!
//! Two modes, chosen by `ui:` in `parameters/calib_params.yaml`: the interactive
//! window (`session.rs`, keys S C W, writes on confirmation) and the headless
//! one-shot (`headless.rs`, accumulates until `frames` or `timeout_s`, writes,
//! exits). The detectors must run the board's family and tag size
//! (`family: april`, `marker_size: 0.185`). No command-line options.

use ips_calib::headless;
use ips_calib::setup::Setup;

const USAGE: &str = "usage: ips_calib  (no options; settings are in parameters/calib_params.yaml)";

fn main() {
    if let Some(a) = std::env::args().nth(1) {
        eprintln!("error: unexpected argument '{a}'");
        eprintln!("{USAGE}");
        std::process::exit(2);
    }
    match run() {
        Ok(0) => {
            eprintln!("error: no camera could be fitted");
            std::process::exit(1);
        }
        Ok(_) => {}
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
    }
}

/// Returns how many `sbNN_full.yaml` were written. The window returns 1 when
/// the user quit without writing: that is a choice, not a failure.
fn run() -> Result<usize, String> {
    let setup = Setup::load()?;
    if setup.params.ui {
        run_ui(&setup)
    } else {
        headless::run(&setup)
    }
}

#[cfg(feature = "ui")]
fn run_ui(setup: &Setup) -> Result<usize, String> {
    ips_calib::session::run(setup)
}

#[cfg(not(feature = "ui"))]
fn run_ui(_setup: &Setup) -> Result<usize, String> {
    Err("built without the `ui` feature (no OpenCV); set ui: false in calib_params.yaml \
         or rebuild with the default features"
        .to_string())
}
