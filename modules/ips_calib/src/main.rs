//! ips_calib: extrinsic calibration of every IPS camera.
//!
//! Subscribes to every detector's marker stream (`ips_detect-$*`) while the
//! cameras look at the IPS calib board, accumulates each camera's marker centres
//! (camera frame, metres), and, using the board geometry in
//! `parameters/ips_calib.yaml`, solves every camera's pose in the world frame by
//! a rigid least-squares (Kabsch) fit of the board marker centres. Writes
//! `calibration/extrinsics/sbNN_full.yaml` per camera (K copied from the
//! camchain, plus R|t).
//!
//! Two modes, chosen by `ui:` in the same `parameters/ips_calib.yaml`: the interactive
//! window (`session.rs`, keys S C W, writes on confirmation) and the headless
//! one-shot (`headless.rs`, accumulates until `frames` or `timeout_s`, asks on the terminal, writes,
//! exits). The detectors must run the board's family and tag size
//! (`family: april`, `marker_size: 0.185`). No command-line options.

use ips_calib::headless;
use ips_calib::setup::Setup;

const USAGE: &str = "usage: ips_calib  (no options; settings are in parameters/ips_calib.yaml)";

fn main() {
    if let Some(a) = std::env::args().nth(1) {
        eprintln!("error: unexpected argument '{a}'");
        eprintln!("{USAGE}");
        std::process::exit(2);
    }
    if let Err(e) = run() {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

/// The window succeeds whenever the operator quits: leaving without writing is
/// a choice, not a failure. The headless run fails when no camera was fitted,
/// since nothing else would tell a script that the run produced nothing.
fn run() -> Result<(), String> {
    let setup = Setup::load()?;
    if setup.params.ui {
        run_ui(&setup)
    } else {
        let out = headless::run(&setup)?;
        if out.fitted == 0 {
            return Err("no camera could be fitted".into());
        }
        Ok(())
    }
}

#[cfg(feature = "ui")]
fn run_ui(setup: &Setup) -> Result<(), String> {
    drop_snap_gtk_env();
    ips_calib::session::run(setup)
}

/// A terminal opened by a snap application (the VS Code snap, say) carries GTK
/// module paths that point into the snap's own core20 runtime. OpenCV's highgui
/// initialises GTK when it creates the window, GTK follows those paths, and the
/// snap's glibc 2.31 `libpthread` is loaded against the host glibc, which fails
/// with `undefined symbol: __libc_pthread_init`. No other node opens a window,
/// so this is handled here: when launched from a snap, forget those paths.
#[cfg(feature = "ui")]
fn drop_snap_gtk_env() {
    const VARS: [&str; 10] = [
        "GTK_PATH",
        "GTK_EXE_PREFIX",
        "GTK_IM_MODULE_FILE",
        "GIO_MODULE_DIR",
        "GDK_PIXBUF_MODULE_FILE",
        "GDK_PIXBUF_MODULEDIR",
        "GSETTINGS_SCHEMA_DIR",
        "LOCPATH",
        "SNAP",
        "SNAP_LIBRARY_PATH",
    ];
    let from_snap = VARS.iter().any(|v| {
        std::env::var(v).map_or(false, |val| val.contains("/snap/"))
    });
    if !from_snap {
        return;
    }
    for var in VARS {
        std::env::remove_var(var);
    }
    eprintln!("snap-launched terminal: GTK module paths dropped for the window");
}

#[cfg(not(feature = "ui"))]
fn run_ui(_setup: &Setup) -> Result<(), String> {
    Err("built without the `ui` feature (no OpenCV); set ui: false in ips_calib.yaml \
         or rebuild with the default features"
        .to_string())
}
