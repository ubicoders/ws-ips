//! One camera from accumulated samples to a written `sbNN_full.yaml`: the fit
//! and the write, shared by the headless run and the window.

use crate::accumulate::Accumulator;
use crate::fit;
use crate::setup::Setup;
use crate::writer::{self, Camchain, FullYaml};

use nalgebra::{Matrix3, Vector3};
use std::path::PathBuf;

/// A fitted camera pose and how it was obtained.
#[derive(Debug, Clone)]
pub struct CamFit {
    /// Camera to world: `world = R * cam + t`.
    pub r: Matrix3<f64>,
    pub t: Vector3<f64>,
    /// RMS of the fitted centres against the measured ones, metres.
    pub rms: f64,
    /// Board ids the fit used, ascending.
    pub ids: Vec<i32>,
    /// Marker frames the medians came from.
    pub frames: u64,
}

/// Fit `inst` from what `acc` holds. `Err` carries the reason it was skipped,
/// worded for the results line.
pub fn fit_camera(setup: &Setup, acc: &Accumulator, inst: &str) -> Result<CamFit, String> {
    let n = acc.frames(inst);
    if n == 0 {
        return Err("no frames received".to_string());
    }
    // Matched (cam, world) pairs by id; median over the frames for each id.
    let (ids, cam_pts, world_pts) = acc.matched(inst, &setup.world);
    if ids.len() < setup.params.min_markers {
        return Err(format!(
            "only {} usable marker(s) ({n} frames), need >={}",
            ids.len(),
            setup.params.min_markers
        ));
    }
    let Some((r, t)) = fit::fit_extrinsics(&cam_pts, &world_pts) else {
        return Err(format!("degenerate marker geometry (ids={ids:?})"));
    };
    let rms = fit::reprojection_residual(&cam_pts, &world_pts, &r, &t);
    Ok(CamFit {
        r,
        t,
        rms,
        ids,
        frames: n,
    })
}

/// Write `inst`'s `sbNN_full.yaml` and return its path.
pub fn write_camera(
    setup: &Setup,
    inst: &str,
    cc: &Camchain,
    f: &CamFit,
) -> Result<PathBuf, String> {
    let out = writer::full_yaml_path(&setup.paths.extrinsics, inst);
    let label = writer::display_relative(&cc.source, &setup.paths.repo);
    FullYaml {
        camera: inst,
        world_frame: &setup.cfg.world_frame,
        camchain: cc,
        camchain_label: &label,
        r: f.r,
        t: f.t,
        residual_rms_m: f.rms,
        markers_used: &f.ids,
        frames: f.frames,
    }
    .write(&out)
    .map_err(|e| format!("cannot write '{}' ({e})", out.display()))?;
    Ok(out)
}

/// The one-line result for a written camera.
pub fn result_line(inst: &str, f: &CamFit, out: &std::path::Path) -> String {
    format!(
        "  {inst}: t=({:>8.4},{:>8.4},{:>8.4}) m  rms={:.4} m  ids={:?}  frames={}  -> {}",
        f.t[0],
        f.t[1],
        f.t[2],
        f.rms,
        f.ids,
        f.frames,
        out.display()
    )
}

/// `residual_rms_m` of an existing `sbNN_full.yaml`, for the overwrite prompt.
/// `None` when there is no file or no such line.
pub fn existing_rms(setup: &Setup, inst: &str) -> Option<f64> {
    let path = writer::full_yaml_path(&setup.paths.extrinsics, inst);
    let text = std::fs::read_to_string(path).ok()?;
    text.lines()
        .find_map(|l| l.strip_prefix("residual_rms_m:"))
        .and_then(|v| v.trim().parse().ok())
}
