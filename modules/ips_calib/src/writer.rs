//! `calibration/extrinsics/sbNN_full.yaml`, and the camchain intrinsics copied into it.
//!
//! The intrinsics are only copied, never used: the detectors already turned the
//! images into metric marker centres. So this reader needs no OpenCV, unlike the
//! detector's.

use crate::board::mat_to_euler_xyz_deg;
use nalgebra::{Matrix3, Vector3};
use std::path::{Path, PathBuf};

/// One camera's Kalibr camchain: pinhole `[fx, fy, cx, cy]` and the four radtan
/// distortion coefficients `[k1, k2, p1, p2]`.
#[derive(Debug, Clone, PartialEq)]
pub struct Camchain {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub dist: [f64; 4],
    /// The yaml the values came from.
    pub source: PathBuf,
}

/// `<dir>/<inst>-camchain.yaml`, e.g. `.../intrinsics/sb00-camchain.yaml`.
pub fn camchain_path(intrinsics_dir: &Path, inst: &str) -> PathBuf {
    intrinsics_dir.join(format!("{inst}-camchain.yaml"))
}

/// `<dir>/<inst>_full.yaml`, e.g. `.../extrinsics/sb00_full.yaml`.
pub fn full_yaml_path(extrinsics_dir: &Path, inst: &str) -> PathBuf {
    extrinsics_dir.join(format!("{inst}_full.yaml"))
}

impl Camchain {
    /// Read one camera's camchain yaml.
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("cannot read camchain '{}': {e}", path.display()))?;
        Self::parse(&text, path)
    }

    /// Parse camchain text; `source` is recorded for the output and for messages.
    pub fn parse(text: &str, source: &Path) -> Result<Self, String> {
        let intr = find_list(text, "intrinsics")
            .ok_or_else(|| format!("'{}': no 'intrinsics: [...]' line", source.display()))?;
        if intr.len() != 4 {
            return Err(format!(
                "'{}': intrinsics has {} values, expected 4 (fx fy cx cy)",
                source.display(),
                intr.len()
            ));
        }
        let dist = find_list(text, "distortion_coeffs")
            .ok_or_else(|| format!("'{}': no 'distortion_coeffs: [...]' line", source.display()))?;
        if dist.len() < 4 {
            return Err(format!(
                "'{}': distortion_coeffs has {} values, expected >=4 (k1 k2 p1 p2)",
                source.display(),
                dist.len()
            ));
        }
        Ok(Self {
            fx: intr[0],
            fy: intr[1],
            cx: intr[2],
            cy: intr[3],
            dist: [dist[0], dist[1], dist[2], dist[3]],
            source: source.to_path_buf(),
        })
    }

    /// One-line summary for the startup banner.
    pub fn summary(&self) -> String {
        format!(
            "fx={:.1} fy={:.1} cx={:.1} cy={:.1}  dist=[{:.4}, {:.4}, {:.5}, {:.5}]",
            self.fx,
            self.fy,
            self.cx,
            self.cy,
            self.dist[0],
            self.dist[1],
            self.dist[2],
            self.dist[3]
        )
    }
}

/// Pull the first `key: [a, b, c]` list out of `text` and parse it as `f64`s.
/// Matches the key at the start of a (whitespace-trimmed) line, followed directly
/// by `:`, so `distortion_coeffs` and `distortion_model` do not collide. Handles
/// scientific notation.
pub fn find_list(text: &str, key: &str) -> Option<Vec<f64>> {
    for line in text.lines() {
        let t = line.trim_start();
        if let Some(rest) = t.strip_prefix(key) {
            if !rest.starts_with(':') {
                continue;
            }
            let lb = rest.find('[')?;
            let rb = rest.find(']')?;
            let mut out = Vec::new();
            for part in rest[lb + 1..rb].split(',') {
                let p = part.trim();
                if p.is_empty() {
                    continue;
                }
                out.push(p.parse().ok()?);
            }
            return Some(out);
        }
    }
    None
}

/// `path` relative to `root` when it lies under it, else `path` as is. For the
/// `# Intrinsics (from ...)` line, so the written file holds no machine path.
pub fn display_relative(path: &Path, root: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .display()
        .to_string()
}

/// Everything one `sbNN_full.yaml` holds.
pub struct FullYaml<'a> {
    /// Instance name, e.g. `sb00`.
    pub camera: &'a str,
    pub world_frame: &'a str,
    pub camchain: &'a Camchain,
    /// How the camchain path is shown in the file, e.g.
    /// `calibration/intrinsics/sb00-camchain.yaml`.
    pub camchain_label: &'a str,
    /// Camera to world: `world = R * cam + t`.
    pub r: Matrix3<f64>,
    pub t: Vector3<f64>,
    pub residual_rms_m: f64,
    pub markers_used: &'a [i32],
    pub frames: u64,
}

impl FullYaml<'_> {
    /// The file text.
    pub fn render(&self) -> String {
        let r = &self.r;
        let m = |i: usize, j: usize| r[(i, j)];
        let e = mat_to_euler_xyz_deg(r);
        let k = self.camchain;
        format!(
            "# {inst}_full.yaml — generated by ips_calib.\n\
             # Intrinsics K + distortion copied from the camchain; extrinsics R|t solved\n\
             # from the detectors' marker stream (rigid Kabsch fit, metres).\n\
             camera: {inst}\n\
             world_frame: {world_frame}\n\
             units: m\n\
             \n\
             # Intrinsics (from {src}).\n\
             K:\n\
             \x20 - [{fx:.6}, 0.0, {cx:.6}]\n\
             \x20 - [0.0, {fy:.6}, {cy:.6}]\n\
             \x20 - [0.0, 0.0, 1.0]\n\
             dist: [{d0:.8}, {d1:.8}, {d2:.8}, {d3:.8}]   # radtan k1 k2 p1 p2\n\
             \n\
             # Extrinsics: camera pose in world. world = R * cam + t (camera->world).\n\
             # Camera centre in world = t (m).\n\
             R:\n\
             \x20 - [{r00:.9}, {r01:.9}, {r02:.9}]\n\
             \x20 - [{r10:.9}, {r11:.9}, {r12:.9}]\n\
             \x20 - [{r20:.9}, {r21:.9}, {r22:.9}]\n\
             # Same rotation as XYZ-Euler degrees (R = Rx·Ry·Rz), for humans; R is authoritative.\n\
             euler_xyz_deg: [{ex:.4}, {ey:.4}, {ez:.4}]\n\
             t: [{tx:.6}, {ty:.6}, {tz:.6}]\n\
             \n\
             # Fit quality / provenance.\n\
             residual_rms_m: {rms:.6}\n\
             markers_used: {ids:?}\n\
             frames: {frames}\n",
            inst = self.camera,
            world_frame = self.world_frame,
            src = self.camchain_label,
            fx = k.fx,
            fy = k.fy,
            cx = k.cx,
            cy = k.cy,
            d0 = k.dist[0],
            d1 = k.dist[1],
            d2 = k.dist[2],
            d3 = k.dist[3],
            r00 = m(0, 0),
            r01 = m(0, 1),
            r02 = m(0, 2),
            r10 = m(1, 0),
            r11 = m(1, 1),
            r12 = m(1, 2),
            r20 = m(2, 0),
            r21 = m(2, 1),
            r22 = m(2, 2),
            ex = e[0],
            ey = e[1],
            ez = e[2],
            tx = self.t[0],
            ty = self.t[1],
            tz = self.t[2],
            rms = self.residual_rms_m,
            ids = self.markers_used,
            frames = self.frames,
        )
    }

    /// Write the file, creating its directory if needed.
    pub fn write(&self, path: &Path) -> std::io::Result<()> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, self.render())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
    }

    /// The renderer reproduces the committed sb00_full.yaml from the values in it,
    /// line by line, except the provenance comment on line 3, which now names the
    /// marker stream.
    #[test]
    fn render_matches_the_committed_format() {
        #[derive(serde::Deserialize)]
        #[allow(non_snake_case)]
        struct Full {
            camera: String,
            world_frame: String,
            R: Vec<Vec<f64>>,
            t: Vec<f64>,
            residual_rms_m: f64,
            markers_used: Vec<i32>,
            frames: u64,
        }
        // The expected file is a copy under tests/data: calibration/extrinsics/
        // is empty at handout, so the test must not depend on it.
        let root = repo();
        let text = std::fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/sb00_full.yaml"),
        )
        .unwrap();
        let f: Full = serde_yaml::from_str(&text).unwrap();
        let cc_path = camchain_path(&root.join("calibration/intrinsics"), &f.camera);
        let cc = Camchain::load(&cc_path).unwrap();
        let label = display_relative(&cc_path, &root);
        assert_eq!(label, "calibration/intrinsics/sb00-camchain.yaml");
        let r = Matrix3::from_fn(|i, j| f.R[i][j]);
        let out = FullYaml {
            camera: &f.camera,
            world_frame: &f.world_frame,
            camchain: &cc,
            camchain_label: &label,
            r,
            t: Vector3::new(f.t[0], f.t[1], f.t[2]),
            residual_rms_m: f.residual_rms_m,
            markers_used: &f.markers_used,
            frames: f.frames,
        }
        .render();
        let want: Vec<&str> = text.lines().collect();
        let got: Vec<&str> = out.lines().collect();
        assert_eq!(got.len(), want.len());
        for (i, (g, w)) in got.iter().zip(&want).enumerate() {
            if i != 2 {
                assert_eq!(g, w, "line {}", i + 1);
            }
        }
    }
}
