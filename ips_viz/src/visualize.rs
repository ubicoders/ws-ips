//! visualize: the parameters, the off-wire geometry and the rerun.io view.
//!
//! The `cube_poses` message carries each cube's pose (centre and orientation)
//! but not the box dimensions, and no camera information. The viewer supplies
//! both: [`CubeModel`] holds the box half-extents from `viz_params.yaml`, and
//! [`CameraExtrinsics`] is one camera's world pose, fetched at startup from
//! ips_track's cam_param service. [`VizParams`] is `parameters/viz_params.yaml`.
//!
//! [`Visualizer`] streams to an already running rerun viewer over gRPC at its
//! default endpoint (`rerun+http://127.0.0.1:9876/proxy`, what `rerun` listens
//! on when launched bare). The binary only has to [`Visualizer::connect`], call
//! [`log_cameras`](Visualizer::log_cameras) once, and
//! [`log_cubes`](Visualizer::log_cubes) per received `CubePoses` message.
//!
//! Entity layout (everything in the world frame, metres; Z vertical, +Z down):
//! ```text
//!   world                     ViewCoordinates::RFD (Z up/down, grid = floor) (static)
//!   world/cameras/<inst>      a labelled dot at the camera centre        (static)
//!   world/cameras/<inst>/axes the camera's R,G,B body axes (= R columns) (static)
//!   world/cubes/<id>          the fused cube as an oriented box + label  (per set)
//!   debug/<inst>              the camera's latest debug color image      (per frame)
//!   plots/position/{x,y,z}/cube_<id>  cube centre coordinate, metres     (per set)
//!   plots/velocity/{x,y,z}/cube_<id>  cube linear velocity, m/s          (per set)
//!   plots/angle/{x,y,z}/cube_<id>     cube XYZ Euler angle, degrees      (per set)
//! ```
//!
//! Camera poses are fixed by the extrinsics, so they are logged once with
//! `log_static`; cube poses change every set and are logged on the `frame`
//! timeline. The `debug/` and `plots/` subtrees are optional: the `debug_show_*`
//! flags in `viz_params.yaml` decide which views are built and so which data is
//! logged; see [`Visualizer::connect`].

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use nalgebra::{Matrix3, Vector3};
use serde_json::Value;

use crate::msgs::CubePose;

type Res<T> = Result<T, Box<dyn std::error::Error>>;

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

fn d_cameras() -> Vec<u32> {
    vec![0, 1, 2, 3]
}

fn d_rerun_hz() -> f64 {
    20.0
}

fn d_cube_viz_scale() -> f64 {
    1.0
}

fn d_show_images() -> bool {
    true
}

/// `parameters/viz_params.yaml`: the cube box geometry (sizes the drawn boxes),
/// which cameras to draw, the rate at which the debug images are forwarded to
/// rerun, and which panels the layout shows. The geometry keys mirror
/// ips_track's so the view matches the tracker's cube model.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct VizParams {
    pub cube_width: f64,
    pub cube_height: f64,
    pub cube_length: f64,
    /// Camera indices; index `i` is camera `sb{i:02}`.
    #[serde(default = "d_cameras")]
    pub cameras: Vec<u32>,
    /// Hz at which the debug color images are drained to the latest frame and
    /// logged to rerun. Cube poses stay event driven. Defaults to 20.
    #[serde(default = "d_rerun_hz")]
    pub rerun_hz: f64,
    /// Rerun-only display scale for the drawn cube box (about its centre). The
    /// tracked pose is unchanged. Defaults to 1.
    #[serde(default = "d_cube_viz_scale")]
    pub cube_viz_scale: f64,
    /// Show the 2x2 grid of per-camera debug images. When false the image
    /// subscriptions are skipped entirely. Defaults to true.
    #[serde(default = "d_show_images")]
    pub debug_show_images: bool,
    /// Show a 3x1 real-time plot of each cube's centre X/Y/Z (metres).
    #[serde(default)]
    pub debug_show_pos_xyz: bool,
    /// Show a 3x1 real-time plot of each cube's linear velocity X/Y/Z (m/s).
    /// Only the `kf` tracker filter fills velocity; otherwise nothing is plotted,
    /// and a non-finite component drops that sample.
    #[serde(default)]
    pub debug_show_vel_xyz: bool,
    /// Show a 3x1 real-time plot of each cube's XYZ Euler angle (degrees).
    #[serde(default)]
    pub debug_show_ang_xyz: bool,
}

impl VizParams {
    /// `CARGO_MANIFEST_DIR/parameters/viz_params.yaml`.
    pub fn default_path() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("parameters")
            .join("viz_params.yaml")
    }

    /// Parse the yaml text.
    pub fn from_yaml(text: &str) -> Result<Self, String> {
        serde_yaml::from_str(text).map_err(|e| e.to_string())
    }

    /// Read and parse the yaml file at `path`.
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| format!("cannot read '{}': {e}", path.display()))?;
        Self::from_yaml(&text).map_err(|e| format!("parse '{}': {e}", path.display()))
    }

    /// The camera instance names, `sbNN`, in yaml order.
    pub fn camera_names(&self) -> Vec<String> {
        self.cameras.iter().map(|&i| camera_name(i)).collect()
    }
}

/// `sbNN` instance name for a 0-based camera index (`0` is `sb00`).
pub fn camera_name(index: u32) -> String {
    format!("sb{index:02}")
}

// ---------------------------------------------------------------------------
// Off-wire geometry
// ---------------------------------------------------------------------------

/// Half-extents of the cube along the body axes (X toward the RIGHT face, Y
/// toward the BACK face, Z toward the BOTTOM mount). For a true cube all three
/// are equal (edge / 2). These set the half-sizes of the box rerun draws.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CubeModel {
    pub hx: f64,
    pub hy: f64,
    pub hz: f64,
}

impl CubeModel {
    /// Build from the box dimensions in metres: width along X (left to right),
    /// length along Y (front to back), height along Z (top to bottom). Same
    /// argument order as ips_track's cube model (width, length, height).
    pub fn new(width: f64, length: f64, height: f64) -> Self {
        Self {
            hx: width / 2.0,
            hy: length / 2.0,
            hz: height / 2.0,
        }
    }
}

/// One camera's world pose, `world = R * cam + t` (camera centre at `t`), from
/// the cam_param service, which serves `sbNN_full.yaml` as JSON.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraExtrinsics {
    pub r: Matrix3<f64>,
    pub t: Vector3<f64>,
}

impl CameraExtrinsics {
    /// Parse `R` (3x3, row major) and `t` (3) out of the cam_param JSON reply
    /// (`{ "R": [[..]], "t": [..], .. }`, the same field shape as the
    /// `sbNN_full.yaml` it serves). Other fields (K, dist, Rt, euler, provenance)
    /// are ignored; `src` labels the camera in error messages.
    pub fn from_json(doc: &Value, src: &str) -> Res<Self> {
        // An unknown camera is answered with a normal `{ "error": .. }` object
        // (the service handler cannot reply_err), not a transport error.
        if let Some(err) = doc.get("error").and_then(|e| e.as_str()) {
            return Err(format!("{src}: service: {err}").into());
        }
        #[derive(serde::Deserialize)]
        struct Extr {
            #[serde(rename = "R")]
            r: Vec<Vec<f64>>,
            t: Vec<f64>,
        }
        let y: Extr = serde_json::from_value(doc.clone())
            .map_err(|e| format!("{src}: missing or invalid R|t in reply: {e}"))?;
        if y.r.len() != 3 || y.r.iter().any(|row| row.len() != 3) {
            return Err(format!("{src}: R must be 3x3").into());
        }
        if y.t.len() != 3 {
            return Err(format!("{src}: t must have 3 values").into());
        }
        #[rustfmt::skip]
        let r = Matrix3::new(
            y.r[0][0], y.r[0][1], y.r[0][2],
            y.r[1][0], y.r[1][1], y.r[1][2],
            y.r[2][0], y.r[2][1], y.r[2][2],
        );
        let t = Vector3::new(y.t[0], y.t[1], y.t[2]);
        Ok(Self { r, t })
    }
}

// ---------------------------------------------------------------------------
// rerun view
// ---------------------------------------------------------------------------

/// The rerun viewer's default gRPC endpoint, the address `rerun` prints when
/// launched bare.
pub const DEFAULT_ENDPOINT: &str = "rerun+http://127.0.0.1:9876/proxy";

/// Length of the drawn camera body axes, metres.
const CAM_AXIS_LEN: f64 = 0.1;

/// Resolution of the debug color frames (`sensor_msgs/Image1280x1024Color`, RGB8).
const DEBUG_IMG_W: u32 = 1280;
const DEBUG_IMG_H: u32 = 1024;

/// A live connection to a rerun viewer. Construct with [`Visualizer::connect`].
pub struct Visualizer {
    rec: rerun::RecordingStream,
    /// Rerun-only display scale applied to the drawn cube box (about its centre).
    cube_viz_scale: f64,
    /// Whether the per-cube position plots are part of the layout.
    show_pos_xyz: bool,
    /// Whether the per-cube velocity plots are part of the layout. Velocity is
    /// only present when the tracker runs the `kf` filter.
    show_vel_xyz: bool,
    /// Whether the per-cube Euler angle plots are part of the layout.
    show_ang_xyz: bool,
    /// Cube ids whose plot lines already had their static `SeriesLines` style
    /// (colour and legend name) logged, so it is sent once and not every frame.
    styled_plot_cubes: HashSet<u32>,
}

impl Visualizer {
    /// Connect to a running rerun viewer over gRPC and set the world frame.
    /// Streaming is best effort: with no viewer up yet, rerun buffers and
    /// reconnects rather than failing.
    ///
    /// The layout is assembled left to right from the enabled debug panels,
    /// with the 3D `world` view always present and rightmost. Each flag adds
    /// one column:
    ///   * `show_images`: a 2x2 grid of 2D image views (origins `/debug/<inst>`),
    ///     one per entry of `cameras` (the `sbNN` instance names);
    ///   * `show_pos_xyz`: a 3x1 stack of the cube centre X/Y/Z (metres);
    ///   * `show_vel_xyz`: a 3x1 stack of the cube linear velocity X/Y/Z (m/s);
    ///   * `show_ang_xyz`: a 3x1 stack of the cube XYZ Euler angles (degrees).
    ///
    /// The world gets a column share equal to the number of debug columns (min
    /// 1), so it stays about half the width whatever is enabled, and full width
    /// when nothing else is shown. `cube_viz_scale` multiplies the drawn box
    /// size only (see [`log_cubes`](Self::log_cubes)).
    pub fn connect(
        cameras: &[String],
        cube_viz_scale: f64,
        show_images: bool,
        show_pos_xyz: bool,
        show_vel_xyz: bool,
        show_ang_xyz: bool,
    ) -> Res<Self> {
        use rerun::blueprint::{
            ContainerLike, Grid, Spatial2DView, Spatial3DView, TimeSeriesView, Vertical,
        };

        // Ship a fixed layout so the viewer does not auto-generate its own. Each
        // enabled panel is one column; `shares` holds each column's relative width.
        let mut columns: Vec<ContainerLike> = Vec::new();
        let mut shares: Vec<f32> = Vec::new();

        // 2x2 image grid. One 2D view per camera up front keeps the grid stable;
        // a camera that never publishes just shows an empty panel.
        if show_images {
            let grid_views: Vec<ContainerLike> = cameras
                .iter()
                .map(|inst| {
                    Spatial2DView::new(format!("{inst} debug"))
                        .with_origin(format!("/debug/{inst}"))
                        .into()
                })
                .collect();
            columns.push(Grid::new(grid_views).with_grid_columns(2).into());
            shares.push(1.0);
        }

        // 3x1 stack of the cube centre coordinates; one line per cube under each
        // shared origin (see `log_cubes`).
        if show_pos_xyz {
            let col = Vertical::new([
                TimeSeriesView::new("position X (m)")
                    .with_origin("/plots/position/x")
                    .into(),
                TimeSeriesView::new("position Y (m)")
                    .with_origin("/plots/position/y")
                    .into(),
                TimeSeriesView::new("position Z (m)")
                    .with_origin("/plots/position/z")
                    .into(),
            ]);
            columns.push(col.into());
            shares.push(1.0);
        }

        // 3x1 stack of the cube linear velocity. Only the tracker's `kf` filter
        // fills velocity, so with any other smoother these views stay empty.
        if show_vel_xyz {
            let col = Vertical::new([
                TimeSeriesView::new("velocity X (m/s)")
                    .with_origin("/plots/velocity/x")
                    .into(),
                TimeSeriesView::new("velocity Y (m/s)")
                    .with_origin("/plots/velocity/y")
                    .into(),
                TimeSeriesView::new("velocity Z (m/s)")
                    .with_origin("/plots/velocity/z")
                    .into(),
            ]);
            columns.push(col.into());
            shares.push(1.0);
        }

        // 3x1 stack of the cube orientation as XYZ Euler degrees.
        if show_ang_xyz {
            let col = Vertical::new([
                TimeSeriesView::new("angle X (deg)")
                    .with_origin("/plots/angle/x")
                    .into(),
                TimeSeriesView::new("angle Y (deg)")
                    .with_origin("/plots/angle/y")
                    .into(),
                TimeSeriesView::new("angle Z (deg)")
                    .with_origin("/plots/angle/z")
                    .into(),
            ]);
            columns.push(col.into());
            shares.push(1.0);
        }

        // The 3D world view is always present and rightmost; its share equals the
        // debug column count (min 1), which keeps it at about half the width.
        let world_share = shares.len().max(1) as f32;
        columns.push(Spatial3DView::new("world").with_origin("/world").into());
        shares.push(world_share);

        let blueprint = rerun::blueprint::Blueprint::new(
            rerun::blueprint::Horizontal::new(columns).with_column_shares(shares),
        )
        .with_auto_views(false)
        .with_auto_layout(false);

        let rec = rerun::RecordingStreamBuilder::new("ips_viz")
            .with_blueprint(blueprint)
            .connect_grpc_opts(DEFAULT_ENDPOINT)?;
        // The cameras are ceiling mounted and coplanar in world Z (about -2 m), so
        // Z is the vertical axis (+Z down toward the floor where the cubes sit).
        // RFD (X right, Y forward, Z down) makes rerun treat Z as up/down: the
        // ground grid becomes the X-Y plane (the floor) and the ceiling camera
        // plane lies parallel to it, instead of standing upright as RDF (Y up)
        // renders it.
        rec.log_static("world", &rerun::ViewCoordinates::RFD())?;
        eprintln!("visualize: streaming to rerun at {DEFAULT_ENDPOINT}");
        Ok(Self {
            rec,
            cube_viz_scale,
            show_pos_xyz,
            show_vel_xyz,
            show_ang_xyz,
            styled_plot_cubes: HashSet::new(),
        })
    }

    /// Log every camera's world pose once (static): a labelled dot at the centre
    /// plus its R/G/B body axes. Column `i` of the extrinsic `R` is the camera's
    /// `i`-th axis expressed in world (`world = R * cam + t`), so the arrows point
    /// along the camera's X (red), Y (green) and Z, the optical axis (blue).
    pub fn log_cameras(&self, exts: &HashMap<String, CameraExtrinsics>) -> Res<()> {
        let mut names: Vec<&String> = exts.keys().collect();
        names.sort();
        for name in names {
            let e = &exts[name];
            let origin = [e.t[0] as f32, e.t[1] as f32, e.t[2] as f32];

            // Centre dot carrying the camera label.
            self.rec.log_static(
                format!("world/cameras/{name}"),
                &rerun::Points3D::new([origin])
                    .with_radii([0.02_f32])
                    .with_colors([rerun::Color::from_rgb(255, 255, 0)])
                    .with_labels([name.as_str()]),
            )?;

            // Body axes: vector = column i of R, scaled; all share the centre origin.
            let axis = |c: usize| {
                [
                    (e.r[(0, c)] * CAM_AXIS_LEN) as f32,
                    (e.r[(1, c)] * CAM_AXIS_LEN) as f32,
                    (e.r[(2, c)] * CAM_AXIS_LEN) as f32,
                ]
            };
            self.rec.log_static(
                format!("world/cameras/{name}/axes"),
                &rerun::Arrows3D::from_vectors([axis(0), axis(1), axis(2)])
                    .with_origins([origin, origin, origin])
                    .with_colors([
                        rerun::Color::from_rgb(255, 0, 0),
                        rerun::Color::from_rgb(0, 255, 0),
                        rerun::Color::from_rgb(0, 0, 255),
                    ]),
            )?;
        }
        Ok(())
    }

    /// Log the fused cube poses of one received `CubePoses` set on the `frame`
    /// timeline. Each cube is an oriented box (half-extents from `model`, scaled
    /// by `cube_viz_scale` for display only, orientation from the fused
    /// quaternion) carrying its id. A recursive clear first drops any box from a
    /// previous set that is not present now, so the view never lingers on a stale
    /// pose. `stamp_secs` is the message stamp (epoch seconds). Cubes with no pose
    /// (`alive=false` heartbeats, whose pose fields are cleared) are skipped.
    ///
    /// With the plot panels enabled, each cube's centre, linear velocity and XYZ
    /// Euler angles are also logged as `Scalars` to
    /// `plots/{position,velocity,angle}/{x,y,z}/cube_<id>`, one line per cube.
    /// Velocity is logged only when the wire message carries it (the `kf` filter)
    /// and every component is finite.
    pub fn log_cubes(
        &mut self,
        seq: u64,
        stamp_secs: f64,
        cubes: &[CubePose],
        model: &CubeModel,
    ) -> Res<()> {
        self.rec.set_time_sequence("frame", seq as i64);
        self.rec.set_timestamp_secs_since_epoch("time", stamp_secs);

        // Drop boxes for cubes that vanished since the last set (re-logged below
        // if still present, at the same timeline tick).
        self.rec.log("world/cubes", &rerun::Clear::recursive())?;

        // Display-only scale about the box centre; the tracked pose is untouched.
        let s = self.cube_viz_scale as f32;
        let half = [
            model.hx as f32 * s,
            model.hy as f32 * s,
            model.hz as f32 * s,
        ];
        for c in cubes {
            let (Some(p), Some(q)) = (c.position.as_ref(), c.orientation.as_ref()) else {
                continue;
            };
            self.rec.log(
                format!("world/cubes/{}", c.cube_id),
                &rerun::Boxes3D::from_centers_and_half_sizes(
                    [[p.x as f32, p.y as f32, p.z as f32]],
                    [half],
                )
                .with_quaternions([rerun::Quaternion::from_xyzw([
                    q.x as f32, q.y as f32, q.z as f32, q.w as f32,
                ])])
                .with_colors([cube_color(c.cube_id)])
                .with_labels([format!("cube {}", c.cube_id)]),
            )?;

            // Per-cube real-time plots, one scalar series per coordinate under the
            // shared plot origins. Logged only for enabled panels; the per-line
            // style (colour and legend name) is logged once.
            if self.show_pos_xyz || self.show_vel_xyz || self.show_ang_xyz {
                let id = c.cube_id;
                self.style_cube_plots(id)?;
                if self.show_pos_xyz {
                    self.log_xyz("position", id, p.x, p.y, p.z)?;
                }
                // Velocity is optional on the wire (only the `kf` filter sets it)
                // and may be NaN; log it only when present and finite.
                if self.show_vel_xyz {
                    if let Some(v) = c.velocity.as_ref() {
                        if v.x.is_finite() && v.y.is_finite() && v.z.is_finite() {
                            self.log_xyz("velocity", id, v.x, v.y, v.z)?;
                        }
                    }
                }
                if self.show_ang_xyz {
                    if let Some(a) = c.euler_deg.as_ref() {
                        self.log_xyz("angle", id, a.x, a.y, a.z)?;
                    }
                }
            }
        }
        Ok(())
    }

    /// Log one X/Y/Z triple to `plots/<group>/{x,y,z}/cube_<id>`.
    fn log_xyz(&self, group: &str, id: u32, x: f64, y: f64, z: f64) -> Res<()> {
        for (coord, value) in [("x", x), ("y", y), ("z", z)] {
            self.rec.log(
                format!("plots/{group}/{coord}/cube_{id}"),
                &rerun::Scalars::single(value),
            )?;
        }
        Ok(())
    }

    /// Log the static `SeriesLines` style (the cube's colour, matching its box,
    /// and a `cube <id>` legend name) for cube `id`'s plot lines, once. Only the
    /// series of enabled panels are styled.
    fn style_cube_plots(&mut self, id: u32) -> Res<()> {
        if !self.styled_plot_cubes.insert(id) {
            return Ok(());
        }
        let color = cube_color(id);
        let name = format!("cube {id}");
        let style = || {
            rerun::SeriesLines::new()
                .with_colors([color])
                .with_names([name.clone()])
        };
        for coord in ["x", "y", "z"] {
            if self.show_pos_xyz {
                self.rec
                    .log_static(format!("plots/position/{coord}/cube_{id}"), &style())?;
            }
            if self.show_vel_xyz {
                self.rec
                    .log_static(format!("plots/velocity/{coord}/cube_{id}"), &style())?;
            }
            if self.show_ang_xyz {
                self.rec
                    .log_static(format!("plots/angle/{coord}/cube_{id}"), &style())?;
            }
        }
        Ok(())
    }

    /// Log one camera's latest debug frame at `debug/<inst>` (RGB8, 1280x1024) on
    /// the `time` timeline; rerun routes it to that camera's panel in the 2x2
    /// grid. `rgb` must hold 1280*1024*3 bytes (a short buffer fails rerun's
    /// resolution check and is skipped). Only called for cameras that publish,
    /// so a silent camera's panel stays empty.
    pub fn log_debug_image(&self, inst: &str, stamp_secs: f64, rgb: Vec<u8>) -> Res<()> {
        self.rec.set_timestamp_secs_since_epoch("time", stamp_secs);
        self.rec.log(
            format!("debug/{inst}"),
            &rerun::Image::from_rgb24(rgb, [DEBUG_IMG_W, DEBUG_IMG_H]),
        )?;
        Ok(())
    }
}

/// A stable per-cube colour so each cube keeps its hue across sets.
fn cube_color(id: u32) -> rerun::Color {
    const PALETTE: [(u8, u8, u8); 6] = [
        (230, 80, 80),
        (80, 200, 120),
        (90, 150, 240),
        (240, 190, 70),
        (200, 110, 230),
        (90, 220, 220),
    ];
    let (r, g, b) = PALETTE[(id as usize) % PALETTE.len()];
    rerun::Color::from_rgb(r, g, b)
}
