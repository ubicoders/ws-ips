//! Parameters file and `--id` camera selection. No camera is needed.

use ips_cam_gcam_pub::capture::{self, CamParams};

#[test]
fn shipped_parameters_file_resolves_id() {
    let path = capture::params_path();
    let p = CamParams::load(&path).unwrap_or_else(|e| panic!("{e}"));
    let id = capture::parse_args(["--id", "sb00"]).unwrap();
    assert_eq!(id.as_deref(), Some("sb00"));
    let selected = capture::select_cameras(&p.cameras, id.as_deref()).unwrap();
    assert_eq!(selected, vec!["sb00".to_string()]);
}

#[test]
fn unknown_id_is_an_error() {
    let path = capture::params_path();
    let p = CamParams::load(&path).unwrap_or_else(|e| panic!("{e}"));
    let err = capture::select_cameras(&p.cameras, Some("sb07")).unwrap_err();
    assert!(err.contains("sb07"), "{err}");
}
