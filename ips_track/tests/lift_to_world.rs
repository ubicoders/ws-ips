//! `lift_to_world` with a hand-built extrinsic, checked against values worked
//! out by hand.

use ips_track::cube::{lift_to_world, CameraExtrinsics};
use nalgebra::{Matrix3, Vector3};

#[test]
fn lifts_a_marker_with_a_known_extrinsic() {
    // Camera yawed +90 deg about world z, centre at (1, 2, 3):
    // world = R * cam + t, so cam +x maps to world +y and cam +y to world -x.
    #[rustfmt::skip]
    let ext = CameraExtrinsics {
        r: Matrix3::new(0.0, -1.0, 0.0,
                        1.0,  0.0, 0.0,
                        0.0,  0.0, 1.0),
        t: Vector3::new(1.0, 2.0, 3.0),
    };

    // A marker 1.5 m in front of the camera, offset (0.1, 0.2), rotated -90 deg
    // about the camera z axis.
    let t_cam_mk = Vector3::new(0.1, 0.2, 1.5);
    #[rustfmt::skip]
    let r_cam_mk = Matrix3::new( 0.0, 1.0, 0.0,
                                -1.0, 0.0, 0.0,
                                 0.0, 0.0, 1.0);

    let (r_world_mk, t_world_mk) = lift_to_world(&ext, &r_cam_mk, &t_cam_mk);

    // R * (0.1, 0.2, 1.5) = (-0.2, 0.1, 1.5); plus t = (0.8, 2.1, 4.5).
    assert!(
        (t_world_mk - Vector3::new(0.8, 2.1, 4.5)).norm() < 1e-12,
        "t: {t_world_mk:?}"
    );
    // +90 deg then -90 deg about the same axis: the marker is world-aligned.
    assert!(
        (r_world_mk - Matrix3::identity()).norm() < 1e-12,
        "R: {r_world_mk:?}"
    );
}
