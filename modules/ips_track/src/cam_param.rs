//! cam_param: the camera-parameter routes on the sb service router
//! (`swarmbotix_io/service.rs`, included as [`crate::service`]).
//!
//! Serves each camera's intrinsics `K` and extrinsics `R`, `t` from the
//! `calibration/extrinsics/sbNN_full.yaml` files, hosted inside the tracker so
//! the API is live whenever ips_track runs. Routes, under
//! `<device>/ws_ips/ips_track/zenoh/service`:
//!
//! - `/`: `{ "module", "routes", "cameras" }`.
//! - `/cam_params`: the camera ids that have a `*_full.yaml`, sorted.
//! - `/cam_param/sbNN`: the whole `sbNN_full.yaml` as JSON, plus `"Rt"`, the
//!   3x4 matrix `[R | t]`. ips_viz parses this shape.
//!
//! Stateless on purpose: each request re-reads the yaml, and the routes glob
//! every `*_full.yaml` in the directory, not only the cameras being tracked.
//! [`crate::service::ServiceApp::start`] is non-blocking (handlers run on zenoh's
//! threads) and opens its own zenoh session.

use std::path::{Path, PathBuf};

use serde_json::{json, Value};

use crate::service;

/// Camera ids (`sbNN`) that have a `*_full.yaml` in `dir`, sorted.
pub fn svc_list_cameras(dir: &Path) -> Vec<String> {
    let mut ids = Vec::new();
    if let Ok(rd) = std::fs::read_dir(dir) {
        for e in rd.flatten() {
            let name = e.file_name();
            let name = name.to_string_lossy();
            if let Some(stem) = name.strip_suffix("_full.yaml") {
                if svc_is_valid_id(stem) {
                    ids.push(stem.to_string());
                }
            }
        }
    }
    ids.sort();
    ids
}

/// Accept only safe filename characters, so a wildcard path segment cannot
/// escape `dir`.
pub fn svc_is_valid_id(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 32
        && id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

/// One camera's params file as JSON, with the 3x4 `Rt = [R | t]` added. Returns
/// an `{ "error": ... }` object on any failure (a handler cannot reply_err).
pub fn svc_load_cam(dir: &Path, id: &str) -> Value {
    if !svc_is_valid_id(id) {
        return json!({ "error": format!("invalid camera id '{id}'") });
    }
    let path = dir.join(format!("{id}_full.yaml"));
    let text = match std::fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) => {
            return json!({
                "error": format!("no params for '{id}': {e}"),
                "available": svc_list_cameras(dir),
            })
        }
    };
    let mut doc: Value = match serde_yaml::from_str(&text) {
        Ok(v) => v,
        Err(e) => return json!({ "error": format!("parse '{id}': {e}") }),
    };
    let rt = svc_augment_rt(&doc); // read R and t while doc is borrowed immutably
    if let (Value::Object(map), Some(rt)) = (&mut doc, rt) {
        map.insert("Rt".to_string(), rt);
    }
    doc
}

/// The 3x4 `[R | t]` from `R` (3x3) and `t` (3), if both are well formed.
pub fn svc_augment_rt(doc: &Value) -> Option<Value> {
    let r = doc.get("R")?.as_array()?;
    let t = doc.get("t")?.as_array()?;
    if r.len() != 3 || t.len() != 3 {
        return None;
    }
    let mut rows = Vec::with_capacity(3);
    for (i, row) in r.iter().enumerate() {
        let row = row.as_array()?;
        if row.len() != 3 {
            return None;
        }
        let mut out = row.clone();
        out.push(t[i].clone());
        rows.push(Value::Array(out));
    }
    Some(Value::Array(rows))
}

/// Start the camera-param service on `dir` and return the running app. The
/// caller keeps it alive (dropping it undeclares the routes). Warns and returns
/// `None` on failure: the tracker runs regardless of the service.
pub fn start_cam_param_service(dir: PathBuf) -> Option<service::ServiceApp> {
    let mut app = service::ServiceApp::open();

    // Root: advertise the routes and the cameras currently available.
    let root_dir = dir.clone();
    app.get("/", move |_req: service::Req| {
        json!({
            "module": service::MODULE,
            "routes": ["/", "/cam_params", "/cam_param/<id>"],
            "cameras": svc_list_cameras(&root_dir),
        })
    });

    // The available camera ids.
    let list_dir = dir.clone();
    app.get("/cam_params", move |_req: service::Req| {
        json!(svc_list_cameras(&list_dir))
    });

    // One camera's K and R|t. The id is the last segment of the matched key.
    let cam_dir = dir.clone();
    app.get("/cam_param/*", move |req: service::Req| {
        let id = req.key.rsplit('/').next().unwrap_or("");
        svc_load_cam(&cam_dir, id)
    });

    match app.start() {
        Ok(()) => {
            println!(
                "cam_param service: live on {}/**  cameras [{}]",
                service::base(),
                svc_list_cameras(&dir).join(", ")
            );
            Some(app)
        }
        Err(e) => {
            eprintln!("warning: cam_param service failed to start ({e}); continuing without it");
            None
        }
    }
}
