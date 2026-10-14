//! The headless one-shot (`ui: false`): accumulate until every camera has its
//! frames or the timeout passes, fit, write, exit.

use crate::accumulate::Accumulator;
use crate::calibrate;
use crate::setup::{MarkerStream, Setup};

use std::time::{Duration, Instant};

/// The whole run. Returns how many `sbNN_full.yaml` were written.
pub fn run(setup: &Setup) -> Result<usize, String> {
    let params = &setup.params;
    let mut acc = Accumulator::new(setup.instances(), params.frames, &setup.cfg.board.family);
    let mut stream = MarkerStream::open()?;

    let start = Instant::now();
    let deadline = start + Duration::from_secs_f64(params.timeout_s);
    let mut last_report = Instant::now();
    while Instant::now() < deadline && !acc.done() {
        let any = stream.drain(|msg| {
            if let Some(notice) = acc.ingest(msg) {
                println!("{notice}");
            }
            !acc.done()
        })?;
        if !any {
            std::thread::sleep(Duration::from_millis(1));
        }
        if last_report.elapsed() >= Duration::from_secs(1) {
            println!("  accumulating  {}", acc.progress());
            last_report = Instant::now();
        }
    }
    if acc.done() {
        println!(
            "  every camera reached {} frames after {:.1} s",
            params.frames,
            start.elapsed().as_secs_f64()
        );
    } else {
        println!("  timeout after {} s  {}", params.timeout_s, acc.progress());
    }
    if stream.undecodable() > 0 {
        println!("  {} undecodable sample(s) ignored", stream.undecodable());
    }
    drop(stream);

    println!("\n=== results ===");
    let mut written = 0;
    for (inst, cc) in &setup.cams {
        let f = match calibrate::fit_camera(setup, &acc, inst) {
            Ok(f) => f,
            Err(why) => {
                println!("  {inst}: {why}, skipped");
                continue;
            }
        };
        match calibrate::write_camera(setup, inst, cc, &f) {
            Ok(out) => {
                written += 1;
                println!("{}", calibrate::result_line(inst, &f, &out));
            }
            Err(e) => println!("  {inst}: {e}, skipped"),
        }
    }
    Ok(written)
}
