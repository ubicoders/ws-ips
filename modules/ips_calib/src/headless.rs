//! The headless one-shot (`ui: false`): accumulate until every camera has its
//! frames or the timeout passes, fit, print every camera's result with the
//! residual of the file on disk beside it, ask `[y/N]` on the terminal, write,
//! exit. A script answers by piping `y` into stdin.

use crate::accumulate::Accumulator;
use crate::calibrate::{self, CamFit};
use crate::setup::{MarkerStream, Setup};
use crate::writer::{self, Camchain};

use std::io::{BufRead, Write};
use std::time::{Duration, Instant};

/// What the run produced.
pub struct Outcome {
    /// Cameras with a successful fit.
    pub fitted: usize,
    /// `sbNN_full.yaml` files written.
    pub written: usize,
}

/// The whole run.
pub fn run(setup: &Setup) -> Result<Outcome, String> {
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
    let mut fits: Vec<(&str, &Camchain, CamFit)> = Vec::new();
    for (inst, cc) in &setup.cams {
        match calibrate::fit_camera(setup, &acc, inst) {
            Ok(f) => {
                println!("{}", calibrate::fit_line(inst, &f, calibrate::existing_rms(setup, inst)));
                fits.push((inst, cc, f));
            }
            Err(why) => println!("  {inst}: {why}, skipped"),
        }
    }
    let fitted = fits.len();
    if fitted == 0 {
        return Ok(Outcome { fitted, written: 0 });
    }

    let dir = writer::display_relative(&setup.paths.extrinsics, &setup.paths.repo);
    if !ask_write(fitted, &dir)? {
        println!("cancelled, nothing written");
        return Ok(Outcome { fitted, written: 0 });
    }

    let mut written = 0;
    for (inst, cc, f) in &fits {
        match calibrate::write_camera(setup, inst, cc, f) {
            Ok(out) => {
                written += 1;
                println!("{}", calibrate::result_line(inst, f, &out));
            }
            Err(e) => println!("  {inst}: {e}, skipped"),
        }
    }
    Ok(Outcome { fitted, written })
}

/// `[y/N]` on the terminal. Anything but `y` or `yes`, including an empty line
/// or a closed stdin, is no.
fn ask_write(n: usize, dir: &str) -> Result<bool, String> {
    print!("\nwrite {n} file(s) to {dir}/ ? [y/N] ");
    std::io::stdout().flush().map_err(|e| format!("stdout: {e}"))?;
    let mut line = String::new();
    std::io::stdin()
        .lock()
        .read_line(&mut line)
        .map_err(|e| format!("stdin: {e}"))?;
    let answer = line.trim().to_ascii_lowercase();
    Ok(answer == "y" || answer == "yes")
}
