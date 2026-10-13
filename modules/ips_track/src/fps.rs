//! fps: sliding-window rate meter.
//!
//! Reports the *actual* arrival rate by timing frames as they land, rather than
//! trusting any nominal rate. Keeps the arrival timestamps from roughly the last
//! `window` and divides the frame count by the span they cover, which is far steadier
//! than the reciprocal of a single inter-frame gap.
//!
//! Feed exactly one [`FpsMeter::tick`] per received frame; it returns the
//! current measured Hz (or `None` until two frames share the window).

use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// Sliding-window rate meter (ips_track feeds it one tick per publish).
pub struct FpsMeter {
    window: Duration,
    ticks: VecDeque<Instant>,
}

impl FpsMeter {
    /// New meter averaging over `window` (e.g. `Duration::from_secs(1)`).
    pub fn new(window: Duration) -> Self {
        Self {
            window,
            ticks: VecDeque::new(),
        }
    }

    /// Record a frame that arrived at `now` and return the measured FPS over the
    /// trailing `window`, or `None` until at least two frames fall inside it.
    pub fn tick(&mut self, now: Instant) -> Option<f64> {
        self.ticks.push_back(now);
        while let Some(&oldest) = self.ticks.front() {
            if now.duration_since(oldest) > self.window {
                self.ticks.pop_front();
            } else {
                break;
            }
        }

        let oldest = *self.ticks.front()?;
        let span = now.duration_since(oldest).as_secs_f64();
        if self.ticks.len() < 2 || span <= 0.0 {
            return None;
        }
        Some((self.ticks.len() - 1) as f64 / span)
    }
}
