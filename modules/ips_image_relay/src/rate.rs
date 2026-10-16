//! rate: a frame counter for the periodic Hz line.
//!
//! Each stream counts the frames it relays; every report period the camera
//! thread reads the rate of each counter over the time since the previous
//! report and resets it. A stream that relayed nothing reads 0.0 Hz, which is
//! the normal state of `debug_image` while ips_detect runs with `debug: false`.

use std::time::{Duration, Instant};

/// How often a camera thread prints its `image .. Hz, debug_image .. Hz` line.
pub const REPORT_EVERY: Duration = Duration::from_secs(5);

/// Counts events since the last [`RateCounter::take_hz`].
pub struct RateCounter {
    count: u64,
    since: Instant,
}

impl RateCounter {
    /// A counter whose first window starts at `now`.
    pub fn new(now: Instant) -> Self {
        Self {
            count: 0,
            since: now,
        }
    }

    /// Record one event.
    pub fn tick(&mut self) {
        self.count += 1;
    }

    /// Events per second since the window began, then start a new window at `now`.
    pub fn take_hz(&mut self, now: Instant) -> f64 {
        let span = now.duration_since(self.since).as_secs_f64();
        let hz = if span > 0.0 {
            self.count as f64 / span
        } else {
            0.0
        };
        self.count = 0;
        self.since = now;
        hz
    }
}
