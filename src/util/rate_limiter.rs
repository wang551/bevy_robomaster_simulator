use std::time::Duration;

/// Averaging rate limiter shared by the ROS2 and Talos diagnostics so their
/// echo logs tick at the same cadence.
#[derive(Debug, Clone)]
pub struct AverageRateLimiter {
    period: Duration,
    elapsed: Duration,
}

impl AverageRateLimiter {
    pub fn new(period: Duration) -> Self {
        Self {
            period,
            elapsed: period,
        }
    }

    pub fn from_hz(hz: f32) -> Self {
        assert!(hz.is_finite() && hz > 0.0);
        Self::new(Duration::from_secs_f32(1.0 / hz))
    }

    pub fn tick(&mut self, delta: Duration) {
        self.elapsed = self.elapsed.saturating_add(delta).min(self.period);
    }

    pub fn allow(&mut self) -> bool {
        if self.elapsed < self.period {
            return false;
        }
        self.elapsed = Duration::ZERO;
        true
    }
}
