"""Logic for "is the recorder actually writing?". Plain Python, no ROS.

data_recorder.py supplies byte totals and the clock;
test/test_recorder_logic.py drives this directly.

A recorder process can be alive and healthy while writing nothing (every
topic it records has no publisher). This checks that the bag directory
grows. Cases covered: first sample (not confirmed yet), a stall starting
mid-run, recovery, and metadata-only growth.
"""

# Write rate is smoothed, since rosbag2 writes in chunks and the rate feeds
# a minutes-remaining readout.
DEFAULT_RATE_EMA_ALPHA = 0.3

# No growth for this long counts as stalled.
DEFAULT_STALL_WARN_SEC = 5.0


class WriteObservation:
    """One poll's verdict."""

    def __init__(self):
        # Bytes grew since the previous poll (`writing` on recording_status).
        self.writing = False
        self.total_bytes = 0
        # Seconds since bytes last grew. 0.0 while healthy.
        self.stall_sec = 0.0
        # stall_sec past the threshold. One poll without growth is normal.
        self.stalled = False
        # True once per stall, so the caller logs the transition only.
        self.newly_stalled = False
        self.recovered = False
        self.bytes_per_s = 0.0
        # False until two samples exist: not confirmed is not the same as
        # confirmed not writing.
        self.confirmed = False


class WriteWatch:
    """Watches a byte total for growth. Holds no clock and reads no disk."""

    def __init__(self, stall_warn_sec=DEFAULT_STALL_WARN_SEC,
                 rate_ema_alpha=DEFAULT_RATE_EMA_ALPHA):
        self.stall_warn_sec = float(stall_warn_sec)
        self.rate_ema_alpha = float(rate_ema_alpha)
        self.reset()

    def reset(self):
        """Start a new run, clearing the previous run's write rate."""
        self.total_bytes = 0
        self.last_sample_at = None
        self.last_growth_at = None
        self.bytes_per_s = 0.0
        self.ever_grew = False
        self._stall_reported = False

    def update(self, total_bytes, now):
        """Feed the current byte total. Returns a WriteObservation."""
        obs = WriteObservation()
        obs.total_bytes = int(total_bytes)

        prev_bytes = self.total_bytes
        prev_at = self.last_sample_at
        self.total_bytes = int(total_bytes)
        self.last_sample_at = now

        if prev_at is None:
            # First sample: not confirmed yet. The stall clock starts here,
            # so a recorder that never writes still trips the alarm.
            self.last_growth_at = now
            obs.bytes_per_s = 0.0
            return obs

        obs.confirmed = True
        dt = now - prev_at
        grew = obs.total_bytes > prev_bytes

        if grew:
            self.last_growth_at = now
            self.ever_grew = True
            if dt > 0:
                inst = (obs.total_bytes - prev_bytes) / dt
                self.bytes_per_s = (self.rate_ema_alpha * inst
                                    + (1.0 - self.rate_ema_alpha) * self.bytes_per_s)
            if self._stall_reported:
                obs.recovered = True
                self._stall_reported = False

        obs.writing = grew
        obs.bytes_per_s = self.bytes_per_s
        obs.stall_sec = max(0.0, now - (self.last_growth_at
                                        if self.last_growth_at is not None else now))
        obs.stalled = obs.stall_sec > self.stall_warn_sec
        if obs.stalled and not self._stall_reported:
            obs.newly_stalled = True
            self._stall_reported = True
        return obs

    def minutes_remaining(self, free_bytes):
        """Minutes of disk left at the observed write rate, or None before a
        rate has been observed."""
        if self.bytes_per_s <= 1e3 or free_bytes is None:
            return None
        return (free_bytes / self.bytes_per_s) / 60.0
