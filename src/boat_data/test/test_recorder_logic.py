"""Logic tests for the recorder's write watcher (write_watch.py).

Plain Python. These don't check that `ros2 bag record` writes, that the
topics exist, or that SIGINT finalises the bag; that needs a real recorder.

Run without ROS:
    python3 -m pytest src/boat_data/test/test_recorder_logic.py -v
    python3 src/boat_data/test/test_recorder_logic.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from boat_data.write_watch import WriteWatch  # noqa: E402

MB = 1_000_000


def test_first_sample_is_unconfirmed_not_a_false_green():
    """The first sample is "not confirmed", not a green light."""
    w = WriteWatch()
    obs = w.update(0, 0.0)
    assert not obs.confirmed
    assert not obs.writing
    assert not obs.stalled


def test_growth_is_reported_as_writing():
    w = WriteWatch()
    w.update(0, 0.0)
    obs = w.update(12 * MB, 1.0)
    assert obs.confirmed and obs.writing and not obs.stalled
    assert obs.total_bytes == 12 * MB
    assert obs.bytes_per_s > 0


def test_a_healthy_run_never_reports_a_stall():
    w = WriteWatch(stall_warn_sec=5.0)
    t, total = 0.0, 0
    w.update(total, t)
    for _ in range(60):
        t += 1.0
        total += 110 * MB          # ~110 MB/s of 640x480x10fps imagery
        obs = w.update(total, t)
        assert obs.writing and not obs.stalled
    assert w.ever_grew


def test_recorder_alive_but_writing_nothing_is_caught():
    """Recorder alive, no publishers, bag never grows: must be reported."""
    w = WriteWatch(stall_warn_sec=5.0)
    t = 0.0
    w.update(0, t)
    stalls = 0
    for _ in range(20):
        t += 1.0
        obs = w.update(0, t)       # metadata written, then nothing, ever
        assert not obs.writing
        if obs.newly_stalled:
            stalls += 1
    assert obs.stalled
    assert obs.stall_sec >= 15.0
    assert stalls == 1, (
        'the alarm must fire exactly once per stall, not every second -- '
        'a warning repeated every tick is one the operator learns to skip')
    assert not w.ever_grew


def test_metadata_only_growth_still_stalls():
    """Startup metadata makes the size non-zero, so the watcher keys on
    growth, not size."""
    w = WriteWatch(stall_warn_sec=5.0)
    t = 0.0
    w.update(0, t)
    t += 1.0
    obs = w.update(4096, t)        # metadata lands
    assert obs.writing
    for _ in range(10):            # and then nothing ever again
        t += 1.0
        obs = w.update(4096, t)
    assert obs.stalled and obs.total_bytes == 4096


def test_stall_then_recovery_resets_the_alarm():
    """Recovery clears the alarm, and a second stall fires again."""
    w = WriteWatch(stall_warn_sec=3.0)
    t, total = 0.0, 0
    w.update(total, t)
    for _ in range(3):
        t += 1.0
        total += 10 * MB
        w.update(total, t)

    for _ in range(6):             # publisher dies
        t += 1.0
        obs = w.update(total, t)
    assert obs.stalled

    t += 1.0                       # publisher comes back
    total += 10 * MB
    obs = w.update(total, t)
    assert obs.writing and obs.recovered and not obs.stalled

    second_alarms = 0
    for _ in range(6):             # and dies again
        t += 1.0
        obs = w.update(total, t)
        if obs.newly_stalled:
            second_alarms += 1
    assert obs.stalled
    assert second_alarms == 1, (
        'the second stall did not raise its own alarm -- a latched alarm '
        'would hide every failure after the first')


def test_a_single_missed_poll_is_not_a_stall():
    """One poll without growth is normal; only a sustained stall counts."""
    w = WriteWatch(stall_warn_sec=5.0)
    t, total = 0.0, 0
    w.update(total, t)
    t += 1.0
    total += 10 * MB
    w.update(total, t)
    t += 1.0
    obs = w.update(total, t)       # chunk boundary: no growth this second
    assert not obs.writing
    assert not obs.stalled, 'one quiet poll should not raise an alarm'


def test_reset_clears_a_previous_runs_rate():
    """A new run doesn't inherit the previous run's write rate."""
    w = WriteWatch()
    t, total = 0.0, 0
    w.update(total, t)
    for _ in range(5):
        t += 1.0
        total += 100 * MB
        w.update(total, t)
    assert w.bytes_per_s > 0
    assert w.minutes_remaining(50e9) is not None

    w.reset()
    assert w.bytes_per_s == 0.0
    assert not w.ever_grew
    assert w.minutes_remaining(50e9) is None, (
        'a fresh run reported a time estimate before writing anything')


def test_minutes_remaining_uses_the_observed_rate():
    """Minutes remaining, not free GB, is what limits a run."""
    w = WriteWatch()
    t, total = 0.0, 0
    w.update(total, t)
    for _ in range(20):
        t += 1.0
        total += 120 * MB          # ~7.2 GB/hour, the real imagery rate
        w.update(total, t)

    mins = w.minutes_remaining(2e9)     # the old MIN_FREE_GB_TO_START
    assert mins is not None
    assert 0.2 < mins < 0.5, (
        'expected the 2 GB threshold to be well under a minute at full '
        'imagery rate, got %.2f min' % mins)

    mins20 = w.minutes_remaining(20e9)  # recommended minimum free space
    assert mins20 > mins * 9


def test_minutes_remaining_is_none_without_an_observed_rate():
    """No estimate from an assumed rate in the measured-rate field."""
    w = WriteWatch()
    w.update(0, 0.0)
    w.update(0, 1.0)
    assert w.minutes_remaining(100e9) is None


def scenario_run_that_goes_wrong():
    """A run that starts fine, loses its camera publisher, and recovers."""
    log = []
    w = WriteWatch(stall_warn_sec=5.0)
    t, total = 0.0, 0

    def step(delta, note):
        nonlocal t, total
        t += 1.0
        total += delta
        obs = w.update(total, t)
        log.append((t, total / 1e6, obs.writing, obs.stalled,
                    obs.bytes_per_s / 1e6, note))

    step(0, 'run starts, nothing written yet')
    for _ in range(4):
        step(110 * MB, 'capturing normally')
    for _ in range(8):
        step(0, 'camera publisher died')
    for _ in range(4):
        step(110 * MB, 'camera back')
    return log


def _run_standalone():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith('test_') and callable(o)]
    failures = 0
    for name, fn in tests:
        try:
            fn()
            print('  PASS  %s' % name)
        except AssertionError as exc:
            failures += 1
            print('  FAIL  %s\n        %s' % (name, exc))
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print('  ERROR %s\n        %r' % (name, exc))

    print('\n%d/%d passed\n' % (len(tests) - failures, len(tests)))

    print('A run that loses its camera publisher and gets it back:')
    print('  %-6s %10s %8s %8s %10s  %s'
          % ('t', 'bag MB', 'writing', 'STALLED', 'MB/s', 'what is happening'))
    prev = None
    for t, mb, writing, stalled, rate, note in scenario_run_that_goes_wrong():
        mark = ' <-' if (writing, stalled) != prev else ''
        print('  %-6.0f %10.1f %8s %8s %10.1f  %s%s'
              % (t, mb, 'yes' if writing else 'no',
                 'YES' if stalled else '-', rate, note, mark))
        prev = (writing, stalled)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(_run_standalone())
