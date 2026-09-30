"""Logic tests for the heading complementary filter (heading.py).

Plain Python with a fake clock. These don't check the real IMU's units or
sign (only the boat can), the real gyro bias, or ROS wiring. There is a
test showing what the wrong sign looks like, so the symptom is recognisable.

Run without ROS:
    python3 -m pytest src/boat_navigation/test/test_heading_logic.py -v
    python3 src/boat_navigation/test/test_heading_logic.py   (prints a trace)
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from boat_navigation.heading import (  # noqa: E402
    HeadingEstimator,
    QUALITY_DEAD_RECKONING,
    QUALITY_FUSED,
    QUALITY_NONE,
    SOURCE_COG,
    SOURCE_MAG,
    course_over_ground_deg,
    normalize_angle_deg,
    wrap_heading_deg,
)

DT = 0.05          # 20 Hz, the node's actual predict rate
CRUISE = 1.5       # m/s, a plausible survey speed


def run_gyro(est, t, seconds, rate_dps, speed=CRUISE, cog=None, cog_every=1.0):
    """Advance the filter for `seconds`, optionally feeding COG. Returns the
    new time. Gyro at 20 Hz, GPS at 1 Hz."""
    next_cog = t + cog_every
    end = t + seconds
    while t < end - 1e-9:
        t += DT
        est.on_ground_speed(speed)
        est.on_gyro_rate(rate_dps, t)
        if cog is not None and t >= next_cog:
            est.on_cog(cog, speed, t)
            next_cog += cog_every
    return t


# Angle arithmetic

def test_normalize_takes_the_short_way_round():
    assert abs(normalize_angle_deg(350.0 - 10.0) - (-20.0)) < 1e-9
    assert abs(normalize_angle_deg(10.0 - 350.0) - 20.0) < 1e-9
    assert normalize_angle_deg(180.0) == 180.0      # (-180, 180], not [-180, 180)
    assert normalize_angle_deg(-180.0) == 180.0
    assert abs(normalize_angle_deg(720.0 + 45.0) - 45.0) < 1e-9


def test_wrap_keeps_headings_in_compass_range():
    assert abs(wrap_heading_deg(-10.0) - 350.0) < 1e-9
    assert abs(wrap_heading_deg(370.0) - 10.0) < 1e-9


def test_fusion_never_averages_across_the_wrap():
    """359 and 1 are two degrees apart. Correct along the wrapped error,
    never average the raw numbers (which gives 180)."""
    est = HeadingEstimator(alpha=0.5)     # exaggerated so the effect is obvious
    est.on_absolute_reference(359.0, 0.0, source=SOURCE_MAG)
    est.on_absolute_reference(1.0, 1.0, source=SOURCE_MAG)
    h = est.evaluate(1.0).heading_deg
    # Halfway between 359 and 1 the short way is 0 (or 360).
    assert min(abs(h - 0.0), abs(h - 360.0)) < 1e-6, (
        'blended across the wrap: got %.2f, expected ~0/360' % h)


def test_cog_helper_matches_compass_convention():
    # Due north.
    assert abs(course_over_ground_deg(60.0, 5.0, 60.001, 5.0)) < 0.5
    # Due east.
    assert abs(course_over_ground_deg(60.0, 5.0, 60.0, 5.001) - 90.0) < 0.5


# Quality states

def test_unreferenced_heading_is_NONE_not_zero():
    """An unreferenced heading is unknown, not zero degrees."""
    est = HeadingEstimator()
    t = run_gyro(est, 0.0, 5.0, rate_dps=3.0, speed=0.0)
    s = est.evaluate(t)
    assert s.quality == QUALITY_NONE
    assert not s.usable
    assert math.isnan(s.heading_deg)
    assert 'reference' in s.detail.lower()


def test_cog_above_the_threshold_gives_a_fused_estimate():
    est = HeadingEstimator()
    t = run_gyro(est, 0.0, 10.0, rate_dps=0.0, speed=CRUISE, cog=90.0)
    s = est.evaluate(t)
    assert s.quality == QUALITY_FUSED
    assert s.usable
    assert s.source == SOURCE_COG
    assert abs(normalize_angle_deg(s.heading_deg - 90.0)) < 1.0
    assert s.cog_valid
    assert s.detail == ''


def test_cog_below_the_speed_threshold_is_rejected_and_the_estimate_coasts():
    """Below the speed threshold COG is GPS noise and must be rejected."""
    est = HeadingEstimator(cog_min_speed_mps=0.5, fused_timeout_sec=5.0)
    t = run_gyro(est, 0.0, 10.0, rate_dps=0.0, speed=CRUISE, cog=90.0)
    assert est.evaluate(t).quality == QUALITY_FUSED

    # Boat slows to a crawl. COG keeps arriving -- and keeps being wrong,
    # because at 0.05 m/s it is noise. It must be ignored.
    before = est.evaluate(t).heading_deg
    t = run_gyro(est, t, 20.0, rate_dps=0.0, speed=0.05, cog=270.0)
    s = est.evaluate(t)
    assert s.quality == QUALITY_DEAD_RECKONING, (
        'accepted a course from a stationary boat')
    assert not s.cog_valid
    assert abs(normalize_angle_deg(s.heading_deg - before)) < 2.0, (
        'noise COG dragged the heading %.1f degrees' % (s.heading_deg - before))
    assert 'below' in s.detail.lower() and 'threshold' in s.detail.lower()
    assert est.rejected_cog_slow > 0


def test_degradation_is_progressive_and_ends_in_unusable():
    """FUSED, then DEAD_RECKONING, then NONE at the configured times. The
    NONE transition is what degrades failsafe RTL to a powered stop."""
    est = HeadingEstimator(fused_timeout_sec=5.0, max_coast_sec=60.0)
    t = run_gyro(est, 0.0, 5.0, rate_dps=0.0, speed=CRUISE, cog=0.0)
    assert est.evaluate(t).quality == QUALITY_FUSED

    # GPS stops. Gyro keeps running.
    t = run_gyro(est, t, 3.0, rate_dps=0.0, speed=CRUISE, cog=None)
    assert est.evaluate(t).quality == QUALITY_FUSED, 'degraded too eagerly'

    t = run_gyro(est, t, 5.0, rate_dps=0.0, speed=CRUISE, cog=None)
    s = est.evaluate(t)
    assert s.quality == QUALITY_DEAD_RECKONING
    assert s.usable, 'a few seconds of coasting is degraded, not unusable'
    assert 0.0 < s.confidence < 1.0

    t = run_gyro(est, t, 60.0, rate_dps=0.0, speed=CRUISE, cog=None)
    s = est.evaluate(t)
    assert s.quality == QUALITY_NONE
    assert not s.usable
    assert 'not steerable' in s.detail or 'unavailable' in s.detail.lower()


def test_confidence_decays_monotonically_while_coasting():
    est = HeadingEstimator(fused_timeout_sec=2.0, max_coast_sec=300.0)
    t = run_gyro(est, 0.0, 3.0, rate_dps=0.0, speed=CRUISE, cog=45.0)
    last = est.evaluate(t).confidence
    for _ in range(8):
        t = run_gyro(est, t, 5.0, rate_dps=0.0, speed=CRUISE, cog=None)
        now = est.evaluate(t).confidence
        assert now <= last + 1e-9, 'confidence went UP while coasting'
        last = now
    assert last < 0.9


# Filtering

def test_gyro_tracks_a_turn_between_gps_fixes():
    """Turning 10 deg/s for 3 s shows ~30 degrees from the gyro, with only
    three GPS fixes."""
    est = HeadingEstimator()
    t = run_gyro(est, 0.0, 5.0, rate_dps=0.0, speed=CRUISE, cog=0.0)
    start = est.evaluate(t).heading_deg
    t = run_gyro(est, t, 3.0, rate_dps=10.0, speed=CRUISE, cog=None)
    turned = normalize_angle_deg(est.evaluate(t).heading_deg - start)
    assert 27.0 < turned < 33.0, 'expected ~30 deg of turn, got %.1f' % turned


def test_cog_pulls_out_accumulated_gyro_drift():
    """The COG's job: it is drift-free, so it slowly corrects the gyro's
    accumulated error. This is the entire reason the two are fused."""
    est = HeadingEstimator(alpha=0.9)   # brisk, so the test is short
    # Seed the filter, then let a badly biased gyro drift it away with no
    # correction available.
    est.on_absolute_reference(0.0, 0.0, source=SOURCE_MAG)
    t = run_gyro(est, 0.0, 20.0, rate_dps=1.0, speed=CRUISE, cog=None)
    drifted = est.evaluate(t).heading_deg
    assert drifted > 15.0, 'expected the biased gyro to drift, got %.1f' % drifted

    # Now GPS comes back and says the boat is actually going due north.
    # The gyro keeps its (wrong) rate; COG must win over time.
    t = run_gyro(est, t, 30.0, rate_dps=1.0, speed=CRUISE, cog=0.0, cog_every=1.0)
    s = est.evaluate(t)
    assert s.quality == QUALITY_FUSED
    assert abs(normalize_angle_deg(s.heading_deg)) < abs(normalize_angle_deg(drifted)), (
        'COG failed to pull the estimate back: %.1f -> %.1f'
        % (drifted, s.heading_deg))


def test_a_single_wild_cog_sample_does_not_snap_the_heading():
    """One bad fix moves the estimate a little, not all the way."""
    est = HeadingEstimator(alpha=0.98)
    t = run_gyro(est, 0.0, 10.0, rate_dps=0.0, speed=CRUISE, cog=0.0)
    before = est.evaluate(t).heading_deg
    est.on_cog(180.0, CRUISE, t)          # one absurd sample
    after = est.evaluate(t).heading_deg
    moved = abs(normalize_angle_deg(after - before))
    assert moved < 5.0, 'one bad fix moved the heading %.1f degrees' % moved


def test_first_reference_is_adopted_outright():
    """The first reference is adopted directly instead of converging slowly."""
    est = HeadingEstimator(alpha=0.98)
    est.on_cog(137.0, CRUISE, 1.0)
    assert abs(est.evaluate(1.0).heading_deg - 137.0) < 1e-6


# Robustness

def test_non_finite_and_absurd_gyro_samples_are_dropped_not_integrated():
    """Non-finite gyro samples are dropped, not integrated."""
    est = HeadingEstimator()
    t = run_gyro(est, 0.0, 5.0, rate_dps=0.0, speed=CRUISE, cog=30.0)
    good = est.evaluate(t).heading_deg

    for bad in (float('nan'), float('inf'), -float('inf'), 5000.0, 'x', None):
        t += DT
        est.on_gyro_rate(bad, t)
    s = est.evaluate(t)
    assert math.isfinite(s.heading_deg), 'a bad sample poisoned the estimate'
    assert abs(normalize_angle_deg(s.heading_deg - good)) < 1.0
    assert est.rejected_rates == 6


def test_long_stall_is_not_integrated_across():
    """Long gaps (a stalled node) are skipped, not integrated at a constant rate."""
    est = HeadingEstimator()
    t = run_gyro(est, 0.0, 5.0, rate_dps=0.0, speed=CRUISE, cog=0.0)
    before = est.evaluate(t).heading_deg

    t += 4.0                              # the node was asleep
    est.on_gyro_rate(20.0, t)             # turning hard when it woke up
    after = est.evaluate(t).heading_deg
    assert abs(normalize_angle_deg(after - before)) < 1.0, (
        'integrated 20 deg/s across a 4 s stall -- 80 degrees of invented turn')


def test_bias_is_learned_while_stationary_and_capped():
    """A gyro sitting still should read zero; whatever it does read is
    bias, and bias is the thing that integrates into unbounded error."""
    est = HeadingEstimator(bias_learn_rate=0.05, max_bias_dps=5.0)
    t = 0.0
    # Tied to the dock: no speed, small constant offset on the gyro.
    for _ in range(4000):
        t += DT
        est.on_ground_speed(0.0)
        est.on_gyro_rate(0.7, t)
    assert 0.3 < est.gyro_bias_dps < 0.75, (
        'bias not learned: %.3f' % est.gyro_bias_dps)
    assert est.bias_samples > 0

    # A wildly out-of-range reading must not become a huge "bias".
    est2 = HeadingEstimator(bias_learn_rate=0.5, max_bias_dps=5.0)
    t = 0.0
    for _ in range(2000):
        t += DT
        est2.on_ground_speed(0.0)
        est2.on_gyro_rate(1.9, t)     # under the 2.0 stationary rate gate
    assert abs(est2.gyro_bias_dps) <= 5.0


def test_bias_is_not_learned_while_pivoting():
    """Bias is only learned when both speed and turn rate are low. A boat
    pivoting in place has near-zero speed but a real rotation."""
    est = HeadingEstimator(bias_learn_rate=0.05)
    t = 0.0
    for _ in range(2000):
        t += DT
        est.on_ground_speed(0.05)      # barely moving
        est.on_gyro_rate(25.0, t)      # but spinning
    assert abs(est.gyro_bias_dps) < 0.05, (
        'learned a real pivot as bias: %.3f' % est.gyro_bias_dps)


def test_learned_bias_is_actually_subtracted():
    est = HeadingEstimator(bias_learn_rate=0.5)
    t = 0.0
    for _ in range(3000):              # learn a +1.0 dps bias at the dock
        t += DT
        est.on_ground_speed(0.0)
        est.on_gyro_rate(1.0, t)
    est.on_absolute_reference(0.0, t, source=SOURCE_MAG)

    # Now the same biased reading while underway: the boat is NOT really
    # turning, so the heading should barely move.
    start = est.evaluate(t).heading_deg
    t = run_gyro(est, t, 30.0, rate_dps=1.0, speed=CRUISE, cog=None)
    drift = abs(normalize_angle_deg(est.evaluate(t).heading_deg - start))
    assert drift < 3.0, (
        'bias not being subtracted: drifted %.1f degrees in 30 s' % drift)


# Other absolute sources

def test_a_magnetometer_drops_in_without_touching_the_filter():
    """A magnetometer uses on_absolute_reference() with its own source."""
    est = HeadingEstimator()
    t = 0.0
    # Stationary, so COG is unavailable -- exactly where a magnetometer
    # earns its keep and where COG fundamentally cannot help.
    for _ in range(100):
        t += DT
        est.on_ground_speed(0.0)
        est.on_gyro_rate(0.0, t)
        est.on_absolute_reference(215.0, t, source=SOURCE_MAG)
    s = est.evaluate(t)
    assert s.quality == QUALITY_FUSED, (
        'a magnetometer should give a FUSED estimate at zero speed, which '
        'is the one thing COG can never do')
    assert s.source == SOURCE_MAG
    assert s.absolute_reference_available
    assert abs(normalize_angle_deg(s.heading_deg - 215.0)) < 1.0


def test_source_weight_scales_a_correction():
    """alpha_for() is the one place the correction gain is decided."""
    strong = HeadingEstimator(alpha=0.5)
    weak = HeadingEstimator(alpha=0.5)
    for est, w in ((strong, 1.0), (weak, 0.1)):
        est.on_absolute_reference(0.0, 0.0, source=SOURCE_MAG)
        est.on_absolute_reference(90.0, 1.0, source=SOURCE_MAG, weight=w)
    assert strong.evaluate(1.0).heading_deg > weak.evaluate(1.0).heading_deg


# Sign convention

def test_a_reversed_gyro_sign_makes_the_filter_fight_itself():
    """Diagnostic: with the gyro sign flipped the filter lags every turn
    and snaps back afterwards, which looks like bad tuning. The adapter
    must negate Imu.angular_velocity.z (rad/s CCW) to get deg/s clockwise."""
    correct = HeadingEstimator(alpha=0.9)
    reversed_ = HeadingEstimator(alpha=0.9)
    t = 0.0
    # Boat genuinely turning right (clockwise) at 10 deg/s, with COG
    # confirming it.
    for est, sign in ((correct, +1.0), (reversed_, -1.0)):
        tt, cog = 0.0, 0.0
        for _ in range(200):            # 10 s
            tt += DT
            cog = wrap_heading_deg(cog + 10.0 * DT)
            est.on_ground_speed(CRUISE)
            est.on_gyro_rate(sign * 10.0, tt)
            est.on_cog(cog, CRUISE, tt)
        t = tt
    err_ok = abs(normalize_angle_deg(correct.evaluate(t).heading_deg - 100.0))
    err_bad = abs(normalize_angle_deg(reversed_.evaluate(t).heading_deg - 100.0))
    assert err_ok < 5.0, 'correct sign should track well: %.1f deg off' % err_ok
    assert err_bad > err_ok * 3, (
        'a reversed gyro sign should track visibly worse (ok=%.1f bad=%.1f)'
        % (err_ok, err_bad))


# Scenario trace

def scenario_survey_leg_then_slow_manoeuvre():
    """Cruise (FUSED), slow near a dock (DEAD_RECKONING), sit still for two
    minutes (NONE), then drive off and re-acquire."""
    log = []
    est = HeadingEstimator(fused_timeout_sec=5.0, max_coast_sec=60.0)
    t = 0.0

    def note(phase):
        s = est.evaluate(t)
        log.append((t, phase, s.quality_name, s.heading_deg, s.confidence,
                    s.ground_speed_mps, s.detail))

    for _ in range(6):
        t = run_gyro(est, t, 5.0, rate_dps=0.0, speed=CRUISE, cog=45.0)
        note('cruising 1.5 m/s')
    for _ in range(4):
        t = run_gyro(est, t, 5.0, rate_dps=2.0, speed=0.2, cog=45.0)
        note('slowed to 0.2 m/s')
    for _ in range(10):
        t = run_gyro(est, t, 10.0, rate_dps=0.3, speed=0.0, cog=None)
        note('stopped')
    for _ in range(5):
        t = run_gyro(est, t, 5.0, rate_dps=0.0, speed=CRUISE, cog=45.0)
        note('under way again')
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

    print('Heading quality through a realistic run:')
    print('  %-7s %-20s %-15s %8s %6s %6s  %s'
          % ('t', 'phase', 'quality', 'heading', 'conf', 'm/s', 'detail'))
    prev = None
    for t, phase, q, h, c, v, detail in scenario_survey_leg_then_slow_manoeuvre():
        mark = ' <-' if q != prev else ''
        hs = '--' if (h != h) else '%.1f' % h    # h != h is the NaN test
        print('  %-7.1f %-20s %-15s %8s %6.2f %6.2f  %s%s'
              % (t, phase, q, hs, c, v, detail[:44], mark))
        prev = q
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(_run_standalone())
