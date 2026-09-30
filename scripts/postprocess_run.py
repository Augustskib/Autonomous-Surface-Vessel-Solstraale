#!/usr/bin/env python3
"""Offline post-processing for a recorded run (MCAP bag from data_recorder).

No ROS needed. Requires mcap, mcap-ros2-support and pandas; pyarrow for
parquet, otherwise CSV.

  1. Checks timestamps per topic: increasing, and gaps larger than the
     nominal rate allows.
  2. Builds a sync index: for each camera frame, the nearest GPS fix and IMU
     sample and their offset in ms.
  3. Computes quality metrics from what is in the bag only.
  4. Writes <run_dir>/derived/sync_index.parquet (or .csv) and
     quality_report.json.
  5. Merges the report into manifest.json under "quality" and sets
     quality.post_processed = True.

    python3 postprocess_run.py /path/to/run_dir
    python3 postprocess_run.py /path/to/run_dir/bag/some_run.mcap

run_dir layout:
    run_dir/
      manifest.json
      run_brief.json
      bag/some_run_0.mcap   (the first .mcap found is used)
      checksums.sha256

Without manifest.json the merge step is skipped and derived/ is written next
to the bag.
"""

import argparse
import json
import sys
from bisect import bisect_left
from pathlib import Path

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

# Nominal rates, used to decide when a gap is suspicious.
EXPECTED_RATES_HZ = {
    '/boat/camera/image_raw': 10.0,
    '/boat/gps/fix': 5.0,
    '/boat/imu': 20.0,
}

# NavSatStatus uses signed int8 with STATUS_NO_FIX = -1. The synthetic test
# bag uses 255 instead; is_no_fix_status() accepts both.

def is_no_fix_status(status) -> bool:
    return status is not None and (status < 0 or status == 255)

CAMERA_TOPIC = '/boat/camera/image_raw'
GPS_TOPIC = '/boat/gps/fix'
IMU_TOPIC = '/boat/imu'

# Sync error above this is flagged per-frame as "poor sync" in the index.
SYNC_WARN_MS = 100.0


def find_bag_file(run_dir: Path) -> Path:
    if run_dir.is_file() and run_dir.suffix == '.mcap':
        return run_dir
    bag_dir = run_dir / 'bag'
    search_dir = bag_dir if bag_dir.is_dir() else run_dir
    candidates = sorted(search_dir.glob('*.mcap'))
    if not candidates:
        raise FileNotFoundError(f'No .mcap file found under {search_dir}')
    return candidates[0]


def load_topic_timestamps(bag_path: Path):
    """Return {topic: sorted list of log_time_ns}."""
    timestamps = {}
    with open(bag_path, 'rb') as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema, channel, message, ros_msg in reader.iter_decoded_messages():
            timestamps.setdefault(channel.topic, []).append(
                (message.log_time, ros_msg))
    for topic in timestamps:
        timestamps[topic].sort(key=lambda pair: pair[0])
    return timestamps


def validate_monotonic_and_gaps(topic: str, entries):
    """Check one topic's (log_time_ns, msg) entries, sorted. Returns a dict
    of findings."""
    times_ns = [t for t, _ in entries]
    out_of_order = 0
    for i in range(1, len(times_ns)):
        if times_ns[i] < times_ns[i - 1]:
            out_of_order += 1

    gaps = []
    nominal_hz = EXPECTED_RATES_HZ.get(topic)
    if nominal_hz and len(times_ns) > 1:
        nominal_period_s = 1.0 / nominal_hz
        # Flag any gap more than 2x the nominal period as a probable drop.
        gap_threshold_s = nominal_period_s * 2.0
        for i in range(1, len(times_ns)):
            dt_s = (times_ns[i] - times_ns[i - 1]) / 1e9
            if dt_s > gap_threshold_s:
                dropped_est = int(round(dt_s / nominal_period_s)) - 1
                gaps.append({
                    'start_s': (times_ns[i - 1] - times_ns[0]) / 1e9,
                    'end_s': (times_ns[i] - times_ns[0]) / 1e9,
                    'duration_s': round(dt_s, 3),
                    'estimated_dropped_messages': max(dropped_est, 0),
                })

    return {
        'topic': topic,
        'message_count': len(times_ns),
        'out_of_order_count': out_of_order,
        'duration_s': round((times_ns[-1] - times_ns[0]) / 1e9, 3) if len(times_ns) > 1 else 0.0,
        'gaps': gaps,
        'total_estimated_dropped_messages': sum(g['estimated_dropped_messages'] for g in gaps),
    }


def nearest_neighbor_index(target_ns: int, sorted_times_ns, max_i):
    """Nearest neighbour by binary search in a sorted list of ints.
    Returns (index, abs_diff_ns), or (None, None) if the list is empty."""
    if not sorted_times_ns:
        return None, None
    i = bisect_left(sorted_times_ns, target_ns)
    candidates = []
    if i < len(sorted_times_ns):
        candidates.append(i)
    if i > 0:
        candidates.append(i - 1)
    best_i = min(candidates, key=lambda c: abs(sorted_times_ns[c] - target_ns))
    return best_i, abs(sorted_times_ns[best_i] - target_ns)


def build_sync_index(timestamps):
    """For each camera frame, the nearest GPS fix and IMU sample by
    log_time, with offsets in ms."""
    camera_entries = timestamps.get(CAMERA_TOPIC, [])
    gps_entries = timestamps.get(GPS_TOPIC, [])
    imu_entries = timestamps.get(IMU_TOPIC, [])

    gps_times = [t for t, _ in gps_entries]
    imu_times = [t for t, _ in imu_entries]

    rows = []
    for frame_i, (cam_t, _cam_msg) in enumerate(camera_entries):
        gps_i, gps_diff_ns = nearest_neighbor_index(cam_t, gps_times, len(gps_times))
        imu_i, imu_diff_ns = nearest_neighbor_index(cam_t, imu_times, len(imu_times))

        gps_sync_ms = (gps_diff_ns / 1e6) if gps_diff_ns is not None else None
        imu_sync_ms = (imu_diff_ns / 1e6) if imu_diff_ns is not None else None

        gps_status = None
        if gps_i is not None:
            gps_msg = gps_entries[gps_i][1]
            gps_status = getattr(gps_msg, 'status', None)

        rows.append({
            'camera_frame_index': frame_i,
            'camera_log_time_ns': cam_t,
            'gps_index': gps_i,
            'gps_sync_error_ms': round(gps_sync_ms, 3) if gps_sync_ms is not None else None,
            'gps_status': gps_status,
            'gps_no_fix': is_no_fix_status(gps_status),
            'imu_index': imu_i,
            'imu_sync_error_ms': round(imu_sync_ms, 3) if imu_sync_ms is not None else None,
            'poor_sync': bool(
                (gps_sync_ms is not None and gps_sync_ms > SYNC_WARN_MS) or
                (imu_sync_ms is not None and imu_sync_ms > SYNC_WARN_MS)
            ),
        })
    return rows


def write_sync_index(rows, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    try:
        import pandas as pd
        df = pd.DataFrame(rows)
        parquet_path = out_dir / 'sync_index.parquet'
        df.to_parquet(parquet_path, index=False)
        return parquet_path
    except ImportError:
        import csv
        csv_path = out_dir / 'sync_index.csv'
        with open(csv_path, 'w', newline='') as f:
            if rows:
                writer = csv.DictWriter(f, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
        return csv_path


def compute_quality_report(topic_validations, sync_rows):
    gps_fix_loss_events = 0
    prev_status = None
    for row in sync_rows:
        status = row['gps_status']
        if status is None:
            continue
        is_no_fix = is_no_fix_status(status)
        prev_no_fix = is_no_fix_status(prev_status)
        if is_no_fix and not prev_no_fix:
            gps_fix_loss_events += 1
        prev_status = status

    sync_errors_gps = [r['gps_sync_error_ms'] for r in sync_rows if r['gps_sync_error_ms'] is not None]
    sync_errors_imu = [r['imu_sync_error_ms'] for r in sync_rows if r['imu_sync_error_ms'] is not None]
    poor_sync_frames = sum(1 for r in sync_rows if r['poor_sync'])

    def stats(values):
        if not values:
            return {'mean_ms': None, 'max_ms': None}
        return {'mean_ms': round(sum(values) / len(values), 3), 'max_ms': round(max(values), 3)}

    camera_val = next((t for t in topic_validations if t['topic'] == CAMERA_TOPIC), None)

    report = {
        'topics': topic_validations,
        'sync': {
            'camera_frames_indexed': len(sync_rows),
            'gps_offset': stats(sync_errors_gps),
            'imu_offset': stats(sync_errors_imu),
            'frames_with_poor_sync_gt_100ms': poor_sync_frames,
        },
        'dropped_camera_frames_estimate': camera_val['total_estimated_dropped_messages'] if camera_val else None,
        'gps_fix_loss_events': gps_fix_loss_events,
        'out_of_order_messages': sum(t['out_of_order_count'] for t in topic_validations),
        'pass': True,
        'issues': [],
    }

    if report['out_of_order_messages'] > 0:
        report['pass'] = False
        report['issues'].append(f"{report['out_of_order_messages']} out-of-order message(s) found.")
    if camera_val and camera_val['total_estimated_dropped_messages'] > 0:
        report['issues'].append(
            f"~{camera_val['total_estimated_dropped_messages']} camera frame(s) likely dropped "
            f"({len(camera_val['gaps'])} gap(s)).")
    if gps_fix_loss_events > 0:
        report['issues'].append(f'{gps_fix_loss_events} GPS fix-loss event(s) detected.')
    if poor_sync_frames > 0:
        report['issues'].append(
            f'{poor_sync_frames} camera frame(s) have sync error > {SYNC_WARN_MS:.0f}ms to GPS or IMU.')

    return report


def merge_into_manifest(run_dir: Path, quality_report: dict):
    manifest_path = run_dir / 'manifest.json'
    if not manifest_path.is_file():
        print(f'(no manifest.json found at {manifest_path}, skipping manifest merge)')
        return
    with open(manifest_path) as f:
        manifest = json.load(f)
    existing_quality = manifest.get('quality', {})
    existing_quality.update(quality_report)
    existing_quality['post_processed'] = True
    manifest['quality'] = existing_quality
    with open(manifest_path, 'w') as f:
        json.dump(manifest, f, indent=2)
    print(f'Merged quality metrics into {manifest_path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_path', help='Path to a run_dir, or directly to a .mcap file')
    args = parser.parse_args()

    run_path = Path(args.run_path)
    bag_path = find_bag_file(run_path)
    run_dir = run_path if run_path.is_dir() else run_path.parent.parent if run_path.parent.name == 'bag' else run_path.parent

    print(f'Reading {bag_path} ...')
    timestamps = load_topic_timestamps(bag_path)
    for topic, entries in timestamps.items():
        print(f'  {topic}: {len(entries)} messages')

    topic_validations = [validate_monotonic_and_gaps(topic, entries) for topic, entries in timestamps.items()]

    print('Building nearest-neighbor sync index (camera as master clock)...')
    sync_rows = build_sync_index(timestamps)

    derived_dir = run_dir / 'derived'
    index_path = write_sync_index(sync_rows, derived_dir)
    print(f'Wrote sync index: {index_path} ({len(sync_rows)} rows)')

    quality_report = compute_quality_report(topic_validations, sync_rows)
    report_path = derived_dir / 'quality_report.json'
    with open(report_path, 'w') as f:
        json.dump(quality_report, f, indent=2)
    print(f'Wrote quality report: {report_path}')

    print()
    print('=== QUALITY SUMMARY ===')
    print(f"PASS: {quality_report['pass']}")
    for issue in quality_report['issues']:
        print(f'  - {issue}')
    if not quality_report['issues']:
        print('  (no issues found)')
    print(f"GPS sync offset:  mean={quality_report['sync']['gps_offset']['mean_ms']}ms  max={quality_report['sync']['gps_offset']['max_ms']}ms")
    print(f"IMU sync offset:  mean={quality_report['sync']['imu_offset']['mean_ms']}ms  max={quality_report['sync']['imu_offset']['max_ms']}ms")

    merge_into_manifest(run_dir, quality_report)


if __name__ == '__main__':
    main()
