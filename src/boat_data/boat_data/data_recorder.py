"""Data recorder: wraps `ros2 bag record` (MCAP) and keeps a run archive.

One directory per run with the bag, a generated manifest.json, the run
brief and checksums. Segment boundaries are tagged both in the bag
(boat/data_events) and in the manifest, so a bag describes itself and runs
can be searched without opening them. Post-processing is done offline.

Not in any launch file. Start it with:

    ros2 run boat_data data_recorder

Control it with JSON on boat/data_command:

    {"action": "start", "site": "lakeeola", "purpose": "docking trials"}
    {"action": "segment_begin", "step_id": "s3", "label": "Dock approach"}
    {"action": "marker", "note": "near miss with kayak"}
    {"action": "segment_end", "step_id": "s3", "outcome": "COMPLETE"}
    {"action": "stop"}
"""

import hashlib
import json
import os
import shutil
import signal
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from boat_data.write_watch import (
    DEFAULT_RATE_EMA_ALPHA,
    DEFAULT_STALL_WARN_SEC,
    WriteWatch,
)

# Topics recorded by default. Commands are recorded with the sensor data so
# runs can be used for controller evaluation and imitation learning.
DEFAULT_TOPICS = [
    # Navigation
    '/boat/gps/fix',
    '/boat/imu',
    # Heading with its quality state. Can't be reconstructed from boat/imu later.
    '/boat/heading',
    '/boat/nav_status',

    # Imagery
    '/boat/camera/image_raw',
    # Intrinsics every run, in case of a lens change or recalibration.
    '/boat/camera/camera_info',
    # Real camera or test pattern (CameraStatus).
    '/boat/camera/status',

    # Control: the arbiter's inputs and mode as well as its output, so every
    # thrust value in the bag says who was driving.
    '/boat/thrust_command',
    '/boat/cmd/manual',
    '/boat/cmd/waypoint',
    '/boat/cmd/failsafe',
    '/boat/mode',
    '/boat/state',
    '/boat/mission_active',
    '/boat/mission_feedback',

    # Safety
    '/boat/safety_status',
    # Failsafe levels change what a run means.
    '/boat/failsafe_state',
    '/boat/estop',

    # Power (real hardware only)
    '/boat/power_status',
    '/boat/system_status',

    # Sim actuators (simulation only)
    '/boat/sim/left_thrust',
    '/boat/sim/right_thrust',

    # Events
    '/boat/data_events',
]

# Topics with no publisher on the current platform are simply empty. The
# manifest's topics_requested field records what was asked for.

# Refuse to start a run below this much free space (GB). 2 GB is only
# 15-20 minutes at 6-8 GB/hour of 640x480 10 fps imagery, so raise it for
# real runs:
#   ros2 run boat_data data_recorder --ros-args -p min_free_gb:=20.0
MIN_FREE_GB_TO_START = 2.0

# `writing` in the status means the bag directory actually grew since the
# last poll. A recorder can be alive while writing nothing (no publishers on
# its topics), so if nothing grows for this long the status goes red.
# The logic is in write_watch.py so it can be tested without ROS.
WRITE_STALL_WARN_SEC = DEFAULT_STALL_WARN_SEC

# Bag size is polled at the 1 Hz status rate.
BYTES_RATE_EMA_ALPHA = DEFAULT_RATE_EMA_ALPHA


def utc_now_iso():
    return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')


def utc_now_compact():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def sha256_of(path, chunk=1024 * 1024):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        while True:
            block = f.read(chunk)
            if not block:
                break
            h.update(block)
    return h.hexdigest()


class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        self.declare_parameter('data_root', str(Path.home() / 'asv-boat' / 'data'))
        self.declare_parameter('topics', DEFAULT_TOPICS)
        # fastwrite: no CRCs or index while recording on the Pi. The index
        # is rebuilt in post-processing.
        self.declare_parameter('mcap_preset', 'fastwrite')
        self.declare_parameter('record_all', False)
        # Parameter, see MIN_FREE_GB_TO_START.
        self.declare_parameter('min_free_gb', MIN_FREE_GB_TO_START)
        self.declare_parameter('write_stall_warn_sec', WRITE_STALL_WARN_SEC)

        self.data_root = Path(self.get_parameter('data_root').value)
        self.runs_dir = self.data_root / 'runs'

        self.command_sub = self.create_subscription(
            String, 'boat/data_command', self.command_callback, 10)

        # Tags are published so they land in the bag too.
        self.events_pub = self.create_publisher(String, 'boat/data_events', 10)
        self.status_pub = self.create_publisher(String, 'boat/recording_status', 10)

        self.proc = None
        self.run_id = None
        self.run_dir = None
        self.started_at = None
        self.manifest = None
        self.open_segments = {}

        # Watches bytes reaching the disk, see WRITE_STALL_WARN_SEC.
        self.write_watch = WriteWatch(
            stall_warn_sec=float(self.get_parameter('write_stall_warn_sec').value))
        self._last_obs = None

        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            'Data recorder ready. Archive root: %s (not recording)' % self.data_root)

    # Command handling

    def command_callback(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn('Malformed boat/data_command (not JSON), ignoring')
            return

        if not isinstance(cmd, dict):
            self.get_logger().warn('data_command must be a JSON object')
            return
        action = cmd.get('action')
        handlers = {
            'start': self.start_run,
            'stop': lambda c: self.stop_run(c),
            'marker': self.add_marker,
            'tag': self.add_tag,
            'segment_begin': self.segment_begin,
            'segment_end': self.segment_end,
        }
        handler = handlers.get(action)
        if handler is None:
            self.get_logger().warn('Unknown data_command action: %r' % action)
            return
        try:
            handler(cmd)
        except Exception as exc:  # noqa: BLE001 - never let a bad command kill recording
            self.get_logger().error('data_command %r failed: %s' % (action, exc))

    # Run lifecycle

    def start_run(self, cmd):
        if self.proc is not None:
            self.get_logger().warn('Already recording %s -- ignoring start' % self.run_id)
            return

        min_free_gb = float(self.get_parameter('min_free_gb').value)
        free_gb = shutil.disk_usage(str(self._ensure_root())).free / 1e9
        if free_gb < min_free_gb:
            self.get_logger().error(
                'Refusing to start: only %.1f GB free (need %.1f). A run that '
                'fills the disk corrupts its own tail.' % (free_gb, min_free_gb))
            return
        # Log the implied run length at the start.
        self.get_logger().info(
            '%.1f GB free -- roughly %.0f minutes of imagery at ~7 GB/hour.'
            % (free_gb, free_gb / 7.0 * 60.0))

        # Reset the write watcher so a previous run can't make this one look
        # healthy.
        self.write_watch.reset()
        self._last_obs = None

        site = cmd.get('site', 'unknown')
        seq = self._next_sequence()
        self.run_id = '%s_%s_run%03d' % (utc_now_compact(), site, seq)
        self.run_dir = self.runs_dir / self.run_id
        (self.run_dir / 'bag').mkdir(parents=True, exist_ok=True)

        topics = list(self.get_parameter('topics').value)
        record_all = bool(self.get_parameter('record_all').value)
        preset = self.get_parameter('mcap_preset').value

        args = ['ros2', 'bag', 'record', '--storage', 'mcap',
                '--output', str(self.run_dir / 'bag' / 'rosbag2')]
        if preset:
            args += ['--storage-preset-profile', preset]
        if record_all:
            args += ['--all']
        else:
            args += topics

        self.get_logger().info('Starting recording: %s' % self.run_id)
        try:
            # Own process group, so stopping signals the recorder's children too.
            self.proc = subprocess.Popen(args, start_new_session=True)
        except FileNotFoundError:
            self.get_logger().error(
                "'ros2' not found on PATH -- is the environment sourced?")
            self.proc = None
            return

        self.started_at = utc_now_iso()
        self.manifest = {
            'run_id': self.run_id,
            'schema_version': '1.0',
            'source': cmd.get('source', 'simulation'),
            'site': site,
            'purpose': cmd.get('purpose', ''),
            'started_utc': self.started_at,
            'ended_utc': None,
            'duration_s': None,
            'software': {
                'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
                'git_commit': self._git_commit(),
            },
            'topics_requested': topics if not record_all else ['<all>'],
            'segments': [],
            'markers': [],
            'quality': {},
        }
        self.open_segments = {}
        self._write_manifest()

        brief_path = self.run_dir / 'run_brief.json'
        if not brief_path.exists():
            # Placeholder for the operator to fill in conditions.
            brief_path.write_text(json.dumps({
                'site': site,
                'purpose': cmd.get('purpose', ''),
                'weather': '', 'wind': '', 'sea_state': '',
                'sun': '', 'water_temp_c': None,
                'operator': '', 'notes': '',
            }, indent=2), encoding='utf-8')

        self._emit_event({'event': 'run_start', 'run_id': self.run_id})

    def stop_run(self, cmd=None):
        if self.proc is None:
            self.get_logger().warn('Not recording -- ignoring stop')
            return

        self.get_logger().info('Stopping recording: %s' % self.run_id)
        self._emit_event({'event': 'run_stop', 'run_id': self.run_id})

        # Close any open segments so an aborted run still has a clean manifest.
        for step_id in list(self.open_segments):
            self.segment_end({'step_id': step_id, 'outcome': 'INTERRUPTED'})

        # SIGINT, not SIGKILL, so rosbag2 finalises the file.
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError) as exc:
            self.get_logger().warn('Could not signal recorder group: %s' % exc)

        try:
            self.proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                'Recorder did not exit within 20s of SIGINT -- killing. The bag '
                'may be missing its index and should be checked before use.')
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        self.proc = None

        ended = utc_now_iso()
        self.manifest['ended_utc'] = ended
        self.manifest['duration_s'] = self._elapsed_s()
        self.manifest['quality'] = self._collect_quality()
        self._write_manifest()
        self._write_checksums()

        self.get_logger().info(
            'Run archived: %s (%.0fs)' % (self.run_dir, self.manifest['duration_s'] or 0.0))
        self.run_id = None
        self.run_dir = None
        self.started_at = None
        self.manifest = None

    # Tagging

    def segment_begin(self, cmd):
        if not self._recording('segment_begin'):
            return
        step_id = cmd.get('step_id') or 'step_%d' % len(self.manifest['segments'])
        seg = {
            'step_id': step_id,
            'label': cmd.get('label', ''),
            'type': cmd.get('type', ''),
            't_start_utc': utc_now_iso(),
            't_end_utc': None,
            'tags': list(cmd.get('tags', [])),
            'outcome': None,
        }
        self.open_segments[step_id] = seg
        self.manifest['segments'].append(seg)
        self._write_manifest()
        self._emit_event({'event': 'segment_begin', 'step_id': step_id,
                          'label': seg['label']})

    def segment_end(self, cmd):
        if not self._recording('segment_end'):
            return
        step_id = cmd.get('step_id')
        seg = self.open_segments.pop(step_id, None)
        if seg is None:
            self.get_logger().warn('segment_end for unknown step %r' % step_id)
            return
        seg['t_end_utc'] = utc_now_iso()
        seg['outcome'] = cmd.get('outcome', 'COMPLETE')
        self._write_manifest()
        self._emit_event({'event': 'segment_end', 'step_id': step_id,
                          'outcome': seg['outcome']})

    def add_tag(self, cmd):
        """Attach a tag to the currently open segment(s)."""
        if not self._recording('tag'):
            return
        tag = cmd.get('tag')
        if not tag:
            return
        targets = ([self.open_segments[cmd['step_id']]]
                   if cmd.get('step_id') in self.open_segments
                   else list(self.open_segments.values()))
        for seg in targets:
            if tag not in seg['tags']:
                seg['tags'].append(tag)
        self._write_manifest()
        self._emit_event({'event': 'tag', 'tag': tag})

    def add_marker(self, cmd):
        """A timestamped point of interest (one press to mark something).
        Could also be wired to a GPIO button on the boat."""
        if not self._recording('marker'):
            return
        marker = {'t_utc': utc_now_iso(), 'note': cmd.get('note', '')}
        self.manifest['markers'].append(marker)
        self._write_manifest()
        self._emit_event({'event': 'marker', 'note': marker['note']})

    # Helpers

    def _recording(self, action):
        if self.proc is None or self.manifest is None:
            self.get_logger().warn('Not recording -- ignoring %s' % action)
            return False
        return True

    def _ensure_root(self):
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        return self.runs_dir

    def _next_sequence(self):
        """Per-day run counter, so run007 means the seventh run today."""
        today = utc_now_compact()[:8]
        existing = [p.name for p in self._ensure_root().glob('%s*' % today)]
        return len(existing) + 1

    def _git_commit(self):
        """Git commit of the software that produced this run.

        Found from this file's resolved path, since the source is reached
        through a symlink and the repository is above it.
        """
        try:
            out = subprocess.run(
                ['git', 'rev-parse', '--short', 'HEAD'],
                cwd=str(Path(__file__).resolve().parent),
                capture_output=True, text=True, timeout=5)
            return out.stdout.strip() or 'unknown'
        except Exception:  # noqa: BLE001 - provenance is best-effort
            return 'unknown'

    def _elapsed_s(self):
        if self.started_at is None:
            return None
        start = datetime.strptime(self.started_at, '%Y-%m-%dT%H:%M:%SZ').replace(
            tzinfo=timezone.utc)
        return (datetime.now(timezone.utc) - start).total_seconds()

    def _collect_quality(self):
        """Run quality evidence computed from what is known without opening
        the bag. Post-processing adds more."""
        bag_dir = self.run_dir / 'bag'
        files = sorted(bag_dir.rglob('*.mcap'))
        total_bytes = sum(f.stat().st_size for f in files)
        return {
            'bag_files': len(files),
            'bytes': total_bytes,
            'segments_recorded': len(self.manifest['segments']),
            'markers': len(self.manifest['markers']),
            # A run that never grew recorded nothing; say so in the manifest.
            'observed_write_rate_bytes_per_s': round(self.write_watch.bytes_per_s, 1),
            'write_confirmed': bool(total_bytes > 0 and self.write_watch.ever_grew),
            'post_processed': False,
        }

    def _write_manifest(self):
        if self.run_dir is None or self.manifest is None:
            return
        # Write then rename, so a crash can't leave a truncated manifest.
        tmp = self.run_dir / 'manifest.json.tmp'
        tmp.write_text(json.dumps(self.manifest, indent=2), encoding='utf-8')
        tmp.replace(self.run_dir / 'manifest.json')

    def _write_checksums(self):
        lines = []
        for path in sorted(self.run_dir.rglob('*')):
            if path.is_file() and path.name != 'checksums.sha256':
                rel = path.relative_to(self.run_dir)
                lines.append('%s  %s' % (sha256_of(path), rel))
        (self.run_dir / 'checksums.sha256').write_text(
            '\n'.join(lines) + '\n', encoding='utf-8')

    def _emit_event(self, payload):
        payload['t_utc'] = utc_now_iso()
        self.events_pub.publish(String(data=json.dumps(payload)))

    def _bag_bytes_now(self):
        """Total bytes under this run's bag/ directory, or None.

        Counts all files, not only *.mcap, so a metadata-only run is
        reported as such instead of as zero bytes.
        """
        if self.run_dir is None:
            return None
        try:
            return sum(p.stat().st_size
                       for p in (self.run_dir / 'bag').rglob('*')
                       if p.is_file())
        except OSError:
            return None

    def _update_write_watch(self, now):
        """Feed the byte total to WriteWatch and log transitions."""
        total = self._bag_bytes_now()
        if total is None:
            return None

        obs = self.write_watch.update(total, now)
        self._last_obs = obs

        # Log transitions only.
        if obs.newly_stalled:
            self.get_logger().error(
                'RECORDING BUT NOT WRITING: the bag has not grown in %.0f s '
                '(%.1f MB total). The recorder process is alive, so this is '
                'almost certainly that the topics it was told to record have '
                'no publishers -- check `ros2 topic list` against the '
                'recorder\'s "topics" parameter. The dashboard is now showing '
                'this as NOT WRITING.' % (obs.stall_sec, total / 1e6))
        elif obs.recovered:
            self.get_logger().warn(
                'Recording resumed -- the bag is growing again (%.1f MB).'
                % (total / 1e6))
        return obs

    def publish_status(self):
        recording = self.proc is not None
        now = time.monotonic()

        # Notice the recorder dying on its own (disk full, crash).
        if recording and self.proc.poll() is not None:
            self.get_logger().error(
                'Recorder subprocess exited unexpectedly (code %s) -- finalising run'
                % self.proc.returncode)
            self.proc = None
            recording = False
            if self.manifest is not None:
                self.manifest['ended_utc'] = utc_now_iso()
                self.manifest['duration_s'] = self._elapsed_s()
                self.manifest['quality'] = self._collect_quality()
                self.manifest['quality']['unexpected_exit'] = True
                self._write_manifest()

        obs = self._update_write_watch(now) if recording else None

        try:
            free_gb = shutil.disk_usage(str(self._ensure_root())).free / 1e9
        except OSError:
            free_gb = None

        # Minutes left at the observed write rate.
        est_minutes = None
        if free_gb is not None:
            mins = self.write_watch.minutes_remaining(free_gb * 1e9)
            est_minutes = round(mins, 1) if mins is not None else None

        self.status_pub.publish(String(data=json.dumps({
            'recording': recording,
            # recording: a recorder process exists.
            # writing:   bytes reached the disk in the last second.
            'writing': bool(obs.writing) if obs else False,
            'write_stall_sec': round(obs.stall_sec, 1) if obs else None,
            'write_stalled': bool(obs.stalled) if obs else False,
            'bag_mb': round(obs.total_bytes / 1e6, 2) if obs else None,
            'write_mb_per_s': (round(obs.bytes_per_s / 1e6, 3)
                               if obs else None),
            'est_minutes_remaining': est_minutes,
            'run_id': self.run_id,
            'elapsed_s': round(self._elapsed_s(), 1) if recording else None,
            'open_segments': list(self.open_segments),
            'free_gb': round(free_gb, 1) if free_gb is not None else None,
            'min_free_gb': float(self.get_parameter('min_free_gb').value),
        })))

    def destroy_node(self):
        # Never leave a recording unfinalised.
        if self.proc is not None:
            self.get_logger().warn('Node shutting down while recording -- finalising')
            self.stop_run()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
