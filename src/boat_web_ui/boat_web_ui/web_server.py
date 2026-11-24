import os
import numpy as np
from boat_web_ui.video_stream import SharedVideo
from boat_web_ui.access import authorized, same_origin
from boat_web_ui.lidar_view import LatestScan
import json
import re
import uuid
import math
import socket
import subprocess
import threading
import time
from pathlib import Path
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, NavSatFix, Imu, PointCloud2
from std_msgs.msg import String, Bool, Empty
from cv_bridge import CvBridge
from boat_msgs.msg import (
    CameraStatus,
    FailsafeState,
    HeadingEstimate,
    MissionCommand,
    NavStatus,
    PowerStatus,
    SafetyStatus,
    ThrustCommand,
)
from foxglove_msgs.msg import GeoJSON
from geometry_msgs.msg import PoseStamped
from flask import Flask, Response, redirect, request, jsonify


# Heading, roll and pitch shown as numbers on the dashboard. Same quaternion
# math as waypoint_follower, duplicated because there is no shared util module.
def imu_orientation_deg(q):
    """Return (heading_deg, roll_deg, pitch_deg) from an IMU quaternion.

    Heading is compass style (0 = north, clockwise). Roll and pitch are body
    frame angles, mainly to see whether the boat is upright.
    """
    x, y, z, w = q.x, q.y, q.z, q.w

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    heading_deg = (90.0 - math.degrees(yaw)) % 360.0

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll_deg = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))  # guard asin() domain error from fp noise
    pitch_deg = math.degrees(math.asin(sinp))

    return heading_deg, roll_deg, pitch_deg


def _nan_to_none(x):
    """JSON has no NaN, so the NaN "not applicable" fields become null."""
    return None if isinstance(x, float) and math.isnan(x) else x


# Battery reserve. Pure function, no ROS or Flask, so it can be unit tested.

# Pack defaults match the reported 3S LiPo. Voltage is only a rough reserve
# indicator; these thresholds do not guarantee return-trip capacity.
DEFAULT_CELLS = 3
DEFAULT_CELL_FULL_V = 4.20
DEFAULT_CELL_RESERVE_V = 3.60   # the FLOOR: enough left to get home
DEFAULT_CELL_EMPTY_V = 3.30     # below this, damage

# Window for the rolling max voltage, used as the rested voltage estimate.
RESTED_VOLTAGE_WINDOW_SEC = 60.0


def battery_reserve(voltage_v, cells=DEFAULT_CELLS,
                    cell_full_v=DEFAULT_CELL_FULL_V,
                    cell_reserve_v=DEFAULT_CELL_RESERVE_V):
    """Fraction of the above-reserve band left, plus a state.

    Returns (fraction, state). fraction is 1.0 at a full pack and 0.0 at the
    reserve floor. state is 'OK', 'WARN', 'AT_RESERVE', 'BELOW_RESERVE' or
    'UNKNOWN'. Zero means "head back now", not "empty": the band below the
    floor is kept for the return trip.

    This is voltage based and therefore rough. Voltage sags under load, the
    LiPo curve is flat in the middle and drops fast at the end. Coulomb
    counting from the INA226 current readings would be better and is not
    implemented. The interpolation is linear on purpose.
    """
    try:
        voltage_v = float(voltage_v)
    except (TypeError, ValueError):
        return None, 'UNKNOWN'
    if not math.isfinite(voltage_v) or voltage_v <= 0.0:
        return None, 'UNKNOWN'

    full = cell_full_v * cells
    floor = cell_reserve_v * cells
    if full <= floor:
        return None, 'UNKNOWN'

    frac = (voltage_v - floor) / (full - floor)
    frac = max(0.0, min(1.0, frac))

    if voltage_v < DEFAULT_CELL_EMPTY_V * cells:
        state = 'BELOW_RESERVE'
    elif voltage_v <= floor:
        state = 'AT_RESERVE'
    elif frac < 0.30:
        # Warn below reserve + 30%.
        state = 'WARN'
    else:
        state = 'OK'
    return frac, state


app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()
# Set only when detector_node publishes boat/perception/detections_image.
latest_detections_frame = None
detections_frame_lock = threading.Lock()
ros_node = None

# Live telemetry, written by the ROS callbacks and read by /telemetry.
# lat/lon stay None until the first fix so the map draws no marker.
latest_recording_time = None
latest_lidar = {}
latest_lidar_time = None
# Newest /boat/lidar/points_preview cloud, flattened on request for the map.
lidar_scan = LatestScan()
telemetry_lock = threading.RLock()  # telemetry() also calls _power_snapshot()
latest_gps = {'lat': None, 'lon': None}
latest_state = None
latest_geojson = None  # parsed dict (trail + current target), or None
latest_imu = {'heading': None, 'roll': None, 'pitch': None}
latest_nav = {}  # parsed boat/nav_status dict from waypoint_follower
latest_safety = {}  # parsed boat/safety_status dict from safety_monitor
# Mirrors boat/estop. A latched e-stop makes mission_manager refuse every
# mission, so it is shown on the dashboard.
latest_estop = False
# Mirrors boat/mode from control_arbiter: IDLE, MANUAL, WAYPOINT, HOLD or
# ESTOP. If a mission is executing but this is not WAYPOINT, something else
# is holding control.
latest_mode = None
latest_operator = {}
latest_operator_time = None
operator_receipts = {}
mission_receipts = {}
latest_mission = {}
latest_mission_time = None
# How long a control route waits for the ROS graph to confirm the result.
CONTROL_ACK_TIMEOUT_SEC = 1.5
CONTROL_ACK_POLL_SEC = 0.02
# Mirrors boat/recording_status from data_recorder. Missing means not
# recording (the recorder is not in the default launch).
latest_recording = {'recording': False, 'run_id': None, 'elapsed_s': None,
                     'open_segments': [], 'free_gb': None}
# Dock / home position. Captured from the first valid GPS fix and movable
# at runtime. Kept on the server so every client sees the same one.
dock_point = {'lat': None, 'lon': None, 'auto': True}
# Dock ArUco marker from dock_marker_node: distance and lateral offset in
# meters, only valid while detected is true. Separate from dock_point.
latest_dock_marker = {'detected': False, 'distance_m': None, 'lateral_m': None}
# boat/failsafe_state from safety_monitor, drives the banner on both pages.
# Empty means no supervisor has published yet.
latest_failsafe = {}
# boat/heading from heading_estimator, with its quality state.
latest_heading = {}
# boat/camera/status from camera_publisher.
latest_camera = {}
# boat/power_status from power_monitor.
latest_power = {'channels': [], 'seen': False}
# boat/system_status from system_monitor (JSON in a String).
latest_system = {}
# Rolling max pack voltage, see _note_pack_voltage().
_pack_voltage_samples = []


# The console is one page. Its markup, style and script live next to this
# module and are joined once at import time. Plain string replacement is used
# instead of a template engine because the files are full of CSS/JS braces.

def _compose():
    """Join console.html with its stylesheet and scripts."""
    here = Path(__file__).parent
    def read(name):
        return (here / name).read_text(encoding='utf-8')
    script = read('console.js').replace('//__OPERATOR_JS__', read('operator_controls.js'))
    out = read('console.html').replace('/*__CONSOLE_CSS__*/', read('console.css'))
    return out.replace('//__CONSOLE_JS__', script)


CONSOLE_PAGE_HTML = _compose()


# Remote use goes through authenticated SSH forwarding to localhost.
@app.before_request
def check_access():
    if request.host not in ('127.0.0.1:5000', 'localhost:5000', '[::1]:5000'):
        return jsonify({'status': 'error', 'error': 'Use the local forwarded dashboard address'}), 400
    if not authorized(request.authorization, os.environ.get('ASV_OPERATOR_PASSWORD', '')):
        return Response('Authentication required', 401,
                        {'WWW-Authenticate': 'Basic realm="ASV operator"'})
    if request.method == 'POST' and not same_origin(request.headers.get('Origin'), request.host_url):
        return jsonify({'status': 'error', 'error': 'Cross-origin command refused'}), 403


@app.before_request
def validate_command_body():
    if request.method == 'POST' and request.is_json:
        if not isinstance(request.get_json(silent=True), dict):
            return jsonify({'status': 'error', 'error': 'JSON body must be an object'}), 400


@app.route('/')
def index():
    limit = ros_node.manual_thrust_limit if ros_node else 0
    return Response(CONSOLE_PAGE_HTML.replace('__MANUAL_THRUST_LIMIT__', '%g' % limit),
                    mimetype='text/html')


@app.route('/data')
def data_page():
    """Old Data Collection address. Recording is a tab of the console now."""
    return redirect('/#recording')


def _await(predicate, timeout_sec=CONTROL_ACK_TIMEOUT_SEC):
    """Wait until predicate() is true or the timeout expires.

    The control routes publish and return immediately, so a 200 would only
    mean "sent". Waiting for the state the ROS callbacks update lets the
    route report what actually happened. Blocking is fine because Flask runs
    in its own thread while rclpy.spin() owns the main thread.

    Returns True if the predicate held before the timeout.
    """
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(CONTROL_ACK_POLL_SEC)
    return predicate()


def _snapshot():
    with telemetry_lock:
        return latest_state, latest_estop, latest_mode


def _note_pack_voltage(voltage_v, now=None):
    """Track the highest pack voltage over the last minute.

    Voltage sags under load, so the peaks (lowest current) are a better
    estimate of the rested voltage than a single sample. Still only an
    estimate.

    Returns the rested voltage estimate.
    """
    global _pack_voltage_samples
    now = time.monotonic() if now is None else now
    _pack_voltage_samples.append((now, voltage_v))
    cutoff = now - RESTED_VOLTAGE_WINDOW_SEC
    _pack_voltage_samples = [s for s in _pack_voltage_samples if s[0] >= cutoff]
    return max(v for _t, v in _pack_voltage_samples)


def _power_snapshot():
    """Build the /telemetry power block, including the reserve verdict."""
    with telemetry_lock:
        channels = list(latest_power.get('channels') or [])
        seen = bool(latest_power.get('seen'))

    # Use the 'battery' channel, or the first channel that read OK.
    pack = None
    for ch in channels:
        if ch['name'] == 'battery' and ch['ok']:
            pack = ch
            break
    if pack is None:
        pack = next((c for c in channels if c['ok']), None)

    rested = None
    frac = None
    state = 'UNKNOWN'
    if pack is not None and pack['voltage_v'] > 0:
        rested = _note_pack_voltage(pack['voltage_v'])
        frac, state = battery_reserve(rested)

    return {
        'seen': seen,
        'channels': channels,
        'pack_channel': pack['name'] if pack else None,
        'pack_voltage_v': round(pack['voltage_v'], 2) if pack else None,
        'pack_rested_v': round(rested, 2) if rested is not None else None,
        'reserve_frac': round(frac, 3) if frac is not None else None,
        'reserve_state': state,
        # Caveat is part of the payload so it travels with the number.
        'method': 'voltage-derived from a 60s rolling maximum; unreliable '
                  'under varying load. Coulomb counting is not '
                  'implemented.',
    }


@app.route('/thrust', methods=['POST'])
@app.route('/control_mode', methods=['POST'])
def legacy_control():
    """Never acknowledge commands the production arbiter cannot accept."""
    return jsonify(status='error', error='Legacy control disabled; reload the dashboard and use /operator'), 410


@app.route('/operator/prepare', methods=['POST'])
@app.route('/operator', methods=['POST'])
def operator_command():
    """An atomic, sequenced command. Success means the arbiter acknowledged it."""
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify(status='error', error='Expected a JSON object'), 400
    session, seq = data.get('session'), data.get('seq')
    if (not isinstance(session, str) or not 1 <= len(session) <= 80
            or type(seq) is not int or seq < 0
            or data.get('intent') not in ('MANUAL', 'AUTO', 'HOLD', 'ESTOP', 'RESET')
            or type(data.get('transition')) is not bool):
        return jsonify(status='error', error='Invalid operator command'), 400
    if not ros_node:
        return jsonify(status='error', error='ROS node not ready'), 503
    try:
        left, right = float(data.get('left', 0)), float(data.get('right', 0))
        if not all(math.isfinite(v) for v in (left, right)):
            raise ValueError()
    except (ValueError, TypeError):
        return jsonify(status='error', error='Invalid thrust'), 400
    limit = ros_node.manual_thrust_limit
    data['left'] = max(-limit, min(limit, left))
    data['right'] = max(-limit, min(limit, right))
    # Each transport attempt gets its own receipt. A duplicate sequence is
    # rejected by the arbiter, never reported as a fresh cached success.
    if not isinstance(data.get('ticket', ''), str) or not isinstance(data.get('authority', ''), str):
        return jsonify(status='error', error='Invalid control authorization'), 400
    key = uuid.uuid4().hex
    data['request_id'] = key
    data['prepare'] = request.path == '/operator/prepare'
    ros_node.operator_pub.publish(String(data=json.dumps(data)))
    def receipt_ready():
        with telemetry_lock:
            return key in operator_receipts
    if not _await(receipt_ready):
        return jsonify(status='error', error='No control acknowledgement. Control will hold on heartbeat timeout.'), 504
    with telemetry_lock:
        result = operator_receipts.get(key, {})
    return jsonify(status='ok' if result.get('accepted') else 'error',
                   error=result.get('reason'), operator=result), (200 if result.get('accepted') else 409)


def _mission_snapshot():
    with telemetry_lock:
        if latest_mission_time is None or time.monotonic() - latest_mission_time > 2.0:
            return None
        return dict(latest_mission)


def _mission_receipt(request_id):
    def ready():
        with telemetry_lock:
            return request_id in mission_receipts
    if not _await(ready):
        return jsonify(status='error', error='Mission acknowledgement timed out; keep Hold and refresh status'), 504
    with telemetry_lock:
        result = mission_receipts[request_id]
    return jsonify(status='ok' if result.get('accepted') else 'error',
                   error=result.get('reason'), execution_id=result.get('execution_id'),
                   state=result.get('state')), (200 if result.get('accepted') else 409)


@app.route('/mission', methods=['POST'])
def mission():
    """Accept only an individually identified route, never infer from EXECUTE."""
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify(status='error', error='Expected a JSON object'), 400
    latitudes, longitudes = data.get('latitudes'), data.get('longitudes')
    if (not isinstance(latitudes, list) or not isinstance(longitudes, list) or
            not 1 <= len(latitudes) <= 1000 or len(latitudes) != len(longitudes)):
        return jsonify(status='error', error='Empty, oversized or mismatched waypoint arrays'), 400
    try:
        latitudes, longitudes = list(map(float, latitudes)), list(map(float, longitudes))
        if (not all(math.isfinite(v) and -90 <= v <= 90 for v in latitudes) or
                not all(math.isfinite(v) and -180 <= v <= 180 for v in longitudes)):
            raise ValueError()
    except (ValueError, TypeError, OverflowError):
        return jsonify(status='error', error='Invalid coordinates'), 400
    status = _mission_snapshot()
    if not ros_node or status is None:
        return jsonify(status='error', error='Fresh mission-manager status required'), 503
    request_id = uuid.uuid4().hex
    ros_node.publish_mission(latitudes, longitudes, request_id, status)
    return _mission_receipt(request_id)


@app.route('/mission/cancel', methods=['POST'])
def mission_cancel():
    """A delayed cancel cannot cancel a different execution."""
    data = request.get_json(silent=True)
    if not isinstance(data, dict) or not isinstance(data.get('execution_id'), str):
        return jsonify(status='error', error='Execution identity is required'), 400
    status = _mission_snapshot()
    if not ros_node or status is None:
        return jsonify(status='error', error='Fresh mission-manager status required'), 503
    if data['execution_id'] != status.get('execution_id'):
        return jsonify(status='error', error='Mission changed; refresh before cancelling'), 409
    request_id = uuid.uuid4().hex
    ros_node.publish_mission_cancel(request_id, status)
    return _mission_receipt(request_id)


@app.route('/estop', methods=['POST'])
def estop():
    data = request.get_json(silent=True) or {}
    if not isinstance(data, dict) or data.get('active', True) is not True:
        return jsonify(status='error', error='Use the sequenced Reset to Hold control'), 400
    active = True
    if not ros_node:
        return jsonify({'status': 'error', 'error': 'ROS node not ready'}), 503
    ros_node.publish_estop(active)
    # boat/estop echoes back through our own subscription, which confirms
    # the round trip.
    if not _await(lambda: _snapshot()[1] == active):
        return jsonify({
            'status': 'error',
            'error': 'e-stop published but the state did not change',
            'estop': _snapshot()[1]}), 504
    return jsonify({'status': 'ok', 'estop': active})


@app.route('/geofence', methods=['POST'])
def geofence():
    """Set, move, resize, enable or disable the geofence at runtime.

    Publishes JSON on boat/geofence_command. safety_monitor applies it and
    echoes it on boat/data_events so it is recorded. Validated here for a
    readable error and again in safety_monitor, which must not trust the
    topic.
    """
    data = request.get_json(silent=True) or {}
    action = str(data.get('action', '')).strip()
    valid = ('set', 'set_here', 'enable', 'disable',
             'set_launch', 'set_launch_here')
    if action not in valid:
        return jsonify({'status': 'error',
                        'error': 'action must be one of %s' % (valid,)}), 400

    cmd = {'action': action}

    if action in ('set', 'set_here'):
        try:
            radius = float(data.get('radius_m', 150.0))
            warn = float(data.get('warn_fraction', 0.9))
        except (TypeError, ValueError):
            return jsonify({'status': 'error',
                            'error': 'radius_m/warn_fraction must be numbers'}), 400
        if not math.isfinite(radius) or radius < 5.0:
            return jsonify({'status': 'error',
                            'error': 'radius_m must be finite and at least 5 m'}), 400
        if not math.isfinite(warn) or not (0.0 < warn < 1.0):
            return jsonify({'status': 'error',
                            'error': 'warn_fraction must be between 0 and 1'}), 400
        cmd['radius_m'] = radius
        cmd['warn_fraction'] = warn
        if action == 'set':
            try:
                cmd['center_lat'] = float(data['center_lat'])
                cmd['center_lon'] = float(data['center_lon'])
            except (KeyError, TypeError, ValueError):
                return jsonify({'status': 'error',
                                'error': 'set requires numeric center_lat/center_lon'}), 400

    if action == 'set_launch':
        try:
            cmd['lat'] = float(data['lat'])
            cmd['lon'] = float(data['lon'])
        except (KeyError, TypeError, ValueError):
            return jsonify({'status': 'error',
                            'error': 'set_launch requires numeric lat/lon'}), 400

    if action == 'disable':
        # Disabling requires a reason.
        reason = str(data.get('reason', '')).strip()
        if not reason:
            return jsonify({'status': 'error',
                            'error': 'disabling the geofence requires a reason, '
                                     'which is recorded into the run data'}), 400
        cmd['reason'] = reason

    if not ros_node:
        return jsonify({'status': 'error', 'error': 'ROS node not ready'}), 503
    ros_node.publish_geofence_command(cmd)

    # No ack wait here. The fence drawn from SafetyStatus on the next
    # telemetry tick shows what is actually enforced.
    return jsonify({'status': 'ok', 'command': cmd})


@app.route('/failsafe/clear', methods=['POST'])
def failsafe_clear():
    """Operator failsafe de-escalation.

    safety_monitor refuses a normal clear while the condition is still
    active. force overrides that and needs a reason, which is recorded.
    """
    data = request.get_json(silent=True) or {}
    force = bool(data.get('force', False))
    payload = {'force': force}
    if force:
        reason = str(data.get('reason', '')).strip()
        if not reason:
            return jsonify({'status': 'error',
                            'error': 'a forced clear requires a typed reason, '
                                     'which is recorded into the run data'}), 400
        payload['reason'] = reason
    if not ros_node:
        return jsonify({'status': 'error', 'error': 'ROS node not ready'}), 503
    ros_node.publish_failsafe_clear(payload)

    # Wait for the level to drop so a refused clear is reported as a failure.
    if not force:
        if not _await(lambda: (latest_failsafe or {}).get('level', 0) == 0):
            level = (latest_failsafe or {}).get('level_name', '?')
            reasons = (latest_failsafe or {}).get('active_reasons') or []
            return jsonify({
                'status': 'error',
                'error': 'failsafe is still at %s -- %s'
                         % (level, reasons[0] if reasons else
                            'safety_monitor refused the clear; check its log'),
            }), 409
    return jsonify({'status': 'ok'})


@app.route('/telemetry')
def telemetry():
    # Each poll is also the ground-control heartbeat that safety_monitor
    # watches for link loss. It only ticks if browser, server, ROS and
    # network all work.
    if ros_node:
        ros_node.publish_gcs_heartbeat()
    with telemetry_lock:
        return jsonify({
            'lat': latest_gps['lat'],
            'lon': latest_gps['lon'],
            'state': latest_state,
            'mode': latest_mode,
            'operator': dict(latest_operator),
            'mission': dict(latest_mission),
            'operator_age_s': (time.monotonic() - latest_operator_time
                               if latest_operator_time is not None else None),
            'geojson': latest_geojson,
            'heading': latest_imu['heading'],
            'roll': latest_imu['roll'],
            'pitch': latest_imu['pitch'],
            'nav': latest_nav,
            'safety': latest_safety,
            'estop': latest_estop,
            'dock': dict(dock_point),
            'recording': dict(latest_recording, status_fresh=bool(latest_recording_time is not None and time.monotonic()-latest_recording_time < 3.0)),
            'dock_marker': latest_dock_marker,
            # Each is an empty dict when its publisher is absent, and the
            # page shows that as "no data".
            'failsafe': latest_failsafe,
            'heading_est': latest_heading,
            'camera': latest_camera,
            'lidar': dict(latest_lidar, fresh=bool(latest_lidar.get('fresh') and latest_lidar_time is not None and time.monotonic()-latest_lidar_time < 2.0)),
            'power': _power_snapshot(),
            'system': latest_system,
        })


@app.route('/lidar/scan')
def lidar_scan_view():
    """Top-down preview points for the map's LiDAR layer. Display only.

    Kept out of /telemetry so the control poll stays small. The page asks
    for this only while the LiDAR layer is switched on.
    """
    with telemetry_lock:
        return jsonify(lidar_scan.snapshot())


def _queue_data_command(cmd):
    if ros_node is None:
        return jsonify({'status': 'error', 'error': 'ROS connection unavailable'}), 503
    ros_node.publish_data_command(cmd)
    return jsonify({'status': 'queued', 'confirmed': False,
                    'detail': 'Request dispatched; check fresh recorder status for outcome'}), 202


@app.route('/data/start', methods=['POST'])
def data_start():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({'status': 'error', 'error': 'JSON object required'}), 400
    cmd = {
        'action': 'start',
        'site': data.get('site') or 'unknown',
        'purpose': data.get('purpose', ''),
        'source': data.get('source') or 'unknown',
    }
    return _queue_data_command(cmd)


@app.route('/data/stop', methods=['POST'])
def data_stop():
    return _queue_data_command({'action': 'stop'})


@app.route('/data/save_name', methods=['POST'])
def data_save_name():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({'status': 'error', 'error': 'JSON object required'}), 400
    name, run_id = data.get('name'), data.get('run_id')
    if (not isinstance(name, str) or not 1 <= len(name.strip()) <= 80
            or any(ord(c) < 32 for c in name)
            or not isinstance(run_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,160}', run_id)):
        return jsonify({'status': 'error', 'error': 'Valid recording ID and name (1 to 80 characters) required'}), 400
    return _queue_data_command({'action': 'save_name', 'run_id': run_id, 'name': name.strip()})


@app.route('/data/marker', methods=['POST'])
def data_marker():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({'status': 'error', 'error': 'JSON object required'}), 400
    return _queue_data_command({'action': 'marker', 'note': data.get('note', '')})


@app.route('/detect/enable', methods=['POST'])
def detect_enable():
    """Turn detector_node inference on or off (not just the displayed feed)."""
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({'status': 'error', 'error': 'JSON object required'}), 400
    enabled = bool(data.get('enabled', True))
    if ros_node:
        ros_node.publish_detect_enable(enabled)
    return jsonify({'status': 'ok'})


@app.route('/data/segment_begin', methods=['POST'])
def data_segment_begin():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({'status': 'error', 'error': 'JSON object required'}), 400
    label = data.get('label', '')
    # step_id is left for the recorder to generate.
    return _queue_data_command({'action': 'segment_begin', 'label': label})


@app.route('/data/segment_end', methods=['POST'])
def data_segment_end():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({'status': 'error', 'error': 'JSON object required'}), 400
    step_id = data.get('step_id')
    if not step_id:
        return jsonify({'status': 'error', 'error': 'step_id required'}), 400
    return _queue_data_command({'action': 'segment_end', 'step_id': step_id})


@app.route('/dock', methods=['POST'])
def set_dock():
    """Move the dock position, from the Set Dock button or a map right-click."""
    data = request.get_json(silent=True) or {}
    try:
        lat = float(data.get('lat'))
        lon = float(data.get('lon'))
    except (TypeError, ValueError):
        return jsonify({'status': 'error', 'error': 'lat/lon must be numbers'}), 400
    if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
        return jsonify({'status': 'error', 'error': 'lat/lon out of range'}), 400
    with telemetry_lock:
        dock_point['lat'] = lat
        dock_point['lon'] = lon
        # Set by hand, so stop auto-capturing from GPS.
        dock_point['auto'] = False
    if ros_node:
        ros_node.get_logger().info('Dock relocated to %.6f, %.6f' % (lat, lon))
    return jsonify({'status': 'ok'})


def _encode_jpeg(frame):
    ok, data = cv2.imencode('.jpg', frame)
    if not ok:
        raise ValueError('JPEG encoding failed')
    return data.tobytes()


def _stale_jpeg():
    frame = np.zeros((180, 480, 3), dtype=np.uint8)
    cv2.putText(frame, 'NO FRESH VIDEO', (35, 95), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (180, 180, 220), 2)
    return _encode_jpeg(frame)


video_stream = SharedVideo(_encode_jpeg, _stale_jpeg())
detections_stream = SharedVideo(_encode_jpeg, _stale_jpeg(), max_hz=5.)


def generate_frames():
    yield from video_stream.stream()


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


def generate_detections_frames():
    yield from detections_stream.stream()


@app.route('/detections_feed')
def detections_feed():
    return Response(generate_detections_frames(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server_node')
        self.declare_parameter('manual_thrust_limit', 100.0)
        self.manual_thrust_limit = float(self.get_parameter('manual_thrust_limit').value)
        if not math.isfinite(self.manual_thrust_limit) or not 0 <= self.manual_thrust_limit <= 100:
            raise ValueError('manual_thrust_limit must be between 0 and 100')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'boat/camera/image_raw', self.image_callback, 10)
        # Only publishes when detector_node is running.
        self.detections_subscription = self.create_subscription(
            Image, 'boat/perception/detections_image',
            self.detections_image_callback, 10)
        # Manual thrust goes to boat/cmd/manual. control_arbiter owns
        # boat/thrust_command.
        self.thrust_pub = self.create_publisher(ThrustCommand, 'boat/cmd/manual', 10)
        self.operator_pub = self.create_publisher(String, 'boat/operator_command', 10)
        self.create_subscription(String, 'boat/operator_status', self.operator_callback, 10)
        self.create_subscription(String, 'boat/mission_status', self.mission_status_callback, 10)
        self.mission_pub = self.create_publisher(MissionCommand, 'boat/mission_command', 10)
        self.estop_pub = self.create_publisher(Bool, 'boat/estop', 10)
        # Manual lease: MANUAL / RELEASE / RESUME, see boat_control/arbitration.py.
        self.control_mode_pub = self.create_publisher(
            String, 'boat/cmd/control_mode', 10)
        # Operator mission cancel.
        self.mission_cancel_pub = self.create_publisher(
            String, 'boat/mission_cancel', 10)
        # data_recorder control, see publish_data_command().
        self.data_command_pub = self.create_publisher(String, 'boat/data_command', 10)
        # detector_node inference on/off.
        self.detect_enable_pub = self.create_publisher(Bool, 'boat/perception/detect_enable', 10)
        # Geofence changes and failsafe clears. JSON in a String because these
        # are low rate and the shape still changes.
        self.geofence_cmd_pub = self.create_publisher(
            String, 'boat/geofence_command', 10)
        self.failsafe_clear_pub = self.create_publisher(
            String, 'boat/failsafe_clear', 10)

        # Map telemetry, same topics Foxglove shows.
        self.gps_sub = self.create_subscription(
            NavSatFix, 'boat/gps/fix', self.gps_callback, 10)
        self.state_sub = self.create_subscription(
            String, 'boat/state', self.state_callback, 10)
        self.geojson_sub = self.create_subscription(
            GeoJSON, 'boat/mission_geojson', self.geojson_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'boat/imu', self.imu_callback, 10)
        self.nav_status_sub = self.create_subscription(
            NavStatus, 'boat/nav_status', self.nav_status_callback, 10)
        self.safety_status_sub = self.create_subscription(
            SafetyStatus, 'boat/safety_status', self.safety_status_callback, 10)
        self.estop_state_sub = self.create_subscription(
            Bool, 'boat/estop', self.estop_state_callback, 10)
        # Source control_arbiter is selecting.
        self.mode_sub = self.create_subscription(
            String, 'boat/mode', self.mode_callback, 10)
        self.recording_status_sub = self.create_subscription(
            String, 'boat/recording_status', self.recording_status_callback, 10)
        # Failsafe level for the banner and the Safety box.
        self.failsafe_sub = self.create_subscription(
            FailsafeState, 'boat/failsafe_state', self.failsafe_callback, 10)
        # Fused heading with quality. imu_callback is kept only for roll and
        # pitch.
        self.heading_sub = self.create_subscription(
            HeadingEstimate, 'boat/heading', self.heading_callback, 10)
        # Camera status, see CameraStatus.msg.
        self.camera_status_sub = self.create_subscription(
            CameraStatus, 'boat/camera/status', self.camera_status_callback, 10)
        # Battery and current readings from power_monitor.
        self.create_subscription(String, 'boat/lidar/status', self.lidar_status_callback, 10)
        self.create_subscription(PointCloud2, '/boat/lidar/points_preview',
                                 self.lidar_points_callback, qos_profile_sensor_data)
        self.power_sub = self.create_subscription(
            PowerStatus, 'boat/power_status', self.power_callback, 10)
        self.system_sub = self.create_subscription(
            String, 'boat/system_status', self.system_status_callback, 10)
        # Only publishes when dock_marker_node is running.
        self.dock_marker_detected_sub = self.create_subscription(
            Bool, 'boat/dock/marker_detected', self.dock_marker_detected_callback, 10)
        self.dock_marker_pose_sub = self.create_subscription(
            PoseStamped, 'boat/dock/marker_pose', self.dock_marker_pose_callback, 10)
        # Ground-control heartbeat for safety_monitor, see /telemetry.
        self.gcs_heartbeat_pub = self.create_publisher(Empty, 'boat/gcs_heartbeat', 10)

        self.get_logger().info('Web dashboard node started, dashboard at :5000')

    def mission_status_callback(self, msg):
        global latest_mission, latest_mission_time
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
        except (ValueError, TypeError):
            return
        with telemetry_lock:
            latest_mission = data
            latest_mission_time = time.monotonic()
            key = data.get('request_id')
            if isinstance(key, str):
                mission_receipts[key] = data
                while len(mission_receipts) > 256:
                    del mission_receipts[next(iter(mission_receipts))]

    def operator_callback(self, msg):
        global latest_operator, latest_operator_time
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
        except (ValueError, TypeError):
            return
        with telemetry_lock:
            latest_operator = data
            latest_operator_time = time.monotonic()
            if isinstance(data.get('request_id'), str):
                operator_receipts[data['request_id']] = data
                while len(operator_receipts) > 256:
                    del operator_receipts[next(iter(operator_receipts))]

    def _video_input(self, msg, stream):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            stream.put(frame)
        except Exception as exc:
            self.get_logger().warn(f'Invalid video frame: {exc}', throttle_duration_sec=5.0)

    def image_callback(self, msg):
        self._video_input(msg, video_stream)

    def detections_image_callback(self, msg):
        self._video_input(msg, detections_stream)

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:  # STATUS_NO_FIX
            return
        # Ignore (0, 0) so the dock is never captured at null island.
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        with telemetry_lock:
            latest_gps['lat'] = msg.latitude
            latest_gps['lon'] = msg.longitude
            if dock_point['auto'] and dock_point['lat'] is None:
                dock_point['lat'] = msg.latitude
                dock_point['lon'] = msg.longitude
                self.get_logger().info(
                    'Dock auto-set to first GPS fix: %.6f, %.6f'
                    % (msg.latitude, msg.longitude))

    def state_callback(self, msg: String):
        global latest_state
        with telemetry_lock:
            latest_state = msg.data

    def imu_callback(self, msg: Imu):
        heading, roll, pitch = imu_orientation_deg(msg.orientation)
        with telemetry_lock:
            latest_imu['heading'] = heading
            latest_imu['roll'] = roll
            latest_imu['pitch'] = pitch

    def nav_status_callback(self, msg: NavStatus):
        # Converted to a dict (NaN to None) for /telemetry.
        global latest_nav
        with telemetry_lock:
            latest_nav = {
                'active': msg.active,
                'waypoint_index': msg.waypoint_index,
                'waypoint_count': msg.waypoint_count,
                'speed_mps': msg.speed_mps,
                'target_lat': _nan_to_none(msg.target_lat),
                'target_lon': _nan_to_none(msg.target_lon),
                'distance_m': _nan_to_none(msg.distance_m),
                'remaining_m': _nan_to_none(msg.remaining_m),
                'bearing_deg': _nan_to_none(msg.bearing_deg),
                'heading_error_deg': _nan_to_none(msg.heading_error_deg),
                'eta_sec': _nan_to_none(msg.eta_sec),
                'gps_age_sec': _nan_to_none(msg.gps_age_sec),
            }

    def safety_status_callback(self, msg: SafetyStatus):
        # level is safety_monitor's own OK/WARN/ERROR (0/1/2), passed through.
        global latest_safety
        with telemetry_lock:
            latest_safety = {
                'level': msg.level,
                'tripped': msg.tripped,
                'reason': msg.reason or None,
                'fence_enabled': msg.fence_enabled,
                'fence_center_lat': msg.fence_center_lat,
                'fence_center_lon': msg.fence_center_lon,
                'fence_radius_m': msg.fence_radius_m,
                'fence_distance_m': _nan_to_none(msg.fence_distance_m),
                'fence_margin_m': _nan_to_none(msg.fence_margin_m),
                'fence_breached': msg.fence_breached,
                'fence_warning': msg.fence_warning,
                'gcs_lost': msg.gcs_lost,
                'gcs_heartbeat_age_sec': _nan_to_none(msg.gcs_heartbeat_age_sec),
                'fence_warn_radius_m': _nan_to_none(msg.fence_warn_radius_m),
                'fence_warn_fraction': _nan_to_none(msg.fence_warn_fraction),
                'fence_outside_sec': _nan_to_none(msg.fence_outside_sec),
                'fence_disabled_reason': msg.fence_disabled_reason or None,
            }

    def failsafe_callback(self, msg: FailsafeState):
        global latest_failsafe
        with telemetry_lock:
            latest_failsafe = {
                'level': msg.level,
                'level_name': msg.level_name,
                'reason': msg.reason or None,
                'active_reasons': list(msg.active_reasons),
                'requested_level': msg.requested_level,
                # Why the boat is doing something other than what it asked
                # for, shown in the banner.
                'degraded_reason': msg.degraded_reason or None,
                'commanding_thrust': msg.commanding_thrust,
                'commanded_left': msg.commanded_left,
                'commanded_right': msg.commanded_right,
                'launch_lat': _nan_to_none(msg.launch_lat),
                'launch_lon': _nan_to_none(msg.launch_lon),
                'launch_point_valid': msg.launch_point_valid,
                'distance_to_launch_m': _nan_to_none(msg.distance_to_launch_m),
                'bearing_to_launch_deg': _nan_to_none(msg.bearing_to_launch_deg),
                'position_valid': msg.position_valid,
                'heading_usable': msg.heading_usable,
                'heading_quality': msg.heading_quality,
                'latched': msg.latched,
                'seconds_at_level': msg.seconds_at_level,
                # Derived here so the banner and Safety box agree.
                'conditions_clear': bool(
                    msg.reason == 'LATCHED'
                    or (msg.level > 0 and not msg.reason)),
            }

    def heading_callback(self, msg: HeadingEstimate):
        global latest_heading
        with telemetry_lock:
            latest_heading = {
                'quality': msg.quality,
                'heading_deg': _nan_to_none(msg.heading_deg),
                'source': msg.source or None,
                'confidence': msg.confidence,
                'age_since_reference_sec': _nan_to_none(msg.age_since_reference_sec),
                'gyro_bias_dps': msg.gyro_bias_dps,
                'cog_valid': msg.cog_valid,
                'ground_speed_mps': _nan_to_none(msg.ground_speed_mps),
                'cog_deg': _nan_to_none(msg.cog_deg),
                'absolute_reference_available': msg.absolute_reference_available,
                'detail': msg.detail or None,
            }

    def lidar_status_callback(self, msg):
        global latest_lidar, latest_lidar_time
        try:
            value = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(value, dict):
            return
        with telemetry_lock:
            latest_lidar = value
            latest_lidar_time = time.monotonic()

    def lidar_points_callback(self, msg):
        with telemetry_lock:
            lidar_scan.put(msg)

    def camera_status_callback(self, msg: CameraStatus):
        global latest_camera
        with telemetry_lock:
            latest_camera = {
                'source': msg.source,
                'mock_mode': msg.mock_mode,
                'healthy': msg.healthy,
                'detail': msg.detail or None,
                'width': msg.width,
                'height': msg.height,
                'measured_fps': msg.measured_fps,
                'configured_fps': msg.configured_fps,
                'frames_published': msg.frames_published,
                'read_errors': msg.read_errors,
                'calibration_measured': msg.calibration_measured,
                'calibration_source': msg.calibration_source or None,
            }

    def power_callback(self, msg: PowerStatus):
        global latest_power
        with telemetry_lock:
            latest_power = {
                'seen': True,
                'channels': [{
                    'name': c.name,
                    'voltage_v': c.voltage_v,
                    'current_a': c.current_a,
                    'power_w': c.power_w,
                    # Keep "read failed" distinct from a real 0.0 A.
                    'ok': c.ok,
                } for c in msg.channels],
            }

    def system_status_callback(self, msg: String):
        global latest_system
        try:
            parsed = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        with telemetry_lock:
            latest_system = parsed

    def mode_callback(self, msg: String):
        global latest_mode
        with telemetry_lock:
            latest_mode = msg.data

    def estop_state_callback(self, msg: Bool):
        global latest_estop
        with telemetry_lock:
            latest_estop = bool(msg.data)

    def recording_status_callback(self, msg: String):
        global latest_recording, latest_recording_time
        try:
            parsed = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        with telemetry_lock:
            latest_recording = parsed
            latest_recording_time = time.monotonic()

    def dock_marker_detected_callback(self, msg: Bool):
        global latest_dock_marker
        with telemetry_lock:
            latest_dock_marker['detected'] = bool(msg.data)
            if not msg.data:
                # Not detected, drop the stale distance and offset.
                latest_dock_marker['distance_m'] = None
                latest_dock_marker['lateral_m'] = None

    def dock_marker_pose_callback(self, msg: PoseStamped):
        # A pose means the marker is detected.
        global latest_dock_marker
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        with telemetry_lock:
            latest_dock_marker['detected'] = True
            latest_dock_marker['distance_m'] = round(math.sqrt(x * x + y * y + z * z), 2)
            latest_dock_marker['lateral_m'] = round(x, 2)

    def publish_gcs_heartbeat(self):
        self.gcs_heartbeat_pub.publish(Empty())

    def geojson_callback(self, msg: GeoJSON):
        global latest_geojson
        try:
            parsed = json.loads(msg.geojson)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn('Received malformed boat/mission_geojson message, ignoring')
            return
        with telemetry_lock:
            latest_geojson = parsed

    def publish_thrust(self, left, right):
        """Publish a manual thrust command clamped to +/-100.

        The /thrust route already rejects non-finite values.
        """
        msg = ThrustCommand()
        msg.left = max(-self.manual_thrust_limit, min(self.manual_thrust_limit, float(left)))
        msg.right = max(-self.manual_thrust_limit, min(self.manual_thrust_limit, float(right)))
        self.thrust_pub.publish(msg)

    def publish_control_mode(self, mode):
        self.control_mode_pub.publish(String(data=str(mode)))

    def publish_mission_cancel(self, request_id, status):
        self.mission_cancel_pub.publish(String(data=json.dumps({
            'request_id': request_id, 'execution_id': status['execution_id'],
            'manager_id': status['manager_id'], 'generation': status['generation']})))

    def publish_mission(self, latitudes, longitudes, execution_id, status):
        msg = MissionCommand()
        msg.execution_id = execution_id
        msg.manager_id = status['manager_id']
        msg.generation = status['generation']
        msg.latitudes = [float(x) for x in latitudes]
        msg.longitudes = [float(x) for x in longitudes]
        self.mission_pub.publish(msg)

    def publish_estop(self, active):
        msg = Bool()
        msg.data = bool(active)
        self.estop_pub.publish(msg)

    def publish_data_command(self, command_dict):
        """Publish a data_recorder command, e.g. {"action": "start", ...}.
        See boat_data/data_recorder.py for the actions."""
        msg = String()
        msg.data = json.dumps(command_dict)
        self.data_command_pub.publish(msg)

    def publish_detect_enable(self, enabled):
        self.detect_enable_pub.publish(Bool(data=bool(enabled)))

    def publish_geofence_command(self, command_dict):
        """See safety_monitor.geofence_command_callback for the actions."""
        self.geofence_cmd_pub.publish(String(data=json.dumps(command_dict)))

    def publish_failsafe_clear(self, payload):
        self.failsafe_clear_pub.publish(String(data=json.dumps(payload)))


def get_reachable_ips():
    """Best effort list of this machine's LAN IPs, printed at startup so
    the dashboard URL is easy to find when the network changes. Never raises.
    """
    ips = []

    # Primary: hostname -I (Linux).
    try:
        result = subprocess.run(
            ['hostname', '-I'],
            capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            for token in result.stdout.split():
                if token not in ips:
                    ips.append(token)
    except (OSError, subprocess.SubprocessError):
        pass

    # Fallback: the local address a UDP socket would use. connect() on UDP
    # sends nothing, so this works without internet.
    if not ips:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(('8.8.8.8', 80))
                ip = s.getsockname()[0]
                if ip:
                    ips.append(ip)
        except OSError:
            pass

    return ips


def run_flask():
    print('ASV dashboard: http://127.0.0.1:5000; remote access requires SSH forwarding.')
    app.run(host='127.0.0.1', port=5000, threaded=True)


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = WebServerNode()

    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(ros_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ros_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
