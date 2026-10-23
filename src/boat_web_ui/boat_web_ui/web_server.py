import json
import math
import socket
import subprocess
import threading
import time
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix, Imu
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
from flask import Flask, Response, render_template_string, request, jsonify


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

# Pack defaults. These assume a 4S LiPo, but the boat runs a 3S pack, so a
# healthy 3S pack reads as BELOW_RESERVE. TODO: set DEFAULT_CELLS = 3 or make
# it a parameter. 4.20 V/cell is full, 3.30 V/cell is the damage floor.
DEFAULT_CELLS = 4
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


# Shared nav bar and failsafe banner. Both pages are plain strings, so the
# shared parts are spliced in at import time by replacing sentinel comments.
# str.format() is not used because the pages are full of CSS/JS braces.

NAV_CSS = """
        /* Navigation bar. Large targets, active page marked by underline,
           background and text (not colour alone), no hover-only controls. */
        .nav {
            display: flex; align-items: stretch; gap: 0; flex-shrink: 0;
            border-bottom: 1px solid var(--line); background: #12151a;
        }
        .nav a {
            display: flex; align-items: center; gap: 7px;
            padding: 10px 20px; text-decoration: none;
            color: var(--dim); font-size: 13px; letter-spacing: 2.6px;
            text-transform: uppercase; font-weight: 600;
            border-right: 1px solid var(--line);
            border-bottom: 3px solid transparent;
            transition: background .12s, color .12s;
        }
        .nav a:hover { background: #1c2128; color: var(--txt); }
        .nav a.active {
            color: var(--acc); background: #221b12;
            border-bottom-color: var(--acc);
        }
        .nav a .nav-ico { font-size: 15px; line-height: 1; }
        .nav .nav-spacer { flex: 1; border-right: none; }
        .nav .nav-note {
            display: flex; align-items: center; padding: 0 16px;
            color: #5d666f; font-size: 10px; letter-spacing: 2px;
        }

        /* Failsafe banner. Full width, sticky, always colour plus a word
           plus a glyph, with the reason in words. */
        .banner {
            display: flex; align-items: center; gap: 14px; flex-shrink: 0;
            padding: 9px 18px; border-bottom: 1px solid var(--line);
            background: #16311a; color: #cfe8c4;
            font-size: 15px; letter-spacing: 2px; font-weight: 700;
            text-transform: uppercase;
        }
        .banner .b-glyph { font-size: 19px; line-height: 1; }
        .banner .b-why {
            font-weight: 400; letter-spacing: 1.1px; font-size: 12px;
            text-transform: none; opacity: 0.92;
            overflow: hidden; text-overflow: ellipsis; white-space: nowrap;
        }
        .banner .b-act { margin-left: auto; display: flex; gap: 7px; }
        .banner button {
            font-size: 10px; padding: 7px 13px; letter-spacing: 1.6px;
        }
        .banner.ok   { background: #16311a; color: #cfe8c4; border-bottom-color: #2f6135; }
        .banner.warn { background: #3a2a10; color: #f3d5a4; border-bottom-color: #7a4e1c; }
        .banner.bad  { background: #4a1414; color: #ffd2d2; border-bottom-color: #8d2a2a;
                       animation: bannerpulse 1.6s ease-in-out infinite; }
        .banner.none { background: #2a2118; color: #e0c9a4; border-bottom-color: #6d5a2b; }
        @keyframes bannerpulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.78; } }
"""

# NAV_ACTIVE_* markers are replaced per page in _compose().
NAV_HTML = """
<nav class="nav">
    <a href="/" class="NAV_ACTIVE_CONTROL"><span class="nav-ico">&#9673;</span>Pilot</a>
    <a href="/data" class="NAV_ACTIVE_DATA"><span class="nav-ico">&#9679;</span>Data Collection</a>
    <span class="nav-spacer"></span>
    <span class="nav-note" id="nav_note">&mdash;</span>
</nav>
<div id="operator_notice" role="status" aria-live="polite" style="padding:10px 18px;white-space:normal">Control changes are confirmed by the arbiter. Hold means zero thrust, not GPS position-holding.</div>
<label style="display:block;padding:0 18px 10px">Safety override reason (only for override/disable): <input id="operator_reason" type="text" maxlength="240" placeholder="Enter a reason before overriding safety"></label>
<div class="banner none" id="fs_banner">
    <span class="b-glyph" id="fs_glyph">&#9679;</span>
    <span id="fs_word">WAITING FOR SUPERVISOR</span>
    <span class="b-why" id="fs_why">safety_monitor has not published boat/failsafe_state yet</span>
    <span class="b-act" id="fs_actions"></span>
</div>
"""

# Banner logic shared by both pages. Each page calls updateBanner(data)
# from its telemetry poll.
BANNER_JS = """
function notifyOperator(message) {
    const el = document.getElementById('operator_notice');
    if (el) el.textContent = message;
}
// Failsafe levels, mirroring boat_msgs/msg/FailsafeState.msg.
const FS_NOMINAL = 0, FS_HOLD = 1, FS_RTL = 2, FS_PSTOP = 3, FS_HARD = 4;

function updateBanner(data) {
    const el = document.getElementById('fs_banner');
    if (!el) return;
    const word = document.getElementById('fs_word');
    const why = document.getElementById('fs_why');
    const glyph = document.getElementById('fs_glyph');
    const actions = document.getElementById('fs_actions');
    const fs = data.failsafe || {};
    const safety = data.safety || {};

    // No supervisor gets its own state, never shown as all clear.
    if (fs.level === undefined || fs.level === null) {
        el.className = 'banner none';
        glyph.innerHTML = '&#9679;';
        word.innerText = 'NO SAFETY SUPERVISOR';
        why.innerText = 'safety_monitor is not publishing boat/failsafe_state '
            + '-- there is no geofence and no failsafe. Start it before launching.';
        actions.innerHTML = '';
        return;
    }

    let cls = 'ok', glyphHtml = '&#10003;', text = 'NOMINAL', reason = '';

    if (data.estop) {
        cls = 'bad'; glyphHtml = '&#9632;'; text = 'ENGINE STOP ENGAGED';
        reason = 'All propulsion cut. Reset to Hold, then choose the next action.';
    } else if ((data.operator || {}).failsafe_stale) {
        cls = 'bad'; glyphHtml = '&#9632;'; text = 'SAFETY LINK STALE';
        reason = 'Propulsion stopped: the safety supervisor stopped reporting.';
    } else if (fs.level >= FS_PSTOP) {
        cls = 'bad'; glyphHtml = '&#9632;'; text = 'POWERED STOP';
        reason = fs.degraded_reason
            ? ('Wanted ' + (fs.requested_level_name || 'a recovery') + ' but ' + fs.degraded_reason)
            : (fs.active_reasons && fs.active_reasons[0]) || fs.reason || '';
    } else if (fs.level === FS_RTL) {
        cls = 'bad'; glyphHtml = '&#8676;';
        text = data.mode === 'FAILSAFE' ? 'RETURN-TO-LAUNCH CONTROL' : 'RETURN-TO-LAUNCH REQUESTED';
        reason = ((fs.active_reasons && fs.active_reasons[0]) || fs.reason || '');
        if (fs.distance_to_launch_m !== null && fs.distance_to_launch_m !== undefined) {
            reason += '  ·  ' + fs.distance_to_launch_m.toFixed(0) + ' m to go';
        }
    } else if (fs.level === FS_HOLD) {
        cls = 'bad'; glyphHtml = '&#9632;'; text = 'HOLDING POSITION';
        reason = (fs.active_reasons && fs.active_reasons[0]) || fs.reason || '';
    } else if (safety.fence_enabled === false) {
        // A disabled geofence stays amber for as long as it lasts.
        cls = 'warn'; glyphHtml = '&#9888;'; text = 'GEOFENCE DISABLED';
        reason = safety.fence_disabled_reason
            ? ('Reason given: ' + safety.fence_disabled_reason)
            : 'No boundary is being enforced.';
    } else if (safety.fence_warning) {
        cls = 'warn'; glyphHtml = '&#9888;'; text = 'APPROACHING GEOFENCE';
        reason = (safety.fence_margin_m !== null && safety.fence_margin_m !== undefined)
            ? (safety.fence_margin_m.toFixed(0) + ' m of margin left before the boundary')
            : 'Inside the warning ring.';
    } else if (fs.heading_usable === false) {
        // Not a failsafe by itself, but it disables return-to-launch.
        cls = 'warn'; glyphHtml = '&#9888;'; text = 'NO USABLE HEADING';
        reason = 'Return-to-launch is unavailable and will degrade to a powered '
               + 'stop. ' + ((data.heading && data.heading.detail) || '');
    } else {
        reason = safety.fence_enabled
            ? ('Geofence armed'
               + ((safety.fence_margin_m !== null && safety.fence_margin_m !== undefined)
                  ? ('  ·  ' + safety.fence_margin_m.toFixed(0) + ' m margin') : ''))
            : '';
    }

    // Latched: conditions gone but the failsafe still holds. Offer clear.
    if (fs.level > FS_NOMINAL && fs.conditions_clear) {
        reason = 'Conditions have cleared. The failsafe stays latched until you clear it.';
    }

    el.className = 'banner ' + cls;
    glyph.innerHTML = glyphHtml;
    word.innerText = text;
    why.innerText = reason;

    if (fs.level > FS_NOMINAL && !data.estop) {
        if (actions.getAttribute('data-shown') !== '1') {
            actions.innerHTML =
                '<button onclick="clearFailsafe(false)">Clear Failsafe</button>' +
                '<button onclick="clearFailsafe(true)">Override</button>';
            actions.setAttribute('data-shown', '1');
        }
    } else if (actions.getAttribute('data-shown') === '1') {
        actions.innerHTML = '';
        actions.removeAttribute('data-shown');
    }
}

function clearFailsafe(force) {
    let body = {force: false};
    if (force) {
        // Force override needs a typed reason, recorded on boat/data_events.
        const reason = document.getElementById('operator_reason').value.trim();
        if (!reason) { notifyOperator('Enter a safety override reason first.'); return; }
        body = {force: true, reason: reason};
    }
    fetch('/failsafe/clear', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(body),
    }).then((r) => r.json()).then((d) => {
        if (d.status !== 'ok') notifyOperator('Clear failed: ' + (d.error || 'unknown error'));
    });
}

// Nav bar summary shown on both pages: recording and camera state.
function updateNavNote(data) {
    const el = document.getElementById('nav_note');
    if (!el) return;
    const rec = data.recording || {};
    const cam = data.camera || {};
    const parts = [];
    if (rec.recording) {
        parts.push(rec.writing === false ? 'REC · NOT WRITING' : 'REC');
    }
    if (cam.mock_mode) parts.push('CAMERA: MOCK');
    else if (cam.healthy === false) parts.push('CAMERA: UNHEALTHY');
    const sys = data.system || {};
    if (sys.cpu_temp_c !== null && sys.cpu_temp_c !== undefined) {
        parts.push(sys.cpu_temp_c.toFixed(0) + '\\u00B0C');
    }
    if (rec.free_gb !== null && rec.free_gb !== undefined) {
        parts.push(rec.free_gb.toFixed(0) + ' GB');
    } else if (sys.disk && sys.disk.free_gb !== undefined) {
        parts.push(sys.disk.free_gb.toFixed(0) + ' GB');
    }
    el.innerText = parts.length ? parts.join('  ·  ') : '\\u2014';
    el.style.color = (rec.recording && rec.writing === false) || cam.mock_mode
        ? 'var(--red)' : '#5d666f';
}
"""

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Ramberg Robotics | ASV Controller</title>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css" />
    <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
    <style>
        /* Graphite and amber palette. Sized to the viewport so the page
           never scrolls; camera and chart panes flex to fill the height. */
        :root {
            --bg: #15181c;
            --panel: #1d2126;
            --panel-hi: #252a31;
            --line: #343a42;
            --txt: #dcd8d0;
            --dim: #838d99;
            --acc: #e8933a;
            --grn: #8bb85f;
            --red: #cf4f4f;
            --cyn: #5fa6b8;
        }
        * { box-sizing: border-box; }
        html, body { height: 100%; }
        body {
            margin: 0; background: var(--bg); color: var(--txt);
            font-family: ui-monospace, 'Cascadia Code', 'Consolas', monospace;
            display: flex; flex-direction: column; overflow: hidden;
            background-image: repeating-linear-gradient(135deg,
                rgba(255,255,255,0.013) 0 2px, transparent 2px 7px);
        }

        /* ---- title bar ---- */
        .topbar {
            display: flex; align-items: center; gap: 16px; flex-shrink: 0;
            padding: 7px 14px; border-bottom: 1px solid var(--line);
            background: linear-gradient(180deg, #262b32 0%, #1a1e23 100%);
        }
        .brand { display: flex; align-items: center; gap: 9px; }
        .mark { width: 27px; height: 27px; flex-shrink: 0; }
        .brand-txt b {
            display: block; color: var(--acc); font-size: 14px;
            letter-spacing: 4.5px; line-height: 1.1; font-weight: 700;
        }
        .brand-txt span {
            display: block; color: var(--dim); font-size: 8px;
            letter-spacing: 4.2px; line-height: 1.2;
        }
        .sysname {
            color: var(--dim); font-size: 10px; letter-spacing: 3.5px;
            text-transform: uppercase; padding-left: 16px;
            border-left: 1px solid var(--line);
        }
        .badges { margin-left: auto; display: flex; gap: 6px; }
        .badge {
            font-size: 10px; letter-spacing: 2px; padding: 4px 11px;
            border: 1px solid var(--line); background: var(--panel); color: var(--dim);
        }
        .badge.ok { color: var(--grn); border-color: #46612f; }
        .badge.warn { color: var(--acc); border-color: #7a4e1c; }
        .badge.bad { color: var(--red); border-color: #6d2626; }

        /* ---- main three-column layout ---- */
        .grid { flex: 1; display: flex; gap: 8px; padding: 8px; min-height: 0; }
        .col { flex: 1; min-width: 0; display: flex; flex-direction: column; }
        .side {
            width: 302px; flex-shrink: 0; display: flex; flex-direction: column;
            gap: 8px; overflow-y: auto;
        }
        .hdr {
            font-size: 9px; letter-spacing: 2.2px; color: var(--dim);
            text-transform: uppercase; padding: 0 2px 5px;
            display: flex; justify-content: space-between; align-items: baseline;
        }
        .hdr em { font-style: normal; color: #5d666f; letter-spacing: 1.5px; }
        .pane {
            flex: 1; min-height: 0; width: 100%;
            border: 1px solid var(--line); background: #0b0d10;
        }
        img.pane { object-fit: cover; }
        #map { cursor: crosshair; }
        /* Darken the OSM tiles to match the console. */
        .leaflet-tile-pane {
            filter: invert(1) hue-rotate(180deg) brightness(0.88) contrast(0.82) saturate(0.45);
        }
        .leaflet-control-attribution {
            background: rgba(21,24,28,0.8) !important; color: #6b7480 !important;
            font-size: 9px !important;
        }
        .leaflet-control-attribution a { color: #7e8894 !important; }
        .strip { padding-top: 8px; flex-shrink: 0; }

        /* ---- sidebar boxes ---- */
        .box { border: 1px solid var(--line); background: var(--panel); flex-shrink: 0; }
        .box-hdr {
            font-size: 9px; letter-spacing: 2.2px; color: var(--acc);
            text-transform: uppercase; padding: 6px 10px;
            background: var(--panel-hi); border-bottom: 1px solid var(--line);
        }
        .kv {
            display: flex; justify-content: space-between; align-items: baseline;
            padding: 5px 10px; border-top: 1px solid #2a2f36;
        }
        .kv:first-of-type { border-top: none; }
        .kv > i {
            font-style: normal; font-size: 9px; letter-spacing: 1.6px; color: var(--dim);
        }
        .kv-stack { display: block; padding: 5px 10px; border-top: 1px solid #2a2f36; }
        .kv-stack > i {
            font-style: normal; display: block; font-size: 9px;
            letter-spacing: 1.6px; color: var(--dim); margin-bottom: 2px;
        }
        .val { font-size: 14px; font-weight: 500; color: var(--txt); }
        .val.sm { font-size: 12px; }
        .grn { color: var(--grn); }
        .amb { color: var(--acc); }
        .red { color: var(--red); }
        .cyn { color: var(--cyn); }

        .btngrid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; padding: 8px; }
        button {
            font-family: inherit; font-size: 10px; letter-spacing: 1.4px;
            text-transform: uppercase; padding: 10px 3px; cursor: pointer;
            border: 1px solid var(--line); background: var(--panel-hi);
            color: var(--txt); transition: background .12s, border-color .12s;
        }
        button:hover { background: #2e343c; border-color: #46505c; }
        button:active { background: #14181c; }
        .btn-go { color: var(--grn); border-color: #46612f; }
        .btn-go:hover { border-color: var(--grn); }
        .btn-estop {
            grid-column: span 2; color: #fff; background: #6b2020;
            border-color: var(--red); font-weight: 700; letter-spacing: 3px;
        }
        .btn-estop:hover { background: var(--red); border-color: #ea7070; }
        .btn-clear { grid-column: span 2; color: var(--grn); border-color: #46612f; }
        .btn-clear:hover { border-color: var(--grn); }

        /* ---- propulsion + mission strips ---- */
        .eng-row { display: flex; align-items: center; gap: 9px; margin-bottom: 5px; }
        .eng-tag { font-size: 9px; letter-spacing: 1.6px; color: var(--dim); width: 38px; }
        .eng-num { font-size: 12px; width: 42px; text-align: right; }
        .bar-container {
            position: relative; flex: 1; height: 14px;
            background: var(--panel); border: 1px solid var(--line); overflow: hidden;
        }
        .bar-center { position: absolute; left: 50%; top: 0; bottom: 0; width: 1px; background: #454c55; }
        .bar {
            position: absolute; top: 0; bottom: 0; left: 50%; width: 0;
            background: var(--grn); transition: width .1s, left .1s;
        }
        .hint { font-size: 9px; color: #5d666f; letter-spacing: 1.4px; padding-top: 3px; }
        .mission-banner {
            border: 1px solid var(--line); background: var(--panel);
            padding: 7px 11px; font-size: 11px; letter-spacing: 1.4px;
            color: var(--dim); text-transform: uppercase;
            white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
        }
        .mission-banner.active { color: var(--acc); border-color: #7a4e1c; background: #241d14; }

        /* ---- waypoint reticles (HUD-style corner brackets) ---- */
        .wp-wrap { background: none; border: none; }
        .wp { position: relative; width: 30px; height: 30px; color: var(--cyn); }
        .wp > i.c { position: absolute; width: 9px; height: 9px; border: 2px solid currentColor; }
        .wp > i.tl { top: 0; left: 0; border-right: 0; border-bottom: 0; }
        .wp > i.tr { top: 0; right: 0; border-left: 0; border-bottom: 0; }
        .wp > i.bl { bottom: 0; left: 0; border-right: 0; border-top: 0; }
        .wp > i.br { bottom: 0; right: 0; border-left: 0; border-top: 0; }
        .wp > i.dot {
            position: absolute; left: 50%; top: 50%; width: 5px; height: 5px;
            margin: -2.5px 0 0 -2.5px; background: currentColor; border-radius: 50%;
        }
        .wp > b {
            position: absolute; left: 50%; top: -13px; transform: translateX(-50%);
            font-size: 10px; letter-spacing: 1px; color: currentColor;
            text-shadow: 0 0 5px #000, 0 0 3px #000;
        }
        .wp.pending { color: var(--acc); }
        .wp.queued { color: var(--cyn); }
        .wp.done { color: var(--grn); opacity: 0.4; }
        .wp.active { color: var(--acc); animation: retpulse 1.15s ease-in-out infinite; }
        @keyframes retpulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.4; } }
        .dock { position: relative; width: 22px; height: 22px; color: var(--grn); }
        .dock > i.ring {
            position: absolute; inset: 3px; border: 2px solid currentColor;
            transform: rotate(45deg);
        }
        .dock > b {
            position: absolute; left: 50%; top: -12px; transform: translateX(-50%);
            font-size: 8px; letter-spacing: 1.5px; color: currentColor;
            text-shadow: 0 0 5px #000, 0 0 3px #000;
        }
        .boat-arrow {
            font-size: 22px; line-height: 22px; color: var(--acc);
            text-align: center; transform-origin: 50% 50%;
            text-shadow: 0 0 6px #000, 0 0 3px #000;
        }
        /*__NAV_CSS__*/

        /* Geofence + power controls */
        .row2 { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; padding: 8px; }
        .row2 input {
            width: 100%; padding: 6px; background: #0e1416; color: #dce6e2;
            border: 1px solid #2a3a36; font-family: inherit; font-size: 11px;
        }
        .fieldlbl {
            font-size: 9px; letter-spacing: 1.6px; color: var(--dim);
            display: block; margin-bottom: 2px;
        }
        .note {
            font-size: 9px; color: #6b7480; letter-spacing: 0.8px;
            padding: 4px 10px 8px; line-height: 1.45;
        }
        .btn-warn { color: var(--acc); border-color: #7a4e1c; }
        .chan {
            display: grid; grid-template-columns: 1fr auto auto auto;
            gap: 6px; padding: 4px 10px; font-size: 11px;
            border-top: 1px solid #2a2f36; align-items: baseline;
        }
        .chan > i { font-style: normal; color: var(--dim); font-size: 9px;
                    letter-spacing: 1.4px; }
        .chan.dead { opacity: 0.45; }
        .chan.dead > b::after { content: ' ?'; color: var(--red); }
        .meter {
            height: 9px; background: #0e1114; border: 1px solid var(--line);
            margin: 6px 10px 2px; position: relative; overflow: hidden;
        }
        .meter > span {
            position: absolute; left: 0; top: 0; bottom: 0;
            background: var(--grn); transition: width .3s;
        }
        .meter.warn > span { background: var(--acc); }
        .meter.bad > span { background: var(--red); }
    </style>
</head>
<body>
<!--__NAV__-->

<header class="topbar">
    <div class="brand">
        <svg class="mark" viewBox="0 0 32 32" xmlns="http://www.w3.org/2000/svg">
            <path d="M16 1.6 L29.1 9.3 L29.1 22.7 L16 30.4 L2.9 22.7 L2.9 9.3 Z"
                  fill="none" stroke="#e8933a" stroke-width="1.5"/>
            <path d="M8.6 18.4 L16 11 L23.4 18.4" fill="none" stroke="#e8933a" stroke-width="2"/>
            <path d="M8.6 23.2 L16 15.8 L23.4 23.2" fill="none" stroke="#e8933a"
                  stroke-width="1.3" opacity="0.45"/>
        </svg>
        <div class="brand-txt"><b>RAMBERG</b><span>ROBOTICS</span></div>
    </div>
    <div class="sysname">Autonomous Surface Vessel Controller</div>
    <div class="badges">
        <span class="badge" id="link_badge">LINK</span>
        <span class="badge" id="state_badge">--</span>
        <span class="badge" id="rec_badge">DATA OFF</span>
    </div>
</header>

<main class="grid">

    <section class="col">
        <div class="hdr"><span>Forward Camera</span><span class="badge" id="det_toggle_badge" onclick="toggleDetections()" style="cursor:pointer;" title="Toggle YOLO detection box overlay (placeholder model)">DETECT: OFF</span><em>LIVE</em></div>
        <img class="pane" id="camera_pane" src="/video_feed">
        <div class="strip">
            <div class="eng-row">
                <div class="eng-tag">PORT</div>
                <div class="bar-container"><div class="bar-center"></div><div class="bar" id="left_bar"></div></div>
                <div class="eng-num" id="left_val">0</div>
            </div>
            <div class="eng-row">
                <div class="eng-tag">STBD</div>
                <div class="bar-container"><div class="bar-center"></div><div class="bar" id="right_bar"></div></div>
                <div class="eng-num" id="right_val">0</div>
            </div>
            <div class="hint">W/S THROTTLE &nbsp;&middot;&nbsp; A/D RUDDER &nbsp;&middot;&nbsp; RELEASE TO NEUTRAL</div>
        </div>
    </section>

    <section class="col">
        <div class="hdr"><span>Chart</span><em>L-CLICK PLOT WAYPOINT &middot; R-CLICK MOVE DOCK</em></div>
        <div class="pane" id="map"></div>
        <div class="strip">
            <div class="mission-banner" id="mission_banner">NO ACTIVE MISSION</div>
        </div>
    </section>

    <aside class="side">
        <div class="box">
            <div class="box-hdr">Vessel</div>
            <!-- Heading from boat/heading, shown with its quality. -->
            <div class="kv"><i>Heading</i><b class="val" id="heading_val">--</b></div>
            <div class="kv"><i>Hdg Quality</i><b class="val sm" id="hdgq_val">--</b></div>
            <div class="kv"><i>Hdg Source</i><b class="val sm" id="hdgsrc_val">--</b></div>
            <div class="kv"><i>Roll</i><b class="val" id="roll_val">--</b></div>
            <div class="kv"><i>Pitch</i><b class="val" id="pitch_val">--</b></div>
            <div class="kv"><i>Ground Speed</i><b class="val" id="speed_val">--</b></div>
            <div class="kv"><i>GPS Age</i><b class="val" id="gpsage_val">--</b></div>
            <div class="kv-stack"><i>Position</i><b class="val sm" id="gps_val">NO FIX</b></div>
        </div>

        <div class="box">
            <div class="box-hdr">Navigation</div>
            <div class="kv"><i>Waypoint</i><b class="val sm" id="wp_val">--</b></div>
            <div class="kv"><i>Distance</i><b class="val sm" id="dist_val">--</b></div>
            <div class="kv"><i>Bearing</i><b class="val sm" id="brg_val">--</b></div>
            <div class="kv"><i>Hdg Error</i><b class="val sm" id="hdgerr_val">--</b></div>
            <div class="kv"><i>Remaining</i><b class="val sm" id="rem_val">--</b></div>
            <div class="kv"><i>ETA</i><b class="val sm" id="eta_val">--</b></div>
            <div class="kv"><i>Progress</i><b class="val sm" id="prog_val">--</b></div>
            <div class="kv"><i>Plotted</i><b class="val sm" id="plotted_val">0</b></div>
        </div>

        <div class="box" id="dock_marker_box">
            <div class="box-hdr">Dock Marker</div>
            <div class="kv"><i>Detected</i><b class="val sm" id="dockm_detected">--</b></div>
            <div class="kv"><i>Distance</i><b class="val sm" id="dockm_dist">--</b></div>
            <div class="kv"><i>Lateral</i><b class="val sm" id="dockm_lat">--</b></div>
        </div>

        <div class="box" id="safety_box">
            <div class="box-hdr">Safety</div>
            <div class="kv"><i>Engine Stop</i><b class="val sm" id="estop_state">--</b></div>
            <div class="kv"><i>Status</i><b class="val sm" id="safe_state">--</b></div>
            <div class="kv"><i>Failsafe</i><b class="val sm" id="fs_state">--</b></div>
            <div class="kv"><i>Geofence</i><b class="val sm" id="fence_state">--</b></div>
            <div class="kv"><i>Fence Margin</i><b class="val sm" id="fence_margin">--</b></div>
            <!-- Counts up while outside the fence. -->
            <div class="kv"><i>Time Outside</i><b class="val sm" id="fence_outside">--</b></div>
            <div class="kv"><i>GCS Link</i><b class="val sm" id="gcs_link">--</b></div>
        </div>

        <div class="box" id="fence_box">
            <div class="box-hdr">Geofence</div>
            <div class="row2">
                <div>
                    <span class="fieldlbl">Radius (m)</span>
                    <input type="number" id="fence_radius_in" value="150" min="5" step="5">
                </div>
                <div>
                    <span class="fieldlbl">Warn ring (frac)</span>
                    <input type="number" id="fence_warn_in" value="0.9" min="0.1" max="0.99" step="0.05">
                </div>
            </div>
            <div class="btngrid">
                <button class="btn-go" onclick="setFenceHere()">Set Fence Here</button>
                <button onclick="setLaunchHere()">Set Launch Here</button>
                <button class="btn-warn" onclick="disableFence()">Disable Fence</button>
                <button onclick="enableFence()">Enable Fence</button>
            </div>
            <!-- The default fence centre is the Lake Eola sim origin, so
                 in Norway it is breached as soon as a fix arrives. -->
            <div class="note">
                Set the fence at the dock, before launching. The default
                centre is the simulator's Lake Eola origin and will be
                breached immediately at any real site. Both rings are drawn
                on the chart from what safety_monitor is actually enforcing.
            </div>
        </div>

        <div class="box" id="power_box">
            <div class="box-hdr">Power &amp; Host</div>
            <div class="kv"><i>Pack</i><b class="val" id="pwr_pack">--</b></div>
            <div class="kv"><i>Reserve</i><b class="val sm" id="pwr_reserve">--</b></div>
            <div class="meter" id="pwr_meter"><span style="width:0%"></span></div>
            <div class="note" id="pwr_caption">
                Reserve is the band ABOVE a floor kept for the trip home,
                so it reaches 0 while charge remains. Voltage-derived and
                therefore weak: a LiPo sags under load and its curve is
                flat mid-range. Coulomb counting would be better and
                is not built yet.
            </div>
            <div id="pwr_channels"></div>
            <div class="kv"><i>CPU Temp</i><b class="val sm" id="sys_temp">--</b></div>
            <div class="kv"><i>Load (1m)</i><b class="val sm" id="sys_load">--</b></div>
            <div class="kv"><i>Free Storage</i><b class="val sm" id="sys_disk">--</b></div>
            <div class="kv"><i>Throttling</i><b class="val sm" id="sys_throttle">--</b></div>
        </div>

        <div class="box">
            <div class="box-hdr">Control Source</div>
            <div class="kv"><i>Arbiter Mode</i><b class="val sm" id="mode_state">--</b></div>
            <div class="kv"><i>Your Control</i><b class="val sm" id="lease_state">not acquired</b></div>
            <div class="btngrid">
                <button onclick="acquireLease('button')">Take Manual</button>
                <button onclick="releaseLease()">Hold</button>
                <button class="btn-go" onclick="resumeAuto()">Continue Mission</button>
            </div>
        </div>

        <div class="box">
            <div class="box-hdr">Controls</div>
            <div class="btngrid">
                <button class="btn-go" onclick="sendMission()">Start Mission</button>
                <button onclick="clearPending()">Clear Points</button>
                <button onclick="cancelMission()">Cancel Mission</button>
                <button onclick="returnToDock()">Return Dock</button>
                <button onclick="setDockHere()">Set Dock Here</button>
                <button onclick="clearActive()">Hide Route Drawing</button>
                <button class="btn-estop" onclick="engineStop()">Engine Stop</button>
                <button class="btn-clear" onclick="resetToIdle()">Reset to Hold</button>
            </div>
            <div style="padding:12px">Motor bars show requested thrust, not measured motor output.</div>
        </div>
    </aside>

</main>

<script>
let left = 0, right = 0;
const keys = {};
const MANUAL_THRUST_LIMIT = {{ manual_thrust_limit }};

// Engine bars grow from the centre: forward right in green, reverse left
// in amber.
function setBar(id, val) {
    const el = document.getElementById(id);
    const pct = Math.abs(val) / 2;  // -100..100 maps to 0..50% of the track
    if (val >= 0) {
        el.style.left = '50%';
        el.style.background = 'var(--grn)';
    } else {
        el.style.left = (50 - pct) + '%';
        el.style.background = 'var(--acc)';
    }
    el.style.width = pct + '%';
}

function fmtDist(m) {
    if (m === null || m === undefined) return '--';
    if (m >= 1000) return (m / 1000).toFixed(2) + ' km';
    return m.toFixed(1) + ' m';
}

function fmtEta(sec) {
    if (sec === null || sec === undefined) return '--';
    if (sec > 5400) return '> 1h 30m';
    const m = Math.floor(sec / 60);
    const s = Math.round(sec % 60);
    return m + 'm ' + String(s).padStart(2, '0') + 's';
}

function setCell(id, text, cls) {
    const el = document.getElementById(id);
    el.innerText = text;
    el.className = cls ? ('val sm ' + cls) : 'val sm';
}

//__OPERATOR_JS__

// --- Live chart (boat/gps/fix + boat/mission_geojson via /telemetry) ---
const map = L.map('map', {zoomControl: true, attributionControl: true}).setView([0, 0], 3);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; OpenStreetMap',
    maxZoom: 19,
}).addTo(map);

// Leaflet has to re-measure when the flex layout changes size.
window.addEventListener('resize', () => map.invalidateSize());
setTimeout(() => map.invalidateSize(), 250);

const boatIcon = L.divIcon({
    className: 'wp-wrap',
    html: '<div id="boat-arrow" class="boat-arrow">&#9650;</div>',
    iconSize: [22, 22], iconAnchor: [11, 11],
});
const boatMarker = L.marker([0, 0], {opacity: 0, icon: boatIcon}).addTo(map);

// Only the trail LineString is taken from boat/mission_geojson. The target
// is drawn locally as a reticle.
let missionLayer = L.geoJSON(null, {
    filter: (f) => f.geometry && f.geometry.type === 'LineString',
    style: () => ({color: '#e8933a', weight: 2, opacity: 0.55}),
}).addTo(map);

let mapCentered = false;
let lastFix = null;  // [lat, lon] of the most recent fix, for Set Dock

// Geofence drawn from boat/safety_status, so it is the fence safety_monitor
// enforces. Redrawn only when it changes.
let fenceCircle = null;
let fenceWarnCircle = null;
let fenceKey = null;
let launchMarker = null;
let fenceInputsTouched = false;

function drawGeofence(s) {
    // Warning ring radius comes from safety_monitor.
    const warnR = (s.fence_warn_radius_m !== null && s.fence_warn_radius_m !== undefined)
        ? s.fence_warn_radius_m : s.fence_radius_m * 0.9;
    const key = [s.fence_center_lat, s.fence_center_lon,
                 s.fence_radius_m, warnR, s.fence_enabled].join(',');
    if (key === fenceKey) return;
    fenceKey = key;
    if (fenceCircle) { map.removeLayer(fenceCircle); fenceCircle = null; }
    if (fenceWarnCircle) { map.removeLayer(fenceWarnCircle); fenceWarnCircle = null; }
    if (!s.fence_enabled) return;
    const centre = [s.fence_center_lat, s.fence_center_lon];
    fenceCircle = L.circle(centre, {
        radius: s.fence_radius_m, color: '#cf4f4f', weight: 1.5,
        opacity: 0.75, fillColor: '#cf4f4f', fillOpacity: 0.035, dashArray: '7,6',
    }).addTo(map);
    fenceWarnCircle = L.circle(centre, {
        radius: warnR, color: '#e8933a', weight: 1,
        opacity: 0.55, fill: false, dashArray: '3,7',
    }).addTo(map);
}

// RETURN_TO_LAUNCH point from safety_monitor, drawn separately from the
// dock so any difference is visible.
function drawLaunchPoint(fs) {
    if (!fs || !fs.launch_point_valid) return;
    const pos = [fs.launch_lat, fs.launch_lon];
    if (launchMarker) {
        launchMarker.setLatLng(pos);
        return;
    }
    launchMarker = L.marker(pos, {
        icon: L.divIcon({
            className: 'wp-wrap',
            html: '<div class="dock" style="color:var(--red)">'
                + '<i class="ring"></i><b>LAUNCH</b></div>',
            iconSize: [22, 22], iconAnchor: [11, 11],
        }),
        interactive: false,
    }).addTo(map);
}

// --- Geofence controls ------------------------------------------------
// Sent on boat/geofence_command. safety_monitor applies it and records it.

function postFence(body, okMsg) {
    return fetch('/geofence', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(body),
    }).then((r) => r.json()).then((d) => {
        if (d.status !== 'ok') notifyOperator('Geofence command failed: ' + (d.error || 'unknown'));
        else if (okMsg) console.log(okMsg);
        fenceKey = null;   // force a redraw
    });
}

function setFenceHere() {
    const radius = parseFloat(document.getElementById('fence_radius_in').value);
    const warn = parseFloat(document.getElementById('fence_warn_in').value);
    if (!isFinite(radius) || radius < 5) {
        notifyOperator('Radius must be a number of at least 5 m.');
        return;
    }
    if (!isFinite(warn) || warn <= 0 || warn >= 1) {
        notifyOperator('Warning-ring fraction must be between 0 and 1 (e.g. 0.9).');
        return;
    }
    postFence({action: 'set_here', radius_m: radius, warn_fraction: warn},
              'fence set to current position');
}

function setLaunchHere() {
    postFence({action: 'set_launch_here'}, 'launch point set');
}

function enableFence() {
    postFence({action: 'enable'}, 'fence enabled');
}

function disableFence() {
    // Disabling needs a typed reason. The banner stays amber while off.
    const reason = document.getElementById('operator_reason').value.trim();
    if (!reason) { notifyOperator('Enter a reason before disabling the geofence.'); return; }
    postFence({action: 'disable', reason: reason}, 'fence disabled');
}

// --- Click-to-set-waypoint mission planning ---
// PENDING points are plotted but not sent. ACTIVE points are the dispatched
// route, kept on the chart and coloured by progress.
let pendingPoints = [];  // [[lat, lon], ...] not yet sent
let pendingMarkers = [];
let activePoints = [];   // dispatched route, [[lat, lon], ...]
let activeMarkers = [];
let activeLine = null;
// Mirrors nav.active, so Clear Points does not wipe a running route.
let missionActive = false;
let routeBusy = false;
let routeRequest = null;

// Corner-bracket reticle marker.
function wpIcon(n, cls) {
    return L.divIcon({
        className: 'wp-wrap',
        html: '<div class="wp ' + cls + '">' +
              '<i class="c tl"></i><i class="c tr"></i>' +
              '<i class="c bl"></i><i class="c br"></i>' +
              '<i class="dot"></i><b>' + n + '</b></div>',
        iconSize: [30, 30], iconAnchor: [15, 15],
    });
}

function setWpClass(marker, cls) {
    const el = marker.getElement();
    if (!el) return;
    const w = el.querySelector('.wp');
    if (w) w.className = 'wp ' + cls;
}

// Right-click moves the dock. Left click is for waypoints.
map.on('contextmenu', (e) => {
    setDock(e.latlng.lat, e.latlng.lng);
});

map.on('click', (e) => {
    pendingPoints.push([e.latlng.lat, e.latlng.lng]);
    const marker = L.marker(e.latlng, {icon: wpIcon(pendingPoints.length, 'pending')}).addTo(map);
    pendingMarkers.push(marker);
    document.getElementById('plotted_val').innerText = pendingPoints.length;
});

function clearPending() {
    pendingMarkers.forEach((m) => map.removeLayer(m));
    pendingMarkers = [];
    pendingPoints = [];
    document.getElementById('plotted_val').innerText = 0;
    // Also clear the dispatched route, but only when no mission is running.
    if (!missionActive) clearActive();
}

function clearActive() {
    activeMarkers.forEach((m) => map.removeLayer(m));
    activeMarkers = [];
    activePoints = [];
    if (activeLine) { map.removeLayer(activeLine); activeLine = null; }
}

function drawActiveRoute(points) {
    clearActive();
    activePoints = points.slice();
    activeLine = L.polyline(activePoints, {
        color: '#5fa6b8', weight: 1.5, opacity: 0.65, dashArray: '5,7',
    }).addTo(map);
    activePoints.forEach((p, i) => {
        const m = L.marker(p, {icon: wpIcon(i + 1, 'queued')}).addTo(map);
        activeMarkers.push(m);
    });
}

// Colour the route: reached = dim green, current = pulsing amber,
// queued = cyan.
function styleActiveRoute(currentIdx) {
    activeMarkers.forEach((m, i) => {
        setWpClass(m, i < currentIdx ? 'done' : i === currentIdx ? 'active' : 'queued');
    });
}

// Route commands never silently replace an executing mission.
async function startRoute(points) {
    if (routeBusy) { notifyOperator('A route request is already pending. Manual and Hold are still available.'); return false; }
    if (!points.length) { notifyOperator('Plot at least one waypoint first.'); return false; }
    if (missionActive) { notifyOperator('A mission is retained. Continue it or cancel it before starting a new route.'); return false; }
    routeBusy = true;
    try {
        if (!await releaseLease()) return false;
        const revision = operator.revision;
        routeRequest = postCommand('/mission', {latitudes:points.map(p=>p[0]), longitudes:points.map(p=>p[1])});
        await routeRequest;
        // A takeover or stop during mission acceptance must win.
        if (operator.revision !== revision) return false;
        missionActive = true;
        drawActiveRoute(points);
        const ok = await operator.transition('AUTO');
        notifyOperator(ok ? 'Mission accepted. Navigation has control unless a safety stop is active.' : 'Mission retained; control is not confirmed.');
        return ok;
    } catch(error) { notifyOperator(error.message + ' Route acceptance is not confirmed; check mission status before retrying.'); return false; }
    finally { routeBusy = false; routeRequest = null; }
}
async function sendMission() {
    const points = pendingPoints.slice();
    if (await startRoute(points)) clearPending();
}

function engineStop() {
    clearDriveKeys();
    notifyOperator('Requesting emergency stop.');
    return operator.transition('ESTOP');
}
function resetToIdle() {
    clearDriveKeys();
    notifyOperator('Clearing E-stop into Hold. This does not resume a mission.');
    return operator.transition('RESET');
}
async function cancelMission() {
    if (!await releaseLease()) return;
    const revision = operator.revision;
    try {
        if (routeRequest) await routeRequest.catch(() => {});
        if (operator.revision !== revision) return;
        await postCommand('/mission/cancel', {});
        if (operator.revision !== revision) return;
        missionActive = false; clearActive();
        notifyOperator('Mission cancelled. Holding zero thrust.');
    } catch(error) { notifyOperator(error.message); }
}

// Dock / home position, owned by the server (see /dock). Auto-captured from
// the first GPS fix.
let dockPoint = null;
let dockMarker = null;

function drawDock(lat, lon) {
    const moved = !dockMarker
        || dockMarker.getLatLng().lat !== lat
        || dockMarker.getLatLng().lng !== lon;
    if (!moved) return;
    if (dockMarker) map.removeLayer(dockMarker);
    dockMarker = L.marker([lat, lon], {
        icon: L.divIcon({
            className: 'wp-wrap',
            html: '<div class="dock"><i class="ring"></i><b>DOCK</b></div>',
            iconSize: [22, 22], iconAnchor: [11, 11],
        }),
        interactive: false,
    }).addTo(map);
}

function setDock(lat, lon) {
    fetch('/dock', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({lat: lat, lon: lon}),
    }).then((r) => r.json()).then((d) => {
        if (d.status !== 'ok') notifyOperator('Set dock failed: ' + (d.error || 'unknown error'));
    });
}

// Set the dock to the boat's current position.
function setDockHere() {
    if (!lastFix) {
        notifyOperator('No GPS fix yet -- cannot set the dock to the boat position.');
        return;
    }
    setDock(lastFix[0], lastFix[1]);
}

function toggleDetections() {
    // Switch between the raw and YOLO feeds and turn detector_node
    // inference on or off. When off, the detector passes raw frames through.
    const img = document.getElementById('camera_pane');
    const badge = document.getElementById('det_toggle_badge');
    const isOn = badge.textContent.indexOf('ON') !== -1;
    const turningOn = !isOn;
    img.src = turningOn ? '/detections_feed' : '/video_feed';
    badge.textContent = turningOn ? 'DETECT: ON' : 'DETECT: OFF';
    fetch('/detect/enable', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({enabled: turningOn}),
    });
}

async function returnToDock() {
    if (!dockPoint) { notifyOperator('Dock position is not set.'); return; }
    if (missionActive) {
        notifyOperator('Cancel the current mission before starting a new route to the dock.');
        return;
    }
    await startRoute([dockPoint]);
}

function attitudeClass(v) {
    if (v === null || v === undefined) return '';
    const a = Math.abs(v);
    if (a > 45) return 'red';
    if (a > 20) return 'amb';
    return 'grn';
}

function updateTelemetry() {
    fetch('/telemetry').then((r) => r.json()).then((data) => {
        const lb = document.getElementById('link_badge');
        const controlFresh = data.operator_age_s != null && data.operator_age_s <= 1;
        lb.innerText = controlFresh ? 'CONTROL LINK OK' : 'CONTROL LINK STALE';
        lb.className = controlFresh ? 'badge ok' : 'badge bad';
        operator.observe(data.operator, data.operator_age_s);

        const state = data.state || '--';
        const sb = document.getElementById('state_badge');
        sb.innerText = state;
        sb.className = 'badge ' + (state === 'EXECUTE' ? 'ok'
                                 : state === 'ABORTED' ? 'bad'
                                 : state === 'IDLE' ? 'warn' : '');

        // Source control_arbiter selects. EXECUTE + WAYPOINT means the boat
        // is driving its route. Manual can drive while that route is paused.
        const mode = controlFresh ? (data.operator.mode || data.mode || '--') : 'UNKNOWN';
        setCell('mode_state', mode,
                mode === 'WAYPOINT' ? 'grn'
              : mode === 'MANUAL' ? 'amb'
              : mode === 'ESTOP' ? 'red'
              : mode === 'HOLD' ? 'amb' : '');
        if (state === 'EXECUTE' && (mode === 'MANUAL' || mode === 'HOLD')) {
            sb.innerText = mode === 'MANUAL' ? 'MISSION PAUSED / MANUAL CONTROL' : 'MISSION PAUSED / HOLD';
            sb.className = 'badge warn';
        }

        // Recording reminder. Full controls are on the Data page.
        const rec = data.recording || {};
        const recBadge = document.getElementById('rec_badge');
        if (rec.recording) {
            recBadge.innerText = 'REC ' + (rec.elapsed_s !== null && rec.elapsed_s !== undefined
                ? rec.elapsed_s.toFixed(0) + 's' : '');
            recBadge.className = 'badge bad';
        } else {
            recBadge.innerText = 'DATA OFF';
            recBadge.className = 'badge';
        }

        // Dock ArUco marker. Missing key means dock_marker_node is not
        // running.
        const dm = data.dock_marker || {};
        const dmDetEl = document.getElementById('dockm_detected');
        const dmDistEl = document.getElementById('dockm_dist');
        const dmLatEl = document.getElementById('dockm_lat');
        if (dmDetEl) {
            dmDetEl.innerText = dm.detected ? 'YES' : 'NO';
            dmDetEl.className = 'val sm ' + (dm.detected ? 'grn' : '');
            dmDistEl.innerText = (dm.distance_m === null || dm.distance_m === undefined)
                ? '--' : dm.distance_m.toFixed(2) + ' m';
            dmLatEl.innerText = (dm.lateral_m === null || dm.lateral_m === undefined)
                ? '--' : (dm.lateral_m > 0 ? '+' : '') + dm.lateral_m.toFixed(2) + ' m';
        }

        if (data.dock && data.dock.lat !== null && data.dock.lat !== undefined) {
            dockPoint = [data.dock.lat, data.dock.lon];
            drawDock(data.dock.lat, data.dock.lon);
        }

        if (data.lat !== null && data.lon !== null) {
            lastFix = [data.lat, data.lon];
            document.getElementById('gps_val').innerText =
                data.lat.toFixed(6) + ', ' + data.lon.toFixed(6);
            boatMarker.setLatLng([data.lat, data.lon]);
            boatMarker.setOpacity(1);
            if (!mapCentered) {
                map.setView([data.lat, data.lon], 19);
                mapCentered = true;
            }
        }

        if (data.geojson) {
            missionLayer.clearLayers();
            missionLayer.addData(data.geojson);
        }

        // Boat arrow: fused heading, or raw IMU if the estimator is not
        // running.
        const hdg = data.heading_est || {};
        const arrowHeading = (hdg.heading_deg !== null && hdg.heading_deg !== undefined
                              && hdg.quality > 0) ? hdg.heading_deg : data.heading;
        if (arrowHeading !== null && arrowHeading !== undefined) {
            const arrow = document.getElementById('boat-arrow');
            if (arrow) arrow.style.transform = 'rotate(' + arrowHeading + 'deg)';
        }

        // Heading 0 = north, clockwise. Roll/pitch amber past 20 deg, red
        // past 45 deg.
        updateHeading(hdg, data.heading);
        const rollEl = document.getElementById('roll_val');
        const pitchEl = document.getElementById('pitch_val');
        rollEl.innerText = data.roll !== null ? data.roll.toFixed(1) + '°' : '--';
        pitchEl.innerText = data.pitch !== null ? data.pitch.toFixed(1) + '°' : '--';
        rollEl.className = 'val ' + attitudeClass(data.roll);
        pitchEl.className = 'val ' + attitudeClass(data.pitch);

        const nav = data.nav || {};
        missionActive = state === 'EXECUTE';

        // nav_status is m/s. km/h is for display only.
        document.getElementById('speed_val').innerText =
            (nav.speed_mps !== undefined && nav.speed_mps !== null)
                ? (nav.speed_mps * 3.6).toFixed(1) + ' km/h' : '--';

        const age = nav.gps_age_sec;
        const ageEl = document.getElementById('gpsage_val');
        ageEl.innerText = (age === undefined || age === null) ? '--' : age.toFixed(1) + ' s';
        ageEl.className = 'val ' + (age === undefined || age === null ? ''
                                  : age > 3 ? 'red' : age > 1.5 ? 'amb' : 'grn');

        const banner = document.getElementById('mission_banner');
        if (nav.active && nav.waypoint_count) {
            const n = nav.waypoint_index + 1;
            let txt = (mode === 'WAYPOINT' ? '' : 'PAUSED · ') + 'WAYPOINT ' + n + ' / ' + nav.waypoint_count;
            if (nav.distance_m !== null && nav.distance_m !== undefined) {
                txt += '  ·  ' + fmtDist(nav.distance_m) + ' OUT';
            }
            if (nav.eta_sec !== null && nav.eta_sec !== undefined) {
                txt += '  ·  ETA ' + fmtEta(nav.eta_sec);
            }
            banner.innerText = txt;
            banner.className = 'mission-banner active';
            styleActiveRoute(nav.waypoint_index);

            setCell('wp_val', n + ' / ' + nav.waypoint_count, 'cyn');
            setCell('dist_val', fmtDist(nav.distance_m), 'amb');
            setCell('brg_val', nav.bearing_deg !== null && nav.bearing_deg !== undefined
                ? nav.bearing_deg.toFixed(1) + '°' : '--');
            const he = nav.heading_error_deg;
            setCell('hdgerr_val',
                (he === null || he === undefined) ? '--' : (he > 0 ? '+' : '') + he.toFixed(1) + '°',
                (he === null || he === undefined) ? '' : Math.abs(he) < 15 ? 'grn' : Math.abs(he) < 60 ? 'amb' : 'red');
            setCell('rem_val', fmtDist(nav.remaining_m));
            setCell('eta_val', fmtEta(nav.eta_sec), 'cyn');
            setCell('prog_val', Math.round(100 * nav.waypoint_index / nav.waypoint_count) + ' %');
        } else {
            banner.innerText = (state === 'ABORTED')
                ? 'MISSION ABORTED · PLOT NEW WAYPOINTS OR RETURN TO DOCK'
                : 'NO ACTIVE MISSION · CLICK CHART TO PLOT WAYPOINTS';
            banner.className = 'mission-banner';
            ['wp_val', 'dist_val', 'brg_val', 'hdgerr_val', 'rem_val', 'eta_val', 'prog_val']
                .forEach((id) => setCell(id, '--'));
        }
        // A latched e-stop is the usual reason missions do nothing, so it
        // gets its own readout.
        const esEl = document.getElementById('estop_state');
        if (data.estop) {
            esEl.innerText = 'ENGAGED';
            esEl.className = 'val sm red';
            banner.innerText = 'ENGINE STOP ENGAGED · RESET TO HOLD, THEN CHOOSE YOUR NEXT ACTION';
            banner.className = 'mission-banner';
            banner.style.color = 'var(--red)';
            banner.style.borderColor = 'var(--red)';
        } else {
            esEl.innerText = 'CLEAR';
            esEl.className = 'val sm grn';
            banner.style.color = '';
            banner.style.borderColor = '';
        }

        updateSafety(data.safety || {}, data.failsafe || {});
        updatePower(data);
        updateBanner(data);
        updateNavNote(data);
        drawLaunchPoint(data.failsafe || {});
        syncFenceInputs(data.safety || {});
    }).catch(() => {
        const lb = document.getElementById('link_badge');
        lb.innerText = 'NO LINK';
        lb.className = 'badge bad';
    });
}

// Inputs show the enforced values until the operator starts editing.
function syncFenceInputs(s) {
    if (fenceInputsTouched) return;
    const r = document.getElementById('fence_radius_in');
    const w = document.getElementById('fence_warn_in');
    if (!r || !w) return;
    if (s.fence_radius_m !== undefined && s.fence_radius_m !== null) {
        r.value = s.fence_radius_m;
    }
    if (s.fence_warn_fraction !== undefined && s.fence_warn_fraction !== null) {
        w.value = s.fence_warn_fraction;
    }
}
['fence_radius_in', 'fence_warn_in'].forEach((id) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener('input', () => { fenceInputsTouched = true; });
});

// Heading quality names, mirroring boat_msgs/msg/HeadingEstimate.msg.
const HDG_Q = ['NONE', 'DEAD RECKONING', 'FUSED'];

function updateHeading(hdg, rawImuHeading) {
    const valEl = document.getElementById('heading_val');
    const qEl = document.getElementById('hdgq_val');
    const sEl = document.getElementById('hdgsrc_val');
    if (!valEl) return;

    if (hdg.quality === undefined || hdg.quality === null) {
        // No estimator. Show the raw IMU value labelled as drifting.
        valEl.innerText = (rawImuHeading !== null && rawImuHeading !== undefined)
            ? rawImuHeading.toFixed(1) + '°' : '--';
        valEl.className = 'val amb';
        qEl.innerText = 'NO ESTIMATOR';
        qEl.className = 'val sm red';
        sEl.innerText = 'raw IMU yaw (drifts)';
        sEl.className = 'val sm amb';
        return;
    }

    const usable = hdg.quality >= 1;
    valEl.innerText = (usable && hdg.heading_deg !== null && hdg.heading_deg !== undefined)
        ? hdg.heading_deg.toFixed(1) + '°' : '--';
    valEl.className = 'val ' + (hdg.quality >= 2 ? 'grn' : usable ? 'amb' : 'red');

    qEl.innerText = HDG_Q[hdg.quality] || ('?' + hdg.quality);
    qEl.className = 'val sm ' + (hdg.quality >= 2 ? 'grn' : usable ? 'amb' : 'red');
    qEl.title = hdg.detail || '';

    // COG is course over ground, not bow heading.
    let src = hdg.source || '--';
    if (src === 'COG') src = 'COG (course, not bow)';
    if (hdg.age_since_reference_sec !== null && hdg.age_since_reference_sec !== undefined
        && hdg.quality < 2) {
        src += '  ·  ' + hdg.age_since_reference_sec.toFixed(0) + 's stale';
    }
    sEl.innerText = src;
    sEl.className = 'val sm ' + (hdg.quality >= 2 ? '' : 'amb');
}

function updatePower(data) {
    const pwr = data.power || {};
    const sys = data.system || {};
    const packEl = document.getElementById('pwr_pack');
    const resEl = document.getElementById('pwr_reserve');
    const meter = document.getElementById('pwr_meter');
    const chans = document.getElementById('pwr_channels');
    if (!packEl) return;

    if (!pwr.seen) {
        // No power data. Different from a 0.0 V reading.
        packEl.innerText = 'NO MONITOR';
        packEl.className = 'val red';
        resEl.innerText = '--';
        meter.firstElementChild.style.width = '0%';
        chans.innerHTML = '';
    } else {
        packEl.innerText = (pwr.pack_voltage_v !== null && pwr.pack_voltage_v !== undefined)
            ? pwr.pack_voltage_v.toFixed(2) + ' V' : '--';
        packEl.className = 'val';

        const frac = pwr.reserve_frac;
        const state = pwr.reserve_state || 'UNKNOWN';
        if (frac === null || frac === undefined) {
            resEl.innerText = state;
            resEl.className = 'val sm';
            meter.firstElementChild.style.width = '0%';
            meter.className = 'meter';
        } else {
            resEl.innerText = Math.round(frac * 100) + ' % above reserve  ·  ' + state;
            const cls = state === 'OK' ? 'grn' : state === 'WARN' ? 'amb' : 'red';
            resEl.className = 'val sm ' + cls;
            meter.firstElementChild.style.width = (frac * 100).toFixed(0) + '%';
            meter.className = 'meter ' + (state === 'OK' ? '' : state === 'WARN' ? 'warn' : 'bad');
        }

        // ok === false means the I2C read failed, not a real zero.
        let html = '';
        (pwr.channels || []).forEach((c) => {
            html += '<div class="chan' + (c.ok ? '' : ' dead') + '">'
                 + '<i>' + c.name + '</i>'
                 + '<b>' + (c.ok ? c.voltage_v.toFixed(2) + ' V' : 'read fail') + '</b>'
                 + '<b>' + (c.ok ? c.current_a.toFixed(2) + ' A' : '') + '</b>'
                 + '<b>' + (c.ok ? c.power_w.toFixed(1) + ' W' : '') + '</b>'
                 + '</div>';
        });
        chans.innerHTML = html;
    }

    // Host telemetry from system_monitor.
    const t = sys.cpu_temp_c;
    const tEl = document.getElementById('sys_temp');
    tEl.innerText = (t === null || t === undefined) ? '--' : t.toFixed(1) + ' °C';
    const tw = sys.temp_warn_c || 70, tc = sys.temp_critical_c || 80;
    tEl.className = 'val sm ' + (t === null || t === undefined ? ''
        : t >= tc ? 'red' : t >= tw ? 'amb' : 'grn');

    const load = sys.load;
    document.getElementById('sys_load').innerText =
        (load && load.length) ? load[0].toFixed(2) : '--';

    const dEl = document.getElementById('sys_disk');
    const rec = data.recording || {};
    let freeGb = (rec.free_gb !== null && rec.free_gb !== undefined)
        ? rec.free_gb : (sys.disk ? sys.disk.free_gb : null);
    if (freeGb === null || freeGb === undefined) {
        dEl.innerText = '--';
        dEl.className = 'val sm';
    } else {
        // Minutes remaining, not just gigabytes.
        let txt = freeGb.toFixed(1) + ' GB';
        const mins = (rec.est_minutes_remaining !== null && rec.est_minutes_remaining !== undefined)
            ? rec.est_minutes_remaining
            : (sys.disk ? sys.disk.est_imagery_minutes : null);
        if (mins !== null && mins !== undefined) txt += '  ·  ~' + Math.round(mins) + ' min';
        dEl.innerText = txt;
        dEl.className = 'val sm ' + (freeGb < 5 ? 'red' : freeGb < 20 ? 'amb' : 'grn');
    }

    const thrEl = document.getElementById('sys_throttle');
    const thr = sys.throttle;
    if (!thr) {
        thrEl.innerText = '--';
        thrEl.className = 'val sm';
    } else if (!thr.flags || !thr.flags.length) {
        thrEl.innerText = 'NONE';
        thrEl.className = 'val sm grn';
    } else {
        // The Pi runs off the motor pack, so under-voltage here means
        // brownouts.
        thrEl.innerText = thr.flags.length + ' FLAG(S)';
        thrEl.className = 'val sm red';
        thrEl.title = thr.flags.join('; ');
    }
}

function updateSafety(s, fs) {
    const stateEl = document.getElementById('safe_state');
    const fenceEl = document.getElementById('fence_state');
    const marginEl = document.getElementById('fence_margin');
    const outsideEl = document.getElementById('fence_outside');
    const fsEl = document.getElementById('fs_state');
    const gcsEl = document.getElementById('gcs_link');
    const box = document.getElementById('safety_box');

    // Failsafe level, with the reason as a tooltip.
    const FS_NAMES = ['NOMINAL', 'HOLD POSITION', 'RETURN TO LAUNCH',
                      'POWERED STOP', 'HARD STOP'];
    if (fs && fs.level !== undefined && fs.level !== null) {
        fsEl.innerText = (FS_NAMES[fs.level] || ('?' + fs.level))
            + (fs.degraded_reason ? ' *' : '');
        fsEl.className = 'val sm ' + (fs.level === 0 ? 'grn'
                                    : fs.level === 2 ? 'amb' : 'red');
        fsEl.title = fs.degraded_reason
            ? ('DEGRADED: ' + fs.degraded_reason)
            : ((fs.active_reasons && fs.active_reasons.join('; ')) || '');
    } else {
        fsEl.innerText = 'NO SUPERVISOR';
        fsEl.className = 'val sm red';
    }

    if (s.fence_enabled === undefined) {
        // safety_monitor has not published. Show "no data", not dashes.
        stateEl.innerText = 'NO MONITOR';
        stateEl.className = 'val sm red';
        fenceEl.innerText = '--'; fenceEl.className = 'val sm';
        marginEl.innerText = '--'; marginEl.className = 'val sm';
        outsideEl.innerText = '--'; outsideEl.className = 'val sm';
        gcsEl.innerText = '--'; gcsEl.className = 'val sm';
        box.style.borderColor = 'var(--red)';
        return;
    }

    // s.level is safety_monitor's own OK/WARN/ERROR (0/1/2).
    const LEVEL_CLASS = ['grn', 'amb', 'red'];
    const LEVEL_BORDER = ['var(--line)', 'var(--acc)', 'var(--red)'];
    const cls = LEVEL_CLASS[s.level] || 'grn';
    if (s.level === 2) {
        stateEl.innerText = s.reason === 'GEOFENCE_BREACH' ? 'FENCE BREACH' : 'LINK LOST';
    } else if (s.level === 1) {
        stateEl.innerText = 'FENCE WARNING';
    } else {
        stateEl.innerText = 'NOMINAL';
    }
    stateEl.className = 'val sm ' + cls;
    box.style.borderColor = LEVEL_BORDER[s.level] || 'var(--line)';

    // Both rings, so the warning band is visible.
    fenceEl.innerText = s.fence_enabled
        ? (s.fence_radius_m.toFixed(0) + ' m  (warn ' +
           ((s.fence_warn_radius_m !== null && s.fence_warn_radius_m !== undefined)
            ? s.fence_warn_radius_m.toFixed(0) : '?') + ' m)')
        : 'DISABLED';
    fenceEl.className = 'val sm ' + (s.fence_enabled ? '' : 'red');
    fenceEl.title = s.fence_enabled ? '' : (s.fence_disabled_reason || '');

    if (s.fence_margin_m === null || s.fence_margin_m === undefined) {
        marginEl.innerText = '--';
        marginEl.className = 'val sm';
    } else if (s.fence_margin_m < 0) {
        // Outside: show how far.
        marginEl.innerText = fmtDist(-s.fence_margin_m) + ' OUTSIDE';
        marginEl.className = 'val sm red';
    } else {
        marginEl.innerText = fmtDist(s.fence_margin_m);
        marginEl.className = 'val sm ' + (s.fence_warning ? 'amb' : 'grn');
    }

    // Keeps counting while the boat is outside.
    if (s.fence_outside_sec === null || s.fence_outside_sec === undefined
        || s.fence_outside_sec === 0) {
        outsideEl.innerText = s.fence_outside_sec === 0 ? 'inside' : '--';
        outsideEl.className = 'val sm ' + (s.fence_outside_sec === 0 ? 'grn' : '');
    } else {
        outsideEl.innerText = s.fence_outside_sec.toFixed(0) + ' s';
        outsideEl.className = 'val sm red';
    }

    const age = s.gcs_heartbeat_age_sec;
    gcsEl.innerText = (age === null || age === undefined) ? 'NO BEAT' : age.toFixed(1) + ' s';
    gcsEl.className = 'val sm ' + (s.gcs_lost ? 'red' : (age !== null && age > 3) ? 'amb' : 'grn');

    drawGeofence(s);
}

//__BANNER_JS__

setInterval(updateTelemetry, 1000);
updateTelemetry();
</script>
</body>
</html>
"""


# Data collection page. Same styling as the control page and the same
# backend routes.
DATA_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Ramberg Robotics | Data Collection</title>
    <style>
        :root {
            --bg: #15181c; --panel: #1d2126; --panel-hi: #252a31; --line: #343a42;
            --txt: #dcd8d0; --dim: #838d99; --acc: #e8933a; --grn: #8bb85f;
            --red: #cf4f4f; --cyn: #5fa6b8;
        }
        * { box-sizing: border-box; }
        html, body { height: 100%; }
        body {
            margin: 0; background: var(--bg); color: var(--txt);
            font-family: ui-monospace, 'Cascadia Code', 'Consolas', monospace;
            display: flex; flex-direction: column; overflow: hidden;
            background-image: repeating-linear-gradient(135deg,
                rgba(255,255,255,0.013) 0 2px, transparent 2px 7px);
        }
        .topbar {
            display: flex; align-items: center; gap: 16px; flex-shrink: 0;
            padding: 7px 14px; border-bottom: 1px solid var(--line);
            background: linear-gradient(180deg, #262b32 0%, #1a1e23 100%);
        }
        .brand { display: flex; align-items: center; gap: 9px; }
        .mark { width: 27px; height: 27px; flex-shrink: 0; }
        .brand-txt b {
            display: block; color: var(--acc); font-size: 14px;
            letter-spacing: 4.5px; line-height: 1.1; font-weight: 700;
        }
        .brand-txt span {
            display: block; color: var(--dim); font-size: 8px;
            letter-spacing: 4.2px; line-height: 1.2;
        }
        .sysname {
            color: var(--dim); font-size: 10px; letter-spacing: 3.5px;
            text-transform: uppercase; padding-left: 16px;
            border-left: 1px solid var(--line);
        }
        .badges { margin-left: auto; display: flex; gap: 6px; }
        .badge {
            font-size: 10px; letter-spacing: 2px; padding: 4px 11px;
            border: 1px solid var(--line); background: var(--panel); color: var(--dim);
        }
        .badge.ok { color: var(--grn); border-color: #46612f; }
        .badge.bad { color: var(--red); border-color: #6d2626; }

        .grid { flex: 1; display: flex; gap: 8px; padding: 8px; min-height: 0; }
        .col { flex: 1.4; min-width: 0; display: flex; flex-direction: column; }
        .side {
            width: 340px; flex-shrink: 0; display: flex; flex-direction: column;
            gap: 8px; overflow-y: auto;
        }
        .hdr {
            font-size: 9px; letter-spacing: 2.2px; color: var(--dim);
            text-transform: uppercase; padding: 0 2px 5px;
            display: flex; justify-content: space-between; align-items: baseline;
        }
        .hdr em { font-style: normal; color: #5d666f; letter-spacing: 1.5px; }
        .pane { flex: 1; min-height: 0; width: 100%; border: 1px solid var(--line); background: #0b0d10; }
        img.pane { object-fit: cover; }

        .box { border: 1px solid var(--line); background: var(--panel); flex-shrink: 0; }
        .box-hdr {
            font-size: 9px; letter-spacing: 2.2px; color: var(--acc);
            text-transform: uppercase; padding: 6px 10px;
            background: var(--panel-hi); border-bottom: 1px solid var(--line);
        }
        .kv { display: flex; justify-content: space-between; align-items: baseline; padding: 5px 10px; border-top: 1px solid #2a2f36; }
        .kv:first-of-type { border-top: none; }
        .kv > i { font-style: normal; font-size: 9px; letter-spacing: 1.6px; color: var(--dim); }
        .val { font-size: 14px; font-weight: 500; color: var(--txt); }
        .val.sm { font-size: 12px; }
        .grn { color: var(--grn); } .amb { color: var(--acc); } .red { color: var(--red); }

        .btngrid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; padding: 8px; }
        button {
            font-family: inherit; font-size: 10px; letter-spacing: 1.4px;
            text-transform: uppercase; padding: 10px 3px; cursor: pointer;
            border: 1px solid var(--line); background: var(--panel-hi);
            color: var(--txt); transition: background .12s, border-color .12s;
        }
        button:hover { background: #2e343c; border-color: #46505c; }
        button:active { background: #14181c; }
        .btn-go { color: var(--grn); border-color: #46612f; }
        .btn-go:hover { border-color: var(--grn); }
        .btn-clear { grid-column: span 2; color: var(--grn); border-color: #46612f; }
        .btn-clear:hover { border-color: var(--grn); }
        input, select {
            width: 100%; margin: 3px 0; padding: 5px; background: #0e1416;
            color: #dce6e2; border: 1px solid #2a3a36; border-radius: 3px;
            font-family: inherit; font-size: 11px;
        }
        .field-group { margin: 6px 0; }
        .badge.warn { color: var(--acc); border-color: #7a4e1c; }
        .note {
            font-size: 9px; color: #6b7480; letter-spacing: 0.8px;
            padding: 4px 10px 8px; line-height: 1.45;
        }
        .capture-bad {
            background: #4a1414; color: #ffd2d2; padding: 8px 10px;
            font-size: 11px; letter-spacing: 1px; line-height: 1.5;
        }
        /*__NAV_CSS__*/
    </style>
</head>
<body>
<!--__NAV__-->

<header class="topbar">
    <div class="brand">
        <svg class="mark" viewBox="0 0 32 32" xmlns="http://www.w3.org/2000/svg">
            <path d="M16 1.6 L29.1 9.3 L29.1 22.7 L16 30.4 L2.9 22.7 L2.9 9.3 Z"
                  fill="none" stroke="#e8933a" stroke-width="1.5"/>
            <path d="M8.6 18.4 L16 11 L23.4 18.4" fill="none" stroke="#e8933a" stroke-width="2"/>
            <path d="M8.6 23.2 L16 15.8 L23.4 23.2" fill="none" stroke="#e8933a"
                  stroke-width="1.3" opacity="0.45"/>
        </svg>
        <div class="brand-txt"><b>RAMBERG</b><span>ROBOTICS</span></div>
    </div>
    <div class="sysname">Data Collection</div>
    <div class="badges">
        <span class="badge" id="link_badge">LINK</span>
        <span class="badge" id="cam_badge">CAM --</span>
        <span class="badge" id="rec_badge">DATA OFF</span>
    </div>
</header>

<main class="grid">

    <section class="col">
        <div class="hdr"><span>Forward Camera</span><span class="badge" id="det_toggle_badge" onclick="toggleDetections()" style="cursor:pointer;" title="Toggle YOLO detection box overlay">DETECT: OFF</span><em>Confirm framing/exposure before starting a run</em></div>
        <img class="pane" id="camera_pane" src="/video_feed">
    </section>

    <aside class="side">
        <!-- Capture status: is the camera real, and are bytes reaching
             the disk. -->
        <div class="box" id="capture_box">
            <div class="box-hdr">Capture Integrity</div>
            <div id="capture_alert"></div>
            <div class="kv"><i>Camera Source</i><b class="val sm" id="cam_source">--</b></div>
            <div class="kv"><i>Camera Health</i><b class="val sm" id="cam_health">--</b></div>
            <div class="kv"><i>Frame Rate</i><b class="val sm" id="cam_fps">--</b></div>
            <div class="kv"><i>Intrinsics</i><b class="val sm" id="cam_calib">--</b></div>
            <div class="note">
                "Nominal" intrinsics are synthesised from a spec-sheet field
                of view with no distortion model. Fine for rough geometry,
                not calibration data &mdash; run camera_calibration and pass
                the result via camera_info_url before collecting footage
                intended for a dataset.
            </div>
        </div>

        <div class="box" id="data_box">
            <div class="box-hdr">Recording Status</div>
            <!-- Recording = a recorder process exists.
                 Writing = bytes reached the disk this second. -->
            <div class="kv"><i>Recording</i><b class="val sm" id="rec_state">OFF</b></div>
            <div class="kv"><i>Writing To Disk</i><b class="val sm" id="rec_writing">--</b></div>
            <div class="kv"><i>Bag Size</i><b class="val sm" id="rec_bag">--</b></div>
            <div class="kv"><i>Write Rate</i><b class="val sm" id="rec_rate">--</b></div>
            <div class="kv"><i>Run ID</i><b class="val sm" id="rec_run_id">--</b></div>
            <div class="kv"><i>Elapsed</i><b class="val sm" id="rec_elapsed">--</b></div>
            <div class="kv"><i>Free Disk</i><b class="val sm" id="rec_free_gb">--</b></div>
            <div class="kv"><i>Time Remaining</i><b class="val sm" id="rec_remaining">--</b></div>
            <div class="kv"><i>Open Segment</i><b class="val sm" id="rec_segment">--</b></div>
        </div>

        <div class="box" id="host_box">
            <div class="box-hdr">Boat Health</div>
            <div class="kv"><i>Pack Voltage</i><b class="val sm" id="d_pack">--</b></div>
            <div class="kv"><i>Reserve</i><b class="val sm" id="d_reserve">--</b></div>
            <div class="kv"><i>CPU Temp</i><b class="val sm" id="d_temp">--</b></div>
            <div class="kv"><i>Heading</i><b class="val sm" id="d_heading">--</b></div>
        </div>

        <div class="box">
            <div class="box-hdr">Start / Stop Run</div>
            <div style="padding: 8px;">
                <div class="field-group">
                    <input type="text" id="data_site" placeholder="Site (e.g. lakeeola)">
                    <input type="text" id="data_purpose" placeholder="Purpose (e.g. docking trials)">
                    <select id="data_source">
                        <option value="real">Real Hardware</option>
                        <option value="simulation">Simulation</option>
                    </select>
                </div>
            </div>
            <div class="btngrid">
                <button class="btn-go" onclick="startDataRun()">Start Data Run</button>
                <button class="btn-clear" onclick="stopDataRun()">Stop Recording</button>
            </div>
        </div>

        <div class="box">
            <div class="box-hdr">Segments</div>
            <div style="padding: 8px;">
                <div class="field-group">
                    <input type="text" id="segment_label" placeholder="Segment label (e.g. Dock approach)">
                </div>
            </div>
            <div class="btngrid">
                <button onclick="beginSegment()">Begin Segment</button>
                <button onclick="endSegment()">End Segment</button>
            </div>
        </div>

        <div class="box">
            <div class="box-hdr">Event Marker</div>
            <div style="padding: 8px;">
                <div class="field-group">
                    <input type="text" id="marker_note" placeholder="Marker note (optional)">
                </div>
            </div>
            <div class="btngrid">
                <button class="btn-clear" onclick="addMarker()">Mark Event</button>
            </div>
        </div>
    </aside>

</main>

<script>
let currentOpenSegments = [];  // from /telemetry recording.open_segments

function toggleDetections() {
    // Also turns detector inference on or off, not just the display.
    const img = document.getElementById('camera_pane');
    const badge = document.getElementById('det_toggle_badge');
    const isOn = badge.textContent.indexOf('ON') !== -1;
    const turningOn = !isOn;
    img.src = turningOn ? '/detections_feed' : '/video_feed';
    badge.textContent = turningOn ? 'DETECT: ON' : 'DETECT: OFF';
    fetch('/detect/enable', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({enabled: turningOn}),
    });
}

function startDataRun() {
    const site = document.getElementById('data_site').value || 'unknown';
    const purpose = document.getElementById('data_purpose').value || '';
    const source = document.getElementById('data_source').value;
    fetch('/data/start', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({site: site, purpose: purpose, source: source}),
    }).then((r) => r.json()).then((data) => {
        if (data.status !== 'ok') notifyOperator('Start failed: ' + (data.error || 'unknown error'));
    });
}

function stopDataRun() {
    fetch('/data/stop', {method: 'POST'});
}

function beginSegment() {
    const label = document.getElementById('segment_label').value || '';
    fetch('/data/segment_begin', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({label: label}),
    }).then((r) => r.json()).then((data) => {
        if (data.status !== 'ok') notifyOperator('Begin segment failed: ' + (data.error || 'unknown error'));
        else document.getElementById('segment_label').value = '';
    });
}

function endSegment() {
    // Ends the oldest open segment.
    if (!currentOpenSegments.length) {
        notifyOperator('No open segment to end.');
        return;
    }
    fetch('/data/segment_end', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({step_id: currentOpenSegments[0]}),
    }).then((r) => r.json()).then((data) => {
        if (data.status !== 'ok') notifyOperator('End segment failed: ' + (data.error || 'unknown error'));
    });
}

function addMarker() {
    const note = document.getElementById('marker_note').value || '';
    fetch('/data/marker', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({note: note}),
    });
    document.getElementById('marker_note').value = '';
}

function updateTelemetry() {
    fetch('/telemetry').then((r) => r.json()).then((data) => {
        const lb = document.getElementById('link_badge');
        lb.innerText = 'LINK OK';
        lb.className = 'badge ok';

        const rec = data.recording || {};
        currentOpenSegments = rec.open_segments || [];
        const recEl = document.getElementById('rec_state');
        recEl.innerText = rec.recording ? 'RECORDING' : 'OFF';
        recEl.className = 'val sm ' + (rec.recording ? 'grn' : '');

        // recording only says a process exists.
        const wEl = document.getElementById('rec_writing');
        if (!rec.recording) {
            wEl.innerText = '--';
            wEl.className = 'val sm';
        } else if (rec.writing) {
            wEl.innerText = 'YES';
            wEl.className = 'val sm grn';
        } else {
            wEl.innerText = 'NO'
                + ((rec.write_stall_sec !== null && rec.write_stall_sec !== undefined)
                   ? ('  ·  ' + rec.write_stall_sec.toFixed(0) + ' s stalled') : '');
            wEl.className = 'val sm red';
        }

        document.getElementById('rec_bag').innerText =
            (rec.bag_mb !== null && rec.bag_mb !== undefined)
                ? rec.bag_mb.toFixed(1) + ' MB' : '--';
        document.getElementById('rec_rate').innerText =
            (rec.write_mb_per_s !== null && rec.write_mb_per_s !== undefined)
                ? rec.write_mb_per_s.toFixed(2) + ' MB/s' : '--';
        document.getElementById('rec_run_id').innerText = rec.run_id || '--';
        document.getElementById('rec_elapsed').innerText =
            (rec.elapsed_s !== null && rec.elapsed_s !== undefined)
                ? rec.elapsed_s.toFixed(0) + ' s' : '--';

        const sys = data.system || {};
        const freeGb = (rec.free_gb !== null && rec.free_gb !== undefined)
            ? rec.free_gb : (sys.disk ? sys.disk.free_gb : null);
        const fgEl = document.getElementById('rec_free_gb');
        fgEl.innerText = (freeGb === null || freeGb === undefined)
            ? '--' : freeGb.toFixed(1) + ' GB';
        fgEl.className = 'val sm ' + (freeGb === null || freeGb === undefined ? ''
            : freeGb < 5 ? 'red' : freeGb < 20 ? 'amb' : 'grn');

        // Minutes, not gigabytes. At about 7 GB/hour, the 2 GB threshold is
        // roughly 15 minutes.
        const mins = (rec.est_minutes_remaining !== null && rec.est_minutes_remaining !== undefined)
            ? rec.est_minutes_remaining
            : (sys.disk ? sys.disk.est_imagery_minutes : null);
        const rmEl = document.getElementById('rec_remaining');
        rmEl.innerText = (mins === null || mins === undefined)
            ? '--' : ('~' + Math.round(mins) + ' min'
                      + (rec.est_minutes_remaining !== null
                         && rec.est_minutes_remaining !== undefined
                         ? ' (measured)' : ' (est.)'));
        rmEl.className = 'val sm ' + (mins === null || mins === undefined ? ''
            : mins < 15 ? 'red' : mins < 45 ? 'amb' : 'grn');

        document.getElementById('rec_segment').innerText =
            currentOpenSegments.length ? currentOpenSegments.join(', ') : '--';

        const recBadge = document.getElementById('rec_badge');
        if (rec.recording) {
            recBadge.innerText = (rec.writing === false ? 'REC! NOT WRITING ' : 'REC ')
                + (rec.elapsed_s !== null && rec.elapsed_s !== undefined
                   ? rec.elapsed_s.toFixed(0) + 's' : '');
            recBadge.className = 'badge bad';
        } else {
            recBadge.innerText = 'DATA OFF';
            recBadge.className = 'badge';
        }

        updateCapture(data);
        updateHealth(data);
        updateBanner(data);
        updateNavNote(data);
    }).catch(() => {
        const lb = document.getElementById('link_badge');
        lb.innerText = 'NO LINK';
        lb.className = 'badge bad';
    });
}

function updateCapture(data) {
    const cam = data.camera || {};
    const rec = data.recording || {};
    const alertEl = document.getElementById('capture_alert');
    const srcEl = document.getElementById('cam_source');
    const healthEl = document.getElementById('cam_health');
    const fpsEl = document.getElementById('cam_fps');
    const calEl = document.getElementById('cam_calib');
    const badge = document.getElementById('cam_badge');
    const box = document.getElementById('capture_box');

    // A test pattern and real footage look the same everywhere else.
    let alerts = [];
    if (cam.mock_mode) {
        alerts.push('CAMERA IS IN MOCK MODE &mdash; the video above is a '
                  + 'synthetic test pattern, not the world. Anything recorded '
                  + 'now is worthless.');
    } else if (cam.healthy === false) {
        alerts.push('CAMERA UNHEALTHY &mdash; ' + (cam.detail || 'no frames arriving.'));
    }
    if (rec.recording && rec.writing === false) {
        alerts.push('RECORDING BUT NOT WRITING &mdash; the bag has not grown. '
                  + 'The recorder process is alive but no data is reaching '
                  + 'the disk; check that the topics it was told to record '
                  + 'have publishers.');
    }
    alertEl.innerHTML = alerts.length
        ? ('<div class="capture-bad">' + alerts.join('<br><br>') + '</div>') : '';
    box.style.borderColor = alerts.length ? 'var(--red)' : 'var(--line)';

    if (cam.source === undefined || cam.source === null) {
        // No boat/camera/status at all.
        srcEl.innerText = 'NO STATUS TOPIC';
        srcEl.className = 'val sm red';
        srcEl.title = 'camera_publisher is not publishing boat/camera/status';
        healthEl.innerText = '--'; healthEl.className = 'val sm';
        fpsEl.innerText = '--'; fpsEl.className = 'val sm';
        calEl.innerText = '--'; calEl.className = 'val sm';
        badge.innerText = 'CAM ?';
        badge.className = 'badge warn';
        return;
    }

    const SRC = ['UNKNOWN', 'REAL CAMERA', 'MOCK TEST PATTERN'];
    srcEl.innerText = SRC[cam.source] || ('?' + cam.source);
    srcEl.className = 'val sm ' + (cam.source === 1 ? 'grn' : 'red');

    healthEl.innerText = cam.healthy ? 'OK' : 'NOT OK';
    healthEl.className = 'val sm ' + (cam.healthy ? 'grn' : 'red');
    healthEl.title = cam.detail || '';

    const fps = cam.measured_fps, want = cam.configured_fps || 10;
    fpsEl.innerText = (fps === null || fps === undefined)
        ? '--' : fps.toFixed(1) + ' / ' + want.toFixed(0) + ' fps';
    // Measured frame rate. Thermal throttling shows up here first.
    fpsEl.className = 'val sm ' + ((fps === null || fps === undefined) ? ''
        : fps < want * 0.6 ? 'red' : fps < want * 0.9 ? 'amb' : 'grn');

    calEl.innerText = cam.calibration_measured ? 'MEASURED' : 'NOMINAL (synthesised)';
    calEl.className = 'val sm ' + (cam.calibration_measured ? 'grn' : 'amb');
    calEl.title = cam.calibration_source || '';

    badge.innerText = cam.mock_mode ? 'CAM MOCK'
        : cam.healthy ? 'CAM OK' : 'CAM BAD';
    badge.className = 'badge ' + (cam.mock_mode || !cam.healthy ? 'bad' : 'ok');
}

function updateHealth(data) {
    const pwr = data.power || {};
    const sys = data.system || {};
    const hdg = data.heading_est || {};

    const packEl = document.getElementById('d_pack');
    packEl.innerText = (pwr.seen && pwr.pack_voltage_v !== null
                        && pwr.pack_voltage_v !== undefined)
        ? pwr.pack_voltage_v.toFixed(2) + ' V'
        : (pwr.seen ? '--' : 'NO MONITOR');
    packEl.className = 'val sm ' + (pwr.seen ? '' : 'red');

    const resEl = document.getElementById('d_reserve');
    if (pwr.reserve_frac === null || pwr.reserve_frac === undefined) {
        resEl.innerText = pwr.reserve_state || '--';
        resEl.className = 'val sm';
    } else {
        resEl.innerText = Math.round(pwr.reserve_frac * 100) + ' %  ·  ' + pwr.reserve_state;
        resEl.className = 'val sm ' + (pwr.reserve_state === 'OK' ? 'grn'
            : pwr.reserve_state === 'WARN' ? 'amb' : 'red');
    }

    const t = sys.cpu_temp_c;
    const tEl = document.getElementById('d_temp');
    tEl.innerText = (t === null || t === undefined) ? '--' : t.toFixed(1) + ' °C';
    tEl.className = 'val sm ' + ((t === null || t === undefined) ? ''
        : t >= (sys.temp_critical_c || 80) ? 'red'
        : t >= (sys.temp_warn_c || 70) ? 'amb' : 'grn');

    const hEl = document.getElementById('d_heading');
    const HDG_Q = ['NONE', 'DEAD RECKONING', 'FUSED'];
    if (hdg.quality === undefined || hdg.quality === null) {
        hEl.innerText = 'NO ESTIMATOR';
        hEl.className = 'val sm red';
    } else {
        hEl.innerText = ((hdg.heading_deg !== null && hdg.heading_deg !== undefined
                          && hdg.quality > 0) ? hdg.heading_deg.toFixed(0) + '°  ·  ' : '')
                      + (HDG_Q[hdg.quality] || hdg.quality);
        hEl.className = 'val sm ' + (hdg.quality >= 2 ? 'grn'
            : hdg.quality === 1 ? 'amb' : 'red');
        hEl.title = hdg.detail || '';
    }
}

//__BANNER_JS__

setInterval(updateTelemetry, 1000);
updateTelemetry();
</script>
</body>
</html>
"""


# Build both pages once at import time.

def _compose(page, active):
    """Splice the nav bar, banner and shared JS into a page template."""
    out = page.replace('/*__NAV_CSS__*/', NAV_CSS)
    out = out.replace('<!--__NAV__-->', NAV_HTML)
    out = out.replace('//__BANNER_JS__', BANNER_JS)
    out = out.replace('//__OPERATOR_JS__', Path(__file__).with_name('operator_controls.js').read_text(encoding='utf-8'))
    out = out.replace('NAV_ACTIVE_CONTROL', 'active' if active == 'control' else '')
    out = out.replace('NAV_ACTIVE_DATA', 'active' if active == 'data' else '')
    return out


CONTROL_PAGE_HTML = _compose(HTML_PAGE, 'control')
DATA_PAGE_HTML = _compose(DATA_PAGE, 'data')


@app.before_request
def validate_command_body():
    if request.method == 'POST' and request.is_json:
        if not isinstance(request.get_json(silent=True), dict):
            return jsonify({'status': 'error', 'error': 'JSON body must be an object'}), 400


@app.route('/')
def index():
    return render_template_string(CONTROL_PAGE_HTML,
                                  manual_thrust_limit=ros_node.manual_thrust_limit if ros_node else 0)


@app.route('/data')
def data_page():
    return render_template_string(DATA_PAGE_HTML)


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
def thrust():
    """Manual thrust from the dashboard.

    Only sent while the browser holds the manual lease, so 0/0 here is a
    real stop command, not missing input.
    """
    data = request.get_json(silent=True) or {}
    try:
        left = float(data.get('left', 0))
        right = float(data.get('right', 0))
    except (TypeError, ValueError):
        return jsonify({'status': 'error', 'error': 'left/right must be numbers'}), 400

    # Reject NaN/inf before clamping. json.loads accepts them and min/max
    # does not clamp NaN.
    if not (math.isfinite(left) and math.isfinite(right)):
        return jsonify({'status': 'error',
                        'error': 'left/right must be finite numbers'}), 400

    if not ros_node:
        return jsonify({'status': 'error', 'error': 'ROS node not ready'}), 503

    if latest_operator.get('session'):
        return jsonify({'status': 'error', 'error': 'Reload dashboard: sequenced controls are active'}), 409

    limit = ros_node.manual_thrust_limit
    left = max(-limit, min(limit, left))
    right = max(-limit, min(limit, right))
    ros_node.publish_thrust(left, right)
    # Echo what was published after clamping.
    return jsonify({'status': 'ok',
                    'left': max(-100.0, min(100.0, left)),
                    'right': max(-100.0, min(100.0, right))})


@app.route('/control_mode', methods=['POST'])
def control_mode():
    """Acquire, hold, release or hand back the manual lease.

    MANUAL   a human is driving. Sent every 200 ms as the heartbeat.
    RELEASE  the operator lets go. Thrust latches to zero, autonomy does not
             resume.
    RESUME   the operator allows autonomy to drive again.
    """
    data = request.get_json(silent=True) or {}
    mode = str(data.get('mode', '')).strip().upper()
    if mode not in ('MANUAL', 'RELEASE', 'RESUME'):
        return jsonify({'status': 'error',
                        'error': 'mode must be MANUAL, RELEASE or RESUME'}), 400
    if not ros_node:
        return jsonify({'status': 'error', 'error': 'ROS node not ready'}), 503

    if latest_operator.get('session'):
        return jsonify({'status': 'error', 'error': 'Reload dashboard: sequenced controls are active'}), 409

    ros_node.publish_control_mode(mode)

    # Do not wait on the 5 Hz MANUAL heartbeat, only on the one-shot
    # transitions.
    if mode == 'MANUAL':
        return jsonify({'status': 'ok', 'mode': mode})

    if mode == 'RELEASE':
        expected = 'HOLD'
        ok = _await(lambda: _snapshot()[2] == 'HOLD')
    else:
        # Lands in WAYPOINT if a mission is running, otherwise IDLE.
        expected = 'IDLE or WAYPOINT'
        ok = _await(lambda: _snapshot()[2] in ('IDLE', 'WAYPOINT'))

    _, _, observed = _snapshot()
    if not ok:
        return jsonify({
            'status': 'error',
            'error': f'arbiter did not reach {expected} within '
                     f'{CONTROL_ACK_TIMEOUT_SEC:.1f}s (boat/mode is {observed}). '
                     f'Is control_arbiter running?',
            'mode': observed,
        }), 504
    return jsonify({'status': 'ok', 'mode': observed})


@app.route('/operator', methods=['POST'])
def operator_command():
    """An atomic, sequenced command. Success means the arbiter acknowledged it."""
    data = request.get_json(silent=True) or {}
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
    key = (session, seq)
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


@app.route('/mission', methods=['POST'])
def mission():
    """Upload a route and report whether mission_manager accepted it.

    A few quick checks give a clear error for common mistakes. Success means
    boat/state reached EXECUTE.
    """
    data = request.get_json(silent=True) or {}
    latitudes = data.get('latitudes', [])
    longitudes = data.get('longitudes', [])
    if not isinstance(latitudes, list) or not isinstance(longitudes, list) or not latitudes or len(latitudes) != len(longitudes):
        return jsonify({'status': 'error', 'error': 'empty or mismatched lat/lon arrays'}), 400
    try:
        latitudes = [float(x) for x in latitudes]
        longitudes = [float(x) for x in longitudes]
    except (TypeError, ValueError):
        return jsonify({'status': 'error', 'error': 'lat/lon must be numbers'}), 400
    if not all(math.isfinite(x) for x in latitudes + longitudes):
        return jsonify({'status': 'error', 'error': 'lat/lon must be finite'}), 400

    if not all(-90 <= x <= 90 for x in latitudes) or not all(-180 <= x <= 180 for x in longitudes):
        return jsonify({'status': 'error', 'error': 'lat/lon out of range'}), 400
    state, estopped, _mode = _snapshot()

    # Quick rejections with a clear reason.
    if estopped:
        return jsonify({'status': 'error',
                        'error': 'engine stop is engaged -- press Reset to Hold first',
                        'state': state}), 409
    if state not in ('IDLE', 'ABORTED'):
        return jsonify({
            'status': 'error',
            'error': f'boat is {state}, not IDLE/ABORTED -- press Cancel to '
                     f'return to IDLE, then send again',
            'state': state}), 409

    if not ros_node:
        return jsonify({'status': 'error', 'error': 'ROS node not ready'}), 503

    ros_node.publish_mission(latitudes, longitudes)

    if not _await(lambda: _snapshot()[0] == 'EXECUTE'):
        observed, _, _ = _snapshot()
        return jsonify({
            'status': 'error',
            'error': f'mission was published but the boat never entered EXECUTE '
                     f'(boat/state is {observed} after {CONTROL_ACK_TIMEOUT_SEC:.1f}s). '
                     f'Check the mission_manager log.',
            'state': observed}), 504

    # Accepted. Warn if the manual lease is held, since the route will not
    # drive until control is handed back.
    _, _, mode = _snapshot()
    warning = None
    if mode in ('MANUAL', 'HOLD'):
        warning = (f'Mission accepted, but control is {mode} -- press '
                   f'"Continue Mission" to let the boat drive it.')
    return jsonify({'status': 'ok', 'state': 'EXECUTE', 'mode': mode,
                    'warning': warning})


@app.route('/mission/cancel', methods=['POST'])
def mission_cancel():
    """Cancel the running mission and return to IDLE."""
    if not ros_node:
        return jsonify({'status': 'error', 'error': 'ROS node not ready'}), 503
    ros_node.publish_mission_cancel()
    if not _await(lambda: _snapshot()[0] not in ('EXECUTE',)):
        observed, _, _ = _snapshot()
        return jsonify({
            'status': 'error',
            'error': f'cancel published but boat/state is still {observed}',
            'state': observed}), 504
    observed, _, _ = _snapshot()
    return jsonify({'status': 'ok', 'state': observed})


@app.route('/estop', methods=['POST'])
def estop():
    data = request.get_json(silent=True) or {}
    active = bool(data.get('active', True))
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
            'recording': latest_recording,
            'dock_marker': latest_dock_marker,
            # Each is an empty dict when its publisher is absent, and the
            # page shows that as "no data".
            'failsafe': latest_failsafe,
            'heading_est': latest_heading,
            'camera': latest_camera,
            'power': _power_snapshot(),
            'system': latest_system,
        })


@app.route('/data/start', methods=['POST'])
def data_start():
    data = request.get_json() or {}
    cmd = {
        'action': 'start',
        'site': data.get('site') or 'unknown',
        'purpose': data.get('purpose', ''),
        'source': data.get('source') or 'simulation',
    }
    if ros_node:
        ros_node.publish_data_command(cmd)
    return jsonify({'status': 'ok'})


@app.route('/data/stop', methods=['POST'])
def data_stop():
    if ros_node:
        ros_node.publish_data_command({'action': 'stop'})
    return jsonify({'status': 'ok'})


@app.route('/data/marker', methods=['POST'])
def data_marker():
    data = request.get_json() or {}
    if ros_node:
        ros_node.publish_data_command({'action': 'marker', 'note': data.get('note', '')})
    return jsonify({'status': 'ok'})


@app.route('/detect/enable', methods=['POST'])
def detect_enable():
    """Turn detector_node inference on or off (not just the displayed feed)."""
    data = request.get_json() or {}
    enabled = bool(data.get('enabled', True))
    if ros_node:
        ros_node.publish_detect_enable(enabled)
    return jsonify({'status': 'ok'})


@app.route('/data/segment_begin', methods=['POST'])
def data_segment_begin():
    data = request.get_json() or {}
    label = data.get('label', '')
    # step_id is left for the recorder to generate.
    if ros_node:
        ros_node.publish_data_command({'action': 'segment_begin', 'label': label})
    return jsonify({'status': 'ok'})


@app.route('/data/segment_end', methods=['POST'])
def data_segment_end():
    data = request.get_json() or {}
    step_id = data.get('step_id')
    if not step_id:
        return jsonify({'status': 'error', 'error': 'step_id required'}), 400
    if ros_node:
        ros_node.publish_data_command({'action': 'segment_end', 'step_id': step_id})
    return jsonify({'status': 'ok'})


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


def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            frame = latest_frame
        if frame is None:
            # No frame yet, sleep instead of spinning.
            time.sleep(0.05)
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


def generate_detections_frames():
    """Like generate_frames() but for the detector overlay. Yields nothing
    if detector_node is not running."""
    global latest_detections_frame
    while True:
        with detections_frame_lock:
            frame = latest_detections_frame
        if frame is None:
            time.sleep(0.05)
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


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
        self.mission_pub = self.create_publisher(MissionCommand, 'boat/mission_command', 10)
        self.estop_pub = self.create_publisher(Bool, 'boat/estop', 10)
        # Manual lease: MANUAL / RELEASE / RESUME, see boat_control/arbitration.py.
        self.control_mode_pub = self.create_publisher(
            String, 'boat/cmd/control_mode', 10)
        # Operator mission cancel.
        self.mission_cancel_pub = self.create_publisher(
            Empty, 'boat/mission_cancel', 10)
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
            if data.get('request_session') is not None:
                operator_receipts[(data['request_session'], data['request_seq'])] = data
                while len(operator_receipts) > 256:
                    del operator_receipts[next(iter(operator_receipts))]

    def image_callback(self, msg):
        global latest_frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with frame_lock:
            latest_frame = frame

    def detections_image_callback(self, msg):
        global latest_detections_frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with detections_frame_lock:
            latest_detections_frame = frame

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
        global latest_recording
        try:
            parsed = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        with telemetry_lock:
            latest_recording = parsed

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

    def publish_mission_cancel(self):
        self.mission_cancel_pub.publish(Empty())

    def publish_mission(self, latitudes, longitudes):
        msg = MissionCommand()
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
    # No authentication, and it binds to all interfaces: anyone on the
    # network can drive the boat. Only use on a trusted network.
    print('WARNING: boat_web_ui has no authentication and is listening on 0.0.0.0:5000')

    # Print every candidate URL since the Pi's IP changes with the network.
    ips = get_reachable_ips()
    if ips:
        print('Dashboard reachable at:')
        for ip in ips:
            print('  http://%s:5000' % ip)
    else:
        print('Could not auto-detect an IP address -- run `hostname -I` '
              'on the Pi manually and open http://<that IP>:5000')

    app.run(host='0.0.0.0', port=5000, threaded=True)


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = WebServerNode()

    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
