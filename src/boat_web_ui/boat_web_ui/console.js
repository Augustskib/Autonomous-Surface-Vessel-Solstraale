// SOLSTRÅLE operator console. One page: map or camera as the main view, the
// other as a corner inset, tab panels in a drawer. No ROS logic here; every
// command goes through the acknowledged HTTP routes in web_server.py.

const $ = (id) => document.getElementById(id);
const isNum = (v) => typeof v === 'number' && Number.isFinite(v);

// Browser-local settings (map view, layers, layout). Never control state.
const store = {
    get(key, fallback) {
        try {
            const raw = localStorage.getItem('asv.' + key);
            return raw === null ? fallback : JSON.parse(raw);
        } catch (_) { return fallback; }
    },
    set(key, value) {
        try { localStorage.setItem('asv.' + key, JSON.stringify(value)); } catch (_) {}
    },
};

function setText(id, text, cls) {
    const el = $(id);
    if (!el) return;
    el.innerText = text;
    el.className = cls || '';
}

function fmtDist(m) {
    if (!isNum(m)) return '--';
    if (m >= 1000) return (m / 1000).toFixed(2) + ' km';
    return m.toFixed(1) + ' m';
}

function fmtEta(sec) {
    if (!isNum(sec)) return '--';
    if (sec > 5400) return '> 1h 30m';
    const m = Math.floor(sec / 60);
    const s = Math.round(sec % 60);
    return m + 'm ' + String(s).padStart(2, '0') + 's';
}

// --- Drive state shared with operator_controls.js -------------------------
let left = 0, right = 0;
const keys = {};
// Mirrors the mission state, so Clear Points does not wipe a running route.
let missionActive = false;

// Thrust bars grow from the centre: forward to the right, reverse to the left.
function setBar(id, val) {
    const el = $(id);
    const pct = Math.abs(val) / 2;  // -100..100 maps to 0..50% of the track
    el.style.left = val >= 0 ? '50%' : (50 - pct) + '%';
    el.style.background = val >= 0 ? 'var(--grn)' : 'var(--amb)';
    el.style.width = pct + '%';
}

// --- Operator messages ----------------------------------------------------
// operator_controls.js writes the text into #operator_notice and calls
// logOperatorEvent. The message shows as a toast and stays in the Log tab.
const operatorEvents = [];
let toastTimer = null;
function logOperatorEvent(message) {
    const log = $('operator_event_log');
    if (log) {
        operatorEvents.unshift(new Date().toLocaleTimeString() + '  ' + message);
        operatorEvents.length = Math.min(operatorEvents.length, 80);
        log.textContent = operatorEvents.join(String.fromCharCode(10));
    }
    const toast = $('toast');
    if (toast) {
        toast.hidden = false;
        clearTimeout(toastTimer);
        toastTimer = setTimeout(dismissToast, 8000);
    }
}
function dismissToast() {
    const toast = $('toast');
    if (toast) toast.hidden = true;
}

//__OPERATOR_JS__

// --- Map ------------------------------------------------------------------
const savedView = store.get('map_view', null);
let mapPlaced = !!savedView;
const map = L.map('map', {zoomControl: true, attributionControl: true, maxZoom: 22})
    .setView(savedView ? [savedView.lat, savedView.lon] : [20, 0], savedView ? savedView.zoom : 2);
// OSM serves tiles to zoom 19. Above that the tiles are upscaled, which is
// what makes a LiDAR scan readable at close range.
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; OpenStreetMap', maxZoom: 22, maxNativeZoom: 19,
}).addTo(map);

map.on('moveend', () => {
    const c = map.getCenter();
    store.set('map_view', {lat: c.lat, lon: c.lng, zoom: map.getZoom()});
});
map.on('move zoom viewreset resize', () => redrawLidar());

// First useful position wins: a saved view, else the boat, else the fence.
function placeMapOnce(lat, lon) {
    if (mapPlaced || !isNum(lat) || !isNum(lon)) return;
    map.setView([lat, lon], 19);
    mapPlaced = true;
}

function refreshMapSize() {
    setTimeout(() => { map.invalidateSize(); redrawLidar(); }, 60);
}
window.addEventListener('resize', refreshMapSize);

const fenceGroup = L.layerGroup().addTo(map);
const routeGroup = L.layerGroup().addTo(map);
const marksGroup = L.layerGroup().addTo(map);

const boatIcon = L.divIcon({
    className: 'wp-wrap',
    html: '<div id="boat_icon" class="boat"><svg viewBox="0 0 28 28">'
        + '<path d="M14 2 L22 24 L14 19 L6 24 Z" fill="#1f7fd1" stroke="#fff" stroke-width="2" stroke-linejoin="round"/>'
        + '</svg></div>',
    iconSize: [28, 28], iconAnchor: [14, 14],
});
const boatMarker = L.marker([0, 0], {opacity: 0, icon: boatIcon, interactive: false}).addTo(map);
// Where the boat is drawn: {lat, lon, heading, assumed}. assumed means there
// is no GPS fix and the geofence centre is used for display only.
let boatPose = null;
let lastFix = null;  // [lat, lon] of the most recent real fix (smoothed when still)

// While heading_estimator reports no motion beyond the GPS noise, the marker
// eases toward each fix instead of hopping with the jitter. A jump of more
// than 50 m (reacquired fix) or any real motion shows the raw fix at once.
// Display only: navigation and the geofence use the raw fix.
const STILL_SMOOTHING = 0.15;
let shownFix = null;
function smoothFix(lat, lon, moving) {
    if (!shownFix || moving) { shownFix = [lat, lon]; return shownFix; }
    const dLat = lat - shownFix[0], dLon = lon - shownFix[1];
    const metres = Math.hypot(dLat * 111320, dLon * 111320 * Math.cos(lat * Math.PI / 180));
    if (metres > 50) { shownFix = [lat, lon]; return shownFix; }
    shownFix = [shownFix[0] + STILL_SMOOTHING * dLat, shownFix[1] + STILL_SMOOTHING * dLon];
    return shownFix;
}

function centerOnBoat() {
    if (!boatPose) { notifyOperator('No boat position to center on yet.'); return; }
    map.setView([boatPose.lat, boatPose.lon], Math.max(map.getZoom(), 19));
}

// --- Layers ---------------------------------------------------------------
const layers = Object.assign(
    {lidar: false, fence: true, route: true, marks: true, follow: false},
    store.get('layers', {}));
let lidarYaw = store.get('lidar_yaw', 0);

function applyLayers() {
    const groups = {fence: fenceGroup, route: routeGroup, marks: marksGroup};
    for (const name of Object.keys(groups)) {
        if (layers[name]) groups[name].addTo(map); else map.removeLayer(groups[name]);
    }
    if (!lidarWanted()) lidarScan = null;
    redrawLidar();
}
for (const name of Object.keys(layers)) {
    const box = $('layer_' + name);
    if (!box) continue;
    box.checked = layers[name];
    box.addEventListener('change', () => {
        layers[name] = box.checked;
        store.set('layers', layers);
        applyLayers();
    });
}
const yawInput = $('lidar_yaw');
if (yawInput) {
    yawInput.value = lidarYaw;
    yawInput.addEventListener('input', () => {
        const v = parseFloat(yawInput.value);
        lidarYaw = isNum(v) ? v : 0;
        store.set('lidar_yaw', lidarYaw);
        redrawLidar();
    });
}

// --- LiDAR overlay ----------------------------------------------------------
// /lidar/scan returns the latest preview cloud flattened to x,y centimetres in
// the sensor frame (x forward, y left). It is drawn around the boat marker and
// rotated by the displayed heading. Display only: nothing navigates on it.
let lidarScan = null;
let lidarBusy = false;

function redrawLidar() {
    const canvas = $('lidar_canvas');
    if (!canvas || !canvas.getContext) return;
    const size = map.getSize();
    if (canvas.width !== size.x || canvas.height !== size.y) {
        canvas.width = size.x;
        canvas.height = size.y;
    }
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (!layers.lidar || !lidarScan || !lidarScan.fresh || !boatPose) return;
    const origin = map.latLngToContainerPoint([boatPose.lat, boatPose.lon]);
    const metresPerPixel = 40075016.686 * Math.cos(boatPose.lat * Math.PI / 180)
        / Math.pow(2, map.getZoom() + 8);
    const h = ((boatPose.heading || 0) + lidarYaw) * Math.PI / 180;
    const sin = Math.sin(h), cos = Math.cos(h);
    const dot = metresPerPixel < 0.1 ? 3 : 2;
    const xy = lidarScan.xy || [];
    ctx.fillStyle = 'rgba(57, 255, 136, 0.9)';
    for (let i = 0; i + 1 < xy.length; i += 2) {
        const x = xy[i] / 100, y = xy[i + 1] / 100;
        const east = x * sin - y * cos;
        const north = x * cos + y * sin;
        ctx.fillRect(origin.x + east / metresPerPixel - dot / 2,
                     origin.y - north / metresPerPixel - dot / 2, dot, dot);
    }
}

// Scans are fetched while the map layer is on or the LiDAR view is showing.
function lidarWanted() { return layers.lidar || mainView === 'lidar'; }

async function pollLidar() {
    if (!lidarWanted() || lidarBusy) return;
    lidarBusy = true;
    try {
        const response = await fetch('/lidar/scan');
        lidarScan = response.ok ? await response.json() : null;
    } catch (_) {
        lidarScan = null;
    } finally {
        lidarBusy = false;
    }
    setText('lidar_points', lidarScan && lidarScan.fresh ? String(lidarScan.count) : '--');
    redrawLidar();
    drawRadar();
}
setInterval(pollLidar, 500);

// --- LiDAR view ----------------------------------------------------------------
// Top-down, boat at the centre, bow up, in the sensor frame (x forward,
// y left) turned by the yaw offset. Independent of GPS and heading, so it
// works indoors and before the mount is measured. The range follows the scan
// unless the operator zooms with the mouse wheel; double click returns to auto.
let radarRangeM = null;  // null = automatic

function niceStep(rangeM) {
    for (const step of [0.5, 1, 2, 5, 10, 20, 50]) if (rangeM / step <= 5) return step;
    return 100;
}

function autoRange(xy) {
    const dists = [];
    for (let i = 0; i + 1 < xy.length; i += 2) dists.push(Math.hypot(xy[i], xy[i + 1]) / 100);
    if (!dists.length) return 10;
    dists.sort((a, b) => a - b);
    const p95 = dists[Math.floor(dists.length * 0.95)];
    return Math.min(100, Math.max(2, p95 * 1.15));
}

function drawRadar() {
    const canvas = $('radar_canvas');
    if (!canvas || mainView !== 'lidar') return;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, w, h);
    const fresh = lidarScan && lidarScan.fresh;
    const xy = fresh ? (lidarScan.xy || []) : [];
    const rangeM = radarRangeM || autoRange(xy);
    const cx = w / 2, cy = h / 2;
    const pxPerM = (Math.min(w, h) / 2 - 20) / rangeM;

    // Range rings and labels.
    const step = niceStep(rangeM);
    ctx.strokeStyle = '#1e2a36';
    ctx.fillStyle = '#5d6b7a';
    ctx.font = '11px system-ui, sans-serif';
    ctx.lineWidth = 1;
    for (let r = step; r <= rangeM + 1e-6; r += step) {
        ctx.beginPath();
        ctx.arc(cx, cy, r * pxPerM, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.fillText(r + ' m', cx + 4, cy - r * pxPerM - 3);
    }
    ctx.beginPath();
    ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
    ctx.moveTo(0, cy); ctx.lineTo(w, cy);
    ctx.stroke();

    // Points. Screen up is the bow.
    const yaw = lidarYaw * Math.PI / 180;
    const sin = Math.sin(yaw), cos = Math.cos(yaw);
    ctx.fillStyle = 'rgba(57, 255, 136, 0.9)';
    for (let i = 0; i + 1 < xy.length; i += 2) {
        const x = xy[i] / 100, y = xy[i + 1] / 100;
        // Same rotation as the map overlay with heading 0.
        const fwd = x * cos + y * sin;
        const right = x * sin - y * cos;
        ctx.fillRect(cx + right * pxPerM - 1, cy - fwd * pxPerM - 1, 2, 2);
    }

    // Boat, bow up.
    ctx.fillStyle = '#1f7fd1';
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cx, cy - 12); ctx.lineTo(cx + 7, cy + 9); ctx.lineTo(cx, cy + 5); ctx.lineTo(cx - 7, cy + 9);
    ctx.closePath(); ctx.fill(); ctx.stroke();

    $('radar_note').textContent = fresh
        ? lidarScan.count + ' points, range ' + rangeM.toFixed(1) + ' m'
          + (radarRangeM ? ' (manual, double click for auto)' : ' (auto)')
        : (lidarScan && lidarScan.error ? 'LiDAR: ' + lidarScan.error : 'Waiting for LiDAR');
}

const radarCanvas = $('radar_canvas');
if (radarCanvas) {
    radarCanvas.addEventListener('wheel', (event) => {
        event.preventDefault();
        const current = radarRangeM || autoRange(lidarScan && lidarScan.fresh ? lidarScan.xy || [] : []);
        radarRangeM = Math.min(100, Math.max(1, current * (event.deltaY > 0 ? 1.2 : 1 / 1.2)));
        drawRadar();
    }, {passive: false});
    radarCanvas.addEventListener('dblclick', () => { radarRangeM = null; drawRadar(); });
}
window.addEventListener('resize', () => drawRadar());

// --- Main view / inset ----------------------------------------------------------
// One main view (map, camera or LiDAR). The inset shows the camera, or the
// map while the camera is the main view.
const VIEWS = ['map', 'cam', 'lidar'];
let mainView = store.get('main_view', store.get('map_main', true) ? 'map' : 'cam');
if (!VIEWS.includes(mainView)) mainView = 'map';
let mapIsMain = mainView === 'map';

function applyViews() {
    mapIsMain = mainView === 'map';
    const pip = mainView === 'cam' ? 'map' : 'cam';
    for (const view of VIEWS) {
        const role = view === mainView ? 'main' : view === pip ? 'pip' : 'off';
        $('view_' + view).className = 'view ' + role;
    }
    document.querySelectorAll('#viewpick button').forEach((button) => {
        button.classList.toggle('active', button.dataset.view === mainView);
    });
    refreshMapSize();
    if (mainView === 'lidar') { pollLidar(); drawRadar(); }
}
function setMainView(view) {
    if (!VIEWS.includes(view)) return;
    mainView = view;
    store.set('main_view', mainView);
    applyViews();
}
// The inset's swap button brings the inset to the main view.
function swapViews() {
    setMainView(mainView === 'cam' ? 'map' : 'cam');
}

// --- Tabs and menus -----------------------------------------------------------
const TABS = ['pilot', 'nav', 'recording', 'safety', 'systems', 'log'];
let activeTab = 'pilot';
function showTab(name) {
    if (!TABS.includes(name)) return;
    // Clicking the open tab again closes the drawer.
    if (name === activeTab && name !== 'pilot') name = 'pilot';
    activeTab = name;
    document.querySelectorAll('.tab').forEach((tab) => {
        tab.classList.toggle('active', tab.dataset.tab === name);
    });
    document.querySelectorAll('.panel').forEach((panel) => {
        panel.hidden = panel.dataset.panel !== name;
    });
    $('drawer').hidden = name === 'pilot';
    $('stage').classList.toggle('drawer-open', name !== 'pilot');
    refreshMapSize();
}
document.querySelectorAll('.tab').forEach((tab) => {
    tab.addEventListener('click', () => showTab(tab.dataset.tab));
});

function closeMenus(except) {
    document.querySelectorAll('.menu-list').forEach((list) => {
        if (list !== except) list.hidden = true;
    });
}
document.querySelectorAll('.menu-btn').forEach((button) => {
    button.addEventListener('click', (event) => {
        event.stopPropagation();
        const list = $(button.dataset.menu);
        closeMenus(list);
        list.hidden = !list.hidden;
    });
});
document.querySelectorAll('.menu-list').forEach((list) => {
    list.addEventListener('click', (event) => {
        event.stopPropagation();
        // Actions close the menu, checkboxes and inputs keep it open.
        if (event.target.tagName === 'BUTTON') list.hidden = true;
    });
});
document.addEventListener('click', () => closeMenus(null));

// --- Geofence -------------------------------------------------------------------
// Drawn from boat/safety_status, so it is the fence safety_monitor enforces.
let fenceKey = null;
let fenceCentre = null;
let launchMarker = null;
let fenceInputsTouched = false;

function drawGeofence(s) {
    const warnR = isNum(s.fence_warn_radius_m) ? s.fence_warn_radius_m : s.fence_radius_m * 0.9;
    const key = [s.fence_center_lat, s.fence_center_lon,
                 s.fence_radius_m, warnR, s.fence_enabled].join(',');
    if (key === fenceKey) return;
    fenceKey = key;
    fenceGroup.clearLayers();
    if (!s.fence_enabled || !isNum(s.fence_center_lat) || !isNum(s.fence_center_lon)) return;
    const centre = [s.fence_center_lat, s.fence_center_lon];
    L.circle(centre, {
        radius: s.fence_radius_m, color: '#e05757', weight: 1.5, opacity: 0.8,
        fillColor: '#e05757', fillOpacity: 0.03, dashArray: '7,6', interactive: false,
    }).addTo(fenceGroup);
    L.circle(centre, {
        radius: warnR, color: '#e0a13a', weight: 1, opacity: 0.6,
        fill: false, dashArray: '3,7', interactive: false,
    }).addTo(fenceGroup);
}

// RETURN_TO_LAUNCH point from safety_monitor, drawn apart from the dock so
// any difference is visible.
function drawLaunchPoint(fs) {
    if (!fs || !fs.launch_point_valid) return;
    const pos = [fs.launch_lat, fs.launch_lon];
    if (launchMarker) { launchMarker.setLatLng(pos); return; }
    launchMarker = L.marker(pos, {
        icon: L.divIcon({
            className: 'wp-wrap',
            html: '<div class="mark launch"><i></i><b>LAUNCH</b></div>',
            iconSize: [20, 20], iconAnchor: [10, 10],
        }),
        interactive: false,
    }).addTo(marksGroup);
}

function postFence(body) {
    return fetch('/geofence', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(body),
    }).then((r) => r.json()).then((d) => {
        if (d.status !== 'ok') notifyOperator('Geofence command failed: ' + (d.error || 'unknown'));
        fenceKey = null;   // force a redraw
    }).catch(() => notifyOperator('Geofence command not sent: no connection.'));
}

function setFenceHere() {
    const radius = parseFloat($('fence_radius_in').value);
    const warn = parseFloat($('fence_warn_in').value);
    if (!isNum(radius) || radius < 5) {
        notifyOperator('Radius must be a number of at least 5 m.');
        return;
    }
    if (!isNum(warn) || warn <= 0 || warn >= 1) {
        notifyOperator('Warning-ring fraction must be between 0 and 1 (e.g. 0.9).');
        return;
    }
    postFence({action: 'set_here', radius_m: radius, warn_fraction: warn});
}
function setLaunchHere() { postFence({action: 'set_launch_here'}); }
function enableFence() { postFence({action: 'enable'}); }

function overrideReason(purpose) {
    const reason = $('operator_reason').value.trim();
    if (reason) return reason;
    if (activeTab !== 'safety') showTab('safety');
    $('operator_reason').focus();
    notifyOperator('Enter a reason in the Safety tab before ' + purpose + '.');
    return null;
}
function disableFence() {
    const reason = overrideReason('disabling the geofence');
    if (reason) postFence({action: 'disable', reason: reason});
}

// Inputs show the enforced values until the operator starts editing.
function syncFenceInputs(s) {
    if (fenceInputsTouched) return;
    if (isNum(s.fence_radius_m)) $('fence_radius_in').value = s.fence_radius_m;
    if (isNum(s.fence_warn_fraction)) $('fence_warn_in').value = s.fence_warn_fraction;
}
['fence_radius_in', 'fence_warn_in'].forEach((id) => {
    $(id).addEventListener('input', () => { fenceInputsTouched = true; });
});

// --- Mission planning -------------------------------------------------------------
// PENDING points are plotted but not sent. ACTIVE points are the dispatched
// route, kept on the map and coloured by progress.
let pendingPoints = [];  // [[lat, lon], ...] not yet sent
let pendingMarkers = [];
let activePoints = [];
let activeMarkers = [];
let activeLine = null;
let routeBusy = false;
let routeRequest = null;
let activeExecutionId = null;

// Only the trail LineString is taken from boat/mission_geojson.
const missionLayer = L.geoJSON(null, {
    filter: (f) => f.geometry && f.geometry.type === 'LineString',
    style: () => ({color: '#4c9bd6', weight: 2, opacity: 0.7}),
}).addTo(routeGroup);

function wpIcon(n, cls) {
    return L.divIcon({
        className: 'wp-wrap',
        html: '<div class="wp ' + cls + '"><i></i><b>' + n + '</b></div>',
        iconSize: [26, 26], iconAnchor: [13, 13],
    });
}

function setWpClass(marker, cls) {
    const el = marker.getElement();
    if (!el) return;
    const w = el.querySelector('.wp');
    if (w) w.className = 'wp ' + cls;
}

function showPlotted() { $('plotted_val').innerText = pendingPoints.length; }

// Left click plots a waypoint, right click moves the dock. Only on the main
// view: clicks on the small inset map do nothing.
map.on('click', (e) => {
    if (!mapIsMain) return;
    pendingPoints.push([e.latlng.lat, e.latlng.lng]);
    pendingMarkers.push(L.marker(e.latlng, {icon: wpIcon(pendingPoints.length, 'pending')}).addTo(map));
    showPlotted();
});
map.on('contextmenu', (e) => {
    if (mapIsMain) setDock(e.latlng.lat, e.latlng.lng);
});

function clearPending() {
    pendingMarkers.forEach((m) => map.removeLayer(m));
    pendingMarkers = [];
    pendingPoints = [];
    showPlotted();
    // Also clear the dispatched route, but only when no mission is running.
    if (!missionActive) clearActive();
}

function clearActive() {
    activeMarkers.forEach((m) => routeGroup.removeLayer(m));
    activeMarkers = [];
    activePoints = [];
    if (activeLine) { routeGroup.removeLayer(activeLine); activeLine = null; }
}

function drawActiveRoute(points) {
    clearActive();
    activePoints = points.slice();
    activeLine = L.polyline(activePoints, {
        color: '#4c9bd6', weight: 1.5, opacity: 0.8, dashArray: '5,7',
    }).addTo(routeGroup);
    activePoints.forEach((p, i) => {
        activeMarkers.push(L.marker(p, {icon: wpIcon(i + 1, 'queued')}).addTo(routeGroup));
    });
}

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
        routeRequest = postCommand('/mission', {latitudes: points.map((p) => p[0]), longitudes: points.map((p) => p[1])});
        const acceptedRoute = await routeRequest;
        activeExecutionId = acceptedRoute.execution_id;
        // A takeover or stop during mission acceptance must win.
        if (operator.revision !== revision) return false;
        missionActive = true;
        drawActiveRoute(points);
        const ok = await operator.transition('AUTO');
        notifyOperator(ok ? 'Mission accepted. Navigation has control unless a safety stop is active.' : 'Mission retained; control is not confirmed.');
        return ok;
    } catch (error) {
        notifyOperator(error.message + ' Route acceptance is not confirmed; check mission status before retrying.');
        return false;
    } finally { routeBusy = false; routeRequest = null; }
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
async function resetToIdle() {
    clearDriveKeys();
    notifyOperator('Clearing E-stop into Hold. This does not resume a mission.');
    if (!await releaseLease()) return false;
    return operator.transition('RESET');
}
async function cancelMission() {
    if (!await releaseLease()) return;
    const revision = operator.revision;
    try {
        if (routeRequest) await routeRequest.catch(() => {});
        if (operator.revision !== revision) return;
        if (!activeExecutionId) throw new Error('No confirmed mission identity; refresh telemetry before cancelling.');
        await postCommand('/mission/cancel', {execution_id: activeExecutionId});
        if (operator.revision !== revision) return;
        missionActive = false; clearActive(); clearPending(); activeExecutionId = null;
        notifyOperator('Mission cancelled. Holding zero thrust.');
    } catch (error) { notifyOperator(error.message); }
}

// Dock / home position, owned by the server (see /dock).
let dockPoint = null;
let dockMarker = null;

function drawDock(lat, lon) {
    if (dockMarker && dockMarker.getLatLng().lat === lat && dockMarker.getLatLng().lng === lon) return;
    if (dockMarker) marksGroup.removeLayer(dockMarker);
    dockMarker = L.marker([lat, lon], {
        icon: L.divIcon({
            className: 'wp-wrap',
            html: '<div class="mark"><i></i><b>DOCK</b></div>',
            iconSize: [20, 20], iconAnchor: [10, 10],
        }),
        interactive: false,
    }).addTo(marksGroup);
}

function setDock(lat, lon) {
    fetch('/dock', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({lat: lat, lon: lon}),
    }).then((r) => r.json()).then((d) => {
        if (d.status !== 'ok') notifyOperator('Set dock failed: ' + (d.error || 'unknown error'));
    }).catch(() => notifyOperator('Set dock not sent: no connection.'));
}

function setDockHere() {
    if (!lastFix) { notifyOperator('No GPS fix yet, cannot set the dock to the boat position.'); return; }
    setDock(lastFix[0], lastFix[1]);
}

async function returnToDock() {
    if (!dockPoint) { notifyOperator('Dock position is not set.'); return; }
    if (missionActive) {
        notifyOperator('Cancel the current mission before starting a new route to the dock.');
        return;
    }
    await startRoute([dockPoint]);
}

// --- Recording --------------------------------------------------------------------
let currentOpenSegments = [];  // from /telemetry recording.open_segments
let recordingRows = [];
let recordingResult = '';
async function recordingRequest(url, body) {
    const output = document.getElementById('recording_feedback');
    try {
        const response = await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
        const result = await response.json();
        output.textContent = response.ok ? 'Request sent, waiting for recorder confirmation. Finalizing a large run may take time.' : (result.error || 'Request failed');
    } catch (_) { output.textContent = 'Connection lost; check recorder status before retrying.'; }
}
function saveRecordingName() {
    recordingRequest('/data/save_name', {run_id:document.getElementById('saved_recordings').value,
        name:document.getElementById('recording_name').value});
}
function renderRecordingLibrary(rec) {
    const select = document.getElementById('saved_recordings');
    const selected = select.value;
    recordingRows = rec.library || [];
    select.replaceChildren();
    for (const row of recordingRows) {
        const option = document.createElement('option');
        option.value = row.run_id;
        option.textContent = (row.name || 'Unnamed') + ' | ' + (row.started_utc || row.run_id) + ' | ' + row.state;
        select.appendChild(option);
    }
    if (recordingRows.some(row => row.run_id === selected)) select.value = selected;
    const row = recordingRows.find(row => row.run_id === select.value);
    document.getElementById('record_start').disabled = !rec.status_fresh || rec.recording;
    document.getElementById('record_stop').disabled = !rec.status_fresh || !rec.recording;
    document.getElementById('record_save').disabled = !rec.status_fresh || !row || !row.can_name;
    document.getElementById('recording_details').textContent = row ?
        (row.bytes == null ? '' : (row.bytes / 1e6).toFixed(1) + ' MB | ') + (row.folder || row.run_id) : 'No recordings listed yet.';
    const result = JSON.stringify(rec.command_result || {});
    if (rec.status_fresh && result !== recordingResult && rec.command_result) {
        document.getElementById('recording_feedback').textContent = rec.command_result.detail;
        recordingResult = result;
    }
}

function toggleDetections() {
    // Switches between the raw and YOLO feeds and turns detector_node
    // inference on or off.
    const badge = $('det_toggle_badge');
    const turningOn = badge.textContent.indexOf('ON') === -1;
    $('camera_pane').src = turningOn ? '/detections_feed' : '/video_feed';
    badge.textContent = turningOn ? 'DETECT: ON' : 'DETECT: OFF';
    fetch('/detect/enable', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({enabled: turningOn}),
    }).catch(() => notifyOperator('Detection toggle not sent: no connection.'));
}

function dataPost(url, body, failLabel, onOk) {
    fetch(url, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(body),
    }).then((r) => r.json()).then((data) => {
        if (data.status !== 'queued') notifyOperator(failLabel + ' failed: ' + (data.error || 'unknown error'));
        else if (onOk) onOk();
    }).catch(() => notifyOperator(failLabel + ' not sent: no connection.'));
}

function startDataRun() {
    const feedback = $('recording_feedback');
    fetch('/data/start', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            site: $('data_site').value || 'unknown',
            purpose: $('data_purpose').value || '',
            source: $('data_source').value,
        }),
    }).then((r) => r.json()).then((data) => {
        feedback.textContent = data.status === 'queued'
            ? 'Start requested, wait for RECORDING and disk writing confirmation.'
            : (data.error || 'Start failed');
    }).catch(() => {
        feedback.textContent = 'Connection lost; recording state is unknown. Check status before retrying.';
    });
}
function stopDataRun() { recordingRequest('/data/stop', {}); }

function beginSegment() {
    dataPost('/data/segment_begin', {label: $('segment_label').value || ''}, 'Begin segment',
             () => { $('segment_label').value = ''; });
}
function endSegment() {
    // Ends the oldest open segment.
    if (!currentOpenSegments.length) { notifyOperator('No open segment to end.'); return; }
    dataPost('/data/segment_end', {step_id: currentOpenSegments[0]}, 'End segment');
}
function addMarker() {
    dataPost('/data/marker', {note: $('marker_note').value || ''}, 'Marker',
             () => { $('marker_note').value = ''; });
}

function updateRecording(data) {
    const rawRec = data.recording || {};
    const rec = rawRec.status_fresh === true ? rawRec : {status_fresh: false};
    const sys = data.system || {};
    currentOpenSegments = rec.open_segments || [];

    setText('rec_state', !rec.status_fresh ? 'UNKNOWN / STALE' : rec.recording ? 'RECORDING' : 'OFF',
            rec.recording ? 'grn' : !rec.status_fresh ? 'amb' : '');
    // recording only says a recorder process exists; writing says bytes
    // reached the disk in the last second.
    if (!rec.recording) setText('rec_writing', '--');
    else if (rec.writing) setText('rec_writing', 'YES', 'grn');
    else setText('rec_writing', 'NO' + (isNum(rec.write_stall_sec) ? ', ' + rec.write_stall_sec.toFixed(0) + ' s stalled' : ''), 'red');

    setText('rec_bag', isNum(rec.bag_mb) ? rec.bag_mb.toFixed(1) + ' MB' : '--');
    setText('rec_rate', isNum(rec.write_mb_per_s) ? rec.write_mb_per_s.toFixed(2) + ' MB/s' : '--');
    setText('rec_run_id', rec.run_id || '--');
    setText('rec_elapsed', isNum(rec.elapsed_s) ? rec.elapsed_s.toFixed(0) + ' s' : '--');
    setText('rec_segment', currentOpenSegments.length ? currentOpenSegments.join(', ') : '--');
    renderRecordingLibrary(rec);

    const freeGb = isNum(rec.free_gb) ? rec.free_gb : (sys.disk ? sys.disk.free_gb : null);
    const freeCls = !isNum(freeGb) ? '' : freeGb < 5 ? 'red' : freeGb < 20 ? 'amb' : 'grn';
    const measured = isNum(rec.est_minutes_remaining);
    const mins = measured ? rec.est_minutes_remaining : (sys.disk ? sys.disk.est_imagery_minutes : null);
    setText('rec_free_gb', isNum(freeGb) ? freeGb.toFixed(1) + ' GB' : '--', freeCls);
    setText('rec_remaining', isNum(mins) ? '~' + Math.round(mins) + ' min' + (measured ? ' (measured)' : ' (est.)') : '--',
            !isNum(mins) ? '' : mins < 15 ? 'red' : mins < 45 ? 'amb' : 'grn');
    setText('sys_disk', isNum(freeGb) ? freeGb.toFixed(1) + ' GB' + (isNum(mins) ? ', ~' + Math.round(mins) + ' min' : '') : '--', freeCls);

    const badge = $('rec_badge');
    if (rec.recording) {
        badge.innerText = (rec.writing === false ? 'REC NOT WRITING ' : 'REC ')
            + (isNum(rec.elapsed_s) ? rec.elapsed_s.toFixed(0) + 's' : '');
        badge.className = 'chip bad';
    } else {
        badge.innerText = rec.status_fresh ? 'REC OFF' : 'REC UNKNOWN';
        badge.className = 'chip' + (rec.status_fresh ? '' : ' warn');
    }

    // A test pattern and real footage look the same in a bag, so say it here.
    const cam = data.camera || {};
    const alerts = [];
    if (!rec.status_fresh) alerts.push('Recorder status is stale. Recording and disk writing are unconfirmed.');
    if (cam.mock_mode) alerts.push('Camera is in mock mode. The video is a synthetic test pattern.');
    else if (cam.healthy === false) alerts.push('Camera unhealthy: ' + (cam.detail || 'no frames arriving.'));
    if (rec.recording && rec.writing === false) alerts.push('Recording but not writing. The bag has not grown; check that the recorded topics have publishers.');
    const alertEl = $('capture_alert');
    alertEl.replaceChildren();
    for (const text of alerts) {
        const div = document.createElement('div');
        div.className = 'capture-bad';
        div.textContent = text;
        alertEl.appendChild(div);
    }
}

// --- Telemetry rendering -------------------------------------------------------------
const HDG_Q = ['NONE', 'DEAD RECKONING', 'FUSED'];

function updateHeading(hdg, rawImuHeading) {
    if (hdg.quality === undefined || hdg.quality === null) {
        // No estimator. Show the raw IMU value labelled as drifting.
        setText('heading_val', isNum(rawImuHeading) ? rawImuHeading.toFixed(1) + '°' : '--', 'amb');
        setText('hdgq_val', 'NO ESTIMATOR', 'red');
        setText('hdgsrc_val', 'raw IMU yaw (drifts)', 'amb');
        return;
    }
    const usable = hdg.quality >= 1;
    const cls = hdg.quality >= 2 ? 'grn' : usable ? 'amb' : 'red';
    setText('heading_val', (usable && isNum(hdg.heading_deg)) ? hdg.heading_deg.toFixed(1) + '°' : '--', cls);
    setText('hdgq_val', HDG_Q[hdg.quality] || ('?' + hdg.quality), cls);
    $('hdgq_val').title = hdg.detail || '';
    // COG is course over ground, not bow heading.
    let src = hdg.source || '--';
    if (src === 'COG') src = 'COG (course, not bow)';
    if (isNum(hdg.age_since_reference_sec) && hdg.quality < 2) {
        src += ', ' + hdg.age_since_reference_sec.toFixed(0) + 's stale';
    }
    setText('hdgsrc_val', src, hdg.quality >= 2 ? '' : 'amb');
}

function attitudeClass(v) {
    if (!isNum(v)) return '';
    const a = Math.abs(v);
    return a > 45 ? 'red' : a > 20 ? 'amb' : 'grn';
}

function updateBoat(data) {
    const safety = data.safety || {};
    const hdg = data.heading_est || {};
    const heading = (isNum(hdg.heading_deg) && hdg.quality > 0) ? hdg.heading_deg
        : isNum(data.heading) ? data.heading : null;
    const note = $('pos_note');

    if (isNum(data.lat) && isNum(data.lon)) {
        const moving = isNum(hdg.ground_speed_mps) && hdg.ground_speed_mps > 0;
        lastFix = smoothFix(data.lat, data.lon, moving);
        boatPose = {lat: lastFix[0], lon: lastFix[1], heading: heading, assumed: false};
        setText('gps_val', data.lat.toFixed(6) + ', ' + data.lon.toFixed(6));
        note.hidden = true;
    } else if (isNum(safety.fence_center_lat) && isNum(safety.fence_center_lon)) {
        shownFix = null;
        // No fix (indoors). Draw the boat at the fence centre so the LiDAR
        // scan has somewhere to go. Display only.
        boatPose = {lat: safety.fence_center_lat, lon: safety.fence_center_lon,
                    heading: heading, assumed: true};
        setText('gps_val', 'NO FIX', 'red');
        note.textContent = 'No GPS fix: boat drawn at the geofence centre';
        note.hidden = false;
    } else {
        boatPose = null;
        setText('gps_val', 'NO FIX', 'red');
        note.hidden = true;
    }

    if (!boatPose) { boatMarker.setOpacity(0); return; }
    boatMarker.setLatLng([boatPose.lat, boatPose.lon]);
    boatMarker.setOpacity(1);
    const icon = $('boat_icon');
    if (icon) {
        icon.className = 'boat' + (boatPose.assumed ? ' assumed' : '');
        icon.style.transform = 'rotate(' + (heading || 0) + 'deg)';
    }
    placeMapOnce(boatPose.lat, boatPose.lon);
    if (layers.follow && !boatPose.assumed) map.panTo([boatPose.lat, boatPose.lon], {animate: false});
}

function updateNav(data, state, mode) {
    const nav = data.nav || {};
    const line = $('mission_banner');
    if (nav.active && nav.waypoint_count) {
        const n = nav.waypoint_index + 1;
        let txt = (mode === 'WAYPOINT' ? '' : 'PAUSED, ') + 'WAYPOINT ' + n + ' / ' + nav.waypoint_count;
        if (isNum(nav.distance_m)) txt += ', ' + fmtDist(nav.distance_m) + ' out';
        if (isNum(nav.eta_sec)) txt += ', ETA ' + fmtEta(nav.eta_sec);
        line.innerText = txt;
        line.className = 'mission-line active';
        styleActiveRoute(nav.waypoint_index);

        const he = nav.heading_error_deg;
        setText('wp_val', n + ' / ' + nav.waypoint_count);
        setText('dist_val', fmtDist(nav.distance_m));
        setText('brg_val', isNum(nav.bearing_deg) ? nav.bearing_deg.toFixed(1) + '°' : '--');
        setText('hdgerr_val', isNum(he) ? (he > 0 ? '+' : '') + he.toFixed(1) + '°' : '--',
                !isNum(he) ? '' : Math.abs(he) < 15 ? 'grn' : Math.abs(he) < 60 ? 'amb' : 'red');
        setText('rem_val', fmtDist(nav.remaining_m));
        setText('eta_val', fmtEta(nav.eta_sec));
        setText('prog_val', Math.round(100 * nav.waypoint_index / nav.waypoint_count) + ' %');
    } else {
        line.innerText = state === 'ABORTED'
            ? 'MISSION ABORTED: plot new waypoints or return to dock'
            : 'NO ACTIVE MISSION';
        line.className = 'mission-line';
        ['wp_val', 'dist_val', 'brg_val', 'hdgerr_val', 'rem_val', 'eta_val', 'prog_val']
            .forEach((id) => setText(id, '--'));
    }
    // A latched e-stop is the usual reason missions do nothing.
    if (data.estop) {
        line.innerText = 'ENGINE STOP ENGAGED: Reset to Hold, then choose the next action';
        line.className = 'mission-line stop';
    }

    // nav_status is m/s. km/h is for display only.
    setText('speed_val', isNum(nav.speed_mps) ? (nav.speed_mps * 3.6).toFixed(1) + ' km/h' : '--');
    const age = nav.gps_age_sec;
    if (!isNum(data.lat)) setText('gpsage_val', 'NO FIX', 'red');
    else setText('gpsage_val', isNum(age) ? age.toFixed(1) + ' s' : 'FIX',
                 !isNum(age) ? '' : age > 3 ? 'red' : age > 1.5 ? 'amb' : 'grn');
}

function updatePower(data) {
    const pwr = data.power || {};
    const sys = data.system || {};
    const meter = $('pwr_meter');
    const chans = $('pwr_channels');

    chans.replaceChildren();
    if (!pwr.seen) {
        // No power data. Different from a 0.0 V reading.
        setText('pwr_pack', 'NO MONITOR', 'amb');
        setText('pwr_reserve', '--');
        meter.firstElementChild.style.width = '0%';
    } else {
        setText('pwr_pack', isNum(pwr.pack_voltage_v) ? pwr.pack_voltage_v.toFixed(2) + ' V' : '--');
        const frac = pwr.reserve_frac;
        const state = pwr.reserve_state || 'UNKNOWN';
        if (!isNum(frac)) {
            setText('pwr_reserve', state);
            meter.firstElementChild.style.width = '0%';
            meter.className = 'meter';
        } else {
            setText('pwr_reserve', Math.round(frac * 100) + ' % above reserve, ' + state,
                    state === 'OK' ? 'grn' : state === 'WARN' ? 'amb' : 'red');
            meter.firstElementChild.style.width = (frac * 100).toFixed(0) + '%';
            meter.className = 'meter ' + (state === 'OK' ? '' : state === 'WARN' ? 'warn' : 'bad');
        }
        // ok === false means the I2C read failed, not a real zero.
        for (const c of (pwr.channels || [])) {
            const row = document.createElement('div');
            row.className = 'chan' + (c.ok ? '' : ' dead');
            const cells = [c.name, c.ok ? c.voltage_v.toFixed(2) + ' V' : 'read fail',
                           c.ok ? c.current_a.toFixed(2) + ' A' : '', c.ok ? c.power_w.toFixed(1) + ' W' : ''];
            cells.forEach((text, i) => {
                const cell = document.createElement(i === 0 ? 'i' : 'b');
                cell.textContent = text;
                row.appendChild(cell);
            });
            chans.appendChild(row);
        }
    }

    const t = sys.cpu_temp_c;
    setText('sys_temp', isNum(t) ? t.toFixed(1) + ' °C' : '--',
            !isNum(t) ? '' : t >= (sys.temp_critical_c || 80) ? 'red' : t >= (sys.temp_warn_c || 70) ? 'amb' : 'grn');
    setText('sys_load', (sys.load && sys.load.length) ? sys.load[0].toFixed(2) : '--');
    const thr = sys.throttle;
    if (!thr) setText('sys_throttle', '--');
    else if (!thr.flags || !thr.flags.length) setText('sys_throttle', 'NONE', 'grn');
    else {
        // Under-voltage here means brownouts.
        setText('sys_throttle', thr.flags.length + ' FLAG(S)', 'red');
        $('sys_throttle').title = thr.flags.join('; ');
    }
}

function updateCamera(cam) {
    if (cam.source === undefined || cam.source === null) {
        setText('cam_source', 'NO STATUS TOPIC', 'red');
        ['cam_health', 'cam_fps', 'cam_calib'].forEach((id) => setText(id, '--'));
        return;
    }
    const SRC = ['UNKNOWN', 'REAL CAMERA', 'MOCK TEST PATTERN'];
    setText('cam_source', SRC[cam.source] || ('?' + cam.source), cam.source === 1 ? 'grn' : 'red');
    setText('cam_health', cam.healthy ? 'OK' : 'NOT OK', cam.healthy ? 'grn' : 'red');
    $('cam_health').title = cam.detail || '';
    const fps = cam.measured_fps, want = cam.configured_fps || 10;
    setText('cam_fps', isNum(fps) ? fps.toFixed(1) + ' / ' + want.toFixed(0) + ' fps' : '--',
            !isNum(fps) ? '' : fps < want * 0.6 ? 'red' : fps < want * 0.9 ? 'amb' : 'grn');
    setText('cam_calib', cam.calibration_measured ? 'MEASURED' : 'NOMINAL (synthesised)',
            cam.calibration_measured ? 'grn' : 'amb');
}

function updateLidarStatus(lidar) {
    const badge = $('lidar_badge');
    badge.innerText = lidar.fresh ? 'LIDAR LIVE' : 'LIDAR OFF';
    badge.className = 'chip' + (lidar.fresh ? ' ok' : '');
    setText('lidar_state', lidar.fresh ? 'LIVE' : (lidar.error || 'OFFLINE'), lidar.fresh ? 'grn' : '');
    setText('lidar_time', lidar.timestamp_mode
        ? lidar.timestamp_mode + (lidar.clock_verified ? ' (clock verified)' : ' (clock unverified)') : '--');
}

function updateSafety(s, fs, estop) {
    setText('estop_state', estop ? 'ENGAGED' : 'CLEAR', estop ? 'red' : 'grn');
    $('reset_btn').hidden = !estop;

    const FS_NAMES = ['NOMINAL', 'HOLD POSITION', 'RETURN TO LAUNCH', 'POWERED STOP', 'HARD STOP'];
    if (fs && isNum(fs.level)) {
        setText('fs_state', (FS_NAMES[fs.level] || ('?' + fs.level)) + (fs.degraded_reason ? ' *' : ''),
                fs.level === 0 ? 'grn' : fs.level === 2 ? 'amb' : 'red');
        $('fs_state').title = fs.degraded_reason
            ? ('DEGRADED: ' + fs.degraded_reason)
            : ((fs.active_reasons && fs.active_reasons.join('; ')) || '');
    } else {
        setText('fs_state', 'NO SUPERVISOR', 'red');
    }

    if (s.fence_enabled === undefined) {
        // safety_monitor has not published.
        setText('safe_state', 'NO MONITOR', 'red');
        ['fence_state', 'fence_margin', 'fence_outside', 'gcs_link'].forEach((id) => setText(id, '--'));
        return;
    }

    // s.level is safety_monitor's own OK/WARN/ERROR (0/1/2).
    setText('safe_state',
            s.level === 2 ? (s.reason === 'GEOFENCE_BREACH' ? 'FENCE BREACH' : 'LINK LOST')
            : s.level === 1 ? 'FENCE WARNING' : 'NOMINAL',
            ['grn', 'amb', 'red'][s.level] || 'grn');

    setText('fence_state', s.fence_enabled
        ? (s.fence_radius_m.toFixed(0) + ' m (warn '
           + (isNum(s.fence_warn_radius_m) ? s.fence_warn_radius_m.toFixed(0) : '?') + ' m)')
        : 'DISABLED', s.fence_enabled ? '' : 'red');
    $('fence_state').title = s.fence_enabled ? '' : (s.fence_disabled_reason || '');

    if (!isNum(s.fence_margin_m)) setText('fence_margin', '--');
    else if (s.fence_margin_m < 0) setText('fence_margin', fmtDist(-s.fence_margin_m) + ' OUTSIDE', 'red');
    else setText('fence_margin', fmtDist(s.fence_margin_m), s.fence_warning ? 'amb' : 'grn');

    if (!isNum(s.fence_outside_sec)) setText('fence_outside', '--');
    else if (s.fence_outside_sec === 0) setText('fence_outside', 'inside', 'grn');
    else setText('fence_outside', s.fence_outside_sec.toFixed(0) + ' s', 'red');

    const age = s.gcs_heartbeat_age_sec;
    setText('gcs_link', isNum(age) ? age.toFixed(1) + ' s' : 'NO BEAT',
            s.gcs_lost ? 'red' : (isNum(age) && age > 3) ? 'amb' : 'grn');

    drawGeofence(s);
    // With no saved view and no fix, start at the fence instead of the world.
    if (s.fence_enabled) placeMapOnce(s.fence_center_lat, s.fence_center_lon);
}

// --- Failsafe strip ---------------------------------------------------------------------
// Failsafe levels, mirroring boat_msgs/msg/FailsafeState.msg.
const FS_NOMINAL = 0, FS_HOLD = 1, FS_RTL = 2, FS_PSTOP = 3;
const FS_REQUEST_NAMES = ['nominal', 'hold position', 'return to launch', 'powered stop', 'hard stop'];

function updateBanner(data) {
    const el = $('fs_banner');
    const actions = $('fs_actions');
    const fs = data.failsafe || {};
    const safety = data.safety || {};
    const firstReason = (fs.active_reasons && fs.active_reasons[0]) || fs.reason || '';

    // No supervisor gets its own state, never shown as all clear.
    if (!isNum(fs.level)) {
        el.className = 'banner none';
        $('fs_glyph').innerHTML = '&#9679;';
        $('fs_word').innerText = 'NO SAFETY SUPERVISOR';
        $('fs_why').innerText = 'safety_monitor is not publishing boat/failsafe_state. '
            + 'There is no geofence and no failsafe, and manual control is refused.';
        actions.replaceChildren();
        actions.removeAttribute('data-shown');
        return;
    }

    let cls = 'ok', glyph = '&#10003;', text = 'NOMINAL', reason = '';
    if (data.estop) {
        cls = 'bad'; glyph = '&#9632;'; text = 'ENGINE STOP ENGAGED';
        reason = 'All propulsion cut. Reset to Hold, then choose the next action.';
    } else if ((data.operator || {}).failsafe_stale) {
        cls = 'bad'; glyph = '&#9632;'; text = 'SAFETY LINK STALE';
        reason = 'Propulsion stopped: the safety supervisor stopped reporting.';
    } else if (fs.level >= FS_PSTOP) {
        cls = 'bad'; glyph = '&#9632;'; text = 'POWERED STOP';
        reason = fs.degraded_reason
            ? ('Wanted ' + (FS_REQUEST_NAMES[fs.requested_level] || 'a recovery') + ' but ' + fs.degraded_reason)
            : firstReason;
    } else if (fs.level === FS_RTL) {
        cls = 'bad'; glyph = '&#8676;';
        text = data.mode === 'FAILSAFE' ? 'RETURN-TO-LAUNCH CONTROL' : 'RETURN-TO-LAUNCH REQUESTED';
        reason = firstReason + (isNum(fs.distance_to_launch_m) ? ', ' + fs.distance_to_launch_m.toFixed(0) + ' m to go' : '');
    } else if (fs.level === FS_HOLD) {
        cls = 'bad'; glyph = '&#9632;'; text = 'HOLDING POSITION';
        reason = firstReason;
    } else if (safety.fence_enabled === false) {
        cls = 'warn'; glyph = '&#9888;'; text = 'GEOFENCE DISABLED';
        reason = safety.fence_disabled_reason
            ? ('Reason given: ' + safety.fence_disabled_reason)
            : 'No boundary is being enforced.';
    } else if (safety.fence_warning) {
        cls = 'warn'; glyph = '&#9888;'; text = 'APPROACHING GEOFENCE';
        reason = isNum(safety.fence_margin_m)
            ? (safety.fence_margin_m.toFixed(0) + ' m of margin left before the boundary')
            : 'Inside the warning ring.';
    } else if (fs.heading_usable === false) {
        // Not a failsafe by itself, but it disables return-to-launch.
        cls = 'warn'; glyph = '&#9888;'; text = 'NO USABLE HEADING';
        reason = 'Return-to-launch is unavailable and will degrade to a powered stop. '
            + ((data.heading_est || {}).detail || '');
    } else if (safety.fence_enabled) {
        reason = 'Geofence armed' + (isNum(safety.fence_margin_m) ? ', ' + safety.fence_margin_m.toFixed(0) + ' m margin' : '');
    }

    // Latched: conditions gone but the failsafe still holds. Offer clear.
    if (fs.level > FS_NOMINAL && fs.conditions_clear) {
        reason = 'Conditions have cleared. The failsafe stays latched until you clear it.';
    }

    el.className = 'banner ' + cls;
    $('fs_glyph').innerHTML = glyph;
    $('fs_word').innerText = text;
    $('fs_why').innerText = reason;

    if (fs.level > FS_NOMINAL && !data.estop) {
        if (actions.getAttribute('data-shown') !== '1') {
            actions.innerHTML =
                '<button onclick="clearFailsafe(false)">Clear Failsafe</button>' +
                '<button onclick="clearFailsafe(true)">Override</button>';
            actions.setAttribute('data-shown', '1');
        }
    } else if (actions.getAttribute('data-shown') === '1') {
        actions.replaceChildren();
        actions.removeAttribute('data-shown');
    }
}

function clearFailsafe(force) {
    let body = {force: false};
    if (force) {
        // A forced clear needs a typed reason, recorded on boat/data_events.
        const reason = overrideReason('overriding the failsafe');
        if (!reason) return;
        body = {force: true, reason: reason};
    }
    fetch('/failsafe/clear', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(body),
    }).then((r) => r.json()).then((d) => {
        if (d.status !== 'ok') notifyOperator('Clear failed: ' + (d.error || 'unknown error'));
    }).catch(() => notifyOperator('Clear not sent: no connection.'));
}

// --- Telemetry loop ------------------------------------------------------------------------
let renderErrorLogged = false;

function renderTelemetry(data) {
    const link = $('link_badge');
    const controlFresh = isNum(data.operator_age_s) && data.operator_age_s <= 1;
    link.innerText = controlFresh ? 'LINK OK' : 'CONTROL STALE';
    link.className = 'chip ' + (controlFresh ? 'ok' : 'bad');
    link.title = controlFresh ? '' : 'The control arbiter is not reporting. Manual and missions will be refused.';
    operator.observe(data.operator, data.operator_age_s);

    if (data.mission && data.mission.execution_id) activeExecutionId = data.mission.execution_id;
    const state = data.state || '--';
    missionActive = state === 'EXECUTE';

    // Source the control arbiter selects. EXECUTE + WAYPOINT means the boat
    // is driving its route. Manual can drive while that route is paused.
    const mode = controlFresh ? ((data.operator || {}).mode || data.mode || '--') : 'UNKNOWN';
    setText('mode_state', mode,
            mode === 'WAYPOINT' ? 'grn' : mode === 'ESTOP' || mode === 'FAILSAFE' ? 'red'
            : mode === 'MANUAL' || mode === 'HOLD' ? 'amb' : '');

    const sb = $('state_badge');
    sb.innerText = state;
    sb.title = 'Mission state';
    sb.className = 'chip ' + (state === 'EXECUTE' ? 'ok' : state === 'ABORTED' ? 'bad' : '');
    if (state === 'EXECUTE' && (mode === 'MANUAL' || mode === 'HOLD')) {
        sb.innerText = 'MISSION PAUSED';
        sb.className = 'chip warn';
    }

    const dm = data.dock_marker || {};
    setText('dockm_detected', dm.detected ? 'YES' : 'NO', dm.detected ? 'grn' : '');
    setText('dockm_dist', isNum(dm.distance_m) ? dm.distance_m.toFixed(2) + ' m' : '--');
    setText('dockm_lat', isNum(dm.lateral_m) ? (dm.lateral_m > 0 ? '+' : '') + dm.lateral_m.toFixed(2) + ' m' : '--');

    if (data.dock && isNum(data.dock.lat)) {
        dockPoint = [data.dock.lat, data.dock.lon];
        drawDock(data.dock.lat, data.dock.lon);
    }
    if (data.geojson) {
        missionLayer.clearLayers();
        missionLayer.addData(data.geojson);
    }

    updateHeading(data.heading_est || {}, data.heading);
    setText('roll_val', isNum(data.roll) ? data.roll.toFixed(1) + '°' : '--', attitudeClass(data.roll));
    setText('pitch_val', isNum(data.pitch) ? data.pitch.toFixed(1) + '°' : '--', attitudeClass(data.pitch));

    updateSafety(data.safety || {}, data.failsafe || {}, !!data.estop);
    updateBoat(data);
    updateNav(data, state, mode);
    updatePower(data);
    updateCamera(data.camera || {});
    updateLidarStatus(data.lidar || {});
    updateRecording(data);
    updateBanner(data);
    drawLaunchPoint(data.failsafe || {});
    syncFenceInputs(data.safety || {});
    redrawLidar();
}

function updateTelemetry() {
    fetch('/telemetry').then((r) => r.json()).then((data) => {
        try {
            renderTelemetry(data);
        } catch (error) {
            // A display bug must not look like a dead link.
            console.error(error);
            if (!renderErrorLogged) {
                renderErrorLogged = true;
                notifyOperator('Dashboard display error: ' + error.message);
            }
        }
    }, () => {
        const link = $('link_badge');
        link.innerText = 'NO LINK';
        link.className = 'chip bad';
        renderRecordingLibrary({status_fresh: false});
        setText('rec_state', 'UNKNOWN / NO LINK', 'red');
    });
}

applyViews();
applyLayers();
showPlotted();
if (typeof location !== 'undefined' && location.hash.length > 1) showTab(location.hash.slice(1));
setInterval(updateTelemetry, 1000);
updateTelemetry();
