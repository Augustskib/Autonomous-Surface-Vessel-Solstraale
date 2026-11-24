# Operator console

One page at `http://127.0.0.1:5000`, sized to the window with no page scroll.

## Layout

- **Main view and inset.** The map fills the window and the camera sits in the
  bottom right corner. The swap button on the inset exchanges them. The choice
  is remembered in the browser.
- **Top bar.** Tabs, status readouts (link, arbiter mode, mission state, GPS,
  heading, speed, battery, recording, LiDAR) and Engine Stop. Reset to Hold
  appears only while the stop is engaged.
- **Failsafe strip.** Always visible under the top bar: state word, reason, and
  Clear / Override when a failsafe is latched.
- **Map tools (top left).** Layers menu (LiDAR, geofence, route and trail,
  dock and launch, follow boat), Mission menu (start, return to dock, clear,
  cancel, set dock) and the detection toggle.
- **Drive card (bottom left).** Take Manual, Hold, Continue Mission and the
  requested thrust bars. W/S/A/D drive while manual is confirmed.
- **Tabs.** Pilot closes the drawer. Nav, Recording, Safety, Systems and Log
  open one panel in a drawer on the right; the main view shrinks beside it.
  `/data` redirects to the Recording tab.
- **Messages.** Operator messages show as a toast for 8 s and are kept in the
  Log tab (this browser session only, newest first, 80 entries).

## Map

Leaflet 1.9.4 with OpenStreetMap tiles, loaded from their public servers, so
the laptop needs internet. The view (centre and zoom) is saved in the browser
and restored on reload. With no saved view the map starts at the boat, or at
the geofence centre when there is no fix. Zoom goes to 22; tiles are upscaled
past 19.

Without a GPS fix the boat is drawn grey at the geofence centre and a note
says so. This is for display only and is what lets the LiDAR layer work
indoors.

## LiDAR layer

`web_server` subscribes to `/boat/lidar/points_preview` and serves the newest
cloud at `/lidar/scan` as top-down x,y points (at most 3000, sensor frame, x
forward, y left). The page polls it twice a second only while the layer is
on, and draws the points around the boat rotated by the displayed heading plus
the yaw offset from the Layers menu. Nothing navigates on this view, and a
scan older than 2 s is not drawn.

## Files

`console.html`, `console.css`, `console.js` and `operator_controls.js` in
`src/boat_web_ui/boat_web_ui/`. `web_server.py` joins them once at start, so
rebuild `boat_web_ui` and restart the launch after editing them.

Control semantics are unchanged and described in
[operator-handover.md](operator-handover.md).

## Tests

```
node --test src/boat_web_ui/test/operator_controls.test.cjs src/boat_web_ui/test/pilot_page.test.cjs src/boat_web_ui/test/recording_page.test.cjs
python3 -m pytest tests/test_lidar_view.py src/boat_web_ui/test/test_operator_endpoint.py
```
