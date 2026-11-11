# Green instrument console

The Pilot page uses the approved green-display / amber-indicator concept. This
is implemented in the existing Flask page, not a generated background image.

## Layout and implementation

- `src/boat_web_ui/boat_web_ui/console.css` contains the shared visual theme and
  responsive layout. `_compose()` embeds it in both Pilot and Data pages, and
  `setup.py` includes it in the installed package.
- `web_server.py` places heading, attitude readings, speed, battery and safety
  in the left instrument stack; Leaflet in the center; camera, requested thrust
  bars and a browser-local operator log on the right. The desktop command deck
  remains below these panels. Navigation details, power diagnostics, docking
  detection and geofence configuration are in an expandable lower panel.
- The compass card rotates counterclockwise under a fixed bow pointer as
  clockwise heading increases. Unknown heading dims the dial; raw IMU heading
  retains its explicit no-estimator warning. Numeric telemetry remains the
  source for the instrument; the dial adds no estimator or control logic.
- Operator messages are rendered as plain text, newest first, bounded to 80
  entries. This is browser-session history, not a ROS event archive. It does
  not invent navigation events or persist across reloads.
- The configured manual thrust ceiling is displayed read-only. An adjustable
  thrust control is separate future functionality.

## Map clarification

This checkout uses Leaflet 1.9.4 with OpenStreetMap tiles. `foxglove_msgs/GeoJSON`
is the route-data message format; the page is not an embedded Foxglove map.
The old tile color inversion was removed. Chart labels, attribution and camera
colors remain intact. Loading Leaflet and map tiles still requires network
access to the existing external providers; this change does not add offline
chart storage.

## Control semantics

The existing control handlers and backend policy are unchanged. Hold means
zero requested thrust, not station keeping. Manual/Auto acknowledgments,
lease renewal, emergency-stop/reset and mission cancellation retain their
existing behavior. Clear Points, Cancel Mission and Hide Route Drawing remain
separate actions; the earlier combined cancel-and-clear request is not part
of this visual patch. Motor bars are requested thrust, not measured output.

## UI verification before the control update

- 16 Node tests pass, covering the actual composed Pilot-page JavaScript,
  operator transitions, new heading display and bounded plain-text history.
- 5 Python endpoint/adapter tests pass using in-memory transport.
- Headless Chrome renders Pilot at 1440x900, 1024x768 and 390x844 without
  horizontal overflow. Desktop Engine Stop remains within the viewport.
- Data page also renders in Chrome. These are disconnected checks; camera,
  ROS, physical motors and live telemetry were not exercised.
- In the isolated browser test, Leaflet was served from a downloaded copy of
  the same version because the sandbox could not resolve the external CDN.
  Production CDN URLs were independently checked and returned HTTP 200.

Rebuild/install `boat_web_ui` in the normal ROS workspace and restart its web
server to deploy. The CSS is loaded when the server composes its templates;
refreshing the browser alone does not reload a running server's templates.

## Control update, 7 November 2026

The earlier behavior description above is historical. Cancel + Clear Route now
cancels the identified mission and removes its drawing after acknowledgement.
See [operator-handover.md](operator-handover.md) for the current control contract.
