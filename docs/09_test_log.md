# Test log

[Back to README](../README.md)

I am keeping a test log so I can see what works, what changed, and what still needs checking. Bench tests, software tests, and water tests go here separately.

## Physical and simulation checks

| Check | Result | Next check |
|---|---|---|
| Manual control and thrusters | I can drive the boat from the control station | Measure performance under load |
| Earlier short motor run | Both motors ran for three seconds at a 15% command | Keep load, current, temperature, and firmware details for the next test |
| E-stop | Tested | Measure timing and test restart behavior |
| IMU | Tested | Calibration and mounting orientation |
| RGB camera | Tested | Calibration |
| Thermal | Not tested yet | Try the thermal feed |
| NEO-6M GPS | Connected and checked, but a usable fix is not verified | Outdoor fix test |
| Connection loss | Not tested yet | Test loss and recovery behavior |
| Waypoint navigation | Used in simulation | Repeatable scenario and water testing |
| Recording and data collection | Not started | First capture and replay after the engineering work |

No on-water result has been added yet.

## Software checks

On October 12, 2026, the hardware-free regression suite passed all 13 tests using code revision `58da36e`:

~~~bash
python3 -m unittest discover -s tests -v
~~~

These tests check selected callbacks using test objects in place of ROS connections. They cover invalid motor values, serial buffering, dashboard locking, GPS validity, heading loss, camera-pipe handling, and malformed commands. They do not run the complete ROS stack or physical hardware.

## Tests I still need to do

- Motor direction, current, voltage sag, and temperature under load.
- Command loss, serial loss, E-stop, and restart response.
- Mission start, manual takeover, HOLD, resume, cancel, completion, and a new mission.
- Outdoor GPS, IMU orientation, heading behavior, and camera calibration.
- Recording, shutdown, file checks, processing, and replay.
- Loaded flotation, leaks, communication range, and a supervised water run.

## Recording a result

For each result, I want to keep the date, software and firmware versions, wiring, conditions, method, and measurements together. The [test template](../evidence/test_record_template.md) gives me a place to add those details and link photos or logs.
