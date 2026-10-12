# Data collection

[Back to README](../README.md)

I have not started recording or collecting datasets yet. Right now I am focused on the engineering and controls.

Once the boat is ready, I want to record the camera, GPS, IMU, and control commands together so I can look back at what it saw and how it moved. Later, I want to use those runs for perception and sensor-fusion experiments.

## Recorder

The recorder uses rosbag2 with MCAP storage. It saves run metadata, segments, event markers, and checksums, and checks whether the bag files are growing.

It starts separately from the main launch:

~~~bash
ros2 run boat_data data_recorder
~~~

Then start a recording through the dashboard or `/boat/data_command`. Starting the node alone does not begin capture.

The default data location is `~/asv-boat/data`, set through the `data_root` parameter. A finalized run uses this layout:

~~~text
<run>/
  manifest.json
  run_brief.json
  checksums.sha256
  bag/rosbag2/
~~~

I still need to test the full recording, shutdown, processing, and replay sequence.

## Data to keep

| Data | Why I need it |
|---|---|
| Camera images, calibration, and camera status | Know what the boat saw and whether the feed was real or mocked |
| GPS, IMU, heading, and navigation status | Follow its position and motion |
| Manual, waypoint, failsafe, and selected commands | Understand what requested each movement |
| Mission, mode, safety, and E-stop events | Explain changes and interruptions |
| Power and Pi system readings | Look for supply, temperature, or storage problems |
| Run notes and markers | Keep track of what I was trying to test |

The topic list includes hardware-only and simulation-only streams. I need to check which streams are required for each run and whether they actually contain data. A growing bag file does not tell me that every sensor is recording.

I also want each run to include the software and firmware versions, sensor settings, calibration, location, conditions, and problems noticed during the test.

## Timing

The streams are not fully synchronized yet. The Arduino IMU messages have no acquisition timestamp, and GPS is stamped when the Pi processes it. Serial buffering and camera delay can shift the streams relative to each other.

I need to measure those delays and keep track of the clocks being used. Recording everything in one bag does not fix the timing by itself.

## Processing problems to fix

The postprocessor looks for MCAP files directly under `bag/`, but the recorder writes them under `bag/rosbag2/`. It also picks only the first MCAP file.

GPS status parsing and missing-stream checks need work. The script can change the manifest after checksums are written, so I need to keep the original recording separate from processing results.

## Storage

Raw 640 × 480 BGR images at 10 fps are about 33 GB/hour before other streams and overhead. At that rate, 2 GB lasts only about 3.6 minutes. I need to measure actual disk use with the recording settings before planning longer runs.

Large recordings and model weights will stay outside the source repository.

## Next step

Record one short run, stop it cleanly, check the files and required streams, and replay it away from live motor control.

After that, I want to work on an annotation workflow using CVAT and SAM 2. Labels and processed data should keep the original run ID so I can trace them back to the recording.
