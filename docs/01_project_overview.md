# Project overview

[Back to README](../README.md)

I started SOLSTRÅLE because I wanted to learn how to build a complete robotic system. Most of my background is in software, so working on the hull, electronics, and motors gives me problems I would not run into just writing code.

I am building it independently and using parts I can afford and modify myself. The hull uses PVC floats, metal cross-members, sheet material, and a central enclosure. The design keeps changing as I find things that need a better solution.

## What I want the boat to do

My first goal is a boat I can control reliably and use to collect useful sensor data. I want to record what the camera sees alongside GPS, IMU, and control commands, then look at the data afterward to understand what happened.

Later, I want to use that data for sensor fusion, obstacle avoidance, and autonomous docking. I am still working on the basic platform before trying to make those features reliable.

## Main components

| Component | What I use it for |
|---|---|
| Raspberry Pi 4 | ROS 2, camera processing, dashboard, and recording |
| Arduino Uno R4 WiFi | Low-level motor control and IMU connection |
| Twin brushed thrusters and BTS7960 drivers | Moving and steering the boat |
| Arduino Modulino Movement | Acceleration and angular-rate measurements |
| NEO-6M GPS | Position and course information |
| RGB Pi camera with thermal | Live view and perception experiments; thermal still to test |
| 3S LiPo | Propulsion power |
| Power bank | Separate supply for the Pi and Arduino |

The propulsion supply has a 12 A fuse. I still need to finish the wiring diagram and add the remaining component specifications.

## Current work

The electronics are mounted in their box, the thrusters are installed, and the boat can be driven from the control station. I have tested manual control, the thrusters, IMU, RGB camera, and E-stop. GPS is connected, but I have not verified a usable fix yet.

The [CAD model](04_mechanical_design.md) helps me work through the layout, but it still needs updating to match the latest build and wiring.

My focus is on finishing the engineering and fixing the smaller problems that show up during testing. Recording and data collection have not started yet. I keep completed checks in the [test log](09_test_log.md) and unfinished work in [next steps](10_known_gaps.md).
