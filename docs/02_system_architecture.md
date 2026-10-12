# System architecture

[Back to README](../README.md)

I use the Raspberry Pi for ROS 2, navigation, the dashboard, and recording. The Arduino handles the motor outputs and IMU connection. They communicate over USB serial.

## How commands reach the motors

~~~mermaid
flowchart LR
    UI[Dashboard] -->|Manual requests| A[Control arbiter]
    UI --> M[Mission manager]
    M --> N[Waypoint follower]
    N -->|Navigation requests| A
    S[Safety monitor] --> A
    A --> D[Motor interface on Pi]
    D -->|USB serial| MCU[Arduino]
    MCU --> L[Left driver and thruster]
    MCU --> R[Right driver and thruster]
~~~

Manual commands use `/boat/cmd/manual`, and waypoint commands use `/boat/cmd/waypoint`. The arbiter chooses the command and publishes it on `/boat/thrust_command`. The motor interface sends the left and right values to the Arduino.

The Arduino turns those values into PWM for the two drivers. Its status messages include the last command values, but the Pi currently ignores those lines. I still need a way to check that a particular command reached the Arduino.

## Sensor connections

GPS connects directly to the Pi's serial interface. The Modulino Movement connects to the Arduino over I2C, and its measurements come back through USB serial. The RGB Pi camera connects through CSI. I have tested the RGB feed, but not thermal yet.

The main sensor topics are:

| Topic | Data |
|---|---|
| `/boat/gps/fix` | GPS position and fix status |
| `/boat/imu` | IMU measurements |
| `/boat/heading` | Heading estimate and quality |
| `/boat/camera/image_raw` | Camera images |

The heading estimator combines gyro measurements with GPS course when available. It is a complementary estimator; a full Kalman filter is future work. GPS course tells me which way the boat is moving, which can be different from where the bow points.

## Stops and control modes

The software has an E-stop, geofence checks, and command timeouts. The Arduino watchdog sets the motors to zero and disables the drivers after 500 ms without a valid command. I have tested the E-stop, but I still need to measure stop timing and test connection loss.

HOLD pauses normal mission output. It does not keep the boat in one place against wind or current, so the boat can drift.

The arbiter gives E-stop the highest priority, followed by manual control, failsafe behavior, HOLD, and waypoint output. I need to revisit whether manual control should override a failsafe stop. I also need to check what happens when the safety monitor stops publishing.

## Simulation

Gazebo replaces the physical sensors and motors while keeping the main control interfaces. The `thrust_splitter` turns the selected commands into simulated propeller forces.

This lets me develop the dashboard and navigation, but the simulation still needs tuning to match the finished boat. The simulated IMU also provides absolute orientation, while the real Modulino has no absolute north reference.
