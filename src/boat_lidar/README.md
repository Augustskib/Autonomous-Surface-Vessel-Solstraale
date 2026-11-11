> Validation update 2026-11-09: the SDK client library and this ROS 2 executable compiled on Ubuntu 24.04 / Jazzy x86-64. The executable loaded and rejected missing host configuration. Pi/ARM compilation and physical scan reception remain untested. Reproducible recipe: scripts/validation/Dockerfile.lidar. The historical preparation notes below are superseded where they describe compilation as untested.

# Optional Quanergy M8 ROS 2 adapter

Source integration only: not yet compiled against ROS 2 and the vendor SDK or tested with the M8-1 REV B. The package defaults to `ASV_BUILD_QUANERGY=OFF`. A successful default workspace build does not validate this executable.

Vendor sources: https://github.com/QuanergySystems/quanergy_client

Reviewed SDK revision: `83d76e1675d14426f7d8c0dc34682bf8f5c75c29`. Install the SDK's documented Boost, PCL and build dependencies on the intended Linux ROS 2 machine. Build it into a separate prefix, retaining its license and calibration/settings files. Its CMake currently requests C++11; a modern PCL installation may require adapting that SDK setting to C++17. That compatibility adjustment and the SDK build have not been validated here.

Example preparation, after dependencies are installed:

```sh
git clone https://github.com/QuanergySystems/quanergy_client.git vendor/quanergy_client
git -C vendor/quanergy_client checkout 83d76e1675d14426f7d8c0dc34682bf8f5c75c29
sed -i 's/CXX_STANDARD 11/CXX_STANDARD 17/g' vendor/quanergy_client/CMakeLists.txt
cmake -S vendor/quanergy_client -B vendor/quanergy-build -DCMAKE_INSTALL_PREFIX="$HOME/asv-sdk" -DNoViz=ON
cmake --build vendor/quanergy-build --target quanergy_client --parallel 2
cmake --install vendor/quanergy-build
# From the ASV workspace, with the intended ROS 2 environment sourced:
colcon build --packages-up-to boat_lidar --cmake-args -DASV_BUILD_QUANERGY=ON -DCMAKE_PREFIX_PATH="$HOME/asv-sdk"
source install/setup.bash
ros2 launch boat_lidar quanergy.launch.py host:=ACTUAL_SENSOR_IP
```

If SDK configuration/build fails, resolve that before enabling the adapter; do not mask it with a fake scan source. Port 4141 is the SDK sample default, not a verified setting for your unit. Address, power and hardware calibration must be verified from your sensor/manual. `settings_file` accepts the vendor pipeline settings XML.

Receive timestamps are the default commissioning mode. Sensor timestamp mode requires explicit `sensor_clock_verified:=true`; this is an operator assertion, not a synchronization service. See `docs/sensor-integration-runbook.md` for calibration, recording and acceptance requirements.
