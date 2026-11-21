# Agribot Pico 2 firmware (micro-ROS)

Low-level firmware for the Agribot drive base on a Raspberry Pi Pico 2 (RP2350).
It subscribes to `/cmd_vel`, runs a 25 Hz feed-forward PI wheel-velocity loop,
and publishes encoder ticks and IMU data to a micro-ROS agent over USB serial.

## Status

| Item | State |
| --- | --- |
| Source in `src/` | Recovered copy of the firmware described in the paper (comment fix only) |
| Command timeout (stop when `/cmd_vel` stops) | **Not in this version.** See safety note |
| `CMakeLists.txt` | Template, not yet build-tested from a clean checkout |

## Safety

This version keeps driving the last wheel setpoint if the agent or the Pi
disappears. Do not run it unattended. Keep wheels off the ground or keep a
power cut-off within reach during any test.

## Interfaces

| Topic | Type | Direction | Rate |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | subscribe | on message |
| `/left_ticks`, `/right_ticks` | `std_msgs/Int32` | publish | 20 Hz |
| `/imu/data_raw` | `sensor_msgs/Imu` (frame `imu_link`) | publish | 20 Hz |

Control loop: 25 Hz (`CONTROL_DT = 0.04 s`). Pins and robot constants are at the top of `src/main.cpp`.

## Build (Ubuntu 24.04, ROS 2 Jazzy)

Fill in the pinned versions after your first clean build:

- pico-sdk: 2.2.0 (from your build log; verify)
- micro_ros_raspberrypi_pico_sdk commit: `<fill in>`
- ARM toolchain: `<fill in: arm-none-eabi-gcc --version>`

```bash
export PICO_SDK_PATH=$HOME/pico-sdk
export MICROROS_SDK_PATH=$HOME/micro_ros_raspberrypi_pico_sdk   # libmicroros already built for Jazzy
cd firmware/pico2
mkdir -p build && cd build
cmake -DPICO_BOARD=pico2 ..
make -j"$(nproc)"
```

`-DPICO_BOARD=pico2` is required. Without it you get an RP2040 image that the Pico 2 will not boot.

## Flash and check

1. Hold BOOTSEL, plug in the Pico 2, copy `build/agribot_firmware.uf2` to the `RP2350` drive.
2. Start the agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`
3. In another terminal: `ros2 topic hz /left_ticks /imu/data_raw` (expect about 20 Hz each).
