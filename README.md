This repository is based on https://github.com/ros-controls/ros2_control_demos with the addition of hardware drivers.

## Dependencies:
- https://github.com/dignakov/ros2_abb_support_minimal (for the ABB driver)
- https://github.com/ros-industrial/abb_libegm (for the ABB driver)
- https://github.com/dignakov/kuka_experimental (for the Kuka driver)

## Compiling:
Please follow the directions here https://github.com/ros-controls/ros2_control_demos to set up ros2_control, ros2_controllers, and ros2_control_demos.

Since libegm is used with `add_subdirectory`, for now the directory structure should look something like this to have everything compile:

```
colcon_ws/src
|
├── abb_libegm (https://github.com/ros-industrial/abb_libegm)
├── kuka_experimental (https://github.com/dignakov/kuka_experimental)
├── ros2_abb_support_minimal (https://github.com/dignakov/ros2_abb_support_minimal)
└── ros2_control_demo_drivers
    |
    ├── ros2_control_abb_demo_driver
    ├── ros2_control_demo_drivers
    └── ros2_control_kuka_demo_driver
```

## ABB Driver:
For testing, this needs a real robot or RobotWare compatible with libegm. More details will be added soon.

## Kuka Driver:
The minimal fork for ros2 has a, somewhat rough, adaptation of the RSI simulator. It can be used to test the driver by callig:
```
ros2 run kuka_rsi_simulator kuka_rsi_simulator
```

In combination with launching the robot hardware (see https://github.com/ros-controls/ros2_control_demos for details on starting the driver).
