# BlueSim ROS 2 Interface

This package contains the ROS 2 interfaces and supporting components used to
integrate the BlueSim Unreal Engine simulation with ROS 2.

## Purpose

The package provides the ROS 2 side of the communication between the Unreal
Engine simulation and external ROS 2 applications.

The initial implementation focuses on publishing simulated RGB camera data
from the robots in BlueSim.

## Robots

The simulation currently contains:

- BlueBoat
- Drone
- Rover

Each robot has a simulated RGB camera.

## Camera configuration

The simulated cameras use:

- Resolution: 640 × 360 pixels
- Frame rate: 30 FPS

The Unreal Engine camera/render-target system is responsible for generating
the simulated images. The ROS 2 integration publishes these images using the
standard:

`sensor_msgs/msg/Image`

message type.

## Planned camera topics

The BlueBoat camera will use:

`/bluesim/blueboat/camera/image_raw`

The corresponding Drone and Rover camera topics will be added using the same
naming convention.

## Unreal Engine integration

The Unreal Engine side uses the `rclUE` ROS 2 plugin.

The communication path is:

Unreal camera
→ Scene Capture
→ Render Target
→ rclUE
→ ROS 2 `sensor_msgs/msg/Image`

## Workspace

This package is part of the BlueSim ROS 2 workspace:

```text
BlueSim/
└── ros2_ws/
    └── src/
        └── bluesim_ros2/