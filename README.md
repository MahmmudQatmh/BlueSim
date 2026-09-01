# 🌊 BlueSim — Multi-Robot Robotics Simulator

<p align="center">
  <img src="https://img.shields.io/badge/Unreal%20Engine-5.4.4-0E1128?logo=unrealengine&logoColor=white" alt="Unreal Engine 5.4.4">
  <img src="https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white" alt="ROS 2 Humble">
  <img src="https://img.shields.io/badge/Platform-Ubuntu%2022.04-E95420?logo=ubuntu&logoColor=white" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/Status-Active%20Development-F39C12" alt="Status">
</p>

<p align="center">
  <strong>A multi-robot simulation platform built with Unreal Engine 5.4.4 for robotics simulation, sensor development, autonomous control, computer vision, and ROS 2-based research.</strong>
</p>

---

# 🎥 BlueSim Simulation Overview

The following video provides an overview of the BlueSim simulation environment, including the simulated **BlueBoat, Drone, and Rover**.

<p align="center">
  <video
    src="./ros2_ws/Recorded%20Videos/BlueSim_Showcase.mp4"
    controls
    autoplay
    muted
    loop
    playsinline
    width="100%">
  </video>
</p>

> **BlueSim Showcase — Unreal Engine 5.4.4**

The showcase video is stored in:

```text
ros2_ws/Recorded Videos/BlueSim_Showcase.mp4
```

Additional experimental recordings from the simulated robot cameras are also stored in this directory.

---

# 📖 Table of Contents

- [Overview](#-overview)
- [Project Objectives](#-project-objectives)
- [Current Simulation Capabilities](#-current-simulation-capabilities)
- [Robotic Platforms](#-robotic-platforms)
  - [BlueBoat](#-blueboat)
  - [Drone](#-drone)
  - [Rover](#-rover)
- [Simulation Environment](#-simulation-environment)
- [Multi-Robot Architecture](#-multi-robot-architecture)
- [Robot Control Architecture](#-robot-control-architecture)
- [Camera and Sensor Architecture](#-camera-and-sensor-architecture)
- [ROS 2 Integration](#-ros-2-integration)
- [Experimental Recordings](#-experimental-recordings)
- [Project Structure](#-project-structure)
- [Development Environment](#-development-environment)
- [Installation and Setup](#-installation-and-setup)
- [Launching BlueSim](#-launching-bluesim)
- [ROS 2 Workspace](#-ros-2-workspace)
- [Manual Robot Control](#-manual-robot-control)
- [Current ROS 2 Interfaces](#-current-ros-2-interfaces)
- [Research Applications](#-research-applications)
- [Future Development](#-future-development)
- [Project Roadmap](#-project-roadmap)
- [Documentation](#-documentation)
- [Author](#-author)

---

# 🌊 Overview

**BlueSim** is a multi-robot robotics simulation platform developed using **Unreal Engine 5.4.4**.

The simulator currently contains three different types of robotic platforms:

```text
                 ┌──────────────────────┐
                 │       BlueSim        │
                 │ Unreal Engine 5.4.4  │
                 └──────────┬───────────┘
                            │
            ┌───────────────┼───────────────┐
            │               │               │
            ▼               ▼               ▼
      ┌───────────┐   ┌───────────┐   ┌───────────┐
      │ BlueBoat  │   │   Drone   │   │   Rover   │
      │  Surface  │   │  Aerial   │   │  Ground   │
      │   Robot   │   │   Robot   │   │   Robot   │
      └───────────┘   └───────────┘   └───────────┘
```

BlueSim is designed to provide a common environment in which different robotic systems can be:

- Simulated
- Controlled manually
- Controlled through ROS 2
- Equipped with simulated sensors
- Used for computer vision
- Used for autonomous robotics
- Used for multi-robot experiments
- Used for repeatable research experiments

The simulator is intended to provide a foundation for testing robotics algorithms in simulation before deploying them to real robotic platforms.

---

# 🎯 Project Objectives

The main objectives of BlueSim are:

### 1. Multi-Robot Simulation

Provide a common simulation environment containing different classes of robots:

```text
Surface Robot  → BlueBoat
Aerial Robot   → Drone
Ground Robot   → Rover
```

### 2. Physics-Based Robot Simulation

Use Unreal Engine physics to simulate robot movement and interactions with the environment.

### 3. Synthetic Sensor Generation

Provide simulated sensors, particularly onboard camera systems, that can expose data to external robotics software.

### 4. ROS 2 Connectivity

Connect the Unreal simulation to ROS 2 using the **rclUE bridge**.

### 5. External Robotics Algorithms

Allow external Python/C++ robotics software to consume simulated sensor data and send commands to simulated robots.

### 6. Multi-Robot Research

Provide a common platform for future experiments involving cooperation between surface, aerial, and ground robots.

---

# 🚀 Current Simulation Capabilities

The current BlueSim prototype provides:

| Capability | Status |
|---|:---:|
| Unreal Engine 5.4.4 Simulation | ✔ |
| BlueBoat Simulation | ✔ |
| Drone Simulation | ✔ |
| Rover Simulation | ✔ |
| Manual Robot Control | ✔ |
| Physics-Based BlueBoat Movement | ✔ |
| Physics-Based Rover Movement | ✔ |
| Drone Movement | ✔ |
| Robot Observer System | ✔ |
| Observer Switching | ✔ |
| Simulated Onboard Cameras | ✔ |
| Camera Render Targets | ✔ |
| ROS 2 Integration through rclUE | ✔ |
| ROS 2 Robot Commands | ✔ |
| ROS 2 Camera Publication | ✔ |
| Python ROS 2 Nodes | ✔ |
| Camera Recording | ✔ |
| LiDAR Simulation | 🚧 |
| Additional Sensor Models | 🚧 |
| Advanced Autonomous Navigation | 🚧 |
| Multi-Robot Cooperative Control | 🚧 |

---

# 🤖 Robotic Platforms

## 🚤 BlueBoat

The BlueBoat represents the surface-water robotic platform within BlueSim.

### Current capabilities

- Physics-based movement
- Forward movement
- Backward movement
- Steering
- Manual keyboard control
- ROS 2 command control
- Onboard simulated camera
- Camera data publication through ROS 2
- Camera recording through the ROS 2/Python interface

### Control concept

```text
Keyboard ───────┐
                │
                ▼
          Boat Command
                ▲
                │
ROS 2 ──────────┘
                │
                ▼
           Boat Physics
                │
          ┌─────┴─────┐
          │           │
       Add Force   Add Torque
          │           │
          └─────┬─────┘
                ▼
             BlueBoat
```

---

# 🚁 Drone

The Drone provides an aerial robotic platform.

### Current capabilities

- Forward movement
- Backward movement
- Left/right movement
- Yaw
- Elevation
- Manual keyboard control
- Onboard simulated camera
- ROS 2 interface

### Control concept

```text
Keyboard / ROS 2
       │
       ▼
Drone Commands
       │
       ├── Throttle
       ├── Steering
       ├── Yaw
       └── Elevation
       │
       ▼
Drone Movement
       │
       ▼
     Drone
```

---

# 🛞 Rover

The Rover provides a ground-robot platform.

### Current capabilities

- Physics-based forward movement
- Physics-based backward movement
- Steering
- Manual keyboard control
- ROS 2 command control
- Onboard simulated camera
- ROS 2 camera publication
- Camera recording
- Physics-based lateral movement handling

### Control concept

```text
Keyboard / ROS 2
       │
       ▼
Rover Commands
       │
       ├── Drive Command
       │
       └── Steering Command
       │
       ▼
      Physics
       │
 ┌─────┼──────────────┐
 │     │              │
 ▼     ▼              ▼
Drive  Friction     Steering
Force                Torque
 │     │              │
 └─────┴──────┬───────┘
              ▼
         RoverPhysics
              │
              ▼
            Rover
```

---

# 🌍 Simulation Environment

BlueSim is built as an Unreal Engine simulation environment containing the robotic platforms and the surrounding environment.

The simulation environment provides the foundation for:

```text
Environment
     │
     ├── Water
     ├── Terrain
     ├── Lighting
     ├── Sky / Atmosphere
     ├── Robot Platforms
     ├── Sensors
     └── Observer System
```

The environment can therefore be extended with additional scenarios and robotic experiments without changing the basic ROS 2 communication architecture.

---

# 🏗️ Multi-Robot Architecture

The current simulation contains three robotic agents inside the same Unreal Engine environment.

```text
                           ┌──────────────────────┐
                           │       BlueSim        │
                           │ Unreal Engine 5.4.4  │
                           └──────────┬───────────┘
                                      │
                 ┌────────────────────┼────────────────────┐
                 │                    │                    │
                 ▼                    ▼                    ▼
          ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
          │   BlueBoat   │     │    Drone     │     │    Rover     │
          │              │     │              │     │              │
          │   Surface    │     │    Aerial    │     │    Ground    │
          │    Robot     │     │    Robot     │     │    Robot     │
          └──────┬───────┘     └──────┬───────┘     └──────┬───────┘
                 │                    │                    │
                 └────────────────────┼────────────────────┘
                                      │
                                      ▼
                               Shared Environment
                                      │
                                      ▼
                              Observer System
```

The robots exist simultaneously in the same simulation world.

---

# 👁️ Observer System

BlueSim contains observer pawns that allow the user to switch between robot viewpoints.

The current switching sequence is:

```text
C
│
▼
Drone
│
▼
BlueBoat
│
▼
Rover
│
▼
Drone
│
└──────────────► ...
```

This allows the simulation to be inspected from different robotic viewpoints during an experiment.

---

# 🎮 Robot Control Architecture

The control architecture separates the **command source** from the **robot physics**.

The intended structure is:

```text
               ┌───────────────────┐
               │     Keyboard      │
               └─────────┬─────────┘
                         │
                         ▼
                  Command Variables
                         ▲
                         │
               ┌─────────┴─────────┐
               │      ROS 2        │
               │ Python / rclpy    │
               └─────────┬─────────┘
                         │
                         ▼
                  Robot Controller
                         │
                         ▼
                    Robot Physics
                         │
                         ▼
                       Robot
```

This design allows the same simulated robot to be controlled manually or externally through ROS 2.

The detailed ROS 2 and rclUE implementation is documented separately.

---

# 📷 Camera and Sensor Architecture

Each robot contains a simulated onboard camera.

The general camera structure is:

```text
Robot
 │
 └── Camera
      │
      └── SceneCaptureComponent2D
               │
               ▼
          Render Target
```

Current camera structures include:

```text
BlueBoat
 └── BoatCamera
       └── BoatCameraCapture
            └── RT_BoatCamera
```

```text
Drone
 └── DroneCamera
       └── DroneCameraCapture
            └── RT_DroneCamera
```

```text
Rover
 └── RoverCamera
       └── RoverCameraCapture
            └── RT_RoverCamera
```

Current camera configuration includes:

```text
Projection Type : Perspective
Field of View   : 90°
Resolution      : 640 × 360
```

The camera system is designed to provide sensor data for external processing and robotics algorithms.

---

# 🔌 ROS 2 Integration

BlueSim uses the **rclUE bridge** to connect Unreal Engine to ROS 2.

The overall communication architecture is:

```text
                   Unreal Engine
                         │
                         ▼
                       rclUE
                         │
              ┌──────────┴──────────┐
              │                     │
              ▼                     ▼
          Publishers            Subscribers
              │                     ▲
              │                     │
              ▼                     │
         Sensor Data           Robot Commands
              │                     ▲
              ▼                     │
             ROS 2 ─────────────────┘
              │
              ▼
         Python / C++
         Robotics Software
```

The detailed implementation of the ROS 2 and Unreal Engine integration is documented in:

```text
ros2_ws/src/bluesim_ros2/README.md
```

That document covers:

- rclUE architecture
- ROS 2 nodes
- Publishers
- Subscribers
- Message wrappers
- Camera publishing
- Robot commands
- Python ROS 2 nodes
- Camera recording
- ROS 2 communication architecture

This main README intentionally does not duplicate that technical documentation.

---

# 🔄 Complete Simulation Data Flow

The complete intended robotics loop is:

```text
┌─────────────────────────────────────────────────────────┐
│                    Unreal Engine                        │
│                                                         │
│  ┌────────────┐       ┌────────────┐      ┌──────────┐  │
│  │   Camera   │       │   Robot    │      │  Robot   │  │
│  │   Sensor   │       │  Physics   │      │  State   │  │
│  └─────┬──────┘       └──────▲─────┘      └──────────┘  │
│        │                      │                         │
│        ▼                      │                         │
│   rclUE Publisher        rclUE Subscriber               │
└────────┼──────────────────────┼─────────────────────────┘
         │                      │
         ▼                      ▲
              ┌──────────────┐
              │    ROS 2     │
              └──────┬───────┘
                     │
          ┌──────────┴──────────┐
          │                     │
          ▼                     ▼
     Sensor Data             Commands
          │                     ▲
          ▼                     │
   Python / C++ Algorithms ─────┘
          │
          ├── Computer Vision
          ├── Machine Learning
          ├── Planning
          ├── Navigation
          └── Control
```

---

# 🎥 Experimental Recordings

Recorded camera experiments are stored in:

```text
ros2_ws/Recorded Videos/
```

The directory is intended to contain recordings from the simulated onboard cameras under different experimental configurations.

For example:

```text
ros2_ws/
└── Recorded Videos/
    ├── BlueSim_Overview.mp4
    ├── BlueSim_BlueBoat_Record.mp4
    ├── BlueSim_Rover_Record.mp4
    └── ...
```

The recordings can be used to compare different camera configurations.

---

# 📊 Camera Experiment Comparison

Future camera experiments may compare:

### Resolution

```text
640 × 360
1280 × 720
1920 × 1080
```

### Publication Frequency

```text
5 Hz
10 Hz
20 Hz
30 Hz
```

### Example comparison structure

```text
Camera Configuration
        │
        ├───────────────┐
        │               │
        ▼               ▼
 Resolution         Frequency
        │               │
        └───────┬───────┘
                ▼
          Image Quality
                │
                ▼
         ROS 2 Throughput
                │
                ▼
       Unreal Performance
```

These recordings are useful for evaluating the relationship between visual quality, sensor frequency, ROS 2 data flow, and simulation performance.

---

# 🧪 Experimental Workflow

A typical BlueSim experiment can follow:

```text
             START
               │
               ▼
       Launch BlueSim
               │
               ▼
      Initialize Simulation
               │
               ▼
       Initialize ROS 2
               │
               ▼
      Start Sensor Streams
               │
               ▼
       Start Robot Control
               │
               ▼
        Run Experiment
               │
       ┌───────┴────────┐
       │                │
       ▼                ▼
  Robot State        Camera Data
       │                │
       └───────┬────────┘
               ▼
         Record Results
               │
               ▼
          Stop Robot
               │
               ▼
         Save Results
               │
               ▼
              END
```

This structure is intended to make experiments repeatable and easy to extend.

---

# 🧠 Research Applications

BlueSim is designed to support a wide range of robotics research applications.

## Computer Vision

```text
Simulated Camera
       │
       ▼
     ROS 2
       │
       ▼
Python
       │
       ▼
OpenCV / Machine Learning
       │
       ▼
Detection / Tracking
```

---

## Autonomous Navigation

```text
Sensors
   │
   ▼
Perception
   │
   ▼
State Estimation
   │
   ▼
Planning
   │
   ▼
Control
   │
   ▼
Robot
```

---

## Visual Control

```text
Camera
  │
  ▼
Image
  │
  ▼
Vision Algorithm
  │
  ▼
Target / State
  │
  ▼
Controller
  │
  ▼
ROS 2 Command
  │
  ▼
Robot
```

---

## Multi-Robot Robotics

```text
                     ROS 2
                       │
          ┌────────────┼────────────┐
          │            │            │
          ▼            ▼            ▼
       BlueBoat      Drone         Rover
          │            │            │
          └────────────┼────────────┘
                       │
                       ▼
              Cooperative System
```

Potential applications include:

- Search and rescue
- Environmental monitoring
- Cooperative perception
- Multi-robot navigation
- Sensor sharing
- Visual tracking
- Cooperative control
- Maritime robotics
- UAV/USV cooperation
- Heterogeneous multi-robot systems

---

# 🧩 Future Sensor Expansion

The current architecture is not limited to cameras.

Future simulated sensors can include:

```text
Camera
LiDAR
Depth Camera
IMU
GPS
Sonar
Radar
Odometry
Point Clouds
```

A generic future sensor pipeline is:

```text
Unreal Sensor
      │
      ▼
Sensor Simulation
      │
      ▼
rclUE
      │
      ▼
ROS 2
      │
      ▼
Python / C++
      │
      ▼
Perception / Control / ML
```

---

# 🗂️ Project Structure

```text
BlueSim/
│
├── README.md
│
├── unreal/
│   └── BlueSim/
│       │
│       ├── BlueSim.uproject
│       │
│       ├── Source/
│       │   └── BlueSim/
│       │       ├── BlueSim.Build.cs
│       │       ├── BlueSimCameraPublisher.h
│       │       └── BlueSimCameraPublisher.cpp
│       │
│       ├── Content/
│       │   ├── BlueBoat/
│       │   ├── Drone/
│       │   ├── Rover/
│       │   ├── Sensors/
│       │   ├── Maps/
│       │   ├── Environment/
│       │   └── ...
│       │
│       └── Plugins/
│           └── rclUE/
│
└── ros2_ws/
    │
    ├── build/
    ├── install/
    ├── log/
    │
    ├── Recorded Videos/
    │   ├── BlueSim_Overview.mp4
    │   ├── BlueSim_BlueBoat_Record.mp4
    │   ├── BlueSim_Rover_Record.mp4
    │   └── ...
    │
    └── src/
        │
        └── bluesim_ros2/
            │
            ├── README.md
            ├── package.xml
            ├── CMakeLists.txt
            │
            └── bluesim_ros2/
                ├── camera_monitor.py
                ├── boat_command.py
                ├── blueboat_autonomy.py
                ├── blueboat_experiment.py
                ├── rover_command.py
                └── ...
```

---

# 🛠️ Development Environment

The current BlueSim development environment uses:

| Component | Version / Technology |
|---|---|
| Operating System | Ubuntu 22.04 LTS |
| Simulation Engine | Unreal Engine 5.4.4 |
| ROS 2 | Humble |
| Programming | C++ / Python |
| ROS 2 Unreal Bridge | rclUE |
| Python ROS 2 API | `rclpy` |
| Image Processing | OpenCV / NumPy |
| Version Control | Git / GitHub |
| GPU | Dedicated NVIDIA GPU recommended |

---

# 📦 Installation and Setup

## 1. Clone the repository

```bash
git clone https://github.com/MahmmudQatmh/BlueSim.git
```

Enter the project directory:

```bash
cd BlueSim
```

---

# 2. Unreal Engine

BlueSim requires:

```text
Unreal Engine 5.4.4
```

The Unreal Engine executable should point to an Unreal Engine 5.4.4 installation/build.

Example:

```bash
/path/to/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
BlueSim/unreal/BlueSim/BlueSim.uproject
```

---

# 3. ROS 2

The current development environment uses:

```text
ROS 2 Humble
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

---

# 🚀 Launching BlueSim

Launch the BlueSim Unreal project using Unreal Engine 5.4.4.

Example:

```bash
/path/to/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
BlueSim/unreal/BlueSim/BlueSim.uproject
```

Once the Unreal Editor opens:

```text
1. Open the BlueSim simulation map.
2. Start the simulation.
3. Use the observer controls to inspect the robots.
```

---

# 🐍 ROS 2 Workspace

The ROS 2 workspace is located at:

```text
BlueSim/ros2_ws
```

The BlueSim ROS 2 package is:

```text
BlueSim/ros2_ws/src/bluesim_ros2
```

Build the workspace:

```bash
cd BlueSim/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install
```

Then source the workspace:

```bash
source install/setup.bash
```

Verify the package:

```bash
ros2 pkg list | grep bluesim
```

Expected:

```text
bluesim_ros2
```

---

# 🔎 ROS 2 Diagnostics

List running ROS 2 nodes:

```bash
ros2 node list
```

List ROS 2 topics:

```bash
ros2 topic list
```

List topics and their message types:

```bash
ros2 topic list -t
```

Inspect a topic:

```bash
ros2 topic info <topic_name> -v
```

Measure the publication frequency:

```bash
ros2 topic hz <topic_name>
```

The detailed ROS 2 integration and debugging procedures are documented in:

```text
ros2_ws/src/bluesim_ros2/README.md
```

---

# 🎮 Manual Robot Control

The current BlueSim robots support direct manual control from their Unreal Engine input mappings.

The simulated platforms are:

```text
🚤 BlueBoat
🚁 Drone
🛞 Rover
```

Manual control is useful for:

- Testing robot movement
- Testing the environment
- Validating physics
- Testing sensors
- Demonstrating simulation functionality
- Preparing experiments before autonomous control

---

# 📡 Current ROS 2 Interfaces

The current BlueSim ROS 2 system exposes camera and command interfaces for the three robots.

Current camera topics:

```text
/camera_BlueBoat
/camera_Drone
/camera_Rover
```

Current robot command topics:

```text
/cmd_BlueBoat_vel
/cmd_drone_vel
/cmd_rover_vel
```

Camera message type:

```text
sensor_msgs/msg/Image
```

Robot command message type:

```text
geometry_msgs/msg/Twist
```

The complete implementation of these interfaces is documented in:

```text
ros2_ws/src/bluesim_ros2/README.md
```

---

# 🧪 Example ROS 2 Robot Command

For example, the Rover can receive a forward command through:

```bash
ros2 topic pub --rate 10 /cmd_rover_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0}, angular: {z: 0.0}}"
```

The ROS 2 communication pipeline is:

```text
Python / ROS 2 CLI
        │
        ▼
geometry_msgs/msg/Twist
        │
        ▼
/cmd_rover_vel
        │
        ▼
rclUE
        │
        ▼
Rover Blueprint
        │
        ▼
Rover Control Variables
        │
        ▼
Rover Physics
```

Detailed command examples are available in:

```text
ros2_ws/src/bluesim_ros2/README.md
```

---

# 📷 Example Camera Inspection

A Rover camera frame can be inspected using:

```bash
ros2 topic echo /camera_Rover --once --field width
```

```bash
ros2 topic echo /camera_Rover --once --field height
```

```bash
ros2 topic echo /camera_Rover --once --field encoding
```

The actual camera publication rate can be measured using:

```bash
ros2 topic hz /camera_Rover
```

---

# 🧭 Project Roadmap

## Phase A — Simulation Environment (Completed)

- [x] Configure Unreal Engine project
- [x] Configure renderer
- [x] Establish simulation environment
- [x] Configure environment and water systems
- [x] Establish simulation world
- [x] Develop simulation level

---

## Phase B — BlueBoat Simulation (Completed)

- [x] Import BlueBoat model
- [x] Configure BlueBoat physics
- [x] Implement forward movement
- [x] Implement backward movement
- [x] Implement steering
- [x] Implement manual control
- [x] Add onboard camera
- [x] Configure camera render target
- [x] Connect camera to ROS 2
- [x] Receive ROS 2 movement commands
- [x] Record camera output

---

## Phase C — Drone Simulation (Completed)

- [x] Import Drone model
- [x] Implement flight movement
- [x] Implement forward/backward movement
- [x] Implement lateral movement
- [x] Implement yaw
- [x] Implement elevation
- [x] Implement manual control
- [x] Add onboard camera
- [x] Connect camera to ROS 2
- [x] Implement ROS 2 interface

---

## Phase D — Rover Simulation (Completed)

- [x] Import Rover model
- [x] Configure Rover physics
- [x] Implement forward movement
- [x] Implement backward movement
- [x] Implement steering
- [x] Implement manual control
- [x] Develop physics-based movement
- [x] Improve lateral movement behavior
- [x] Add onboard camera
- [x] Connect camera to ROS 2
- [x] Implement ROS 2 command interface
- [x] Record camera output

---

## Phase E — ROS 2 Integration (Completed)

- [x] Integrate rclUE
- [x] Create ROS 2 nodes
- [x] Create ROS 2 publishers
- [x] Create ROS 2 subscribers
- [x] Integrate `sensor_msgs/msg/Image`
- [x] Integrate `geometry_msgs/msg/Twist`
- [x] Publish simulated camera data
- [x] Receive robot movement commands
- [x] Create Python ROS 2 nodes
- [x] Record camera data through ROS 2

Detailed documentation:

```text
ros2_ws/src/bluesim_ros2/README.md
```

---

## Phase F — Multi-Robot System (Completed)

- [x] BlueBoat
- [x] Drone
- [x] Rover
- [x] Shared simulation environment
- [x] Manual control
- [x] Robot observer system
- [x] Observer switching
- [x] Individual robot cameras
- [x] ROS 2 interfaces
- [x] External Python ROS 2 control

---

## Phase G — Sensors & Autonomous Robotics (Planned)

- [ ] Improve sensor simulation
- [ ] Add LiDAR
- [ ] Add depth sensors
- [ ] Add IMU
- [ ] Add GPS
- [ ] Add odometry
- [ ] Add additional maritime sensors
- [ ] Improve camera performance
- [ ] Improve camera readback architecture
- [ ] Develop autonomous navigation
- [ ] Develop vision-based control
- [ ] Develop perception pipelines
- [ ] Develop autonomous missions

---

## Phase H — Multi-Robot Cooperative Robotics (Planned)

- [ ] Cooperative perception
- [ ] Formation control
- [ ] Search and rescue scenarios
- [ ] Sensor sharing
- [ ] Cooperative navigation
- [ ] Heterogeneous robot coordination
- [ ] UAV/USV cooperation
- [ ] Multi-agent control
- [ ] Experimental validation

---

# 🔬 Long-Term Research Architecture

The long-term architecture of BlueSim is:

```text
                         ┌─────────────────────┐
                         │       BlueSim       │
                         │ Unreal Engine 5.4.4 │
                         └──────────┬──────────┘
                                    │
                  ┌─────────────────┼─────────────────┐
                  │                 │                 │
                  ▼                 ▼                 ▼
              BlueBoat            Drone             Rover
                  │                 │                 │
                  └─────────────────┼─────────────────┘
                                    │
                                   rclUE
                                    │
                                   ROS 2
                                    │
                     ┌──────────────┼──────────────┐
                     │              │              │
                     ▼              ▼              ▼
                 Perception      Planning       Control
                     │              │              │
                     └──────────────┼──────────────┘
                                    │
                                    ▼
                               Python / C++
                                    │
                                    ▼
                             Robotics Algorithms
```

This architecture is intended to allow the same high-level robotics algorithms to operate independently of the simulator implementation.

---

# 🔭 Future BlueSim Vision

The intended evolution of BlueSim is:

```text
                    ┌───────────────────────┐
                    │       BlueSim         │
                    │ Multi-Robot Simulator │
                    └───────────┬───────────┘
                                │
              ┌─────────────────┼─────────────────┐
              │                 │                 │
              ▼                 ▼                 ▼
          Surface            Aerial             Ground
           Robot              Robot              Robot
              │                 │                 │
          BlueBoat             Drone             Rover
              │                 │                 │
              └─────────────────┼─────────────────┘
                                │
                              ROS 2
                                │
                    ┌───────────┴───────────┐
                    │                       │
                 Sensors                 Control
                    │                       │
                    ▼                       ▼
                Perception              Planning
                    │                       │
                    └───────────┬───────────┘
                                │
                                ▼
                       Cooperative Robotics
```

The eventual goal is to provide a flexible simulation platform for testing heterogeneous robotic systems and their interaction through ROS 2.

---

# 📚 Documentation

BlueSim documentation is separated into two levels.

## Main BlueSim Documentation

This README covers:

- Project overview
- Simulation capabilities
- Robotic platforms
- Environment
- Multi-robot architecture
- Camera/sensor overview
- Project structure
- Setup
- Roadmap
- Research applications
- Experimental recordings

## ROS 2 / rclUE Documentation

Detailed ROS 2 and Unreal Engine integration is documented separately:

```text
ros2_ws/src/bluesim_ros2/README.md
```

That documentation covers:

- `rclUE`
- ROS 2 node architecture
- Publishers
- Subscribers
- ROS 2 messages
- Camera publisher implementation
- `BlueSimCameraPublisher.h`
- `BlueSimCameraPublisher.cpp`
- Python ROS 2 interfaces
- Camera recording
- Robot command interfaces
- ROS 2 debugging

---

# 📁 Important Files

### Main Unreal Project

```text
unreal/BlueSim/BlueSim.uproject
```

### BlueSim Unreal C++ Source

```text
unreal/BlueSim/Source/BlueSim/
```

### BlueSim Camera Publisher

```text
unreal/BlueSim/Source/BlueSim/BlueSimCameraPublisher.h
unreal/BlueSim/Source/BlueSim/BlueSimCameraPublisher.cpp
```

### rclUE Plugin

```text
unreal/BlueSim/Plugins/rclUE/
```

### ROS 2 Workspace

```text
ros2_ws/
```

### ROS 2 Package

```text
ros2_ws/src/bluesim_ros2/
```

### ROS 2 Documentation

```text
ros2_ws/src/bluesim_ros2/README.md
```

### Recorded Camera Experiments

```text
ros2_ws/Recorded Videos/
```

---

# 🧰 Technology Stack

<p align="center">

| Component | Technology |
|---|---|
| Simulation | Unreal Engine 5.4.4 |
| Programming | C++ / Python |
| Robot Physics | Unreal Engine Physics |
| ROS Middleware | ROS 2 |
| Unreal ↔ ROS 2 Bridge | rclUE |
| ROS 2 Python Interface | `rclpy` |
| Image Processing | OpenCV / NumPy |
| Version Control | Git / GitHub |
| Platform | Ubuntu 22.04 LTS |

</p>

---

# 📌 Current Project Status

```text
┌────────────────────────────────────────────────────────┐
│                    BlueSim Status                      │
├────────────────────────────────────────────────────────┤
│ Unreal Engine Simulation            ✅ Complete        │
│ BlueBoat                            ✅ Complete        │
│ Drone                               ✅ Complete        │
│ Rover                               ✅ Complete        │
│ Manual Robot Control                ✅ Complete        │
│ Onboard Camera Simulation           ✅ Complete        │
│ rclUE / ROS 2 Integration           ✅ Complete        │
│ ROS 2 Robot Commands                ✅ Complete        │
│ Python ROS 2 Interface              ✅ Complete        │
│ Camera Recording                    ✅ Complete        │
│ LiDAR                               🚧 Planned         │
│ Advanced Autonomous Control         🚧 In Progress     │
│ Multi-Robot Cooperation             🚧 Future          │
└────────────────────────────────────────────────────────┘
```

---

# 👤 Author

**Mahmmud Qatmh**

Robotics and Autonomous Systems  
University of Turku

---

# ⭐ BlueSim

```text
                         BLUE SIM
                            │
          ┌─────────────────┼─────────────────┐
          │                 │                 │
          ▼                 ▼                 ▼
       BlueBoat           Drone             Rover
          │                 │                 │
          └─────────────────┼─────────────────┘
                            │
                           rclUE
                            │
                           ROS 2
                            │
                  ┌─────────┴─────────┐
                  │                   │
                  ▼                   ▼
               Sensors             Commands
                  │                   ▲
                  ▼                   │
             Python / C++ ────────────┘
                  │
                  ▼
           Robotics Algorithms
```

**BlueSim provides a common simulation environment for surface, aerial, and ground robots, with Unreal Engine handling simulation and ROS 2 providing the interface for external robotics software.**