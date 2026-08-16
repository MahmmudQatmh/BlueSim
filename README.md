# 🌊 BlueSim — Autonomous Maritime Vehicle Simulator

[![Unreal Engine](https://img.shields.io/badge/UnrealEngine-5.4-blue?logo=unrealengine)](https://www.unrealengine.com/)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20/%20Jazzy-brightgreen?logo=ros)](https://docs.ros.org/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-red)](https://ardupilot.org/)
[![Status](https://img.shields.io/badge/Roadmap-Phase_F_\(In_Progress\)-orange)](#-project-roadmap--progress)

> High-fidelity simulation platform built in **Unreal Engine 5** designed to emulate the **BlueRobotics BlueBoat** in realistic ocean environments, integrated with **ROS 2** and **ArduPilot SITL** for multi-agent cooperative control research.

---

## 📌 Context & Objectives

This project is developed as part of research activities at the **University of Turku** during Summer 2026.

The primary objective is to bridge physical hardware experimentation with software-in-the-loop simulation. Alongside deploying ROS 2 on real hardware (configuring the Raspberry Pi onboard the BlueBoat), **BlueSim** serves as a digital twin for safe testing, sensor validation, and algorithm development.

### Key Goals

* 🌊 **Physics-accurate Ocean Simulation**: Model realistic buoyancy, hydrodynamic drag, and wave interaction for the BlueBoat USV.
* 🛰️ **Hardware-In-The-Loop / SITL**: Connect seamlessly to ArduPilot SITL and QGroundControl for realistic navigation behavior.
* 🤖 **ROS 2 Bridge**: Stream synthetic sensor outputs (cameras, odometry, IMU) and accept actuation controls (`cmd_vel`).
* 🚁 **Multi-Robot Cooperative Control**: Support collaborative operations between the BlueBoat, autonomous drones, and ground rovers.

---

## 🚀 Current Simulation Capabilities

The current prototype contains three robotic agents in the Unreal Engine environment:

* 🚤 **BlueBoat** — manually controllable with physics-based movement.
* 🚁 **Drone** — manually controllable within the simulation.
* 🛞 **Rover** — manually controllable with physics-based forward/backward movement and steering.

Each robot has its own observer pawn, allowing the user to switch between robot viewpoints during simulation.

The observer system currently supports:

```text
C → Drone → Boat → Rover → Drone → ...
```

The current prototype therefore provides a basic **multi-robot simulation and manual-control framework** that can be extended with autonomous control, sensors, ROS 2, and ArduPilot integration.

---

## 🏗️ Current Multi-Robot Setup

```text
                    ┌──────────────────────┐
                    │       BlueSim        │
                    │    Unreal Engine 5   │
                    └──────────┬───────────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
              ▼                ▼                ▼
        ┌──────────┐     ┌──────────┐     ┌──────────┐
        │  Drone   │     │ BlueBoat │     │  Rover   │
        │          │     │          │     │          │
        │ Movement │     │ Movement │     │ Movement │
        └────┬─────┘     └────┬─────┘     └────┬─────┘
             │                │                │
             ▼                ▼                ▼
        ┌──────────┐     ┌──────────┐     ┌──────────┐
        │ Observer │     │ Observer │     │ Observer │
        │   Pawn   │     │   Pawn   │     │   Pawn   │
        └────┬─────┘     └────┬─────┘     └────┬─────┘
             │                │                │
             └────────────────┼────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │ Observer Switch  │
                    │   C: 0 → 1 → 2  │
                    └──────────────────┘
```

The current implementation focuses on establishing a reliable manual simulation foundation before integrating autonomous control and external middleware.

---

## 🚀 Project Architecture

The targeted execution pipeline establishes a closed-loop system between simulation physics, autopilot software, ROS 2 nodes, and Ground Control Stations:

```text
┌─────────────────┐       SITL        ┌─────────────┐
│  ArduPilot      │ ◄───────────────► │   BlueSim   │
│  (Navigation)   │                   │ (UE5 Ocean) │
└────────┬────────┘                   └──────┬──────┘
         │                                   │ Sensor Data /
         │ Telemetry                         │ Camera Feeds
         ▼                                   ▼
┌─────────────────┐                   ┌─────────────┐
│ QGroundControl  │                   │    ROS 2    │
│  (GCS Interface)│                   │ (Middleware)│
└─────────────────┘                   └──────┬──────┘
                                             │
                                             ▼
                                  ┌────────────────────┐
                                  │  Multi-Robot /     │
                                  │  Coop Control      │
                                  └────────────────────┘
```

---

## 🗺️ Project Roadmap & Progress

The development of BlueSim is structured across **seven planned phases**.

### Current Status: ⚙️ Phase F — Multi-Robot System Expansion *(In Progress)*

* [x] **Phase A — Build the Ocean Environment**

  * [x] Configure Unreal Engine 5 project settings & renderer
  * [x] Enable Water Plugin, Landmass, and Modeling Tools
  * [x] Set up the initial Ocean, Sky Atmosphere, and Directional Sun Lighting
  * [x] Establish the base simulation environment

* [ ] **Phase B — Floating Boat Physics**

  * [ ] Implement realistic buoyancy model
  * [x] Import BlueBoat 3D mesh model & physics asset
  * [x] Implement initial physics-based manual movement
  * [ ] Validate realistic hull hydrodynamics and water interaction

* [ ] **Phase C — Synthetic Sensor Integration**

  * [ ] Add onboard RGB camera sensors to the BlueBoat
  * [ ] Configure render targets and frame rate synchronization inside UE5
  * [ ] Add additional simulated sensors

* [ ] **Phase D — ROS 2 Middleware Integration**

  * [ ] Establish ROS 2 bridge node
  * [ ] Publish synthetic sensor topics (`/camera/image`, `/odom`, `/tf`)
  * [ ] Subscribe to control commands (`/cmd_vel`)

* [ ] **Phase E — ArduPilot SITL Integration**

  * [ ] Establish socket connection between UE5 and ArduPilot SITL
  * [ ] Connect telemetry streams to QGroundControl
  * [ ] Validate autonomous waypoint navigation on water

* [ ] **Phase F — Multi-Robot System Expansion** *(Current)*

  * [x] Introduce BlueBoat, Aerial Drone, and Ground Rover into the simulation
  * [x] Implement manual movement for the simulated robots
  * [x] Implement physics-based Rover movement and steering
  * [x] Add individual observer pawns for each robot
  * [x] Implement observer switching between Drone, BlueBoat, and Rover
  * [ ] Isolate individual ROS 2 namespaces and sensor streams per vehicle
  * [ ] Integrate autonomous control for the individual agents

* [ ] **Phase G — Cooperative Control & Field Testing**

  * [ ] Implement multi-agent algorithms (Formation Control, Search & Rescue, Sensor Sharing)
  * [ ] Integrate cooperative perception and control
  * [ ] Cross-validate simulation performance against physical BlueBoat field deployment data

---

## 🛠️ Prerequisites & Setup

### Requirements

* **OS**: Ubuntu 22.04 LTS (or compatible Linux distro)
* **GPU**: Vulkan 1.2+ compatible dedicated graphics card (NVIDIA RTX recommended)
* **Engine**: Unreal Engine 5.4+
* **Middleware**: ROS 2 (Humble / Jazzy)

### Quick Start

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/MahmmudQatmh/BlueSim.git
   cd BlueSim
   ```
